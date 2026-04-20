#!/usr/bin/env python3
"""BERR EXO - Combined HTTP + WebSocket server.

Serves the frontend at / and a WebSocket at /ws.
Runs the torque control loop with disconnect-safe auto-disarm.
Binds to Tailscale interface only so it is invisible on public WiFi.

Layout expected:
    exo_server.py
    frontend/index.html
    logs/                  (auto-created)
"""
import asyncio
import csv
import json
import math
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from aiohttp import web, WSMsgType
import odrive
from odrive.enums import AxisState, ControlMode, InputMode


# ---- Config ----
HTTP_PORT = 8080
HEARTBEAT_SEC = 2.0      # aiohttp WS ping interval; drops dead clients
DT_DEFAULT = 0.02        # 50 Hz control loop
SCRIPT_DIR = Path(__file__).resolve().parent
FRONTEND_DIR = SCRIPT_DIR / "frontend"
LOG_DIR = SCRIPT_DIR / "logs"


# ---- Curve math (Catmull-Rom, unchanged from original) ----
def catmull_rom(p0, p1, p2, p3, t):
    t2 = t * t
    t3 = t2 * t
    v = 0.5 * (
        (2 * p1)
        + (-p0 + p2) * t
        + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
        + (-p0 + 3 * p1 - 3 * p2 + p3) * t3
    )
    return max(0.0, min(1.0, v))


def evaluate_curve(curve, normalized_pos):
    if normalized_pos <= 0.0:
        return curve[0]
    if normalized_pos >= 1.0:
        return curve[-1]
    n = len(curve)
    fi = normalized_pos * (n - 1)
    i = int(fi)
    frac = fi - i
    p0 = curve[max(0, i - 1)]
    p1 = curve[i]
    p2 = curve[min(n - 1, i + 1)]
    p3 = curve[min(n - 1, i + 2)]
    return catmull_rom(p0, p1, p2, p3, frac)


# ---- Session metrics ----
# Rep detection uses normalized ROM position [0..1] with hysteresis.
# Thresholds deliberately loose: PT patients often don't reach their
# ceiling on every rep, so 70/30 hysteresis counts legitimate reps
# without false-triggering on small oscillations.
REP_TOP_THRESHOLD = 0.70
REP_BOTTOM_THRESHOLD = 0.30
TUT_TORQUE_THRESHOLD = 0.2     # Nm, minimum commanded torque for TUT
TUT_VEL_THRESHOLD = 0.03       # turns/s (~10 deg/s), "is moving"
TUT_POS_THRESHOLD = 0.15       # normalized, "is above rest position"
SLOW_READ_EVERY = 10           # read slow-changing ODrive props every N ticks (5 Hz)


class SessionMetrics:
    """Accumulates workout metrics during a session."""

    def __init__(self):
        self.total_work_J = 0.0
        self.peak_torque_Nm = 0.0
        self.peak_velocity_rps = 0.0
        self.peak_power_W = 0.0
        self.peak_motor_temp_C = 0.0
        self.peak_rom_deg = 0.0
        self.time_under_tension_s = 0.0
        self._torque_samples_sum = 0.0
        self._torque_samples_n = 0
        self.reps = 0
        self._rep_state = "bottom"

    def update(self, t_now, torque_cmd, vel_turns_s, motor_temp,
               normalized, pos_delta_deg, dt):
        torque_abs = abs(torque_cmd)
        ang_vel_rad_s = abs(vel_turns_s) * 2.0 * math.pi
        power_W = torque_abs * ang_vel_rad_s

        self.total_work_J += power_W * dt
        if torque_abs > self.peak_torque_Nm:
            self.peak_torque_Nm = torque_abs
        if abs(vel_turns_s) > self.peak_velocity_rps:
            self.peak_velocity_rps = abs(vel_turns_s)
        if power_W > self.peak_power_W:
            self.peak_power_W = power_W
        if motor_temp > self.peak_motor_temp_C:
            self.peak_motor_temp_C = motor_temp
        if pos_delta_deg > self.peak_rom_deg:
            self.peak_rom_deg = pos_delta_deg

        if torque_abs > TUT_TORQUE_THRESHOLD:
            # TUT requires patient to actually be loaded: either moving
            # against resistance, or held above the rest position.
            # Sitting at bottom with torque commanded (armed idle) does NOT
            # count as time under tension.
            user_active = (abs(vel_turns_s) > TUT_VEL_THRESHOLD
                           or normalized > TUT_POS_THRESHOLD)
            if user_active:
                self.time_under_tension_s += dt
                self._torque_samples_sum += torque_abs
                self._torque_samples_n += 1

        # Rep state machine with hysteresis
        if self._rep_state == "bottom" and normalized > REP_TOP_THRESHOLD:
            self._rep_state = "top"
        elif self._rep_state == "top" and normalized < REP_BOTTOM_THRESHOLD:
            self._rep_state = "bottom"
            self.reps += 1

    def finalize(self, duration_s, max_torque_cfg, pos_range_deg, curve,
                 slew_rate, session_id):
        avg_torque = (self._torque_samples_sum / self._torque_samples_n
                      if self._torque_samples_n else 0.0)
        avg_power = (self.total_work_J / duration_s) if duration_s > 0 else 0.0

        return {
            "session_id": session_id,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "duration_s": round(duration_s, 1),
            "config": {
                "max_torque_Nm": round(abs(max_torque_cfg), 2),
                "pos_range_deg": pos_range_deg,
                "slew_rate_Nm_s": slew_rate,
                "curve": curve,
            },
            "metrics": {
                "reps": self.reps,
                "total_work_J": round(self.total_work_J, 1),
                "peak_torque_Nm": round(self.peak_torque_Nm, 2),
                "avg_torque_Nm": round(avg_torque, 2),
                "peak_rom_deg": round(self.peak_rom_deg, 1),
                "rom_pct_of_target": round(
                    min(100.0, self.peak_rom_deg / pos_range_deg * 100)
                    if pos_range_deg > 0 else 0, 1),
                "peak_velocity_rps": round(self.peak_velocity_rps, 2),
                "peak_power_W": round(self.peak_power_W, 1),
                "avg_power_W": round(avg_power, 1),
                "peak_motor_temp_C": round(self.peak_motor_temp_C, 1),
                "time_under_tension_s": round(self.time_under_tension_s, 1),
                "volume_Nm_reps": round(self.peak_torque_Nm * self.reps, 1),
            },
        }


# ---- Server ----
class ExoServer:
    def __init__(self):
        self.odrv = None
        self.axis = None
        self.session_active = False
        self.session_task = None

    def connect_odrive(self):
        print("Connecting to ODrive...")
        self.odrv = odrive.find_any(timeout=10.0)
        if self.odrv is None:
            print("ERROR: no ODrive found. Is it powered and plugged in?")
            sys.exit(1)
        self.axis = self.odrv.axis0
        print(f"ODrive connected, VBUS={self.odrv.vbus_voltage:.2f}V")
        if not self.axis.commutation_mapper.config.offset_valid:
            print("ERROR: motor not calibrated. Run calibrate.py first.")
            sys.exit(1)

    async def disarm(self):
        """Safely return motor to IDLE. Safe to call multiple times."""
        try:
            self.axis.controller.input_torque = 0.0
            self.axis.requested_state = AxisState.IDLE
            print("Motor disarmed")
        except Exception as e:
            print(f"Disarm error (ignored): {e}")

    async def run_session(self, ws, config):
        curve = config.get("curve", [0.8] * 12)
        max_torque = float(config.get("max_torque", -1.0))
        slew_rate = float(config.get("slew_rate", 5.0))
        pos_range_deg = float(config.get("rom", 120.0))
        dt = float(config.get("dt", DT_DEFAULT))

        print(f"Starting session: max_torque={max_torque} Nm, "
              f"rom={pos_range_deg} deg, slew={slew_rate} Nm/s")

        # Arm
        self.odrv.clear_errors()
        self.axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.axis.controller.config.input_mode = InputMode.PASSTHROUGH
        self.axis.controller.input_torque = 0.0
        self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        await asyncio.sleep(0.3)

        if self.axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
            err = (f"Arm failed: state={self.axis.current_state}, "
                   f"errors={self.axis.active_errors}, "
                   f"disarm_reason={self.axis.disarm_reason}")
            print(err)
            if not ws.closed:
                await ws.send_str(json.dumps({"type": "error", "message": err}))
            self.session_active = False
            return

        pos_start = self.axis.pos_vel_mapper.pos_rel
        pos_range = pos_range_deg / 360.0
        current_torque = 0.0
        metrics = SessionMetrics()

        # Logging
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = LOG_DIR / f"exo_log_{ts}.csv"
        meta_path = LOG_DIR / f"exo_log_{ts}_meta.json"

        with open(meta_path, "w") as mf:
            json.dump({
                "curve": curve,
                "max_torque_Nm": max_torque,
                "slew_rate_Nm_s": slew_rate,
                "pos_range_deg": pos_range_deg,
                "dt_s": dt,
                "pos_start_turns": pos_start,
            }, mf, indent=2)

        csvfile = open(csv_path, "w", newline="")
        writer = csv.writer(csvfile)
        writer.writerow([
            "time_s", "pos_turns", "pos_deg", "normalized",
            "velocity_turns_s", "curve_multiplier",
            "commanded_torque_Nm", "desired_torque_Nm",
            "torque_estimate_Nm", "input_iq_A",
            "motor_temp_C", "fet_temp_C",
            "vbus_V", "ibus_A",
            "power_elec_W", "power_mech_W",
            "active_errors",
        ])
        print(f"Logging to {csv_path}")

        t_start = time.time()
        tick = 0

        # Slow-changing ODrive properties are read every SLOW_READ_EVERY ticks
        # (5 Hz instead of 50 Hz). Halves USB traffic and gives the control
        # loop real headroom. These values update slowly anyway.
        motor_temp = self.axis.motor.motor_thermistor.temperature
        fet_temp = self.axis.motor.fet_thermistor.temperature
        vbus = self.odrv.vbus_voltage
        ibus = self.odrv.ibus
        power_elec = self.axis.motor.electrical_power
        power_mech = self.axis.motor.mechanical_power

        try:
            while self.session_active:
                tick += 1
                t_now = time.time() - t_start
                pos = self.axis.pos_vel_mapper.pos_rel
                vel = self.axis.pos_vel_mapper.vel

                normalized = abs(pos - pos_start) / pos_range
                normalized = min(1.0, normalized)

                curve_mult = evaluate_curve(curve, normalized)
                desired = max_torque * curve_mult

                max_change = slew_rate * dt
                if desired > current_torque:
                    current_torque = min(desired, current_torque + max_change)
                else:
                    current_torque = max(desired, current_torque - max_change)

                self.axis.controller.input_torque = current_torque

                # Fast telemetry (read every tick, needed for safety + feedback)
                input_iq = self.axis.motor.input_iq
                torque_est = self.axis.motor.torque_estimate
                errors = self.axis.active_errors

                # Slow telemetry (read every SLOW_READ_EVERY ticks)
                if tick % SLOW_READ_EVERY == 0:
                    motor_temp = self.axis.motor.motor_thermistor.temperature
                    fet_temp = self.axis.motor.fet_thermistor.temperature
                    vbus = self.odrv.vbus_voltage
                    ibus = self.odrv.ibus
                    power_elec = self.axis.motor.electrical_power
                    power_mech = self.axis.motor.mechanical_power

                # Update session metrics
                pos_delta_deg = abs(pos - pos_start) * 360
                metrics.update(t_now, current_torque, vel, motor_temp,
                               normalized, pos_delta_deg, dt)

                writer.writerow([
                    f"{t_now:.3f}", f"{pos:.5f}",
                    f"{(pos - pos_start) * 360:.1f}", f"{normalized:.3f}",
                    f"{vel:.3f}", f"{curve_mult:.4f}",
                    f"{current_torque:.4f}", f"{desired:.4f}",
                    f"{torque_est:.4f}", f"{input_iq:.3f}",
                    f"{motor_temp:.1f}", f"{fet_temp:.1f}",
                    f"{vbus:.2f}", f"{ibus:.3f}",
                    f"{power_elec:.2f}", f"{power_mech:.2f}",
                    f"{errors}",
                ])

                telemetry = {
                    "type": "telemetry",
                    "torque": round(current_torque, 2),
                    "torque_estimate": round(torque_est, 2),
                    "pos_deg": round(abs(pos - pos_start) * 360, 1),
                    "vel": round(vel, 2),
                    "current": round(input_iq, 2),
                    "motor_temp": round(motor_temp, 1),
                    "fet_temp": round(fet_temp, 1),
                    "power": round(vbus * ibus, 1),
                    "power_elec": round(power_elec, 1),
                    "power_mech": round(power_mech, 1),
                    "vbus": round(vbus, 1),
                    "curve_mult": round(curve_mult, 2),
                    "active_errors": errors,
                    "time": round(t_now, 1),
                    # Live workout metrics
                    "reps": metrics.reps,
                    "work_J": round(metrics.total_work_J, 1),
                    "tut_s": round(metrics.time_under_tension_s, 1),
                }

                # Abort if client dropped mid-session
                if ws.closed:
                    print("WS closed mid-session, aborting")
                    break
                try:
                    await ws.send_str(json.dumps(telemetry))
                except ConnectionResetError:
                    print("WS reset mid-session, aborting")
                    break

                await asyncio.sleep(dt)
        except Exception as e:
            print(f"Session loop error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            csvfile.close()
            print(f"Log saved: {csv_path}")

            # Write session summary JSON for the history feed
            try:
                duration_s = time.time() - t_start
                summary = metrics.finalize(
                    duration_s=duration_s,
                    max_torque_cfg=max_torque,
                    pos_range_deg=pos_range_deg,
                    curve=curve,
                    slew_rate=slew_rate,
                    session_id=ts,
                )
                summary_path = LOG_DIR / f"exo_log_{ts}_summary.json"
                with open(summary_path, "w") as sf:
                    json.dump(summary, sf, indent=2)
                print(f"Summary: {metrics.reps} reps, "
                      f"{metrics.total_work_J:.1f} J, "
                      f"peak {metrics.peak_torque_Nm:.2f} Nm")
            except Exception as e:
                print(f"Summary write failed (ignored): {e}")

            await self.disarm()
            self.session_active = False
            if not ws.closed:
                try:
                    await ws.send_str(json.dumps(
                        {"type": "status", "message": "stopped"}))
                except Exception:
                    pass

    # ---- Routes ----
    async def index_handler(self, request):
        index_path = FRONTEND_DIR / "index.html"
        if not index_path.exists():
            return web.Response(
                status=500,
                text=f"frontend/index.html not found at {index_path}",
            )
        return web.FileResponse(index_path)

    async def sessions_handler(self, request):
        """Return list of past session summaries plus lifetime totals."""
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        summaries = []
        for path in sorted(LOG_DIR.glob("exo_log_*_summary.json"), reverse=True):
            try:
                with open(path) as f:
                    summaries.append(json.load(f))
            except Exception as e:
                print(f"Skipping bad summary {path.name}: {e}")

        lifetime = {
            "session_count": len(summaries),
            "total_reps": sum(
                s.get("metrics", {}).get("reps", 0) for s in summaries),
            "total_work_J": round(sum(
                s.get("metrics", {}).get("total_work_J", 0)
                for s in summaries), 1),
            "total_tut_s": round(sum(
                s.get("metrics", {}).get("time_under_tension_s", 0)
                for s in summaries), 1),
            "total_duration_s": round(sum(
                s.get("duration_s", 0) for s in summaries), 1),
        }
        return web.json_response(
            {"lifetime": lifetime, "sessions": summaries})

    async def clear_sessions_handler(self, request):
        """Delete all session summary JSONs. CSV raw logs are preserved
        so the underlying engineering data is never lost."""
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        deleted = 0
        for path in LOG_DIR.glob("exo_log_*_summary.json"):
            try:
                path.unlink()
                deleted += 1
            except Exception as e:
                print(f"Failed to delete {path.name}: {e}")
        print(f"Cleared {deleted} session summaries (CSVs kept)")
        return web.json_response({"deleted": deleted})

    async def ws_handler(self, request):
        ws = web.WebSocketResponse(heartbeat=HEARTBEAT_SEC)
        await ws.prepare(request)
        client = request.headers.get("X-Forwarded-For", request.remote)
        print(f"Client connected from {client}")

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                    except json.JSONDecodeError:
                        continue
                    cmd = data.get("command")
                    if cmd == "start":
                        if self.session_active:
                            print("Start ignored: session already active")
                            continue
                        self.session_active = True
                        self.session_task = asyncio.create_task(
                            self.run_session(ws, data.get("config", {}))
                        )
                    elif cmd == "stop":
                        print("Stop command received")
                        self.session_active = False
                elif msg.type == WSMsgType.ERROR:
                    print(f"WS error: {ws.exception()}")
                    break
        finally:
            # Safety net: any time the WS handler exits with a session running,
            # kill it and disarm. Covers browser close, tab refresh, WiFi drop,
            # heartbeat timeout, etc.
            if self.session_active:
                print("Client disconnected mid-session, forcing disarm")
                self.session_active = False
                if self.session_task:
                    try:
                        await asyncio.wait_for(self.session_task, timeout=2.0)
                    except asyncio.TimeoutError:
                        print("Session task did not end in time, disarming directly")
                        await self.disarm()
            print(f"Client disconnected: {client}")
        return ws


def get_tailscale_ip():
    """Return Tailscale IPv4, or None if tailscale unavailable."""
    try:
        out = subprocess.check_output(
            ["tailscale", "ip", "-4"], timeout=3, text=True
        ).strip()
        ip = out.splitlines()[0].strip()
        if ip and ip.startswith("100."):
            return ip
    except Exception as e:
        print(f"Tailscale query failed: {e}")
    return None


async def main(server):
    app = web.Application()
    app.router.add_get("/", server.index_handler)
    app.router.add_get("/ws", server.ws_handler)
    app.router.add_get("/api/sessions", server.sessions_handler)
    app.router.add_post("/api/sessions/clear", server.clear_sessions_handler)

    bind_ip = get_tailscale_ip()
    if bind_ip:
        print(f"Binding to Tailscale interface: {bind_ip}:{HTTP_PORT}")
        print("Server invisible on public WiFi, reachable only via tailnet.")
    else:
        bind_ip = "127.0.0.1"
        print(f"WARNING: Tailscale not available, binding to loopback only "
              f"({bind_ip}:{HTTP_PORT})")
        print("Fix tailscale and restart to enable remote access.")

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, bind_ip, HTTP_PORT)
    await site.start()

    print(f"\nReady. Open http://exo:{HTTP_PORT} from any tailnet device.\n")

    try:
        await asyncio.Event().wait()
    finally:
        await runner.cleanup()


if __name__ == "__main__":
    # ODrive connection must happen OUTSIDE the asyncio loop because
    # odrive.find_any() runs its own asyncio.run() internally and nested
    # event loops are not allowed.
    _server = ExoServer()
    _server.connect_odrive()
    try:
        asyncio.run(main(_server))
    except KeyboardInterrupt:
        print("\nShutdown.")