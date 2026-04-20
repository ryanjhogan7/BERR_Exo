#!/usr/bin/env python3
"""BERR EXO - Simple interactive torque tester.

Arms the ODrive in torque-control closed loop, accepts torque commands
from stdin, and auto-disarms on temp limit, active errors, or quit.

Commands (type at prompt):
    <number>    Set torque target in Nm (smoothly ramps, clamped to MAX_TORQUE)
    0 or blank  Ramp to zero torque
    z           Hard zero (no ramp, immediate)
    s           Print status snapshot
    e           Clear ODrive errors
    arm         Enter closed-loop (from IDLE)
    disarm      Return to IDLE (motor free)
    q / quit    Disarm and exit
    Ctrl+C      Panic disarm and exit

Safety:
    - Torque clamped to +/- MAX_TORQUE
    - Auto-disarm if motor temp > TEMP_LIMIT
    - Auto-disarm if active_errors != 0
    - Auto-disarm if closed-loop state is lost unexpectedly
"""
import signal
import sys
import time

import odrive
from odrive.enums import AxisState, ControlMode, InputMode


# ---- Config ----
MAX_TORQUE = 3.0       # Nm hard clamp on commanded torque
TEMP_LIMIT = 85.0      # motor C, auto-disarm above this
SLEW_STEP = 0.05       # Nm per tick during ramp
SLEW_TICK = 0.02       # s between ramp ticks (50 Hz)


class Tester:
    def __init__(self):
        self.odrv = None
        self.axis = None
        self.cmd_torque = 0.0
        self.armed = False

    # ---- Connection ----
    def connect(self):
        print("Connecting to ODrive...")
        self.odrv = odrive.find_any(timeout=10)
        if self.odrv is None:
            print("ERROR: no ODrive found")
            sys.exit(1)
        self.axis = self.odrv.axis0
        print(f"Connected: serial={self.odrv.serial_number}, "
              f"VBUS={self.odrv.vbus_voltage:.2f}V")

        if not self.axis.commutation_mapper.config.offset_valid:
            print("ERROR: motor not calibrated. Run calibrate.py first.")
            sys.exit(1)

        # Ensure torque-control passthrough is configured
        self.axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.axis.controller.config.input_mode = InputMode.PASSTHROUGH
        self.axis.controller.input_torque = 0.0

    # ---- Arm / disarm ----
    def arm(self):
        if self.armed:
            print("Already armed.")
            return
        self.odrv.clear_errors()
        self.axis.controller.input_torque = 0.0
        self.cmd_torque = 0.0
        self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        time.sleep(0.3)
        if self.axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
            print(f"ARM FAILED: state={self.axis.current_state}, "
                  f"errors={self.axis.active_errors}, "
                  f"disarm_reason={self.axis.disarm_reason}")
            return
        self.armed = True
        print("ARMED. Motor holding at 0 Nm.")

    def disarm(self, panic=False):
        if not self.axis:
            return
        try:
            if panic:
                self.axis.controller.input_torque = 0.0
            else:
                self._ramp_to(0.0)
            self.axis.requested_state = AxisState.IDLE
            self.cmd_torque = 0.0
            self.armed = False
            print("DISARMED.")
        except Exception as e:
            print(f"Disarm error (ignored): {e}")

    # ---- Motion ----
    def _ramp_to(self, target):
        target = max(-MAX_TORQUE, min(MAX_TORQUE, float(target)))
        while abs(self.cmd_torque - target) > SLEW_STEP:
            if self.cmd_torque < target:
                self.cmd_torque = min(target, self.cmd_torque + SLEW_STEP)
            else:
                self.cmd_torque = max(target, self.cmd_torque - SLEW_STEP)
            self.axis.controller.input_torque = self.cmd_torque
            time.sleep(SLEW_TICK)
            if not self._safe():
                return
        self.cmd_torque = target
        self.axis.controller.input_torque = target

    def hard_zero(self):
        self.cmd_torque = 0.0
        self.axis.controller.input_torque = 0.0
        print("Hard zero applied.")

    # ---- Safety watchdog ----
    def _safe(self):
        if not self.armed:
            return True
        motor_t = self.axis.motor.motor_thermistor.temperature
        if motor_t > TEMP_LIMIT:
            print(f"\n!! TEMP LIMIT: motor={motor_t:.1f}C > {TEMP_LIMIT}C")
            self.disarm(panic=True)
            return False
        if self.axis.active_errors:
            print(f"\n!! ERRORS: {self.axis.active_errors}")
            self.disarm(panic=True)
            return False
        if self.axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
            print(f"\n!! LOST CLOSED LOOP: state={self.axis.current_state}, "
                  f"disarm_reason={self.axis.disarm_reason}")
            self.armed = False
            return False
        return True

    # ---- Status ----
    def status(self):
        motor_t = self.axis.motor.motor_thermistor.temperature
        fet_t = self.axis.motor.fet_thermistor.temperature
        pos = self.axis.pos_vel_mapper.pos_rel
        vel = self.axis.pos_vel_mapper.vel
        iq = self.axis.motor.input_iq
        est = self.axis.motor.torque_estimate
        vbus = self.odrv.vbus_voltage
        errs = self.axis.active_errors
        state = self.axis.current_state
        print(f"  state={state} armed={self.armed}")
        print(f"  cmd={self.cmd_torque:+.2f} Nm  est={est:+.2f} Nm  "
              f"Iq={iq:+.2f} A")
        print(f"  pos={pos * 360:+7.1f} deg  vel={vel:+.2f} turn/s")
        print(f"  motor={motor_t:.1f}C  FET={fet_t:.1f}C  VBUS={vbus:.2f}V  "
              f"errors={errs}")

    # ---- REPL ----
    def loop(self):
        print()
        print("Commands: <number> | 0 | z | s | e | arm | disarm | q")
        print(f"Limits: +/-{MAX_TORQUE} Nm, auto-disarm at {TEMP_LIMIT}C")
        print()
        while True:
            if self.armed and not self._safe():
                # watchdog disarmed us, stay in loop so user can inspect or re-arm
                pass

            prompt = f"[{self.cmd_torque:+.2f} Nm | "
            prompt += "ARMED" if self.armed else "idle"
            prompt += "] > "
            try:
                cmd = input(prompt).strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                return

            if cmd in ("q", "quit", "exit"):
                return
            elif cmd == "arm":
                self.arm()
            elif cmd == "disarm":
                self.disarm()
            elif cmd == "s" or cmd == "status":
                self.status()
            elif cmd == "e" or cmd == "clear":
                self.odrv.clear_errors()
                print("Errors cleared.")
            elif cmd == "z":
                if self.armed:
                    self.hard_zero()
                else:
                    print("Not armed.")
            elif cmd == "" or cmd == "0":
                if self.armed:
                    self._ramp_to(0.0)
                    self.status()
                else:
                    print("Not armed.")
            else:
                try:
                    target = float(cmd)
                except ValueError:
                    print("Unknown command.")
                    continue
                if not self.armed:
                    print("Not armed. Type 'arm' first.")
                    continue
                clamped = max(-MAX_TORQUE, min(MAX_TORQUE, target))
                if clamped != target:
                    print(f"Clamped {target} -> {clamped} Nm")
                self._ramp_to(clamped)
                self.status()


def main():
    tester = Tester()

    def on_sigint(signum, frame):
        print("\nCtrl+C: panic disarm")
        tester.disarm(panic=True)
        sys.exit(0)

    signal.signal(signal.SIGINT, on_sigint)

    tester.connect()
    try:
        tester.loop()
    finally:
        tester.disarm()
        print("Goodbye.")


if __name__ == "__main__":
    main()