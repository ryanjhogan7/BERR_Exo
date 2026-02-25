#!/usr/bin/env python3
"""BERR EXO — Position-dependent torque control with curve profiles + CSV logging.

Replaces the flat on/off torque model with a user-definable torque curve
that maps normalized arm position (0-1) to a torque multiplier (0-1),
scaled by max_torque. Curves can be presets or loaded from a JSON file
(matching the 12-bar EQ editor in the frontend).
"""

import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import odrive
from odrive.enums import AxisState, ControlMode, InputMode

# ── Preset Curves ───────────────────────────────────────────────────────────
# Each preset is 12 normalized values (0.0–1.0) at evenly spaced positions
# through the ROM, matching the frontend EQ bar count.

NUM_BARS = 12


PRESETS = {
    # Constant resistance — baseline strength testing
    "flat": [0.8] * NUM_BARS,

    # Linear ramp — accommodating resistance, increases with mechanical advantage
    "linear": [i / (NUM_BARS - 1) for i in range(NUM_BARS)],

    # Bell curve — matches muscle length-tension relationship
    # Peak force at mid-sarcomere length (mid-ROM)
    "bell": [math.sin(i / (NUM_BARS - 1) * math.pi) for i in range(NUM_BARS)],

    # Eccentric focus — heavier resistance on return phase (higher indices)
    # Eccentric overload: greater hypertrophy & tendon adaptation
    # Refs: Roig et al. 2009 (BJSM), Lorenz & Reiman 2011
    "eccentric": [0.3 + 0.7 * (i / (NUM_BARS - 1)) ** 2 for i in range(NUM_BARS)],

    # Ramp down — protect end-range, load early ROM
    "ramp_down": [1.0 - i / (NUM_BARS - 1) for i in range(NUM_BARS)],

    # Plateau — ramp up, hold, ramp down. Protects end-ranges.
    "plateau": [
        (t / 0.2) * 0.9 if t < 0.2
        else ((1 - t) / 0.2) * 0.9 if t > 0.8
        else 0.9
        for t in (i / (NUM_BARS - 1) for i in range(NUM_BARS))
    ],
}


# ── Interpolation ───────────────────────────────────────────────────────────

def catmull_rom(p0: float, p1: float, p2: float, p3: float, t: float) -> float:
    """Catmull-Rom spline interpolation between p1 and p2."""
    t2 = t * t
    t3 = t2 * t
    v = 0.5 * (
        (2 * p1)
        + (-p0 + p2) * t
        + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
        + (-p0 + 3 * p1 - 3 * p2 + p3) * t3
    )
    return max(0.0, min(1.0, v))


def evaluate_curve(curve: List[float], normalized_pos: float) -> float:
    """Evaluate torque multiplier (0-1) at a normalized position (0-1).

    Uses Catmull-Rom interpolation across the bar values for a smooth profile.
    Returns 0 outside [0, 1].
    """
    if normalized_pos <= 0.0 or normalized_pos >= 1.0:
        return 0.0

    n = len(curve)
    fi = normalized_pos * (n - 1)
    i = int(fi)
    frac = fi - i

    p0 = curve[max(0, i - 1)]
    p1 = curve[i]
    p2 = curve[min(n - 1, i + 1)]
    p3 = curve[min(n - 1, i + 2)]

    return catmull_rom(p0, p1, p2, p3, frac)


# ── Curve Loading ───────────────────────────────────────────────────────────

def load_curve(preset: Optional[str], curve_file: Optional[str]) -> List[float]:
    """Load a torque curve from a preset name or a JSON file.

    JSON format: {"curve": [0.1, 0.5, 0.8, ...]}  (any length >= 2)
    or just a bare array: [0.1, 0.5, 0.8, ...]
    """
    if curve_file:
        path = Path(curve_file)
        if not path.exists():
            print(f"ERROR: Curve file not found: {curve_file}")
            sys.exit(1)
        with open(path) as f:
            data = json.load(f)

        if isinstance(data, list):
            curve = data
        elif isinstance(data, dict) and "curve" in data:
            curve = data["curve"]
        else:
            print("ERROR: JSON must be an array or {\"curve\": [...]}")
            sys.exit(1)

        if len(curve) < 2:
            print("ERROR: Curve must have at least 2 points")
            sys.exit(1)

        # Clamp values
        curve = [max(0.0, min(1.0, float(v))) for v in curve]
        print(f"Loaded custom curve from {curve_file} ({len(curve)} points)")
        return curve

    name = preset or "flat"
    if name not in PRESETS:
        print(f"ERROR: Unknown preset '{name}'. Options: {list(PRESETS.keys())}")
        sys.exit(1)

    print(f"Using preset curve: {name}")
    return PRESETS[name]


# ── ODrive Connection ───────────────────────────────────────────────────────

def connect_axis(timeout: float = 10.0):
    """Connect to the ODrive and return (odrv, axis0)."""
    print("Connecting to ODrive...")
    try:
        odrv = odrive.find_any(timeout=timeout)
    except Exception as exc:
        print(f"FAILED to connect: {exc}")
        sys.exit(1)
    return odrv, odrv.axis0


# ── Main Control Loop ───────────────────────────────────────────────────────

def run(
    curve: List[float],
    max_torque: float = -1.8,
    slew_rate: float = 5.0,
    pos_range_deg: float = 120.0,
    dt: float = 0.02,
    direction: int = 1,
) -> None:
    """Position-dependent torque control with curve lookup and CSV logging.

    Args:
        curve: Normalized torque multiplier array (0-1 values).
        max_torque: Peak torque in Nm (negative = resistive for typical setup).
        slew_rate: Max torque change rate in Nm/s.
        pos_range_deg: Active ROM in degrees from start position.
        dt: Control loop period in seconds.
        direction: 1 or -1, flips the curve for left/right arm or
                   concentric vs eccentric phase emphasis.
    """
    odrv, axis = connect_axis()

    odrv.clear_errors()
    axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    axis.controller.config.input_mode = InputMode.PASSTHROUGH
    axis.controller.input_torque = 0

    print("Entering closed-loop control...")
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(0.3)

    if axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
        print(f"FAILED — state: {axis.current_state}, errors: {axis.active_errors}")
        print(f"  disarm_reason: {axis.disarm_reason}")
        axis.controller.input_torque = 0
        axis.requested_state = AxisState.IDLE
        sys.exit(1)

    # ── Zero position ──
    input("\nMove arm to START position (extended), press Enter...")
    pos_start = axis.pos_vel_mapper.pos_rel
    pos_range = pos_range_deg / 360.0
    print(f"Start: {pos_start:.3f} turns | End: {pos_start + pos_range:.3f} turns")

    # Print the active curve
    print(f"\nTorque curve ({len(curve)} pts): ", end="")
    print(" ".join(f"{v:.0%}" for v in curve))
    print(f"Max torque: {max_torque} Nm | Slew: {slew_rate} Nm/s | ROM: {pos_range_deg}°\n")

    current_torque = 0.0

    # ── CSV setup ──
    log_dir = Path("./logs")
    log_dir.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = log_dir / f"berr_exo_log_{timestamp}.csv"

    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "time_s", "pos_turns", "pos_deg", "normalized",
            "velocity_turns_s", "curve_multiplier",
            "commanded_torque_Nm", "desired_torque_Nm",
            "torque_estimate_Nm", "input_iq_A",
            "effective_current_lim_A",
            "motor_temp_C", "fet_temp_C",
            "vbus_V", "ibus_A",
            "power_elec_W", "power_mech_W", "power_loss_W",
            "active_errors",
        ])

        # Also save curve metadata as a sidecar JSON
        meta_file = log_dir / f"berr_exo_log_{timestamp}_meta.json"
        with open(meta_file, "w") as mf:
            json.dump({
                "curve": curve,
                "max_torque_Nm": max_torque,
                "slew_rate_Nm_s": slew_rate,
                "pos_range_deg": pos_range_deg,
                "direction": direction,
                "dt_s": dt,
                "pos_start_turns": pos_start,
            }, mf, indent=2)

        print(f"Logging to: {filename}")
        print("Press Ctrl+C to stop.\n")

        t_start = time.time()

        try:
            while True:
                t_now = time.time() - t_start
                pos = axis.pos_vel_mapper.pos_rel
                vel = axis.pos_vel_mapper.vel

                # Normalized position through ROM [0, 1]
                normalized = (pos - pos_start) / pos_range
                normalized = max(0.0, min(1.0, normalized))

                # Flip curve if direction is reversed
                lookup_pos = normalized if direction == 1 else (1.0 - normalized)

                # ── Curve evaluation ──
                curve_mult = evaluate_curve(curve, lookup_pos)
                desired = max_torque * curve_mult

                # ── Slew rate limiting ──
                max_change = slew_rate * dt
                if desired > current_torque:
                    current_torque = min(desired, current_torque + max_change)
                else:
                    current_torque = max(desired, current_torque - max_change)

                axis.controller.input_torque = current_torque

                # ── Telemetry ──
                motor_temp = axis.motor.motor_thermistor.temperature
                fet_temp = axis.motor.fet_thermistor.temperature
                vbus = odrv.vbus_voltage
                ibus = odrv.ibus
                input_iq = axis.motor.input_iq
                eff_lim = axis.motor.effective_current_lim
                power_elec = axis.motor.electrical_power
                power_mech = axis.motor.mechanical_power
                power_loss = axis.motor.loss_power
                torque_est = axis.motor.torque_estimate
                errors = axis.active_errors

                writer.writerow([
                    f"{t_now:.3f}",
                    f"{pos:.5f}",
                    f"{pos * 360:.1f}",
                    f"{normalized:.3f}",
                    f"{vel:.3f}",
                    f"{curve_mult:.4f}",
                    f"{current_torque:.4f}",
                    f"{desired:.4f}",
                    f"{torque_est:.4f}",
                    f"{input_iq:.3f}",
                    f"{eff_lim:.3f}",
                    f"{motor_temp:.1f}",
                    f"{fet_temp:.1f}",
                    f"{vbus:.2f}",
                    f"{ibus:.3f}",
                    f"{power_elec:.2f}",
                    f"{power_mech:.2f}",
                    f"{power_loss:.2f}",
                    f"{errors}",
                ])

                print(
                    f"t={t_now:.1f}s  pos={pos*360:.0f}°  "
                    f"norm={normalized:.2f}  curve={curve_mult:.0%}  "
                    f"τ={current_torque:.2f}Nm  τ_est={torque_est:.2f}Nm  "
                    f"Tmot={motor_temp:.0f}°C  Tfet={fet_temp:.0f}°C  "
                    f"P={vbus*ibus:.1f}W"
                )

                time.sleep(dt)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            axis.controller.input_torque = 0
            axis.requested_state = AxisState.IDLE

    print(f"Done. Data → {filename}")
    print(f"Meta → {meta_file}")


# ── CLI ─────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="BERR EXO — Position-dependent torque with curve profiles"
    )
    p.add_argument("--preset", type=str, default="flat",
                   choices=list(PRESETS.keys()),
                   help="Built-in curve preset (default: flat)")
    p.add_argument("--curve-file", type=str, default=None,
                   help="JSON file with custom curve array (overrides --preset)")
    p.add_argument("--max-torque", type=float, default=-1.0,
                   help="Peak torque in Nm (default: -1.0)")
    p.add_argument("--slew-rate", type=float, default=5.0,
                   help="Torque ramp rate in Nm/s (default: 5.0)")
    p.add_argument("--range-deg", type=float, default=120.0,
                   help="Active ROM in degrees (default: 120.0)")
    p.add_argument("--dt", type=float, default=0.02,
                   help="Control loop period in seconds (default: 0.02)")
    p.add_argument("--direction", type=int, default=1, choices=[1, -1],
                   help="Curve direction: 1=normal, -1=reversed (default: 1)")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    curve = load_curve(args.preset, args.curve_file)
    run(
        curve=curve,
        max_torque= -1.8,
        slew_rate=args.slew_rate,
        pos_range_deg=args.range_deg,
        dt=args.dt,
        direction=args.direction,
    )


if __name__ == "__main__":
    
    main()