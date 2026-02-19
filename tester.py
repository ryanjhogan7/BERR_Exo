#!/usr/bin/env python3
"""Strategy 4b: Position-based torque with fixed range + CSV logging"""
import odrive
from odrive.enums import AxisState, ControlMode, InputMode
import time
import csv
from datetime import datetime

print("Connecting...")
odrv = odrive.find_any(timeout=10)
axis = odrv.axis0

odrv.clear_errors()
axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
axis.controller.config.input_mode = InputMode.PASSTHROUGH
axis.controller.input_torque = 0

print("Entering closed-loop control...")
axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.3)

if axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
    print(f"FAILED - State: {axis.current_state}, Errors: {axis.active_errors}")
    print(f"  disarm_reason: {axis.disarm_reason}")
    print(f"  procedure_result: {axis.procedure_result}")
    exit()

# === SETUP ===
input("\nMove arm to START position (extended), press Enter...")
pos_start = axis.pos_vel_mapper.pos_rel
print(f"Start position: {pos_start:.3f} turns")

POS_RANGE = 120.0 / 360.0

print(f"End position will be: {pos_start + POS_RANGE:.3f} turns")

# === TUNE THESE ===
MAX_TORQUE = -2.5
SLEW_RATE = 5.0

current_torque = 0.0
dt = 0.02

# === CSV SETUP ===
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"berr_exo_log_{timestamp}.csv"
csvfile = open(filename, 'w', newline='')
writer = csv.writer(csvfile)
writer.writerow([
    'time_s',
    'pos_turns',
    'pos_deg',
    'normalized',
    'velocity_turns_s',
    'commanded_torque_Nm',
    'desired_torque_Nm',
    'torque_estimate_Nm',
    'input_iq_A',
    'effective_current_lim_A',
    'motor_temp_C',
    'fet_temp_C',
    'vbus_V',
    'ibus_A',
    'power_elec_W',
    'power_mech_W',
    'power_loss_W',
    'active_errors'
])

print(f"\nLogging to: {filename}")
print(f"Position-based torque active: max={MAX_TORQUE} Nm")
print("Press Ctrl+C to stop.\n")

t_start = time.time()

try:
    while True:
        t_now = time.time() - t_start
        pos = axis.pos_vel_mapper.pos_rel
        vel = axis.pos_vel_mapper.vel

        normalized = (pos - pos_start) / POS_RANGE
        normalized = max(0, min(1, normalized))

        if normalized <= 0:
            desired = 0.0
        elif normalized >= 1:
            desired = 0.0
        else:
            desired = MAX_TORQUE

        max_change = SLEW_RATE * dt
        if desired > current_torque:
            current_torque = min(desired, current_torque + max_change)
        else:
            current_torque = max(desired, current_torque - max_change)

        axis.controller.input_torque = current_torque

        # Read telemetry
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
            f"{errors}"
        ])

        print(f"t={t_now:.1f}s  pos={pos*360:.0f}°  norm={normalized:.2f}  "
              f"τ={current_torque:.2f}Nm  τ_est={torque_est:.2f}Nm  "
              f"Tmotor={motor_temp:.0f}°C  Tfet={fet_temp:.0f}°C  "
              f"P={vbus*ibus:.1f}W")

        time.sleep(dt)

except KeyboardInterrupt:
    print("\nStopping...")

axis.controller.input_torque = 0
axis.requested_state = AxisState.IDLE
csvfile.close()
print(f"Done. Data saved to {filename}")