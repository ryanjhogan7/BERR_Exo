#!/usr/bin/env python3
"""ODrive Pro setup script - D6374 + AMT21xB encoder + thermistor"""
import odrive
from odrive.enums import *
import time
import sys

def save_and_reconnect(odrv, msg="Saving configuration..."):
    print(msg)
    try:
        odrv.save_configuration()
    except Exception:
        pass  # reboots and disconnects
    time.sleep(5)
    print("Reconnecting...")
    return odrive.find_any(timeout=10)

print("Connecting to ODrive...")
odrv = odrive.find_any(timeout=10)
print(f"Found ODrive {odrv.serial_number}")

# === ERASE OLD CONFIG ===
print("Erasing old configuration...")
try:
    odrv.erase_configuration()
except Exception:
    pass
time.sleep(5)
print("Reconnecting...")
odrv = odrive.find_any(timeout=10)
axis = odrv.axis0

# === MOTOR CONFIG ===
axis.config.motor.motor_type = MotorType.HIGH_CURRENT
axis.config.motor.pole_pairs = 7
axis.config.motor.torque_constant = 8.27 / 149
axis.config.motor.calibration_current = 5.0            # match what works in GUI
axis.config.motor.resistance_calib_max_voltage = 4.0    # bumped up - D6374 is low resistance
axis.config.motor.current_soft_max = 40.0
axis.config.motor.current_hard_max = 60.0

# === THERMISTOR CONFIG ===
axis.motor.motor_thermistor.config.enabled = True
axis.motor.motor_thermistor.config.r_ref = 10000
axis.motor.motor_thermistor.config.beta = 3435
axis.motor.motor_thermistor.config.temp_limit_lower = 100.0
axis.motor.motor_thermistor.config.temp_limit_upper = 130.0

# === RS485 ENCODER (AMT21xB-V-OD) ===
odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_EVENT_DRIVEN
axis.config.load_encoder = EncoderId.RS485_ENCODER0
axis.config.commutation_encoder = EncoderId.RS485_ENCODER0

# === REDUCE CALIBRATION ROTATION ===
axis.config.calibration_lockin.current = 5.0     # match motor calibration current
axis.config.calibration_lockin.vel = 20.0
axis.config.calibration_lockin.ramp_distance = 3.14
axis.config.calibration_lockin.ramp_time = 0.4
axis.config.calibration_lockin.accel = 20.0

# === CONTROLLER CONFIG ===
axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
axis.controller.config.input_mode = InputMode.PASSTHROUGH

# === SAVE & REBOOT ===
odrv = save_and_reconnect(odrv)
axis = odrv.axis0

# === MOTOR CALIBRATION ===
print("\nRunning motor calibration (motor should beep)...")
odrv.clear_errors()
axis.requested_state = AxisState.MOTOR_CALIBRATION
time.sleep(1)  # give it a moment to start

# Wait for completion
timeout = 20
start = time.time()
while axis.current_state != AxisState.IDLE:
    time.sleep(0.5)
    if time.time() - start > timeout:
        print("TIMEOUT waiting for motor calibration!")
        sys.exit(1)

# Check for errors
if axis.active_errors:
    print(f"Motor calibration FAILED!")
    print(f"  active_errors: {axis.active_errors}")
    print(f"  disarm_reason: {axis.disarm_reason}")
    print(f"  procedure_result: {axis.procedure_result}")
    sys.exit(1)

r = axis.config.motor.phase_resistance
l = axis.config.motor.phase_inductance
print(f"  Phase resistance: {r:.4f} Ω")
print(f"  Phase inductance: {l:.6f} H")

# Validate - D6374 should be ~0.04Ω resistance, ~15-30µH inductance
if r < 0.001 or l < 0.000001:
    print("ERROR: Calibration values are near zero - motor didn't actually calibrate!")
    print("  Check: motor phases connected? Power supply on? Try higher resistance_calib_max_voltage?")
    print(f"  procedure_result: {axis.procedure_result}")
    sys.exit(1)

print("Motor calibration OK ✓")

# === ENCODER OFFSET CALIBRATION ===
print("\nRunning encoder offset calibration (motor will spin slowly)...")
odrv.clear_errors()
axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION

start = time.time()
while axis.current_state != AxisState.IDLE:
    time.sleep(0.5)
    if time.time() - start > timeout:
        print("TIMEOUT waiting for encoder calibration!")
        sys.exit(1)

if axis.active_errors:
    print(f"Encoder calibration FAILED!")
    print(f"  active_errors: {axis.active_errors}")
    print(f"  disarm_reason: {axis.disarm_reason}")
    print(f"  procedure_result: {axis.procedure_result}")
    sys.exit(1)
print("Encoder calibration OK ✓")

# Save calibration results
odrv = save_and_reconnect(odrv, "Saving calibration...")
axis = odrv.axis0

# === VERIFY ===
print("\n=== Configuration Summary ===")
print(f"Motor: {axis.config.motor.pole_pairs} pole pairs, KT={axis.config.motor.torque_constant:.4f}")
print(f"Phase R: {axis.config.motor.phase_resistance:.4f} Ω, L: {axis.config.motor.phase_inductance:.6f} H")
print(f"Encoder: RS485 AMT21 event-driven")
print(f"Thermistor: enabled={axis.motor.motor_thermistor.config.enabled}, "
      f"temp={axis.motor.motor_thermistor.temperature:.1f}°C, "
      f"limit={axis.motor.motor_thermistor.config.temp_limit_upper}°C")
print(f"Calibration lockin vel: {axis.config.calibration_lockin.vel}")
print("\n✓ Setup complete! Ready for torque control.")