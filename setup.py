#!/usr/bin/env python3
import odrive
from odrive.enums import *
import time
import sys

print("=" * 60)
print("ODrive Setup - BERR Exoskeleton")
print("=" * 60)

odrv = odrive.find_any(timeout=10)
print(f"✓ Connected (Serial: {odrv.serial_number})")

axis = odrv.axis0
odrv.clear_errors()

# Motor config
axis.config.motor.pole_pairs = 7
axis.config.motor.motor_type = 0
axis.config.motor.torque_constant = 0.0551
axis.config.motor.current_soft_max = 25
axis.config.calibration_lockin.current = 10

# Encoder config
odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_EVENT_DRIVEN
axis.config.load_encoder = EncoderId.RS485_ENCODER0
axis.config.commutation_encoder = EncoderId.RS485_ENCODER0

print("\n--- Motor Calibration ---")
input("Press ENTER (motor will spin)...")
axis.requested_state = AxisState.MOTOR_CALIBRATION

start = time.time()
while axis.current_state == AxisState.MOTOR_CALIBRATION:
    time.sleep(0.1)
    if time.time() - start > 15:
        print("✗ Timeout!")
        sys.exit(1)

if axis.disarm_reason != 0:
    print(f"✗ Failed! Disarm: {axis.disarm_reason}")
    sys.exit(1)

print("✓ Motor calibrated")

print("\n--- Encoder Calibration ---")
input("Press ENTER (motor will move)...")
axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION

start = time.time()
while axis.current_state == AxisState.ENCODER_OFFSET_CALIBRATION:
    time.sleep(0.1)
    if time.time() - start > 15:
        print("✗ Timeout!")
        sys.exit(1)

if axis.disarm_reason != 0:
    print(f"✗ Failed! Disarm: {axis.disarm_reason}")
    sys.exit(1)

print("✓ Encoder calibrated")

# ODrive Pro uses these flags instead of pre_calibrated
axis.config.motor.phase_resistance_valid = True
axis.config.motor.phase_inductance_valid = True

print("\nSaving (ODrive will reboot)...")
try:
    odrv.save_configuration()
except:
    pass

print("\n✓ DONE! Wait 5s then: python3 read_encoder.py")