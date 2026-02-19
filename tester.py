#!/usr/bin/env python3
"""Strategy 4b: Position-based torque with fixed range"""
import odrive
from odrive.enums import AxisState, ControlMode, InputMode
import time
import math

print("Connecting...")
odrv = odrive.find_any(timeout=10)
axis = odrv.axis0

odrv.clear_errors()
axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
axis.controller.config.input_mode = InputMode.PASSTHROUGH
axis.controller.input_torque = 0

axis.motor.motor_thermistor.temperature

print("Entering closed-loop control...")
axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.3)

if axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
    print(f"FAILED - State: {axis.current_state}, Errors: {axis.active_errors}")
    exit()

# === SETUP ===
input("\nMove arm to START position (extended), press Enter...")
pos_start = axis.pos_vel_mapper.pos_rel
print(f"Start position: {pos_start:.3f} turns")

# 120 degrees = 1/3 turn
POS_RANGE = 120.0 / 360.0  # 0.333 turns

print(f"End position will be: {pos_start + POS_RANGE:.3f} turns")

# === TUNE THESE ===
MAX_TORQUE = -1


SLEW_RATE = 5.0

current_torque = 0.0
dt = 0.02

print(f"\nPosition-based torque active: max={MAX_TORQUE} Nm")
print("Press Ctrl+C to stop.\n")

try:
    while True:
        pos = axis.pos_vel_mapper.pos_rel
        
        # How far into the range (0 to 1)
        normalized = (pos - pos_start) / POS_RANGE
        normalized = max(0, min(1, normalized))  # Clamp 0-1
        
        # Constant torque within range, zero outside
        if normalized <= 0:
            desired = 0.0
        elif normalized >= 1:
            desired = 0.0
        else:
            desired = MAX_TORQUE
        
        # Rate limit
        max_change = SLEW_RATE * dt
        if desired > current_torque:
            current_torque = min(desired, current_torque + max_change)
        else:
            current_torque = max(desired, current_torque - max_change)
        
        axis.controller.input_torque = current_torque
        
        temp = axis.motor.motor_thermistor.temperature
        print(f"pos={pos:.3f}  norm={normalized:.2f}  torque={current_torque:.2f}  temp={temp:.1f}°C")
        time.sleep(dt)

except KeyboardInterrupt:
    print("\nStopping...")

axis.controller.input_torque = 0
axis.requested_state = AxisState.IDLE
print("Done.")