#!/usr/bin/env python3
"""
Resistive Braking Control - Velocity Mode with Zero Target
Creates viscous drag/damping effect
"""
import odrive
from odrive.enums import *
import time

print("=" * 50)
print("Resistive Braking Mode - Auto Setup")
print("=" * 50)

# Connect
print("\n[1/6] Connecting...")
odrv = odrive.find_any(timeout=10)
axis = odrv.axis0
print("     ✓ Connected")

# Set torque constant
print("\n[2/6] Checking torque constant...")
if abs(axis.config.motor.torque_constant - 0.0551) > 0.001:
    print("     Setting torque constant = 0.0551 Nm/A")
    axis.config.motor.torque_constant = 0.0551
    odrv.save_configuration()
    print("     Rebooting...")
    time.sleep(3)
    odrv = odrive.find_any(timeout=10)
    axis = odrv.axis0
print(f"     ✓ Torque constant: {axis.config.motor.torque_constant}")

# Set current limit
print("\n[3/6] Checking current limit...")
if axis.config.motor.current_soft_max < 10:
    print("     Setting current limit = 25 A")
    axis.config.motor.current_soft_max = 25
    odrv.save_configuration()
    print("     Rebooting...")
    time.sleep(3)
    odrv = odrive.find_any(timeout=10)
    axis = odrv.axis0
print(f"     ✓ Current limit: {axis.config.motor.current_soft_max} A")

# Configure VELOCITY mode for resistive braking
print("\n[4/6] Configuring resistive braking mode...")
axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
axis.controller.config.input_mode = InputMode.PASSTHROUGH

# KEY SETTINGS for resistive braking:
axis.controller.config.vel_integrator_gain = 0  # Zero = no spring-back
axis.controller.input_vel = 0  # Target velocity = 0

# Resistance level (higher = more resistance)
# vel_gain units: (Nm)/(rev/s) or (Nm·s)/rev
resistance_level = 2.0  # Start conservative
axis.controller.config.vel_gain = resistance_level

# Optional: Enable back-EMF feedforward for better performance
axis.controller.config.enable_torque_mode_vel_limit = False

print(f"     ✓ Velocity mode configured")
print(f"     ✓ Integrator gain: {axis.controller.config.vel_integrator_gain}")
print(f"     ✓ Resistance level: {resistance_level} (Nm·s)/rev")

# Enter closed loop
print("\n[5/6] Entering closed loop control...")
axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)

if axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
    print(f"     ✗ Failed! State: {axis.current_state}")
    print(f"     Error: {axis.error}")
    print(f"     Disarm reason: {axis.disarm_reason}")
    exit(1)

print("     ✓ Closed loop active")

print("\n" + "=" * 50)
print(f"RESISTIVE BRAKING ACTIVE")
print(f"Resistance: {resistance_level} (Nm·s)/rev")
print("=" * 50)
print("\nTry turning the motor by hand!")
print("You should feel smooth, speed-dependent resistance")
print("(faster = more resistance)")
print("\nPress Ctrl+C to stop\n")

# Display feedback with current and torque
try:
    print("Time | Velocity | Current | Est. Torque | Position")
    print("-" * 60)
    start_time = time.time()
    
    while True:
        t = time.time() - start_time
        vel = axis.pos_vel_mapper.vel  # rev/s
        current = axis.motor.foc.Iq_measured  # A
        torque = current * axis.config.motor.torque_constant  # Nm (motor shaft)
        torque_output = torque  # Nm 
        pos = axis.pos_vel_mapper.pos_rel  # revolutions
        
        print(f"\r{t:5.1f}s | {vel:7.3f} r/s | {current:6.2f} A | "
              f"{torque_output:7.2f} Nm | {pos:7.2f} rev", 
              end='', flush=True)
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\nShutting down...")
    axis.requested_state = AxisState.IDLE
    print("✓ Done")
