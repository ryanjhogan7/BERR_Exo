#!/usr/bin/env python3
"""Step 2: Configure for Torque Control"""
import odrive
from odrive.enums import ControlMode, InputMode

print("Connecting...")
odrv = odrive.find_any(timeout=10)
axis = odrv.axis0

print("Setting torque control mode...")

# Set to torque control
axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
axis.controller.config.input_mode = InputMode.PASSTHROUGH

# Disable velocity limiter (important for constant torque)
axis.controller.config.enable_torque_mode_vel_limit = False

print("\n=== NEW CONFIG ===")
print(f"Control Mode: {axis.controller.config.control_mode} (should be 1 for TORQUE)")
print(f"Input Mode: {axis.controller.config.input_mode} (should be 1 for PASSTHROUGH)")
print(f"Torque Vel Limit Enabled: {axis.controller.config.enable_torque_mode_vel_limit} (should be False)")

print("\nSaving config (ODrive will reboot)...")
try:
    odrv.save_configuration()
except:
    pass

print("Done. Wait 3 seconds, then run diagnostic.py again to confirm.")