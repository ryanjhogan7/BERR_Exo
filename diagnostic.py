#!/usr/bin/env python3
"""Step 3e: Check and fix velocity limit"""
import odrive

print("Connecting...")
odrv = odrive.find_any(timeout=10)
axis = odrv.axis0

print("\n=== CURRENT VELOCITY SETTINGS ===")
print(f"vel_limit: {axis.controller.config.vel_limit} turns/s")
print(f"vel_limit_tolerance: {axis.controller.config.vel_limit_tolerance}")
print(f"enable_torque_mode_vel_limit: {axis.controller.config.enable_torque_mode_vel_limit}")

# The vel_limit might be very low, causing instant fault
# Let's raise it significantly
print("\n=== FIXING ===")
axis.controller.config.vel_limit = 50.0  # 50 turns/sec is plenty
axis.controller.config.vel_limit_tolerance = 1.5  # More tolerance before fault

print(f"New vel_limit: {axis.controller.config.vel_limit}")
print(f"New vel_limit_tolerance: {axis.controller.config.vel_limit_tolerance}")

print("\nSaving (will reboot)...")
try:
    odrv.save_configuration()
except:
    pass

print("Done. Wait 3 sec, then run the torque test again.")