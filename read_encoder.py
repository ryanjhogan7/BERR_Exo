#!/usr/bin/env python3
"""
Simple Encoder Position Reader
Displays real-time encoder position from AMT212B-V-OD
Press Ctrl+C to exitaimport odrive
odrv = odrive.find_any(timeout=10)
axis = odrv.axis0
m = axis.motor
print([a for a in dir(m) if not a.startswith('_')])
"""

import odrive
import time
import sys

print("=" * 50)
print("Encoder Position Reader")
print("=" * 50)

# Connect to ODrive
print("\nSearching for ODrive...")
try:
    odrv = odrive.find_any(timeout=10)
    print(f"✓ Connected to ODrive\n")
except Exception as e:
    print(f"✗ Failed to connect: {e}")
    sys.exit(1)

# Select axis
axis = odrv.axis0

# Clear any errors
odrv.clear_errors()

print("Reading encoder position (turns)...")
print("Rotate the motor shaft to see values change")
print("Press Ctrl+C to exit\n")
print("-" * 50)

try:
    while True:
        # Read position in turns (1 turn = 360 degrees)
        position = axis.pos_vel_mapper.pos_rel
        velocity = axis.pos_vel_mapper.vel
        
        # Convert to degrees for easier reading
        degrees = position * 360
        
        # Display with clear formatting
        print(f"\rPosition: {position:8.3f} turns  |  {degrees:9.1f}°  |  Vel: {velocity:6.2f} turns/s", end='', flush=True)
        
        time.sleep(0.05)  # Update at 20 Hz

except KeyboardInterrupt:
    print("\n\n✓ Encoder reader stopped")
    sys.exit(0)