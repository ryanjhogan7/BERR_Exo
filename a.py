import odrive
from odrive.enums import *
import time

print("=== Simple Motor Spin ===\n")

# Connect
odrv0 = odrive.find_any()
odrv0.clear_errors()
print("Connected!\n")

# Set velocity
velocity = 10.0  # turns per second
print(f"Setting velocity to {velocity} turns/sec...")
odrv0.axis0.controller.input_vel = velocity

# Start motor
print("Starting motor...")
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Keep it running
print("Motor is spinning! (Press Ctrl+C to stop)")
try:
    while True:
        print(odrv0.axis0.current_state)
        # Keep motor in closed loop
        if odrv0.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # Keep velocity set
        odrv0.axis0.controller.input_vel = velocity
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n\nStopping motor...")
    odrv0.axis0.controller.input_vel = 0.0
    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    print("Stopped!")