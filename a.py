import odrive
from odrive.enums import *
import time

print("=== ODrive Motor Test (No Encoder) ===\n")

# Step 1: Connect to ODrive
print("Step 1: Connecting to ODrive...")
odrv0 = odrive.find_any()
print("✓ Connected!\n")

# Step 2: Configure for sensorless/no encoder operation
print("Step 2: Configuring for operation without encoder...")
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VOLTAGE_CONTROL
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

# Disable encoder requirements
odrv0.axis0.config.startup_encoder_offset_calibration = False
odrv0.axis0.config.startup_encoder_index_search = False
print("✓ Configuration set\n")

# Step 3: Run motor calibration
print("Step 3: Running motor calibration (motor will beep/twitch)...")
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION

# Wait for calibration to complete
while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

# Check calibration status
if odrv0.axis0.error != 0:
    print("✗ Motor calibration failed!")
    print(f"Axis error: {odrv0.axis0.error}")
    print(f"Motor error: {odrv0.axis0.motor.error}")
    exit()
else:
    print("✓ Motor calibration successful!\n")

# Step 4: Mark motor as pre-calibrated
print("Step 4: Saving calibration...")
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.save_configuration()
print("✓ Calibration saved\n")

# Step 5: Reboot ODrive
print("Step 5: Rebooting ODrive...")
odrv0.reboot()
time.sleep(3)
print("✓ Reboot complete\n")

# Step 6: Reconnect
print("Step 6: Reconnecting to ODrive...")
odrv0 = odrive.find_any()
print("✓ Reconnected!\n")

# Step 7: Enter closed loop control
print("Step 7: Entering closed loop control...")
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(1)

# Check for errors
errors = odrv0.axis0.error
if errors != 0:
    print(f"✗ Error entering closed loop: {errors}")
    dump_errors(odrv0)
    exit()

print("✓ Closed loop control active\n")

# Step 8: Spin the motor!
print("Step 8: Spinning motor...\n")
print("Starting at 2V...")
odrv0.axis0.controller.input_voltage = 2.0
time.sleep(3)

print("Increasing to 5V...")
odrv0.axis0.controller.input_voltage = 5.0
time.sleep(3)

print("Increasing to 8V...")
odrv0.axis0.controller.input_voltage = 8.0
time.sleep(3)

print("Reversing direction...")
odrv0.axis0.controller.input_voltage = -5.0
time.sleep(3)

print("Stopping motor...")
odrv0.axis0.controller.input_voltage = 0.0
time.sleep(1)

# Step 9: Return to idle
print("\nStep 9: Returning to idle state...")
odrv0.axis0.requested_state = AXIS_STATE_IDLE
print("✓ Motor test complete!\n")

print("=== Test Summary ===")
print(f"Motor pre-calibrated: {odrv0.axis0.motor.config.pre_calibrated}")
print(f"Motor resistance: {odrv0.axis0.motor.config.phase_resistance} Ω")
print(f"Motor inductance: {odrv0.axis0.motor.config.phase_inductance} H")
print("\nMotor is ready for testing. Encoder can be added later for closed-loop control.")