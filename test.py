import odrive
import time

# ODrive Enums - Define as constants
MOTOR_TYPE_HIGH_CURRENT = 0
ENCODER_MODE_SENSORLESS = 1
CONTROL_MODE_VELOCITY_CONTROL = 2
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

print("Finding ODrive...")
odrv0 = odrive.find_any()
print("ODrive found!")

# Clear any errors
odrv0.clear_errors()

# ===== MOTOR CONFIGURATION =====
print("Configuring motor...")
odrv0.axis0.config.motor.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.config.motor.current_lim = 10  # Adjust for your motor
odrv0.axis0.config.motor.calibration_current = 5
odrv0.axis0.config.motor.resistance_calib_max_voltage = 4
odrv0.axis0.config.motor.pole_pairs = 7  # Adjust for your motor

# ===== SENSORLESS CONFIGURATION =====
print("Configuring sensorless mode...")
odrv0.axis0.config.encoder.mode = ENCODER_MODE_SENSORLESS
odrv0.axis0.config.controller.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.config.controller.vel_limit = 10  # turns/sec
odrv0.axis0.config.controller.vel_ramp_rate = 0.5

# Sensorless estimator settings
odrv0.axis0.config.sensorless_estimator.pm_flux_linkage = 5.51328895422 / (7 * 270)
odrv0.axis0.config.sensorless_estimator.observer_gain = 1000  # Default is usually fine

# ===== STARTUP CONFIGURATION =====
odrv0.axis0.config.startup_motor_calibration = True
odrv0.axis0.config.startup_sensorless_control = True
odrv0.axis0.config.startup_closed_loop_control = True

print("Saving configuration and rebooting...")
odrv0.save_configuration()
odrv0.reboot()
print("Configuration complete! ODrive will now run in sensorless mode on startup.")
print("\nTo manually control the motor, use:")
print("  odrv0.axis0.controller.input_vel = 5  # Set velocity in turns/sec")
print("  odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL  # Start motor")