#!/usr/bin/env python3
"""
ODrive Torque Tester
Simple script to test resistive torque control for exoskeleton project.
Press Ctrl+C to exit safely.
"""

import odrive
from odrive.enums import AxisState, ControlMode, InputMode
import time
import sys
import threading

print("=" * 60)
print("ODrive Torque Tester - BERR Exoskeleton Project")
print("=" * 60)

# Connect to ODrive
print("\nSearching for ODrive...")
try:
    odrv = odrive.find_any(timeout=10)
    print(f"✓ Connected to ODrive (Serial: {odrv.serial_number})")
except Exception as e:
    print(f"✗ Failed to connect to ODrive: {e}")
    sys.exit(1)

# Select axis (change to axis1 if using motor on that port)
axis = odrv.axis0

# Check for errors
if axis.disarm_reason != 0:
    print(f"\n⚠ Warning: Axis has errors (disarm_reason: {axis.disarm_reason})")
    print("Run 'odrv.clear_errors()' in odrivetool if needed")

# Set control mode to TORQUE_CONTROL
print("\nConfiguring torque control mode...")
axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
axis.controller.config.input_mode = InputMode.PASSTHROUGH

# CRITICAL: Disable velocity limiter for pure torque control
# Without this, torque will be limited even at 0 rpm
print("Disabling velocity limiter...")
axis.controller.config.enable_torque_mode_vel_limit = False

# Enter closed loop control
print("Entering closed loop control...")
axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)

# Check if we're in closed loop
if axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
    print(f"✗ Failed to enter closed loop. Current state: {axis.current_state}")
    print("Check motor calibration and encoder setup")
    sys.exit(1)

print("✓ Torque control active!\n")

# Display current motor parameters
print("-" * 60)
print("Motor Information:")
print(f"  Torque Constant: {axis.config.motor.torque_constant:.4f} Nm/A")
print(f"  Current Limit: {axis.config.motor.current_soft_max:.1f} A")
print(f"  Max Torque (~): {axis.config.motor.torque_constant * axis.config.motor.current_soft_max:.2f} Nm")
print(f"\n  Expected Current for 0.1 Nm: {0.1 / axis.config.motor.torque_constant:.2f} A")
print(f"  Expected Current for 1.0 Nm: {1.0 / axis.config.motor.torque_constant:.2f} A")
print("-" * 60)

print("\nVirtual Spring Resistance Controls:")
print("  Enter maximum resistance torque in Nm (e.g., '1' for 1 Nm)")
print("  Motor will resist movement like a spring, capped at max torque")
print("  Perfect for exoskeleton resistance training")
print("  Type a number and press Enter to change resistance while running")
print("  Enter '0' to set zero torque (free movement)")
print("  Press Ctrl+C to exit\n")

# Shared variables for user interface
resistive_torque_magnitude = 0.0
shutdown_flag = threading.Event()

# Get initial resistance torque value from user
try:
    user_input = input("Enter maximum resistance (Nm): ").strip().lower()
    resistive_torque_magnitude = abs(float(user_input))  # Always positive magnitude
    
    # Safety check
    max_torque = axis.config.motor.torque_constant * axis.config.motor.current_soft_max
    if resistive_torque_magnitude > max_torque:
        print(f"⚠ Warning: Requested torque ({resistive_torque_magnitude:.2f} Nm) exceeds safe limit ({max_torque:.2f} Nm)")
        print(f"Limiting to {max_torque:.2f} Nm")
        resistive_torque_magnitude = max_torque
    
    print(f"✓ Maximum resistance set to {resistive_torque_magnitude:.2f} Nm")
    print("  Motor will resist like a spring. Type a number to change resistance.\n")
except ValueError:
    print("✗ Invalid input. Setting torque to 0 Nm (free movement)")
    resistive_torque_magnitude = 0.0
except KeyboardInterrupt:
    print("\n\nExiting...")
    sys.exit(0)

# User input thread
def user_input_thread():
    global resistive_torque_magnitude, shutdown_flag
    while not shutdown_flag.is_set():
        try:
            user_input = input().strip().lower()
            if user_input == 'q':
                shutdown_flag.set()
                break
            
            try:
                new_torque = abs(float(user_input))
                # Safety check
                max_torque = axis.config.motor.torque_constant * axis.config.motor.current_soft_max
                if new_torque > max_torque:
                    print(f"⚠ Warning: Requested torque ({new_torque:.2f} Nm) exceeds safe limit ({max_torque:.2f} Nm)")
                    print(f"Limiting to {max_torque:.2f} Nm")
                    new_torque = max_torque
                
                resistive_torque_magnitude = new_torque
                print(f"✓ Maximum resistance changed to {resistive_torque_magnitude:.2f} Nm")
            except ValueError:
                print("✗ Invalid input. Please enter a number or 'q' to quit.")
        except (EOFError, KeyboardInterrupt):
            shutdown_flag.set()
            break

# Start user input thread
input_thread = threading.Thread(target=user_input_thread, daemon=True)
input_thread.start()

# Main control loop - Virtual spring with limited force
SPRING_STIFFNESS = 50.0  # Nm per turn - how stiff the virtual spring is
POSITION_UPDATE_RATE = 0.5  # How fast locked position follows (0=never, 1=instant)
VELOCITY_THRESHOLD = 0.01  # turns/sec - below this, update locked position

locked_position = None  # The position we're trying to hold
estimated_position = 0.0  # Position estimated by integrating velocity
encoder_position_available = False  # Track if we can read encoder position

try:
    last_update_time = time.time()
    
    # Check once if encoder position is available
    try:
        test_pos = None
        if hasattr(axis, 'pos_vel_mapper') and hasattr(axis.pos_vel_mapper, 'pos'):
            test_pos = axis.pos_vel_mapper.pos
        elif hasattr(axis, 'encoder') and hasattr(axis.encoder, 'pos_estimate'):
            test_pos = axis.encoder.pos_estimate
        
        if test_pos is not None and test_pos != 0.0:
            encoder_position_available = True
            estimated_position = test_pos
            print("✓ Encoder position available")
        else:
            print("⚠ Encoder position not available - will integrate velocity to estimate position")
            print("  (Encoder may need configuration - see ODrive docs for RS485 encoder setup)")
    except:
        print("⚠ Encoder position not available - will integrate velocity to estimate position")
    
    while not shutdown_flag.is_set():
        try:
            current_time = time.time()
            dt = current_time - last_update_time
            last_update_time = current_time
            dt = min(dt, 0.1)  # Cap dt to prevent large jumps
            
            # Get current velocity
            velocity = axis.pos_vel_mapper.vel  # in turns/sec
            abs_vel = abs(velocity)
            
            # Get or estimate position
            if encoder_position_available:
                # Try to read from encoder
                try:
                    if hasattr(axis, 'pos_vel_mapper') and hasattr(axis.pos_vel_mapper, 'pos'):
                        position = axis.pos_vel_mapper.pos
                    elif hasattr(axis, 'encoder') and hasattr(axis.encoder, 'pos_estimate'):
                        position = axis.encoder.pos_estimate
                    else:
                        position = estimated_position
                except:
                    position = estimated_position
            else:
                # Integrate velocity to estimate position
                estimated_position += velocity * dt
                position = estimated_position
            
            # Initialize or update locked position
            if locked_position is None:
                # First time - lock to current position
                locked_position = position
            elif abs_vel < VELOCITY_THRESHOLD:
                # Nearly stationary - lock to current position
                locked_position = position
            else:
                # Moving - slowly update locked position (allows slow movement under sustained force)
                locked_position += (position - locked_position) * POSITION_UPDATE_RATE * dt
            
            # Calculate spring force: proportional to displacement from locked position
            position_error = position - locked_position  # in turns
            spring_torque = -position_error * SPRING_STIFFNESS  # Opposes displacement
            
            # Limit torque to maximum resistance
            if abs(spring_torque) > resistive_torque_magnitude:
                # Cap at maximum
                if spring_torque > 0:
                    resistive_torque = resistive_torque_magnitude
                else:
                    resistive_torque = -resistive_torque_magnitude
            else:
                # Within limit, apply spring force
                resistive_torque = spring_torque
            
            # Apply the torque
            axis.controller.input_torque = resistive_torque
        except Exception as e:
            # If there's an error reading velocity or setting torque, continue
            print(f"\n⚠ Error in control loop: {e}")
            time.sleep(0.1)
            continue
        
        # Display current status
        # Try to get actual torque estimate from motor
        try:
            if hasattr(axis.motor, 'torque_estimate'):
                actual_torque = axis.motor.torque_estimate
            else:
                actual_torque = None
        except (AttributeError, TypeError):
            actual_torque = None
        
        # Try to get measured current, fall back to calculated from set torque
        try:
            # Try different possible attribute paths for measured current
            if hasattr(axis.motor, 'Iq_measured'):
                measured_current = axis.motor.Iq_measured
            elif hasattr(axis.controller, 'Iq_measured'):
                measured_current = axis.controller.Iq_measured
            elif hasattr(axis.motor, 'current_meas_phB'):
                measured_current = axis.motor.current_meas_phB
            else:
                # Fall back to calculating from set torque
                measured_current = abs(resistive_torque) / axis.config.motor.torque_constant if axis.config.motor.torque_constant != 0 else 0.0
        except (AttributeError, ZeroDivisionError):
            # Fall back to calculating from set torque
            measured_current = abs(resistive_torque) / axis.config.motor.torque_constant if axis.config.motor.torque_constant != 0 else 0.0
        
        # Calculate estimated torque from current (use absolute value for display)
        estimated_torque = abs(measured_current) * axis.config.motor.torque_constant
        velocity_rpm = velocity * 60  # Convert to RPM
        
        # Calculate displacement and status
        displacement_deg = position_error * 360  # Convert turns to degrees
        at_limit = abs(resistive_torque) >= resistive_torque_magnitude * 0.99
        status = "AT LIMIT" if at_limit else "RESISTING"
        
        # Debug: show position and locked position
        if actual_torque is not None:
            print(f"\r[{status}] MaxRes: {resistive_torque_magnitude:5.2f} Nm | "
                  f"Applied: {resistive_torque:6.2f} Nm | "
                  f"Actual: {actual_torque:6.2f} Nm | "
                  f"Pos: {position:7.3f} | Lock: {locked_position:7.3f} | "
                  f"Disp: {displacement_deg:6.1f}° | "
                  f"Vel: {velocity_rpm:6.1f} RPM", end="", flush=True)
        else:
            print(f"\r[{status}] MaxRes: {resistive_torque_magnitude:5.2f} Nm | "
                  f"Applied: {resistive_torque:6.2f} Nm | "
                  f"Est: {estimated_torque:6.2f} Nm | "
                  f"Pos: {position:7.3f} | Lock: {locked_position:7.3f} | "
                  f"Disp: {displacement_deg:6.1f}° | "
                  f"Vel: {velocity_rpm:6.1f} RPM", end="", flush=True)
        
        time.sleep(0.01)  # Fast update rate for continuous torque

except KeyboardInterrupt:
    print("\n\nInterrupted by user")
    shutdown_flag.set()

finally:
    # Safe shutdown
    shutdown_flag.set()
    print("\n\nShutting down safely...")
    # Gradually reduce torque to zero (apply in both directions to be safe)
    try:
        for i in range(20):
            # Ramp down the magnitude
            ramp_torque = resistive_torque_magnitude * (1.0 - i / 20.0)
            # Get current velocity to determine direction
            try:
                vel = axis.pos_vel_mapper.vel
                if abs(vel) > 0.01:
                    axis.controller.input_torque = -ramp_torque if vel > 0 else ramp_torque
                else:
                    axis.controller.input_torque = 0.0
            except:
                axis.controller.input_torque = 0.0
            time.sleep(0.01)
        axis.controller.input_torque = 0
    except Exception as e:
        print(f"⚠ Error during shutdown: {e}")
        axis.controller.input_torque = 0
    time.sleep(0.2)
    axis.requested_state = AxisState.IDLE
    print("✓ Motor set to idle. Safe to power off.")
    print("\nTorque tester closed.")