#!/usr/bin/env python3
"""
ODrive Setup Script with RS485 Encoder Configuration
Configures encoder and prepares for torque control
Press Ctrl+C to exit safely.
"""

import odrive
from odrive.enums import AxisState, ControlMode, EncoderId, Rs485EncoderMode
import time
import sys

print("=" * 60)
print("ODrive Setup - BERR Exoskeleton Project")
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

# Clear any errors
print("\nClearing errors...")
odrv.clear_errors()
time.sleep(0.5)

# Check if encoder is configured
print("\nChecking encoder configuration...")
encoder_configured = False
try:
    if (axis.config.load_encoder == EncoderId.RS485_ENCODER0 and 
        axis.config.commutation_encoder == EncoderId.RS485_ENCODER0):
        print("✓ RS485 encoder already configured")
        encoder_configured = True
    else:
        print("⚠ Encoder not configured")
except:
    print("⚠ Encoder not configured")

# Configure encoder if needed
if not encoder_configured:
    print("\nConfiguring RS485 encoder...")
    print("Which encoder do you have?")
    print("  1. AMT212B-V (use POLLING mode)")
    print("  2. AMT212B-V-OD (use EVENT_DRIVEN mode)")
    choice = input("Enter 1 or 2: ").strip()
    
    try:
        # Set encoder mode
        if choice == "1":
            odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_POLLING
            print("✓ Set encoder mode to AMT21_POLLING")
        else:
            odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_EVENT_DRIVEN
            print("✓ Set encoder mode to AMT21_EVENT_DRIVEN")
        
        # Configure axis to use RS485 encoder
        odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
        odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0
        print("✓ Configured axis to use RS485 encoder")
        
        # Save configuration
        print("\nSaving configuration...")
        odrv.save_configuration()
        print("✓ Configuration saved")
        
        print("\n⚠ ODrive will reboot. Please run this script again after reboot.")
        sys.exit(0)
        
    except Exception as e:
        print(f"✗ Encoder configuration failed: {e}")
        print("Continuing without encoder configuration...")

# Check if encoder offset calibration is needed
print("\nChecking encoder calibration...")
calibration_needed = False
try:
    # Check if encoder is reporting valid data
    if hasattr(axis, 'encoder') and hasattr(axis.encoder, 'pos_estimate'):
        test_pos = axis.encoder.pos_estimate
        if test_pos == 0.0:
            calibration_needed = True
    else:
        calibration_needed = True
except:
    calibration_needed = True

if calibration_needed:
    print("⚠ Encoder needs calibration")
    run_cal = input("Run encoder offset calibration now? (y/n): ").strip().lower()
    
    if run_cal == 'y':
        print("\n⚠ Starting encoder offset calibration...")
        print("  Motor will move slowly - do NOT touch it!")
        try:
            axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
            
            # Wait for calibration
            while axis.current_state == AxisState.ENCODER_OFFSET_CALIBRATION:
                print(".", end="", flush=True)
                time.sleep(1)
            
            print("\n✓ Calibration complete!")
            
            # Save calibration
            odrv.save_configuration()
            print("✓ Calibration saved")
        except Exception as e:
            print(f"\n✗ Calibration failed: {e}")
else:
    print("✓ Encoder calibrated")

# Set control mode to TORQUE_CONTROL
print("\nConfiguring torque control mode...")
axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL

# CRITICAL: Disable velocity limiter for pure torque control
print("Disabling velocity limiter...")
axis.controller.config.enable_torque_mode_vel_limit = False

# Enter closed loop control
print("Entering closed loop control...")
axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)

# Check if we're in closed loop
if axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
    print(f"✗ Failed to enter closed loop. Current state: {axis.current_state}")
    print(f"  Disarm reason: {axis.disarm_reason}")
    print("Check motor calibration and encoder setup")
    sys.exit(1)

print("✓ Torque control active!\n")

# Display current motor parameters
print("-" * 60)
print("Motor Information:")
print(f"  Torque Constant: {axis.config.motor.torque_constant:.4f} Nm/A")
print(f"  Current Limit: {axis.config.motor.current_soft_max:.1f} A")
print(f"  Max Torque (~): {axis.config.motor.torque_constant * axis.config.motor.current_soft_max:.2f} Nm")
print("-" * 60)

print("\nTorque Tester Controls:")
print("  Enter torque value in Nm (e.g., '5' for 5 Nm)")
print("  Enter '0' to set zero torque (free movement)")
print("  Enter 'q' or press Ctrl+C to exit\n")

# Main control loop
try:
    current_torque = 0.0
    
    while True:
        # Display current status
        measured_current = axis.motor.current_control.Iq_measured
        estimated_torque = measured_current * axis.config.motor.torque_constant
        velocity = axis.pos_vel_mapper.vel * 60  # Convert to RPM
        
        print(f"\rStatus - Set: {current_torque:6.2f} Nm | "
              f"Measured: {estimated_torque:6.2f} Nm | "
              f"Current: {measured_current:6.2f} A | "
              f"Velocity: {velocity:7.1f} RPM", end="", flush=True)
        
        # Check for user input (non-blocking)
        try:
            user_input = input("\n\nEnter torque (Nm): ").strip().lower()
            
            if user_input == 'q':
                break
            
            # Parse torque value
            torque_value = float(user_input)
            
            # Safety check
            max_torque = axis.config.motor.torque_constant * axis.config.motor.current_soft_max
            if abs(torque_value) > max_torque:
                print(f"⚠ Warning: Requested torque ({torque_value:.2f} Nm) exceeds safe limit ({max_torque:.2f} Nm)")
                print(f"Limiting to {max_torque:.2f} Nm")
                torque_value = max_torque if torque_value > 0 else -max_torque
            
            # Set the torque
            axis.controller.input_torque = torque_value
            current_torque = torque_value
            print(f"✓ Torque set to {current_torque:.2f} Nm")
            
        except ValueError:
            print("✗ Invalid input. Please enter a number or 'q' to quit.")
        except KeyboardInterrupt:
            break
        except EOFError:
            break
        
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\nInterrupted by user")

finally:
    # Safe shutdown
    print("\nShutting down safely...")
    axis.controller.input_torque = 0
    time.sleep(0.2)
    axis.requested_state = AxisState.IDLE
    print("✓ Motor set to idle. Safe to power off.")
    print("\nTorque tester closed.")