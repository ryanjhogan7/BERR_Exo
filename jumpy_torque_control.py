#!/usr/bin/env python3
"""Step 6: Motion-gated torque with hysteresis"""
import odrive
from odrive.enums import AxisState, ControlMode, InputMode
import time

print("Connecting...")
odrv = odrive.find_any(timeout=10)
axis = odrv.axis0

odrv.clear_errors()

axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
axis.controller.config.input_mode = InputMode.PASSTHROUGH
axis.controller.input_torque = 0

print("Entering closed-loop control...")
axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.3)

if axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
    print(f"FAILED - State: {axis.current_state}, Errors: {axis.active_errors}")
    exit()

# === TUNE THESE ===
HOLD_TORQUE = 0.02
RESIST_TORQUE = 0.2       # Increase this as needed

VEL_ENTER_RESIST = 0.07   # Start resisting when vel exceeds this
VEL_EXIT_TO_HOLD = 0.02   # Drop to hold only when vel falls below this
VEL_EXIT_TO_RUNAWAY = 1.2 # Runaway if vel exceeds this

state = "HOLD"

print("Tuning mode - watch the output and adjust parameters.")
print("Press Ctrl+C to stop.\n")

try:
    while True:
        vel = abs(axis.pos_vel_mapper.vel)
        
        # State machine with hysteresis
        if state == "HOLD":
            if vel > VEL_ENTER_RESIST:
                state = "RESIST"
        
        elif state == "RESIST":
            if vel < VEL_EXIT_TO_HOLD:
                state = "HOLD"
            elif vel > VEL_EXIT_TO_RUNAWAY:
                state = "RUNAWAY"
        
        elif state == "RUNAWAY":
            if vel < VEL_ENTER_RESIST:
                state = "HOLD"
        
        # Apply torque based on state
        if state == "HOLD":
            torque = HOLD_TORQUE
        elif state == "RESIST":
            torque = RESIST_TORQUE
        else:
            torque = HOLD_TORQUE
        
        axis.controller.input_torque = torque
        
        print(f"vel={vel:.3f}  torque={torque:.2f}  [{state}]")
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nStopping...")

axis.controller.input_torque = 0
axis.requested_state = AxisState.IDLE
print("Done.")