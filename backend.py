#!/usr/bin/env python3
import asyncio
import websockets
import json
import math
import time
from datetime import datetime

import odrive
from odrive.enums import AxisState, ControlMode, InputMode

# ── Interpolation Math ──
def catmull_rom(p0, p1, p2, p3, t):
    t2 = t * t
    t3 = t2 * t
    v = 0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)
    return max(0.0, min(1.0, v))

def evaluate_curve(curve, normalized_pos):
    if normalized_pos <= 0.0 or normalized_pos >= 1.0:
        return 0.0
    n = len(curve)
    fi = normalized_pos * (n - 1)
    i = int(fi)
    frac = fi - i
    p0 = curve[max(0, i - 1)]
    p1 = curve[i]
    p2 = curve[min(n - 1, i + 1)]
    p3 = curve[min(n - 1, i + 2)]
    return catmull_rom(p0, p1, p2, p3, frac)

# ── Global State ──
ODRV = None
AXIS = None
SESSION_ACTIVE = False

# ── Async Control Loop ──
async def run_session(websocket, config):
    global SESSION_ACTIVE, ODRV, AXIS
    
    # Extract UI Config
    curve = config.get("curve", [0.8]*12)
    max_torque = float(config.get("max_torque", -1.0)) # Changed default to -1.0
    slew_rate = float(config.get("slew_rate", 5.0))
    pos_range_deg = float(config.get("rom", 120.0))
    dt = float(config.get("dt", 0.02))

    # Setup ODrive for Session
    print("Configuring ODrive for session...")
    ODRV.clear_errors()
    AXIS.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    AXIS.controller.config.input_mode = InputMode.PASSTHROUGH
    AXIS.controller.input_torque = 0
    AXIS.requested_state = AxisState.CLOSED_LOOP_CONTROL
    
    await asyncio.sleep(0.3) # Give it a moment to enter closed loop

    if AXIS.current_state != AxisState.CLOSED_LOOP_CONTROL:
        await websocket.send(json.dumps({"type": "error", "message": "Failed to enter closed-loop control."}))
        SESSION_ACTIVE = False
        return

    # Auto-Zero position based on current physical location
    pos_start = AXIS.pos_vel_mapper.pos_rel
    pos_range = pos_range_deg / 360.0
    current_torque = 0.0

    print(f"Session started! Start Pos: {pos_start:.3f} turns. Range: {pos_range_deg} deg.")
    
    t_start = time.time()

    try:
        while SESSION_ACTIVE:
            t_now = time.time() - t_start
            pos = AXIS.pos_vel_mapper.pos_rel
            vel = AXIS.pos_vel_mapper.vel

            # Normalize position
            normalized = (pos - pos_start) / pos_range
            normalized = max(0.0, min(1.0, normalized))

            # Evaluate curve & Torque Slew Limiting
            curve_mult = evaluate_curve(curve, normalized)
            desired = max_torque * curve_mult
            
            max_change = slew_rate * dt
            if desired > current_torque:
                current_torque = min(desired, current_torque + max_change)
            else:
                current_torque = max(desired, current_torque - max_change)

            AXIS.controller.input_torque = current_torque

            # Fetch Telemetry
            motor_temp = AXIS.motor.motor_thermistor.temperature
            fet_temp = AXIS.motor.fet_thermistor.temperature
            vbus = ODRV.vbus_voltage
            ibus = ODRV.ibus
            input_iq = AXIS.motor.input_iq

            # Send Telemetry to UI
            telemetry = {
                "type": "telemetry",
                "torque": round(current_torque, 2),
                "pos_deg": round((pos - pos_start) * 360, 1),
                "vel": round(vel, 2),
                "current": round(input_iq, 2),
                "motor_temp": round(motor_temp, 1),
                "fet_temp": round(fet_temp, 1),
                "power": round(vbus * ibus, 1),
                "vbus": round(vbus, 1),
                "curve_mult": round(curve_mult, 2)
            }
            await websocket.send(json.dumps(telemetry))

            # Yield control back to the event loop so the server can receive incoming messages (like "stop")
            await asyncio.sleep(dt)

    except Exception as e:
        print(f"Session Error: {e}")
    finally:
        print("Session Ended. Disarming...")
        AXIS.controller.input_torque = 0
        AXIS.requested_state = AxisState.IDLE
        SESSION_ACTIVE = False
        await websocket.send(json.dumps({"type": "status", "message": "stopped"}))

# ── WebSocket Server Router ──
async def ws_handler(websocket):
    global SESSION_ACTIVE
    print("UI Client Connected")
    
    async for message in websocket:
        data = json.loads(message)
        cmd = data.get("command")

        if cmd == "start":
            if not SESSION_ACTIVE:
                SESSION_ACTIVE = True
                asyncio.create_task(run_session(websocket, data.get("config", {})))
        
        elif cmd == "stop":
            print("Stop command received from UI")
            SESSION_ACTIVE = False

async def main():
    print("Starting WebSocket Server on ws://localhost:8765")
    async with websockets.serve(ws_handler, "localhost", 8765):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    # PRE-SESSION SETUP: Connect to ODrive BEFORE starting the asyncio loop
    print("Connecting to ODrive... (Pre-session setup)")
    try:
        ODRV = odrive.find_any(timeout=10.0)
        if ODRV is None:
            print("Timeout finding ODrive. Is it plugged in and powered?")
            exit(1)
            
        AXIS = ODRV.axis0
        print(f"ODrive Connected successfully! VBUS: {ODRV.vbus_voltage:.2f}V")
        
        # Now that ODrive is connected, start the WebSocket server
        if hasattr(asyncio, 'WindowsSelectorEventLoopPolicy'):
            asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
        asyncio.run(main())
        
    except Exception as e:
        print(f"Startup Failed: {e}")