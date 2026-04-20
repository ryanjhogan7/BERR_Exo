#!/usr/bin/env python3
"""BERR EXO - ODrive Pro calibration script.

D6374 motor + AMT21xB RS485 encoder + NTC thermistor.
Runs full setup from erased config through closed-loop verification.

Motor MUST be free to spin (detached from arm) before running.

Usage:
    python3 calibrate.py           # interactive, prompts before encoder cal
    python3 calibrate.py --auto    # no prompts, assumes motor is detached
"""
import argparse
import logging
import sys
import time
from datetime import datetime
from pathlib import Path

import odrive
from odrive.enums import (
    AxisState,
    ControlMode,
    EncoderId,
    InputMode,
    MotorType,
    Rs485EncoderMode,
)


LOG_DIR = Path.home() / "berr_exo" / "calibration_logs"


def setup_logging():
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = LOG_DIR / f"calibration_{ts}.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)-7s | %(message)s",
        handlers=[logging.FileHandler(path), logging.StreamHandler(sys.stdout)],
    )
    logging.info(f"Log: {path}")


def save_and_reconnect(odrv, msg="Saving config..."):
    logging.info(msg)
    try:
        odrv.save_configuration()
    except Exception:
        pass  # save triggers reboot, disconnect is expected
    time.sleep(5)
    logging.info("Reconnecting...")
    return odrive.find_any(timeout=10)


def wait_for_idle(axis, timeout=20):
    start = time.time()
    while axis.current_state != AxisState.IDLE:
        time.sleep(0.5)
        if time.time() - start > timeout:
            return False
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--auto", action="store_true",
                        help="skip interactive detach prompt")
    args = parser.parse_args()

    setup_logging()

    logging.info("Connecting to ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        logging.error(f"Connect failed: {e}")
        sys.exit(1)
    logging.info(f"Found ODrive {odrv.serial_number}, VBUS={odrv.vbus_voltage:.2f}V")

    logging.info("Erasing old configuration...")
    try:
        odrv.erase_configuration()
    except Exception:
        pass
    time.sleep(5)
    odrv = odrive.find_any(timeout=10)
    axis = odrv.axis0

    # Motor
    axis.config.motor.motor_type = MotorType.HIGH_CURRENT
    axis.config.motor.pole_pairs = 7
    axis.config.motor.torque_constant = 8.27 / 149
    axis.config.motor.calibration_current = 5.0
    axis.config.motor.resistance_calib_max_voltage = 4.0
    axis.config.motor.current_soft_max = 60.0
    axis.config.motor.current_hard_max = 80.0

    # Thermistor
    axis.motor.motor_thermistor.config.enabled = True
    axis.motor.motor_thermistor.config.r_ref = 10000
    axis.motor.motor_thermistor.config.beta = 3435
    axis.motor.motor_thermistor.config.temp_limit_lower = 100.0
    axis.motor.motor_thermistor.config.temp_limit_upper = 130.0

    # RS485 encoder (AMT21xB-V-OD)
    odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_EVENT_DRIVEN
    axis.config.load_encoder = EncoderId.RS485_ENCODER0
    axis.config.commutation_encoder = EncoderId.RS485_ENCODER0

    # Calibration lockin
    axis.config.calibration_lockin.current = 5.0
    axis.config.calibration_lockin.vel = 20.0
    axis.config.calibration_lockin.ramp_distance = 3.14
    axis.config.calibration_lockin.ramp_time = 0.4
    axis.config.calibration_lockin.accel = 20.0

    # Controller default
    axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    axis.controller.config.input_mode = InputMode.PASSTHROUGH

    odrv = save_and_reconnect(odrv)
    axis = odrv.axis0

    # Motor calibration
    logging.info("Running motor calibration (expect beep)...")
    odrv.clear_errors()
    axis.requested_state = AxisState.MOTOR_CALIBRATION
    time.sleep(1)
    if not wait_for_idle(axis):
        logging.error("Motor cal TIMEOUT")
        sys.exit(1)
    if axis.active_errors:
        logging.error(f"Motor cal FAILED: errors={axis.active_errors}, "
                      f"result={axis.procedure_result}")
        sys.exit(1)

    r = axis.config.motor.phase_resistance
    l = axis.config.motor.phase_inductance
    logging.info(f"  Phase R: {r:.4f} ohm, L: {l:.6f} H")
    if r < 0.001 or l < 0.000001:
        logging.error("Zero calibration values, motor not detected")
        sys.exit(1)
    logging.info("Motor calibration OK")

    # Encoder calibration
    if not args.auto:
        input("\nConfirm motor is FREE TO SPIN (detached from arm), press Enter...")
    else:
        logging.warning("--auto flag set, skipping detach prompt")

    logging.info("Running encoder offset calibration...")
    odrv.clear_errors()
    axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    if not wait_for_idle(axis, timeout=30):
        logging.error("Encoder cal TIMEOUT")
        sys.exit(1)

    offset = axis.commutation_mapper.config.offset
    valid = axis.commutation_mapper.config.offset_valid
    logging.info(f"  offset={offset:.4f}, valid={valid}, "
                 f"result={axis.procedure_result}")
    if not valid:
        logging.error(f"Encoder cal FAILED: errors={axis.active_errors}")
        sys.exit(1)
    logging.info("Encoder calibration OK")

    odrv = save_and_reconnect(odrv, "Saving calibration...")
    axis = odrv.axis0

    if not axis.commutation_mapper.config.offset_valid:
        logging.error("Offset did not persist through reboot")
        sys.exit(1)

    # Closed-loop verify
    logging.info("Verifying closed-loop control...")
    odrv.clear_errors()
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(0.5)
    if axis.current_state == AxisState.CLOSED_LOOP_CONTROL:
        logging.info("Closed-loop OK")
        axis.requested_state = AxisState.IDLE
    else:
        logging.error(f"Closed-loop FAILED: state={axis.current_state}, "
                      f"disarm_reason={axis.disarm_reason}")
        sys.exit(1)

    logging.info("=== Summary ===")
    logging.info(f"Pole pairs: {axis.config.motor.pole_pairs}")
    logging.info(f"KT: {axis.config.motor.torque_constant:.4f} Nm/A")
    logging.info(f"Phase R: {axis.config.motor.phase_resistance:.4f} ohm")
    logging.info(f"Phase L: {axis.config.motor.phase_inductance:.6f} H")
    logging.info(f"Encoder offset: {offset:.4f}")
    logging.info(f"Motor temp: {axis.motor.motor_thermistor.temperature:.1f}C")
    logging.info("Setup complete. Reattach motor to arm.")


if __name__ == "__main__":
    main()