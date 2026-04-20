import time
import board
import busio
import digitalio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

# Pin setup
reset = digitalio.DigitalInOut(board.D17)
reset.direction = digitalio.Direction.OUTPUT
reset.value = True

int_pin = digitalio.DigitalInOut(board.D27)
int_pin.direction = digitalio.Direction.INPUT

i2c = busio.I2C(board.SCL, board.SDA)
imu = None


def hard_reset():
    """Toggle the reset pin to power-cycle the BNO08x state machine."""
    reset.value = False
    time.sleep(0.01)
    reset.value = True
    time.sleep(1.0)

    # Wait for INT to go low (sensor ready)
    timeout = time.monotonic() + 3.0
    while int_pin.value and time.monotonic() < timeout:
        time.sleep(0.01)


def init_imu():
    """Hard reset the sensor and enable the rotation vector feature."""
    global imu
    hard_reset()
    imu = BNO08X_I2C(i2c, address=0x4B, reset=None)

    for attempt in range(10):
        try:
            print(f"Enable attempt {attempt + 1}...")
            time.sleep(0.5)
            imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            print("IMU initialized!")
            return True
        except (RuntimeError, OSError) as e:
            print(f"Error: {e}, retrying...")
            time.sleep(1.0)

    print("Failed to initialize after 10 attempts")
    return False


# Initial setup
if not init_imu():
    exit(1)

# Main loop with INT-driven reads and hard-reset recovery
consecutive_errors = 0
MAX_ERRORS_BEFORE_RESET = 5

while True:
    try:
        # Only read when INT is asserted (active low = data ready)
        if not int_pin.value:
            quat = imu.quaternion
            if quat is not None:
                i, j, k, real = quat
                if all(abs(v) <= 1.0 for v in quat):
                    print(f"i: {i:.3f}  j: {j:.3f}  k: {k:.3f}  real: {real:.3f}")
                    consecutive_errors = 0
        time.sleep(0.001)

    except (OSError, TimeoutError, RuntimeError) as e:
        consecutive_errors += 1
        print(f"Read error ({consecutive_errors}/{MAX_ERRORS_BEFORE_RESET}): {e}")

        if consecutive_errors >= MAX_ERRORS_BEFORE_RESET:
            print("Too many errors, hard-resetting IMU...")
            if init_imu():
                consecutive_errors = 0
            else:
                print("Reinit failed, exiting")
                exit(1)
        else:
            time.sleep(0.1)