import spidev
import time
import RPi.GPIO as GPIO
from hal_plus_motordriver import Motor
from encoder import AMT22Encoder
from aero_logic import AeroLogic

# ==========================
# DEBUG
# ==========================
def print_status(motor):
    print("---- STATUS ----")
    print("Lower limit:", motor.get_lower_limit())
    print("Upper limit:", motor.get_upper_limit())
    print("----------------\n")


# ==========================
# INIT
# ==========================
print("=== INIT ===")

elevator_motor = Motor()
elevator_motor.set_elevator_motor_speed(0)
yaw_motor = Motor()
yaw_motor.set_yaw_motor_speed(0)
encoder = AMT22Encoder(yaw_motor.pi, yaw_motor.PINS["CS"])
time.sleep(1)
print_status(elevator_motor)
print_status(yaw_motor)

try:
    print("=== ELEVATOR MOTOR RUN ===")
    elevator_motor.set_elevator_motor_speed(-200)
    print("Elevator motor command set to 100. Press Ctrl+C to stop.")

    while True:
        print_status(elevator_motor)
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopped by user")


finally:
    # ==========================
    # CLEANUP
    # ==========================
    print("=== CLEANUP ===")

    yaw_motor.set_yaw_motor_speed(0)
    elevator_motor.set_elevator_motor_speed(0)

    try:
        yaw_motor.stop_all()
        elevator_motor.stop_all()
    except:
        pass
