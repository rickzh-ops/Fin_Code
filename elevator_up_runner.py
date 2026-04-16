import signal
import sys
import time

from hal_plus_motordriver import Motor

RUN_SPEED = -250
RUN_DURATION = 1.0
PRINT_INTERVAL = 1.0


def main():
    motor = Motor()
    if motor.get_upper_limit() == 0:
        print("Upper limit already pressed. Up runner will not move.")
        motor.close()
        return

    motor.set_elevator_motor_speed(RUN_SPEED)
    print(
        f"Elevator up runner started at speed {RUN_SPEED} for up to "
        f"{RUN_DURATION:.1f}s."
    )

    def shutdown(*_args):
        motor.set_elevator_motor_speed(0)
        try:
            motor.close()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    last_print_time = 0.0
    start_time = time.perf_counter()
    try:
        while True:
            now = time.perf_counter()
            if motor.get_upper_limit() == 0:
                print("Upper limit pressed. Stopping up runner.")
                break
            if now - last_print_time >= PRINT_INTERVAL:
                print(
                    "up runner | "
                    f"lower={motor.get_lower_limit()} upper={motor.get_upper_limit()} "
                    f"cmd={RUN_SPEED}"
                )
                last_print_time = now
            if now - start_time >= RUN_DURATION:
                break
            time.sleep(0.1)
    finally:
        motor.set_elevator_motor_speed(0)
        try:
            motor.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
