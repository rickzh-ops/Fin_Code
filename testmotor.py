import time
from hal import init, cleanup, devices
from motordriver import MotorSystem
from encoder import AMT22Encoder


# ==========================
# DEBUG
# ==========================
def print_gpio_states():
    print("---- GPIO STATES ----")
    for name, dev in devices.items():
        try:
            if hasattr(dev, "value"):
                print(f"{name}: {dev.value}")
        except Exception:
            print(f"{name}: (no read)")
    print("---------------------\n")


def check_pwm_safety():
    cw = devices["CW_PWM"].value
    ccw = devices["CCW_PWM"].value

    if cw > 0 and ccw > 0:
        print("ERROR: CW & CCW both active!")
    else:
        print("PWM OK")


# ==========================
# INIT
# ==========================
print("=== INIT ===")

init()

motors = MotorSystem()
encoder = AMT22Encoder()
motors.disable_all()
time.sleep(1)

print_gpio_states()


try:
    # ==========================
    # ENABLE TEST
    # ==========================
    print("=== ENABLE TEST ===")
    motors.enable_all()
    motors.stop_all()
    time.sleep(1)

    # ==========================
    # YAW MOTOR TEST
    # ==========================
    print("=== YAW TEST ===")

    for speed in [0.3, 0.6, 1.0]:
        print(f"Forward speed: {speed}")
        motors.yaw.set_speed(speed)
        check_pwm_safety()
        time.sleep(1)

    for speed in [-0.3, -0.6, -1.0]:
        print(f"Reverse speed: {speed}")
        motors.yaw.set_speed(speed)
        check_pwm_safety()
        time.sleep(1)

    print("Stop")
    motors.yaw.stop()
    time.sleep(1)

    # ==========================
    # STEPPER TEST
    # ==========================
    print("=== STEPPER TEST ===")

    motors.elevator.step(10, delay=0.01, direction=1)
    time.sleep(1)

    motors.elevator.step(10, delay=0.01, direction=-1)
    time.sleep(1)

    # ==========================
    # ENCODER TEST
    # ==========================
    '''print("=== ENCODER TEST ===")

    for i in range(10):
        angle = encoder.get_position()

        if angle is None:
            print("Encoder read failed")
        else:
            print(f"Angle: {angle:.2f} deg")

        time.sleep(0.5)
    '''
    # ==========================
    # LIMIT SWITCH TEST
    # ==========================
    '''
    print("=== LIMIT SWITCH TEST ===")

    for i in range(10):
        print(
            "LOWER:",
            devices["LIM_LOWER_1"].is_pressed,
            devices["LIM_LOWER_2"].is_pressed,
            "UPPER:",
            devices["LIM_UPPER_1"].is_pressed,
            devices["LIM_UPPER_2"].is_pressed,
        )
        time.sleep(0.5)
    '''
    # ==========================
    # SIMPLE CLOSED LOOP（可选）
    # ==========================
    '''
    print("=== SIMPLE POSITION HOLD ===")

    target = 0  # 目标角度
    Kp = 0.01

    for _ in range(100):
        angle = encoder.get_position()

        if angle is None:
            continue

        error = target - angle
        speed = max(-1, min(1, Kp * error))

        motors.yaw.set_speed(speed)

        print(f"Target: {target:.1f} | Angle: {angle:.2f} | Speed: {speed:.2f}")

        time.sleep(0.05)
    '''
    motors.yaw.stop()


except KeyboardInterrupt:
    print("\nStopped by user")


finally:
    print("=== CLEANUP ===")

    motors.stop_all()
    motors.disable_all()
    encoder.close()
    cleanup()
