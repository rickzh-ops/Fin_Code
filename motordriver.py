import time
from hal import devices


class MotorFault(Exception):
    pass


def _get(name):
    if name not in devices:
        raise MotorFault(f"{name} not initialized")
    return devices[name]


class YawMotor:
    def __init__(self):
        self.enabled = False

    def enable(self):
        _get("YAW_EN").off()
        self.enabled = True

    def disable(self):
        _get("YAW_EN").on()
        self.stop()
        self.enabled = False

    def set_speed(self, speed):
        if not self.enabled:
            raise MotorFault("Yaw motor not enabled")

        speed = max(-1, min(1, speed))

        if speed > 0:
            _get("CW_PWM").value = speed
            _get("CCW_PWM").value = 0
        elif speed < 0:
            _get("CCW_PWM").value = -speed
            _get("CW_PWM").value = 0
        else:
            self.stop()

    def stop(self):
        _get("CW_PWM").value = 0
        _get("CCW_PWM").value = 0


class StepperMotor:
    def __init__(self):
        self.enabled = False

    def enable(self):
        _get("ELEVATOR_EN").off()
        self.enabled = True

    def disable(self):
        _get("ELEVATOR_EN").on()
        self.enabled = False

    def step(self, steps=1, delay=0.001, direction=1):
        if not self.enabled:
            raise MotorFault("Stepper not enabled")

        _get("ELEVATOR_DIR").value = (direction > 0)

        for _ in range(steps):
            _get("ELEVATOR_STEP").on()
            time.sleep(delay)
            _get("ELEVATOR_STEP").off()
            time.sleep(delay)


class MotorSystem:
    def __init__(self):
        self.yaw = YawMotor()
        self.elevator = StepperMotor()

    def enable_all(self):
        self.yaw.enable()
        self.elevator.enable()

    def disable_all(self):
        self.yaw.disable()
        self.elevator.disable()

    def stop_all(self):
        self.yaw.stop()
