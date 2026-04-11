"""
Main loop for fin yaw control.

References:
- Yaw motor usage follows testmotor_v1.py
- Encoder usage follows encodertest.py and encoder.py

Behavior:
- Read target angle from lookup table data
- Read encoder angle in real time
- Adjust yaw motor output from the encoder error
"""

import time

from aero_logic import AeroLogic
from encoder import AMT22Encoder
from hal_plus_motordriver import Motor
from pid_controller import PIDController

CONTROL_HZ = 100
CONTROL_DT = 1.0 / CONTROL_HZ
LOOKUP_TARGET_INDEX = 8


class FinController:
    def __init__(self):
        self.aero = AeroLogic()
        self.yaw_motor = None
        self.encoder = None
        self.pid = PIDController(
            kp=1.5,
            ki=0.1,
            kd=0.05,
            setpoint=0.0,
            output_limits=(-480, 480),
            deadzone=0.3,
        )

    def initialize(self):
        self.yaw_motor = Motor()
        self.yaw_motor.set_yaw_motor_speed(0)
        self.encoder = AMT22Encoder(self.yaw_motor.pi, self.yaw_motor.PINS["CS"])
        time.sleep(0.2)

    def shutdown(self):
        if self.yaw_motor is not None:
            try:
                self.yaw_motor.set_yaw_motor_speed(0)
                self.yaw_motor.set_elevator_motor_speed(0)
            except Exception:
                pass

        if self.encoder is not None:
            try:
                self.encoder.close()
            except Exception:
                pass

    def get_lookup_target(self):
        targets = self.aero.target_angles
        if not targets:
            return 0.0

        index = max(0, min(LOOKUP_TARGET_INDEX, len(targets) - 1))
        return float(targets[index])

    def read_current_angle(self):
        if self.encoder is None:
            return None

        for method_name in ("get_position", "get_pos", "get_angle", "read_angle"):
            method = getattr(self.encoder, method_name, None)
            if callable(method):
                value = method()
                return float(value) if value is not None else None

        if hasattr(self.encoder, "read_amt22") and callable(self.encoder.read_amt22):
            result = self.encoder.read_amt22()
            if isinstance(result, tuple) and len(result) >= 3:
                return float(result[2])

        raise AttributeError(
            "AMT22Encoder does not expose a supported angle-reading method. "
            "Expected one of: get_position, get_pos, get_angle, read_angle."
        )

    def limit_reached_for_output(self, output):
        if self.yaw_motor is None:
            return False

        if output > 0 and self.yaw_motor.get_upper_limit():
            return True
        if output < 0 and self.yaw_motor.get_lower_limit():
            return True
        return False

    def apply_yaw_output(self, output):
        if self.yaw_motor is None:
            return

        if self.limit_reached_for_output(output):
            self.yaw_motor.set_yaw_motor_speed(0)
            self.pid.reset()
            return

        self.yaw_motor.set_yaw_motor_speed(int(output))

    def control_step(self):
        target_angle = self.get_lookup_target()
        self.pid.set_setpoint(target_angle)

        current_angle = self.read_current_angle()
        if current_angle is None:
            self.yaw_motor.set_yaw_motor_speed(0)
            return None

        yaw_output = self.pid.update(current_angle)
        self.apply_yaw_output(yaw_output)

        return {
            "target_angle": target_angle,
            "current_angle": current_angle,
            "yaw_output": yaw_output,
        }

    def run(self):
        self.initialize()
        try:
            while True:
                started = time.perf_counter()
                state = self.control_step()

                if state is not None:
                    print(
                        "target={target_angle:6.2f} deg | "
                        "angle={current_angle:6.2f} deg | "
                        "yaw={yaw_output:7.2f}".format(**state)
                    )

                elapsed = time.perf_counter() - started
                time.sleep(max(0.0, CONTROL_DT - elapsed))
        finally:
            self.shutdown()


def main():
    controller = FinController()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\nStopped by user")


if __name__ == "__main__":
    main()
