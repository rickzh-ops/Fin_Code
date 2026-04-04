import pigpio
import time

class Motor:
    MAX_SPEED = 480

    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise IOError("Cannot connect to pigpio")

        # =========================
        # PIN 
        # =========================
        self.PINS = {
            # Motor Driver
            "M1_PWM": 12,
            "M2_PWM": 13,
            "M1_DIR": 24,
            "M2_DIR": 25,
            "M1_EN": 22,
            "M2_EN": 23,
            "M1_DIAG": 5,
            "M2_DIAG": 6,

            # Yaw
            "YAW_PWM_CW": 18,
            "YAW_PWM_CCW": 19,
            "YAW_EN": 27,

            # Elevator
            "STEP": 20,
            "DIR": 21,
            "EN": 16,

            # Limit switches
            "LIM_LOWER": 26,
            "LIM_UPPER": 17,
        }

        self._init_pins()

    # =========================
    # init
    # =========================
    def _init_pins(self):
        for name, pin in self.PINS.items():
            if "LIM" in name:
                self.pi.set_mode(pin, pigpio.INPUT)
                self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
            else:
                self.pi.set_mode(pin, pigpio.OUTPUT)

        # Default
        self.pi.write(self.PINS["M1_EN"], 1)
        self.pi.write(self.PINS["M2_EN"], 1)
        self.pi.write(self.PINS["YAW_EN"], 1)
        self.pi.write(self.PINS["EN"], 1)

    # =========================
    # Motor Control
    # =========================
    
    def set_motor_speed(self, m1, m2):
        self._set_single_motor("M1", m1)
        self._set_single_motor("M2", m2)

    def _set_single_motor(self, prefix, speed):
        if speed < 0:
            direction = 1
            speed = -speed
        else:
            direction = 0

        if speed > self.MAX_SPEED:
            speed = self.MAX_SPEED

        self.pi.write(self.PINS[f"{prefix}_DIR"], direction)
        duty = int(speed * 6250 / 3)
        self.pi.hardware_PWM(self.PINS[f"{prefix}_PWM"], 20000, duty)
    
    # =========================
    # Yaw Control
    # =========================
    def set_yaw(self, speed):
        """
        speed > 0 → CW
        speed < 0 → CCW
        """
        if speed == 0:
            self.pi.hardware_PWM(self.PINS["YAW_PWM_CW"], 20000, 0)
            self.pi.hardware_PWM(self.PINS["YAW_PWM_CCW"], 20000, 0)
            return

        duty = int(min(abs(speed), self.MAX_SPEED) * 6250 / 3)

        if speed > 0:
            self.pi.hardware_PWM(self.PINS["YAW_PWM_CW"], 20000, duty)
            self.pi.hardware_PWM(self.PINS["YAW_PWM_CCW"], 20000, 0)
        else:
            self.pi.hardware_PWM(self.PINS["YAW_PWM_CW"], 20000, 0)
            self.pi.hardware_PWM(self.PINS["YAW_PWM_CCW"], 20000, duty)
    # =========================
    # Elevator Motor
    # =========================
    def move_elevator(self, steps, direction, delay=0.001):
        """
        direction: 1=up, 0=down
        """
        self.pi.write(self.PINS["DIR"], direction)

    # Limit Switch
        for i in range(steps):
            if direction == 1 and self.get_upper_limit():
                print("Upper limit reached!")
                break
            if direction == 0 and self.get_lower_limit():
                print("Lower limit reached!")
                break

            self.pi.write(self.PINS["STEP"], 1)
            time.sleep(delay)
            self.pi.write(self.PINS["STEP"], 0)
            time.sleep(delay)

    def get_lower_limit(self):
        self.pi.set_mode(self.PINS["LIM_LOWER"], pigpio.INPUT)
        self.pi.set_pull_up_down(self.PINS["LIM_LOWER"], pigpio.PUD_UP)
        return self.pi.read(self.PINS["LIM_LOWER"])

    def get_upper_limit(self):
        self.pi.set_mode(self.PINS["LIM_UPPER"], pigpio.INPUT)
        self.pi.set_pull_up_down(self.PINS["LIM_UPPER"], pigpio.PUD_UP)
        return self.pi.read(self.PINS["LIM_UPPER"])
    # =========================
    # Stop
    # =========================
    def stop_all(self):
        global _pi
        _pi.stop()
        _pi = pigpio.pi()
        self.set_motor_speed(0, 0)
        self.set_yaw(0)
