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
            "CS": 16,
            #"MISO": 9,
            #"MOSI": 10,
            #"SCLK": 11,
            # Limit switches
            "LIM_LOWER": 17,
            "LIM_UPPER": 26
        }

        self._init_pins()

    # =========================
    # init
    # =========================
    def _init_pins(self):
        for name, pin in self.PINS.items():
            if name == "CS":
                continue
            if "LIM" in name:
                self.pi.set_mode(pin, pigpio.INPUT)
                self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
            else:
                self.pi.set_mode(pin, pigpio.OUTPUT)

        # Match the working motor-driver example: EN high means enabled.
        self.pi.write(self.PINS["M1_EN"], 1)
        self.pi.write(self.PINS["M2_EN"], 1)

    # =========================
    # Motor Control
    # =========================
    def set_elevator_motor_speed(self, m1):
        self._set_single_motor("M1", m1)

    def set_yaw_motor_speed(self, m2):
        self._set_single_motor("M2", m2)

    def _set_single_motor(self, prefix, speed):
        enable_pin = self.PINS[f"{prefix}_EN"]

        if speed == 0:
            self.pi.hardware_PWM(self.PINS[f"{prefix}_PWM"], 20000, 0)
            self.pi.write(enable_pin, 1)
            return

        if speed < 0:
            direction = 1
            speed = -speed
        else:
            direction = 0

        if speed > self.MAX_SPEED:
            speed = self.MAX_SPEED

        self.pi.write(enable_pin, 1)
        self.pi.write(self.PINS[f"{prefix}_DIR"], direction)
        duty = int(speed * 6250 / 3)
        self.pi.hardware_PWM(self.PINS[f"{prefix}_PWM"], 20000, duty)
    '''
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
    '''
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
        self.set_elevator_motor_speed(0)
        self.set_yaw_motor_speed(0)

    def close(self):
        self.stop_all()
        self.pi.stop()
