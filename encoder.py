import spidev
import time
import RPi.GPIO as GPIO
from hal_plus_motordriver import Motor


class AMT22Encoder:
    AMT22_NOP = 0x00
    AMT22_RESET = 0x60
    AMT22_ZERO = 0x70

    def __init__(self, pi_instance, cs_pin, bus=0, device=0, speed_hz=1000000):
        self.pi = pi_instance
        self.cs_pin = cs_pin
        GPIO.setmode(GPIO.BCM)
        # 初始化 SPI
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.no_cs = True # 我们使用 pigpio 手动控制 CS
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0
        self.spi.bits_per_word = 8
        # 确保 CS 初始为高电平
        self._cs_high()

    def _cs_low(self):
        self.pi.write(self.cs_pin, 0)
        time.sleep(0.000003) # 确保建立时间 (3us)

    def _cs_high(self):
        self.pi.write(self.cs_pin, 1)
        time.sleep(0.000003) # 确保保持时间

    def send_cmd(self, cmd):
        try:
            GPIO.output(self.cs_pin, GPIO.LOW)
            time.sleep(0.00001)
            rx = self.spi.xfer2([0x00, 0x00])
            time.sleep(0.00001)
            GPIO.output(self.cs_pin, GPIO.HIGH)
            return (rx[0] << 8) | rx[1]
        except Exception:
            self._cs_high()
            return None

    def check_parity(self, value):
        if value is None:
            return False

        odd = 0
        even = 0

        for i in range(0, 14, 2):
            even ^= (value >> i) & 1
            odd ^= (value >> (i + 1)) & 1

        odd = not odd
        even = not even

        return ((value >> 15) & 1) == odd and ((value >> 14) & 1) == even

    def get_raw_position(self, retries=3):
        for i in range(retries):
            val = self.send_cmd(self.AMT22_NOP)
            if val is not None and self.check_parity(val):
                return val & 0x3FFF
        return None

    def get_position(self):
        raw = self.get_raw_position()
        if raw is None:
            return None

        angle = (raw / 16384.0) * 360.0
        return (angle + 180) % 360 - 180

    def close(self):
        self.spi.close()
