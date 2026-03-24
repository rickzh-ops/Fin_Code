import spidev
import time
from hal import devices


class AMT22Encoder:
    AMT22_NOP = 0x00
    AMT22_RESET = 0x60
    AMT22_ZERO = 0x70

    def __init__(self, bus=0, device=0, speed_hz=1000000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.no_cs = True 
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0

        self._cs_high()

    def _cs_low(self):
        devices["ELEVATOR_CS"].off()

    def _cs_high(self):
        devices["ELEVATOR_CS"].on()

    def send_cmd(self, cmd):
        try:
            self._cs_low()
            resp = self.spi.xfer2([self.AMT22_NOP, cmd])
            self._cs_high()
            return (resp[0] << 8) | resp[1]
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
