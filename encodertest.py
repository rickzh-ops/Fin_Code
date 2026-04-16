import spidev
import os
import time
import RPi.GPIO as GPIO

CS_PIN = 16   # BCM numbering
CS_HOLD_SECONDS = float(os.environ.get("CS_HOLD_SECONDS", "0.00001"))
PRINT_CS = os.environ.get("PRINT_CS", "0") == "1"

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT, initial=GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(1, 0)              
spi.no_cs = True
spi.max_speed_hz = 500000   
spi.mode = 0b00
spi.bits_per_word = 8

def read_amt22():
    GPIO.output(CS_PIN, GPIO.LOW)
    if PRINT_CS:
        print(f"CS LOW readback={GPIO.input(CS_PIN)}")
    time.sleep(CS_HOLD_SECONDS)

    rx = spi.xfer2([0x00, 0x00])

    time.sleep(CS_HOLD_SECONDS)
    GPIO.output(CS_PIN, GPIO.HIGH)
    if PRINT_CS:
        print(f"CS HIGH readback={GPIO.input(CS_PIN)}")

    raw = (rx[0] << 8) | rx[1]
    pos = raw & 0x3FFF      # 14-bit position
    angle = pos * 360.0 / 16384.0
    return raw, pos, angle

try:
    while True:
        raw, pos, angle = read_amt22()
        print(f"raw=0x{raw:04X}, pos={pos:5d}, angle={angle:7.2f} deg")
        time.sleep(0.1)

except KeyboardInterrupt:
    spi.close()
    GPIO.cleanup()
