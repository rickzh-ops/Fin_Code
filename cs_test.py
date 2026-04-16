import time
import RPi.GPIO as GPIO

CS_PIN = 16  # BCM GPIO16, physical pin 36

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT, initial=GPIO.HIGH)

try:
    while True:
        GPIO.output(CS_PIN, GPIO.LOW)
        print("CS LOW")
        time.sleep(1)

        GPIO.output(CS_PIN, GPIO.HIGH)
        print("CS HIGH")
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.output(CS_PIN, GPIO.HIGH)
    GPIO.cleanup()
