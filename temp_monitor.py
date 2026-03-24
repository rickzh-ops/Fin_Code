import time
import os


def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp = float(f.read()) / 1000.0
        return temp
    except:
        return None


try:
    print("=== Raspberry Pi Temperature Monitor ===")

    while True:
        temp = get_cpu_temp()

        if temp is None:
            print("Failed to read temperature")
        else:
            status = "OK"

            if temp > 70:
                status = "HOT"
            if temp > 80:
                status = "OVERHEAT"

            print(f"CPU Temp: {temp:.2f} °C | {status}")

        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopped")
                
