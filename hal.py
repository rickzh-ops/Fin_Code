from gpiozero import DigitalOutputDevice, Button, PWMOutputDevice

PINS = {
    "ELEVATOR_CS": 17,
    "ELEVATOR_STEP": 27,
    "ELEVATOR_DIR": 23,
    "ELEVATOR_EN": 24,
    "ELEVATOR_RESET": 5,
    "E_BRAKE": 6,

    "LIM_LOWER_1": 15,
    "LIM_LOWER_2": 16,
    "LIM_UPPER_1": 20,
    "LIM_UPPER_2": 21,

    "YAW_EN": 22,
    "CCW_PWM": 12,
    "CW_PWM": 13,
    "DEBUG_LED": 25,
}

devices = {}
_initialized = False


def init():
    global _initialized
    if _initialized:
        return

    # Output pins
    for name in [
        "ELEVATOR_CS", "ELEVATOR_STEP", "ELEVATOR_DIR",
        "ELEVATOR_EN", "ELEVATOR_RESET", "E_BRAKE",
        "YAW_EN", "DEBUG_LED"
    ]:
        devices[name] = DigitalOutputDevice(PINS[name])

    # Input pins
    for name in [
        "LIM_LOWER_1", "LIM_LOWER_2",
        "LIM_UPPER_1", "LIM_UPPER_2"
    ]:
        devices[name] = Button(PINS[name], pull_up=False)

    # PWM
    devices["CCW_PWM"] = PWMOutputDevice(PINS["CCW_PWM"])
    devices["CW_PWM"] = PWMOutputDevice(PINS["CW_PWM"])

    # Default
    devices["CW_PWM"].value = 0
    devices["CCW_PWM"].value = 0
    devices["YAW_EN"].on()          # disable
    devices["ELEVATOR_EN"].on()     # disable
    devices["ELEVATOR_CS"].on()     # CS HIGH

    _initialized = True


def cleanup():
    for d in devices.values():
        d.close()
    devices.clear()
