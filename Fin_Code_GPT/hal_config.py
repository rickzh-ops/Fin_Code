"""
Module: Hardware Abstraction Layer Config
#Author: He
#Original Ref: pinmap.cpp
#Description: 
    Centralized configuration for Raspberry Pi pin mapping. 
    Maps physical GPIO pins to logical objects (e.g., Motor Enable, 
    Limit Switches, SPI Chip Selects) and initializes the GPIO modes.
"""
from gpiozero import DigitalOutputDevice, PWMOutputDevice, Button

PINS = {

    # Stepper
    "STEP": 2,
    "DIR": 3,
    "EN": 4,

    # Limit switches
    "LIM_TOP": 20,
    "LIM_BOTTOM": 21,

    # Yaw Motor PWM
    "PWM_CCW": 12,
    "PWM_CW": 13
}

devices = {}


def setup_gpio():

    devices["STEP"] = DigitalOutputDevice(PINS["STEP"])
    devices["DIR"] = DigitalOutputDevice(PINS["DIR"])
    devices["EN"] = DigitalOutputDevice(PINS["EN"])

    devices["PWM_CCW"] = PWMOutputDevice(PINS["PWM_CCW"], frequency=1000)
    devices["PWM_CW"] = PWMOutputDevice(PINS["PWM_CW"], frequency=1000)

    devices["LIM_TOP"] = Button(PINS["LIM_TOP"], pull_up=True)
    devices["LIM_BOTTOM"] = Button(PINS["LIM_BOTTOM"], pull_up=True)
