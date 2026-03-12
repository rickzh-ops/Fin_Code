Module: Hardware Abstraction Layer Config
Author: He
Original Ref: pinmap.cpp
Description: 
    Centralized configuration for Raspberry Pi pin mapping. 
    Maps physical GPIO pins to logical objects (e.g., Motor Enable, 
    Limit Switches, SPI Chip Selects) and initializes the GPIO modes.
"""
from gpiozero import DigitalOutputDevice, Button, PWMOutputDevice
from signal import pause

PINS = {
    # Elevator
    "ELEVATOR_CS":    1,   
    "ELEVATOR_STEP":  2,   
    "ELEVATOR_DIR":   3,   
    "ELEVATOR_EN":    4,   
    "ELEVATOR_RESET": 10,  
    "E_BRAKE":        6,   

    # Limit switch
    "LIM_LOWER_1":    15,  
    "LIM_LOWER_2":    16,  
    "LIM_UPPER_1":    20, 
    "LIM_UPPER_2":    21,  
    
    # Yaw Control
    "YAW_EN":         22,  
    "CCW_PWM":        12,  
    "CW_PWM":         13, 
    "DEBUG_LED":      25, 
}

devices = {}

def setup_gpio():
    # DigitalOutputDevice
    # initial_value=False
    out_pin_names = [
        "ELEVATOR_CS", "ELEVATOR_STEP", "ELEVATOR_DIR",
        "ELEVATOR_EN", "ELEVATOR_RESET", "E_BRAKE",
        "YAW_EN", "DEBUG_LED"
    ]
    for name in out_pin_names:
        devices[name] = DigitalOutputDevice(PINS[name], initial_value=False)

    # Button
    # pull_up=True
    in_pin_names = ["LIM_LOWER_1", "LIM_LOWER_2", "LIM_UPPER_1", "LIM_UPPER_2"]
    for name in in_pin_names:
        devices[name] = Button(PINS[name], pull_up=True)

    # Initialize PWM (PWMOutputDevice)
    devices["CCW_PWM"] = PWMOutputDevice(PINS["CCW_PWM"], frequency=1000, initial_value=0)

def cleanup():
    for device in devices.values():
        device.close()
