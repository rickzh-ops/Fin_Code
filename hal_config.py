"""
Module: Hardware Abstraction Layer Config
Author: He
Original Ref: pinmap.cpp
Description: 
    Centralized configuration for Raspberry Pi pin mapping. 
    Maps physical GPIO pins to logical objects (e.g., Motor Enable, 
    Limit Switches, SPI Chip Selects) and initializes the GPIO modes.
"""
import RPi.GPIO as GPIO
import time
PINS = {
    #Elevator
    "ELEVATOR_CS":    1,   
    "ELEVATOR_STEP":  2,   
    "ELEVATOR_DIR":   3,   
    "ELEVATOR_EN":    4,   
    "ELEVATOR_RESET": 10,  
    "E_BRAKE":        6,   

    #Limit switch
    "LIM_LOWER_1":    15,  
    "LIM_LOWER_2":    16,  
    "LIM_UPPER_1":    20, 
    "LIM_UPPER_2":    21,  
    
    #Yaw Control
    "YAW_EN":         22,  
    "CCW_PWM":        12,  
    "CW_PWM":         13, 
    "DEBUG_LED":      25, 
}

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Out pins
    out_pins = [
        PINS["ELEVATOR_CS"], PINS["ELEVATOR_STEP"], PINS["ELEVATOR_DIR"],
        PINS["ELEVATOR_EN"], PINS["ELEVATOR_RESET"], PINS["E_BRAKE"],
        PINS["YAW_EN"], PINS["DEBUG_LED"]
    ]
    GPIO.setup(out_pins, GPIO.OUT, initial=GPIO.LOW)

    # In pins
    in_pins = [
        PINS["LIM_LOWER_1"], PINS["LIM_LOWER_2"], 
        PINS["LIM_UPPER_1"], PINS["LIM_UPPER_2"]
    ]
    GPIO.setup(in_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    #Set pwm
    global ccw_pwm_ctrl
    GPIO.setup(PINS["CCW_PWM"], GPIO.OUT)
    ccw_pwm_ctrl = GPIO.PWM(PINS["CCW_PWM"], 1000) 
    ccw_pwm_ctrl.start(0)

def cleanup():
    if 'ccw_pwm_ctrl' in globals():
        ccw_pwm_ctrl.stop()
    GPIO.cleanup()
