"""
Module: Motor Driver Interface
Author: He
Original Ref: stepper.cpp / yawDC.cpp
Description: 
    Low-level hardware abstraction for motor control. Includes PWM 
    generation for the DRV8835 (Yaw DC Motor) and pulse generation 
    for the Stepper Motor (Elevator). Provides simple APIs like set_speed().
"""
"""
Motor Driver (Unified gpiozero version)

Contains:
1. Stepper Motor Driver (SPI + STEP/DIR)
2. Yaw DC Motor Driver (DRV8835 PWM)
"""

import time
import spidev

from gpiozero import DigitalOutputDevice, PWMOutputDevice
from hal_config import devices
import amt22_encoder as encoder


# =====================================================
# GLOBAL STATE
# =====================================================

motors_enabled = False

MAX_ANGLE = 20
PWM_PERIOD = 20000
PWM_FREQ = 50

STEP_DELAY = 0.0005


# =====================================================
# SPI DRIVER (STEPPER)
# =====================================================

spi = spidev.SpiDev()

REG_CTRL = 0x00
REG_TORQUE = 0x01
REG_OFF = 0x02
REG_BLANK = 0x03
REG_DECAY = 0x04
REG_STALL = 0x05
REG_DRIVE = 0x06


ctrl = 0xC10
torque = 0x1FF
off = 0x030
blank = 0x080
decay = 0x110
stall = 0x040
drive = 0xA59


# =====================================================
# CHIP SELECT
# =====================================================

def set_cs_low():
    devices["ELEVATOR_CS"].off()


def set_cs_high():
    devices["ELEVATOR_CS"].on()


# =====================================================
# SPI COMMAND
# =====================================================

def send_cmd(reg, value, is_read=False):

    if is_read:
        command = (0x8 | (reg & 0x07)) << 12
    else:
        command = ((reg & 0x07) << 12) | (value & 0xFFF)

    buf = [(command >> 8) & 0xFF, command & 0xFF]

    set_cs_low()
    resp = spi.xfer2(buf)
    set_cs_high()

    return (resp[0] << 8) | resp[1]


def write_reg(reg, value):
    send_cmd(reg, value, False)


def read_reg(reg):
    return send_cmd(reg, 0, True) & 0xFFF


# =====================================================
# STEPPER INITIALIZATION
# =====================================================

def init_stepper():

    spi.open(0, 0)
    spi.max_speed_hz = 500000
    spi.mode = 0

    set_cs_high()

    write_reg(REG_TORQUE, torque)
    write_reg(REG_OFF, off)
    write_reg(REG_BLANK, blank)
    write_reg(REG_DECAY, decay)
    write_reg(REG_STALL, stall)
    write_reg(REG_DRIVE, drive)
    write_reg(REG_CTRL, ctrl)


# =====================================================
# STEPPER STEP CONTROL
# =====================================================

def step(direction):

    devices["ELEVATOR_DIR"].value = direction

    devices["ELEVATOR_STEP"].on()
    time.sleep(STEP_DELAY)
    devices["ELEVATOR_STEP"].off()


def move_steps(steps, direction):

    for _ in range(steps):

        step(direction)
        time.sleep(0.001)


# =====================================================
# YAW MOTOR INITIALIZATION
# =====================================================

def init_yaw():

    global motors_enabled

    devices["PIN_EN"].on()      # disable
    devices["PIN_SLEEP"].on()   # wake

    stop()


# =====================================================
# YAW MOTOR CONTROL
# =====================================================

def turn_clockwise():

    global motors_enabled
    motors_enabled = True

    devices["PIN_EN"].on()

    devices["CCW_PWM"].value = 0

    devices["PIN_EN"].off()


def turn_counter_clockwise():

    global motors_enabled
    motors_enabled = True

    devices["PIN_EN"].on()

    devices["CW_PWM"].value = 0

    devices["PIN_EN"].off()


def stop():

    global motors_enabled

    motors_enabled = False

    devices["CW_PWM"].value = 0
    devices["CCW_PWM"].value = 0

    devices["PIN_EN"].on()


# =====================================================
# POWER CONTROL
# =====================================================

def sleep():

    devices["PIN_SLEEP"].off()


def wake():

    devices["PIN_SLEEP"].on()


# =====================================================
# PWM CONTROL
# =====================================================

def set_duty_cycle(percent, direction):

    percent = max(0, min(99, percent))

    duty = percent / 100.0

    if direction:
        devices["CCW_PWM"].value = duty
    else:
        devices["CW_PWM"].value = duty


# =====================================================
# ENCODER
# =====================================================

def read_current_val(motor_side):

    return encoder.get_val()


def get_current_angle():

    return encoder.get_pos()
