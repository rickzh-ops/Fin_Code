"""
Module: Motor Driver Interface
Author: He
Original Ref: stepper.cpp / yawDC.cpp
Description: 
    Low-level hardware abstraction for motor control. Includes PWM 
    generation for the DRV8835 (Yaw DC Motor) and pulse generation 
    for the Stepper Motor (Elevator). Provides simple APIs like set_speed().
"""
import spidev
import time
import RPi.GPIO as GPIO

class Stepper:
    REG_CTRL   = 0x00
    REG_TORQUE = 0x01
    REG_OFF    = 0x02
    REG_BLANK  = 0x03
    REG_DECAY  = 0x04
    REG_STALL  = 0x05
    REG_DRIVE  = 0x06

    def __init__(self, bus=0, device=0):
        # Pin Setup
        self.ELEVATOR_EN = 17
        self.ELEVATOR_CS = 8
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.ELEVATOR_EN, self.ELEVATOR_CS], GPIO.OUT)
        
        # SPI Setup
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 500000
        self.spi.mode = 0b00

        # Register values
        self.ctrl   = 0xC10
        self.torque = 0x1FF
        self.off    = 0x030
        self.blank  = 0x080
        self.decay  = 0x110
        self.stall  = 0x040
        self.drive  = 0xA59

    def init(self):
        self.set_cs_high()
        
        # Write initial values
        self.write_reg(self.REG_TORQUE, self.torque)
        self.write_reg(self.REG_OFF, self.off)
        self.write_reg(self.REG_BLANK, self.blank)
        self.write_reg(self.REG_DECAY, self.decay)
        self.write_reg(self.REG_STALL, self.stall)
        self.write_reg(self.REG_DRIVE, self.drive)
        self.write_reg(self.REG_CTRL, self.ctrl)

    def set_cs_low(self):
        GPIO.output(self.ELEVATOR_CS, GPIO.LOW)

    def set_cs_high(self):
        GPIO.output(self.ELEVATOR_CS, GPIO.HIGH)

    def sleep(self):
        GPIO.output(self.ELEVATOR_EN, GPIO.LOW)

    def wakeup(self):
        GPIO.output(self.ELEVATOR_EN, GPIO.HIGH)

    def send_cmd(self, reg, value, is_read=False):
        if is_read:
            # Command for READ
            command = (0x8 | (reg & 0x07)) << 12
        else:
            # Command for WRITE
            command = ((reg & 0x07) << 12) | (value & 0xFFF)

        # Split 16-bit command into two 8-bit bytes
        buf = [(command >> 8) & 0xFF, command & 0xFF]
        
        self.set_cs_high() 
        time.sleep(0.001)  
        
        # Sends buf and returns received bytes
        resp = self.spi.xfer2(buf)
        
        time.sleep(0.001)
        self.set_cs_low()
        
        return (resp[0] << 8) | resp[1]

    def write_reg(self, reg, value):
        self.send_cmd(reg, value, is_read=False)

    def read_reg(self, reg):
        return self.send_cmd(reg, 0, is_read=True) & 0xFFF

    def enable_driver(self):
        # Flips the ENBL bit (Bit 0) in the CTRL register
        self.ctrl |= 0x001
        self.write_reg(self.REG_CTRL, self.ctrl)

# Configuration
pi = pigpio.pi()

# Pin Definitions
PIN_EN = 17
PIN_SLEEP = 27
PIN_CW_PWM = 13
PIN_CCW_PWM = 18

# Constants
MAX_ANGLE = 20
PWM_PERIOD = 20000
PWM_FREQ = 50
motors_enabled = False

def init():
    global motors_enabled
    
    # GPIO Setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([PIN_EN, PIN_SLEEP], GPIO.OUT)
    
    # Initial State
    GPIO.output(PIN_EN, GPIO.HIGH)    # Disable
    GPIO.output(PIN_SLEEP, GPIO.HIGH) # Wake
    
    # PWM Setup
    pi.set_mode(PIN_CW_PWM, pigpio.OUTPUT)
    pi.set_mode(PIN_CCW_PWM, pigpio.OUTPUT)
    pi.set_PWM_frequency(PIN_CW_PWM, PWM_FREQ)
    pi.set_PWM_frequency(PIN_CCW_PWM, PWM_FREQ)
    pi.set_PWM_range(PIN_CW_PWM, PWM_PERIOD)
    pi.set_PWM_range(PIN_CCW_PWM, PWM_PERIOD)
    
    stop()

def turn_clockwise():
    global motors_enabled
    motors_enabled = True
    GPIO.output(PIN_EN, GPIO.HIGH) # Disable during switch
    pi.set_PWM_dutycycle(PIN_CCW_PWM, 0)
    GPIO.output(PIN_EN, GPIO.LOW)  # Enable

def turn_counter_clockwise():
    global motors_enabled
    motors_enabled = True
    GPIO.output(PIN_EN, GPIO.HIGH)
    pi.set_PWM_dutycycle(PIN_CW_PWM, 0)
    GPIO.output(PIN_EN, GPIO.LOW)

def stop():
    global motors_enabled
    motors_enabled = False
    pi.set_PWM_dutycycle(PIN_CW_PWM, 0)
    pi.set_PWM_dutycycle(PIN_CCW_PWM, 0)
    GPIO.output(PIN_EN, GPIO.HIGH)

def sleep():
    GPIO.output(PIN_SLEEP, GPIO.LOW)

def wake():
    GPIO.output(PIN_SLEEP, GPIO.HIGH)

def set_duty_cycle(percent, direction):
    percent = max(0, min(99, percent))
    pulse_width = int((percent / 100.0) * PWM_PERIOD)
    
    if direction:
        pi.set_PWM_dutycycle(PIN_CCW_PWM, pulse_width)
    else:
        pi.set_PWM_dutycycle(PIN_CW_PWM, pulse_width)

def read_current_val(motor_side):
    return encoder.get_val()

def get_current_angle():
    return encoder.get_pos()
    if clockwise != (output <= 0):
        stop()
    flag = True
            
    if flag:
        clockwise = (output <= 0)
    if output > 0:
            turn_counter_clockwise()
        else:
        turn_clockwise()
    flag = False

    # Apply PWM
    set_duty_cycle(int(abs(output)), output > 0)

    # Save State
    prev_error = error
        
    # Precise 1ms timing (mimics vTaskDelayUntil)
    last_tick += ts_s
    sleep_time = last_tick - time.perf_counter()
    if sleep_time > 0:
        time.sleep(sleep_time)

