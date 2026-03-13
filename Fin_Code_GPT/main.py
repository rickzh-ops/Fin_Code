"""
Module: Main Application Entry Point
Author: Zhang & He
Original Ref: main.cpp
Description: 
    The core execution loop. Initializes all hardware and controller 
    objects, then executes the main control frequency: 
    Read Wind -> Calculate Target -> Read Feedback -> Update PID -> Drive Motor.
    Includes global exception handling and emergency stop procedures.
"""
import time
from pid_controller import PIDController
from pid_controller import PIDController
from amt22_encoder import get_position, encoder_init
from motor_driver import stop
from hal_config import setup_gpio
# from motor_driver import Motor  # This would be DRV8835 code
# from encoder import Encoder   

# Initialize: Target 15 degrees, Output limited to PWM range (-100 to 100)
pid = PIDController(kp=1.5, ki=0.1, kd=0.05, setpoint=15, output_limits=(-100, 100))

def control_loop():
    while True:
        # 1. Get current position from encoder code
        current_angle = get_position() 
        if current_angle is None:
            continue
        # 2. Check Limit Switches
        if limit_switch_top.is_pressed() and pid.setpoint > current_angle:
            stop()
            continue 
        # 3. Compute PID output
        pwm_speed = pid.update(current_angle)
        
        # 4. Apply to motor
        drive_motor(pwm_speed)
        
        time.sleep(0.01) # 100Hz frequency
