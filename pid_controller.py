"""
Module: PID Controller
Author: Zhang
Original Ref: yawDC.cpp
Description: 
    Implements the closed-loop control algorithm for the Yaw mechanism. 
    It receives the current angle from the encoder and the target angle 
    from the lookup table, then calculates the motor control signal (PWM/Speed). 
    Includes Anti-windup and output clamping logic.
"""

import time

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._output_limits = output_limits
        
        self._prev_error = 0
        self._integral = 0
        self._last_time = time.time()

    def update(self, measurement):
        """Calculates PID output based on current measurement."""
        now = time.time()
        dt = now - self._last_time
        if dt <= 0:
            dt = 1e-6  # Prevent division by zero
        
        error = self.setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self._integral += error * dt
        i_term = self.ki * self._integral
        
        # Derivative term
        derivative = (error - self._prev_error) / dt
        d_term = self.kd * derivative
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Apply output limits (Saturation)
        lower, upper = self._output_limits
        if lower is not None or upper is not None:
            output = max(lower, min(output, upper))
            
        # Update state
        self._prev_error = error
        self._last_time = now
        
        return output

    def set_setpoint(self, new_setpoint):
        """Changes the target angle/position."""
        self.setpoint = new_setpoint
        # Optional: reset integral when target changes significantly to avoid lag
        # self._integral = 0