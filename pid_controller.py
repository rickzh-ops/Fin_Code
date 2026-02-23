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
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(-100, 100), deadzone=0.5):
        """
        Initialize the PID controller.
        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param setpoint: Target value (e.g., target angle)
        :param output_limits: (min, max) tuple for PWM output clamping
        :param deadzone: Minimum error threshold to trigger motor movement
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._min_output, self._max_output = output_limits
        self.deadzone = deadzone
        
        # Internal state variables
        self._prev_error = 0
        self._integral = 0
        self._last_time = time.time()

    def update(self, measurement):
        """
        Calculates PID output based on current measurement.
        :param measurement: Current feedback value (e.g., current encoder angle)
        :return: Controlled output value (e.g., motor PWM duty cycle)
        """
        now = time.time()
        dt = now - self._last_time
        
        # Avoid division by zero if update is called too fast
        if dt <= 0:
            dt = 1e-6 
        
        # Calculate error
        error = self.setpoint - measurement
        
        # Deadzone logic: ignore small errors to prevent "hunting" or jitter
        if abs(error) < self.deadzone:
            return 0

        # 1. Proportional term
        p_term = self.kp * error
        
        # 2. Integral term
        self._integral += error * dt
        i_term = self.ki * self._integral
        
        # 3. Derivative term
        derivative = (error - self._prev_error) / dt
        d_term = self.kd * derivative
        
        # Calculate raw output
        output = p_term + i_term + d_term
        
        # 4. Output Clamping & Anti-Windup
        # If the output hits a limit, we stop increasing the integral to prevent "windup"
        if output > self._max_output:
            if self.ki != 0:
                self._integral -= error * dt  # Simple anti-windup: undo integration
            output = self._max_output
        elif output < self._min_output:
            if self.ki != 0:
                self._integral -= error * dt
            output = self._min_output
            
        # Update state for next iteration
        self._prev_error = error
        self._last_time = now
        
        return output

    def set_setpoint(self, new_setpoint):
        """
        Dynamically update the target position.
        """
        if self.setpoint != new_setpoint:
            self.setpoint = new_setpoint
            # Optional: Reset integral when target changes significantly to avoid overshoot
            # self.reset()

    def reset(self):
        """
        Resets the controller internal state.
        Useful when the system is re-enabled or hits a limit switch.
        """
        self._prev_error = 0
        self._integral = 0
        self._last_time = time.time()

    def update_gains(self, kp=None, ki=None, kd=None):
        """
        Allows tuning of PID parameters during runtime.
        """
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd