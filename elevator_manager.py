"""
Module: Elevator Manager
Author: Zhang
Description:
    Manages elevator deployment/stowage using limit switches as the only
    source of truth for end-position detection.

Behavior:
    - If should_be_active is True:
        move upward until the top limit switch is triggered.
    - If should_be_active is False:
        only stow when fin angle is near zero, then move downward until
        the bottom limit switch is triggered.

Notes:
    - This version does NOT use timeout.
    - This version assumes limit_switch_top.is_pressed and
      limit_switch_bottom.is_pressed are real hardware readings.
    - This version is blocking during motion.
"""

import time


class ElevatorManager:
    def __init__(self, motor_driver, limit_switch_top, limit_switch_bottom):
        self.motor = motor_driver
        self.ls_top = limit_switch_top          # must provide .is_pressed
        self.ls_bottom = limit_switch_bottom    # must provide .is_pressed

        self.MOVING = "MOVING"
        self.DEPLOYED = "DEPLOYED"
        self.STOWED = "STOWED"
        self.UNKNOWN = "UNKNOWN"

    @property
    def is_fully_deployed(self):
        """True only when the physical top limit switch is pressed."""
        return self.ls_top.is_pressed

    @property
    def is_fully_stowed(self):
        """True only when the physical bottom limit switch is pressed."""
        return self.ls_bottom.is_pressed

    def get_status(self):
        """
        Determine current elevator status from limit switches only.
        """
        if self.is_fully_deployed and not self.is_fully_stowed:
            return self.DEPLOYED
        elif self.is_fully_stowed and not self.is_fully_deployed:
            return self.STOWED
        elif not self.is_fully_deployed and not self.is_fully_stowed:
            return self.UNKNOWN
        else:
            # Both switches pressed at once is usually abnormal
            return self.UNKNOWN

    def update_position(self, should_be_active, current_fin_angle):
        """
        Main decision logic.

        Parameters
        ----------
        should_be_active : bool
            From aero logic. True means elevator should deploy.
            False means elevator should stow.
        current_fin_angle : float
            Current fin angle in degrees. Elevator is only allowed to stow
            when fin is near zero for safety.
        """
        if should_be_active:
            if not self.is_fully_deployed:
                self._move_to_top()
            else:
                self.motor.stop()
        else:
            if abs(current_fin_angle) < 0.5:
                if not self.is_fully_stowed:
                    self._move_to_bottom()
                else:
                    self.motor.stop()
            else:
                print(
                    f"Elevator Safety: Waiting for fin to zero "
                    f"(Current: {current_fin_angle:.2f}°)"
                )
                self.motor.stop()

    def _move_to_top(self):
        """
        Move upward until the top limit switch is physically triggered.
        """
        print("Elevator: Deploying to top limit...")

        while not self.is_fully_deployed:
            self.motor.step(direction=1)   # 1 = Up
            time.sleep(0.002)

        self.motor.stop()
        print("Elevator: Deployment confirmed by top limit switch.")

    def _move_to_bottom(self):
        """
        Move downward until the bottom limit switch is physically triggered.
        """
        print("Elevator: Stowing to bottom limit...")

        while not self.is_fully_stowed:
            self.motor.step(direction=0)   # 0 = Down
            time.sleep(0.002)

        self.motor.stop()
        print("Elevator: Stowage confirmed by bottom limit switch.")

    def emergency_stop(self):
        """
        Immediately stop the motor.
        """
        self.motor.stop()
        print("Elevator: EMERGENCY STOP triggered.")