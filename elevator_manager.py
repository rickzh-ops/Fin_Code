"""
Module: Elevator Manager (State Machine)
Author: Zhang
Original Ref: elevator.cpp / limswitch.cpp
Description: 
    Enhanced version. Manages deployment based on lookup table validity.
    Uses limit switches as the source of truth for "Deployed" and "Stowed" states.
"""

import time

class ElevatorManager:
    def __init__(self, motor_driver, limit_switch_top, limit_switch_bottom):
        self.motor = motor_driver
        self.ls_top = limit_switch_top      # Expected to have .is_pressed property
        self.ls_bottom = limit_switch_bottom # Expected to have .is_pressed property
        
        # Internal state tracking
        self.MOVING = "MOVING"
        self.DEPLOYED = "DEPLOYED"
        self.STOWED = "STOWED"
        self.UNKNOWN = "UNKNOWN"

    @property
    def is_fully_deployed(self):
        """Returns True only if the top limit switch is physically pressed."""
        return self.ls_top.is_pressed

    @property
    def is_fully_stowed(self):
        """Returns True only if the bottom limit switch is physically pressed."""
        return self.ls_bottom.is_pressed

    def get_status(self):
        if self.is_fully_deployed:
            return self.DEPLOYED
        if self.is_fully_stowed:
            return self.STOWED
        return self.UNKNOWN

    def update_position(self, should_be_active):
        """
        Decision-making logic called by Zhang's main loop.
        :param should_be_active: Boolean from aero_logic (True if fin can generate lift)
        """
        if should_be_active:
            if not self.is_fully_deployed:
                self._move_to_top()
        else:
            if not self.is_fully_stowed:
                self._move_to_bottom()

    def _move_to_top(self):
        print("Elevator: Deploying to top limit...")
        while not self.is_fully_deployed:
            self.motor.step(direction=1) # Direction 1 = Up
            time.sleep(0.002) 
        self.motor.stop()
        print("Elevator: Deployment confirmed by limit switch.")

    def _move_to_bottom(self):
        print("Elevator: Stowing to bottom limit...")
        while not self.is_fully_stowed:
            self.motor.step(direction=0) # Direction 0 = Down
            time.sleep(0.002)
        self.motor.stop()
        print("Elevator: Stowage confirmed by limit switch.")

    def emergency_stop(self):
        self.motor.stop()
        print("Elevator: EMERGENCY STOP triggered.")


#operation time
    def _move_to_top(self):
        print("Elevator: deploying...")
       
        start_time = time.time()
        while time.time() - start_time < 10:
            self.motor.step(direction=1)

        self.ls_top.is_pressed = True 
        self.ls_bottom.is_pressed = False
        self.motor.stop()