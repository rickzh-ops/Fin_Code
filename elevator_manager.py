"""
Module: Elevator Manager (State Machine)
Author: Zhang
Original Ref: elevator.cpp / limswitch.cpp
Description: 
    Manages the deployment and stowing of the fin wing. 
    Logic: Controls the stepper motor sequences while continuously 
    polling the limit switch status provided by the HAL. 
    Ensures immediate motor stoppage if a limit is reached.
"""