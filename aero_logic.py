"""
Module: Aero Lookup Table Logic
Author: Zhang
Original Ref: OptimalFinAngle.cpp
Description: 
    Aerodynamic decision-making module. It takes the measured wind speed 
    from the CAN bus and finds the corresponding target fin angle using 
    a Lookup Table (LUT) based on pre-calculated aerodynamic data.
"""

import numpy as np

class AeroLogic:
    def __init__(self):
        """
        Initialize the Lookup Table with experimental data.
        The data maps: Wind Speed (m/s) -> Optimal Fin Angle (degrees).
        """
        # --- Example Lookup Table Data ---
        # Replace these with your actual experimental data points
        self.wind_speeds = [0, 5, 10, 15, 20, 25, 30, 35, 40]  # m/s
        self.target_angles = [0, 2, 5, 9, 14, 18, 21, 23, 25]  # degrees
        # ---------------------------------

    def get_target_angle(self, measured_wind_speed):
        """
        Translates wind speed to target angle via linear interpolation.
        :param measured_wind_speed: Current wind speed from CAN bus.
        :return: Target fin angle for the PID controller.
        """
        # Use numpy.interp to handle values between defined data points
        # It also handles edge cases (speeds below min or above max)
        target_angle = np.interp(
            measured_wind_speed, 
            self.wind_speeds, 
            self.target_angles
        )
        
        return float(target_angle)

# --- Quick Test ---
if __name__ == "__main__":
    aero = AeroLogic()
    test_speed = 12.5  # Example wind speed from CAN
    target = aero.get_target_angle(test_speed)
    print(f"Wind Speed: {test_speed} m/s -> Target Angle: {target:.2f} deg")