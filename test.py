#This code is for testing only.


import numpy as np
import matplotlib.pyplot as plt
import random
from pid_controller import PIDController
from aero_logic import AeroLogic
from elevator_manager import ElevatorManager

USE_RANDOM_WIND = False  # Set to True for stochastic wind
DT = 0.005               # Time step (seconds)

# --- Mock Hardware ---
class MockMotor:
    def step(self, direction): pass
    def stop(self): pass

class MockLimitSwitch:
    def __init__(self): self.is_pressed = False

aero = AeroLogic()

# PID Config
pid = PIDController(kp=4.5, ki=1.5, kd=0, output_limits=(-100, 100))

motor = MockMotor()
ls_top, ls_bottom = MockLimitSwitch(), MockLimitSwitch()
elevator = ElevatorManager(motor, ls_top, ls_bottom)

total_time = 60 
steps = int(total_time / DT)
times = np.linspace(0, total_time, steps)

TRANSITION_STEPS = int(10.0 / DT) 


# Wind Generation
if USE_RANDOM_WIND:
    wind_profile = []
    curr_w = 25.0
    for _ in range(steps):
        curr_w += random.uniform(-1.0, 1.0) + (25.0 - curr_w) * 0.05
        wind_profile.append(max(0, curr_w))
else:
    wind_profile = 25 + 15 * np.sin(0.15 * times)

# Variables
current_angle = 0.0
current_state = "STOWED"
transition_timer = 0

history = {"wind":[], "target_lut":[], "actual_angle":[], "state":[]}

print("Starting simulation: Direct LUT Tracking...")

for i in range(steps):
    wind_speed = wind_profile[i]
    should_be_active = 5.0 < wind_speed < 35.0
    
    # 1. State Machine (10s Transition)
    if should_be_active:
        if current_state == "STOWED":
            current_state = "MOVING"
            transition_timer = 0
        elif current_state == "MOVING":
            transition_timer += 1
            if transition_timer >= TRANSITION_STEPS:
                current_state = "DEPLOYED"
    else:
        if current_state == "DEPLOYED":
            current_state = "MOVING"
            transition_timer = 0
        elif current_state == "MOVING":
            transition_timer += 1
            if transition_timer >= TRANSITION_STEPS:
                current_state = "STOWED"

    # 2. Direct Target Logic
    raw_target = aero.get_target_angle(wind_speed)
    
    if current_state == "DEPLOYED":
        # No more ramped_target, use raw_target directly
        pid.set_setpoint(raw_target)
        control_output = pid.update(current_angle)
        
        # Increased physics multiplier to ensure the motor can "keep up"
        current_angle += (control_output * 0.25) 
    else:
        # Hard reset to 0 when not deployed
        current_angle = 0.0
        pid.reset()

    # 3. Data Logging
    history["wind"].append(wind_speed)
    history["target_lut"].append(raw_target if current_state == "DEPLOYED" else 0)
    history["actual_angle"].append(current_angle)
    
    s_map = {"STOWED": 0, "MOVING": 0.5, "DEPLOYED": 1}
    history["state"].append(s_map[current_state])

# --- Plotting ---
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

ax1.plot(times, history["wind"], label='Wind Speed (m/s)')
ax1.axhline(35, color='r', linestyle='--')
ax1.set_ylabel("Wind")
ax1.legend()

# Green is raw, Blue is tracking
ax2.plot(times, history["target_lut"], 'g--', label='Target (LUT)')
ax2.plot(times, history["actual_angle"], 'b-', label='Actual (PID)')
ax2.set_ylabel("Degrees")
ax2.legend()

ax3.plot(times, history["state"], color='purple')
ax3.set_ylim(-0.1, 1.1)
ax3.set_yticks([0, 0.5, 1])
ax3.set_yticklabels(['STOWED', 'MOVING', 'DEPLOYED'])
ax3.set_xlabel("Time (s)")

plt.tight_layout()
plt.show()