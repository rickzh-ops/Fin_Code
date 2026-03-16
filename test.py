import numpy as np
import matplotlib.pyplot as plt
import random
from pid_controller import PIDController
from aero_logic import AeroLogic
from elevator_manager import ElevatorManager

# --- Simulation Settings ---
USE_RANDOM_WIND = False  
DT = 0.01                
TOTAL_TIME = 60          

# --- Physical Constraints (Worm gear characteristics) ---
MAX_DEG_PER_SEC = 30.0   
MAX_STEP_CHANGE = MAX_DEG_PER_SEC * DT 

# --- Mock Hardware ---
class MockMotor:
    def step(self, direction): pass
    def stop(self): pass

class MockLimitSwitch:
    def __init__(self): self.is_pressed = False

# Initialize components
aero = AeroLogic()

# Set deadzone=0.0 to ensure Ki can eliminate steady-state error
pid = PIDController(kp=10.0, ki=15.0, kd=0, output_limits=(-100, 100), deadzone=0.0) 

motor = MockMotor()
ls_top, ls_bottom = MockLimitSwitch(), MockLimitSwitch()
elevator = ElevatorManager(motor, ls_top, ls_bottom)

steps = int(TOTAL_TIME / DT)
times = np.linspace(0, TOTAL_TIME, steps)

# Continuously varying wind: Sine wave simulates wind fluctuation
base_wind = 25 + 15 * np.sin(0.15 * times)
wind_profile = np.where(times < 5, 0, base_wind)

# Initial state
current_angle = 0.0
current_state = "STOWED"
transition_timer = 0
TRANSITION_STEPS = int(10.0 / DT) 

history = {"wind":[], "target_lut":[], "actual_angle":[], "state":[]}

for i in range(steps):
    wind_speed = wind_profile[i]
    # Wind operational range: 5.0 < wind < 35.0
    should_be_active = 5.0 < wind_speed < 35.0
    
    # 1. State machine logic (including safety interlock)
    if should_be_active:
        if current_state == "STOWED":
            current_state = "MOVING"
            transition_timer = 0
        elif current_state == "MOVING":
            transition_timer += 1
            if transition_timer >= TRANSITION_STEPS:
                current_state = "DEPLOYED"
    else:
        # When wind speed is out of range (too high or too low)
        if current_state == "DEPLOYED":
            # Core modification: Retraction to MOVING state only allowed when angle is near 0 (error < 0.5)
            if abs(current_angle) < 0.5:
                current_state = "MOVING"
                transition_timer = 0
            else:
                # Otherwise remain in DEPLOYED state, PID will force target to 0 below
                pass
        elif current_state == "MOVING":
            transition_timer += 1
            if transition_timer >= TRANSITION_STEPS:
                current_state = "STOWED"

    # 2. Core: PID tracking with physical constraints
    # Determine PID target: Set to 0 if not in operational range
    raw_target = aero.get_target_angle(wind_speed) if should_be_active else 0.0
    
    # --- 1. Calculate expected physical displacement (Independent physics layer) ---
    # PID should work as long as it's not in the fully retracted (STOWED) state
    if current_state != "STOWED":
        # Target logic: Follow LUT during deployment; target is 0 during retraction or waiting
        active_target = aero.get_target_angle(wind_speed) if (should_be_active and current_state == "DEPLOYED") else 0.0
        
        pid.set_setpoint(active_target)
        control_output = pid.update(current_angle)
        
        # Unified physical slew rate limiting logic
        desired_change = (control_output / 100.0) * MAX_STEP_CHANGE
        actual_change = np.clip(desired_change, -MAX_STEP_CHANGE, MAX_STEP_CHANGE)
        current_angle += actual_change
    else:
        # Force zero and reset PID only after full retraction
        current_angle = 0.0
        pid.reset()

    # --- 2. State machine only handles logic transitions, not current_angle modification ---
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
            # Safety interlock: Elevator starts retraction only when PID brings angle back within 0.5 deg
            if abs(current_angle) < 0.5:
                current_state = "MOVING"
                transition_timer = 0
        elif current_state == "MOVING":
            transition_timer += 1
            if transition_timer >= TRANSITION_STEPS:
                current_state = "STOWED"

    # Record data
    history["wind"].append(wind_speed)
    history["target_lut"].append(raw_target if current_state == "DEPLOYED" else 0)
    history["actual_angle"].append(current_angle)
    
    s_map = {"STOWED": 0, "MOVING": 0.5, "DEPLOYED": 1}
    history["state"].append(s_map[current_state])

# --- Plotting ---
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

# Plot 1: Wind speed
ax1.plot(times, history["wind"], label='Wind Speed (m/s)', color='tab:blue')
ax1.axhline(35, color='r', linestyle='--', label='Cut-off (35m/s)')
ax1.set_ylabel("Wind Speed")
ax1.grid(True, alpha=0.3)
ax1.legend()

# Plot 2: Angle tracking
ax2.plot(times, history["target_lut"], 'g--', label='Target (look-up table)', alpha=0.6)
ax2.plot(times, history["actual_angle"], 'b-', label='Actual Angle (Physical)')
ax2.set_ylabel("Degrees")
ax2.set_title("Fin Angle Tracking")
ax2.grid(True, alpha=0.3)
ax2.legend()

# Plot 3: System state (Observe the timing of DEPLOYED to MOVING transition)
ax3.plot(times, history["state"], color='purple', linewidth=2, label='System State')
ax3.set_ylim(-0.1, 1.1)
ax3.set_yticks([0, 0.5, 1])
ax3.set_yticklabels(['STOWED', 'MOVING', 'DEPLOYED'])
ax3.set_ylabel("Status")
ax3.set_xlabel("Time (s)")
ax3.grid(True, alpha=0.3)
ax3.legend()

plt.tight_layout()
plt.show()