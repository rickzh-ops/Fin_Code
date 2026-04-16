"""
Main entry point for fin yaw and elevator control.

Modes:
1. Encoder calibration mode
2. Manual wind speed mode
3. 15-second sinusoidal wind tracking mode
4. Angle vs. settling time mode
5. Elevator run mode
"""

import math
import os
import select
import subprocess
import sys
import termios
import threading
import time
import tty
from datetime import datetime

from aero_logic import AeroLogic
from encoder import AMT22Encoder
from hal_plus_motordriver import Motor
from pid_controller import PIDController


CONTROL_HZ = 50
CONTROL_DT = 1.0 / CONTROL_HZ
ENABLE_LIMIT_SWITCH_PROTECTION = False
ENCODER_JOG_SPEED = 180
ENCODER_JOG_DURATION = 0.15
SINE_DURATION_SECONDS = 15.0
SINE_WIND_AMPLITUDE = 20.0
SINE_WIND_OMEGA = math.pi / 2.0
SETTLING_TEST_START_ANGLE = -10
SETTLING_TEST_END_ANGLE = 10
SETTLING_TEST_STEP = 1
SETTLING_TOLERANCE_DEG = 1.0
SETTLING_HOLD_SECONDS = 0.3
SETTLING_MOVE_TIMEOUT = 8.0
ELEVATOR_RUN_SPEED = 100
LIMIT_SIGNAL_PRINT_INTERVAL = 1.0
ELEVATOR_WIND_THRESHOLD = 30.0
LIMIT_SWITCH_ACTIVE_STATE = 0
ELEVATOR_HOME_TIMEOUT = 30.0
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ELEVATOR_DOWN_RUNNER = os.path.join(BASE_DIR, "elevator_down_runner.py")
ELEVATOR_UP_RUNNER = os.path.join(BASE_DIR, "elevator_up_runner.py")


def normalize_angle(angle):
    return ((angle + 180.0) % 360.0) - 180.0


class FinController:
    def __init__(self):
        self.aero = AeroLogic()
        self.yaw_motor = None
        self.elevator_motor = None
        self.encoder = None
        self.encoder_zero_offset = 0.0
        self.elevator_motion_target = None
        self.elevator_current_speed = 0
        self.pid = PIDController(
            kp=200.0,
            ki=5,
            kd=1,
            setpoint=0.0,
            output_limits=(-480, 480),
            deadzone=1.0,
        )

    def initialize_hardware(self):
        self.elevator_motor = Motor()
        self.elevator_motor.set_elevator_motor_speed(0)

        self.yaw_motor = Motor()
        self.yaw_motor.set_yaw_motor_speed(0)

        self.encoder = AMT22Encoder(self.yaw_motor.pi, self.yaw_motor.PINS["CS"])
        time.sleep(1.0)
        self.print_status()
        self.home_elevator_to_lower_limit()

    def shutdown(self):
        for motor in (self.yaw_motor, self.elevator_motor):
            if motor is None:
                continue
            try:
                motor.set_yaw_motor_speed(0)
            except Exception:
                pass
            try:
                motor.set_elevator_motor_speed(0)
            except Exception:
                pass
            close_method = getattr(motor, "close", None)
            if callable(close_method):
                try:
                    close_method()
                except Exception:
                    pass

        if self.encoder is not None:
            try:
                self.encoder.close()
            except Exception:
                pass

    def print_status(self):
        if self.yaw_motor is None:
            return
        print("---- STATUS ----")
        print("Lower limit:", self.yaw_motor.get_lower_limit())
        print("Upper limit:", self.yaw_motor.get_upper_limit())
        print("----------------")

    def read_encoder_state(self, apply_offset=True):
        if self.encoder is None:
            return None

        read_data = getattr(self.encoder, "read_data", None)
        if callable(read_data):
            raw, pos, angle = read_data()
            if angle is None:
                return None
            corrected_angle = (
                normalize_angle(float(angle) - self.encoder_zero_offset)
                if apply_offset
                else normalize_angle(float(angle))
            )
            return {"raw": raw, "pos": pos, "angle": corrected_angle}

        for method_name in ("get_position", "get_pos", "get_angle", "read_angle"):
            method = getattr(self.encoder, method_name, None)
            if callable(method):
                angle = method()
                if angle is None:
                    return None
                corrected_angle = (
                    normalize_angle(float(angle) - self.encoder_zero_offset)
                    if apply_offset
                    else normalize_angle(float(angle))
                )
                return {"raw": None, "pos": None, "angle": corrected_angle}

        raise AttributeError(
            "AMT22Encoder does not expose a supported angle-reading method. "
            "Expected read_data, get_position, get_pos, get_angle, or read_angle."
        )

    def jog_encoder_alignment(self, speed):
        if not self.yaw_allowed_by_upper_limit():
            self.yaw_motor.set_yaw_motor_speed(0)
            print("\nUpper limit switch is not pressed. Yaw motor remains stopped.")
            return
        self.yaw_motor.set_yaw_motor_speed(speed)
        time.sleep(ENCODER_JOG_DURATION)
        self.yaw_motor.set_yaw_motor_speed(0)

    def calibrate_encoder_zero(self):
        print("=== ENCODER CALIBRATION MODE ===")
        print("Use Up/Down arrow to jog the yaw motor.")
        print("Press Enter to accept the current position as encoder zero.")

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while True:
                state = self.read_encoder_state(apply_offset=False)
                if state is not None:
                    raw_text = "----" if state["raw"] is None else f"{state['raw']:04X}"
                    pos_text = "----" if state["pos"] is None else f"{state['pos']:5d}"
                    print(
                        f"\rraw=0x{raw_text} | pos={pos_text} | "
                        f"angle={state['angle']:7.2f} deg   ",
                        end="",
                        flush=True,
                    )

                char = sys.stdin.read(1)
                if char in ("\r", "\n"):
                    final_state = self.read_encoder_state(apply_offset=False)
                    if final_state is not None:
                        self.encoder_zero_offset = final_state["angle"]
                    self.yaw_motor.set_yaw_motor_speed(0)
                    print(f"\nEncoder zero set to {self.encoder_zero_offset:.2f} deg")
                    break

                if char != "\x1b":
                    continue

                next_one = sys.stdin.read(1)
                next_two = sys.stdin.read(1)
                if next_one != "[":
                    continue

                if next_two == "A":
                    self.jog_encoder_alignment(ENCODER_JOG_SPEED)
                elif next_two == "B":
                    self.jog_encoder_alignment(-ENCODER_JOG_SPEED)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.yaw_motor.set_yaw_motor_speed(0)

    def limit_reached_for_output(self, output):
        if not ENABLE_LIMIT_SWITCH_PROTECTION:
            return False
        if self.yaw_motor is None:
            return False
        if output > 0 and self.yaw_motor.get_upper_limit():
            return True
        if output < 0 and self.yaw_motor.get_lower_limit():
            return True
        return False

    def get_elevator_limits(self):
        if self.elevator_motor is None:
            return {"lower": None, "upper": None}
        return {
            "lower": self.elevator_motor.get_lower_limit(),
            "upper": self.elevator_motor.get_upper_limit(),
        }

    def yaw_allowed_by_upper_limit(self):
        limits = self.get_elevator_limits()
        return limits["upper"] == LIMIT_SWITCH_ACTIVE_STATE

    def apply_yaw_output(self, output):
        if self.yaw_motor is None:
            return 0

        if not self.yaw_allowed_by_upper_limit():
            self.yaw_motor.set_yaw_motor_speed(0)
            self.pid.reset()
            return 0

        if self.limit_reached_for_output(output):
            self.yaw_motor.set_yaw_motor_speed(0)
            self.pid.reset()
            return 0

        commanded_speed = int(output)
        self.yaw_motor.set_yaw_motor_speed(commanded_speed)
        return commanded_speed

    def stop_elevator_motion(self):
        if self.elevator_motor is not None:
            self.elevator_motor.set_elevator_motor_speed(0)
        self.elevator_current_speed = 0
        self.elevator_motion_target = None

    def home_elevator_to_lower_limit(self):
        print("Homing elevator to lower limit...")
        start_time = time.perf_counter()
        last_print_time = 0.0

        while True:
            limits = self.get_elevator_limits()
            if limits["lower"] == LIMIT_SWITCH_ACTIVE_STATE:
                self.stop_elevator_motion()
                print("Lower limit pressed. Elevator homing complete.")
                return True

            subprocess.run([sys.executable, ELEVATOR_DOWN_RUNNER], cwd=BASE_DIR, check=False)
            limits = self.get_elevator_limits()
            if limits["lower"] == LIMIT_SWITCH_ACTIVE_STATE:
                self.stop_elevator_motion()
                print("Lower limit pressed. Elevator homing complete.")
                return True

            now = time.perf_counter()
            if now - last_print_time >= LIMIT_SIGNAL_PRINT_INTERVAL:
                print(
                    "Homing elevator | "
                    f"lower={limits['lower']} upper={limits['upper']} cmd=runner_down"
                )
                last_print_time = now

            if now - start_time >= ELEVATOR_HOME_TIMEOUT:
                self.stop_elevator_motion()
                print("Elevator homing timed out before lower limit was pressed.")
                return False

            time.sleep(CONTROL_DT)

    def run_elevator_runner_until_limit(
        self,
        script_path,
        speed,
        motion_target,
        stop_limit_key,
        reached_message,
        moving_state,
    ):
        limits = self.get_elevator_limits()
        if limits[stop_limit_key] == LIMIT_SWITCH_ACTIVE_STATE:
            if self.elevator_motion_target == motion_target or self.elevator_current_speed != 0:
                print(reached_message)
            self.stop_elevator_motion()
            return False, stop_limit_key, limits

        self.elevator_motion_target = motion_target
        self.elevator_current_speed = speed
        subprocess.run([sys.executable, script_path], cwd=BASE_DIR, check=False)
        limits = self.get_elevator_limits()
        if limits[stop_limit_key] == LIMIT_SWITCH_ACTIVE_STATE:
            print(reached_message)
            self.stop_elevator_motion()
            return False, stop_limit_key, limits
        return True, moving_state, limits

    def update_elevator_for_wind(self, wind_speed):
        if abs(wind_speed) <= ELEVATOR_WIND_THRESHOLD:
            return self.run_elevator_runner_until_limit(
                script_path=ELEVATOR_UP_RUNNER,
                speed=-ELEVATOR_RUN_SPEED,
                motion_target="upper_runner",
                stop_limit_key="upper",
                reached_message="Elevator reached upper limit.",
                moving_state="moving_upper",
            )

        return self.run_elevator_runner_until_limit(
            script_path=ELEVATOR_DOWN_RUNNER,
            speed=ELEVATOR_RUN_SPEED,
            motion_target="lower_runner",
            stop_limit_key="lower",
            reached_message="Elevator reached lower limit.",
            moving_state="moving_lower",
        )

    def control_step_for_wind(self, wind_speed):
        target_angle = self.aero.get_target_angle(wind_speed)
        elevator_moving, elevator_state, elevator_limits = self.update_elevator_for_wind(
            wind_speed
        )

        upper_limit_pressed = (
            elevator_limits["upper"] == LIMIT_SWITCH_ACTIVE_STATE
            if elevator_limits is not None
            else False
        )

        if elevator_moving or not upper_limit_pressed:
            self.yaw_motor.set_yaw_motor_speed(0)
            self.pid.reset()
            encoder_state = self.read_encoder_state()
            if encoder_state is None:
                return None
            return {
                "wind_speed": float(wind_speed),
                "target_angle": target_angle,
                "current_angle": encoder_state["angle"],
                "pid_output": 0.0,
                "commanded_speed": 0,
                "raw": encoder_state["raw"],
                "pos": encoder_state["pos"],
                "elevator_state": elevator_state,
                "elevator_speed": self.elevator_current_speed,
                "elevator_lower_limit": elevator_limits["lower"],
                "elevator_upper_limit": elevator_limits["upper"],
            }

        return self.control_step_for_target_angle(
            target_angle,
            wind_speed=wind_speed,
            elevator_state=elevator_state,
            elevator_speed=self.elevator_current_speed,
            elevator_limits=elevator_limits,
        )

    def control_step_for_target_angle(
        self,
        target_angle,
        wind_speed=None,
        elevator_state=None,
        elevator_speed=0,
        elevator_limits=None,
    ):
        self.pid.set_setpoint(target_angle)

        encoder_state = self.read_encoder_state()
        if encoder_state is None:
            self.yaw_motor.set_yaw_motor_speed(0)
            return None

        current_angle = encoder_state["angle"]
        pid_output = self.pid.update(current_angle)
        commanded_speed = self.apply_yaw_output(pid_output)

        return {
            "wind_speed": None if wind_speed is None else float(wind_speed),
            "target_angle": target_angle,
            "current_angle": current_angle,
            "pid_output": pid_output,
            "commanded_speed": commanded_speed,
            "raw": encoder_state["raw"],
            "pos": encoder_state["pos"],
            "elevator_state": elevator_state,
            "elevator_speed": elevator_speed,
            "elevator_lower_limit": None if elevator_limits is None else elevator_limits["lower"],
            "elevator_upper_limit": None if elevator_limits is None else elevator_limits["upper"],
        }

    def format_state(self, state):
        raw_text = "----" if state["raw"] is None else f"{state['raw']:04X}"
        pos_text = "----" if state["pos"] is None else f"{state['pos']:5d}"
        wind_text = "  n/a" if state["wind_speed"] is None else f"{state['wind_speed']:6.2f}"
        base_text = (
            "wind={wind_text} m/s | "
            "target={target_angle:7.2f} deg | "
            "angle={current_angle:7.2f} deg | "
            "pid={pid_output:7.2f} | "
            "cmd={commanded_speed:4d} | "
            "raw=0x{raw_text} | pos={pos_text}"
        ).format(wind_text=wind_text, raw_text=raw_text, pos_text=pos_text, **state)
        elevator_state = state.get("elevator_state")
        if elevator_state is None:
            return base_text

        return (
            f"{base_text} | elev={elevator_state}({state.get('elevator_speed', 0):4d})"
            f" | lim=({state.get('elevator_lower_limit')},{state.get('elevator_upper_limit')})"
        )

    def run_manual_mode(self):
        print("=== MANUAL WIND MODE ===")
        print("Enter wind speed repeatedly. Type 'q' to return to the menu.")

        wind_holder = {"value": None}
        stop_event = threading.Event()

        def control_loop():
            print("Waiting for first wind speed input...")
            while not stop_event.is_set():
                if wind_holder["value"] is None:
                    self.yaw_motor.set_yaw_motor_speed(0)
                    time.sleep(CONTROL_DT)
                    continue

                started = time.perf_counter()
                state = self.control_step_for_wind(wind_holder["value"])
                if state is not None:
                    print(self.format_state(state))

                elapsed = time.perf_counter() - started
                time.sleep(max(0.0, CONTROL_DT - elapsed))

        thread = threading.Thread(target=control_loop, daemon=True)
        thread.start()

        try:
            while True:
                user_text = input("wind speed (m/s): ").strip()
                if not user_text:
                    continue
                if user_text.lower() in {"q", "quit", "exit"}:
                    break

                try:
                    new_speed = float(user_text)
                except ValueError:
                    print(f"Invalid wind speed: {user_text}")
                    continue

                wind_holder["value"] = new_speed
                self.pid.reset()
                print(f"Updated wind speed to {new_speed:.2f} m/s")
        finally:
            stop_event.set()
            self.yaw_motor.set_yaw_motor_speed(0)
            self.stop_elevator_motion()
            thread.join(timeout=1.0)

    def plot_sine_tracking(self, timestamps, target_angles, encoder_angles):
        import matplotlib

        if not os.environ.get("DISPLAY"):
            matplotlib.use("Agg")

        import matplotlib.pyplot as plt

        figure_path = os.path.abspath(
            f"fin_tracking_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        )

        plt.figure(figsize=(10, 6))
        plt.plot(timestamps, target_angles, label="Target Angle from Wind")
        plt.plot(timestamps, encoder_angles, label="Encoder Angle")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (deg)")
        plt.title("Fin Angle Tracking for Wind = 20*sin(pi/2*t)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(figure_path)
        plt.close()

        print(f"Tracking plot saved to {figure_path}")

    def run_sine_tracking_mode(self):
        print("=== SINE TRACKING MODE ===")
        print("wind(t) = 20 * sin(pi/2 * t)")
        print(f"Duration: {SINE_DURATION_SECONDS:.1f} s")

        timestamps = []
        target_angles = []
        encoder_angles = []

        start_time = time.perf_counter()
        last_print_time = -1.0

        try:
            while True:
                elapsed = time.perf_counter() - start_time
                if elapsed > SINE_DURATION_SECONDS:
                    break

                wind_speed = SINE_WIND_AMPLITUDE * math.sin(SINE_WIND_OMEGA * elapsed)
                step_started = time.perf_counter()
                state = self.control_step_for_wind(wind_speed)

                if state is not None:
                    timestamps.append(elapsed)
                    target_angles.append(state["target_angle"])
                    encoder_angles.append(state["current_angle"])

                    if elapsed - last_print_time >= 0.1:
                        print(self.format_state(state))
                        last_print_time = elapsed

                elapsed_step = time.perf_counter() - step_started
                time.sleep(max(0.0, CONTROL_DT - elapsed_step))
        finally:
            self.yaw_motor.set_yaw_motor_speed(0)
            self.stop_elevator_motion()

        if timestamps:
            self.plot_sine_tracking(timestamps, target_angles, encoder_angles)

    def move_to_angle(self, target_angle, timeout_seconds, log_prefix):
        start_time = time.perf_counter()
        settled_since = None

        while True:
            elapsed_total = time.perf_counter() - start_time
            if elapsed_total > timeout_seconds:
                self.yaw_motor.set_yaw_motor_speed(0)
                return None

            step_started = time.perf_counter()
            state = self.control_step_for_target_angle(target_angle)
            if state is not None:
                print(f"{log_prefix} {self.format_state(state)}")
                error = abs(target_angle - state["current_angle"])
                if error <= SETTLING_TOLERANCE_DEG:
                    if settled_since is None:
                        settled_since = time.perf_counter()
                    elif time.perf_counter() - settled_since >= SETTLING_HOLD_SECONDS:
                        self.yaw_motor.set_yaw_motor_speed(0)
                        return time.perf_counter() - start_time
                else:
                    settled_since = None

            elapsed_step = time.perf_counter() - step_started
            time.sleep(max(0.0, CONTROL_DT - elapsed_step))

    def plot_settling_results(self, target_angles, settling_times):
        import matplotlib

        if not os.environ.get("DISPLAY"):
            matplotlib.use("Agg")

        import matplotlib.pyplot as plt

        figure_path = os.path.abspath(
            f"settling_time_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        )

        plt.figure(figsize=(10, 6))
        plt.plot(target_angles, settling_times, marker="o")
        plt.xlabel("Target Angle (deg)")
        plt.ylabel("Settling Time (s)")
        plt.title("Angle vs. Settling Time")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(figure_path)
        plt.close()

        print(f"Settling-time plot saved to {figure_path}")

    def run_settling_time_mode(self):
        print("=== SETTLING TIME MODE ===")
        target_angles = list(
            range(SETTLING_TEST_START_ANGLE, SETTLING_TEST_END_ANGLE + 1, SETTLING_TEST_STEP)
        )

        initial_state = self.read_encoder_state()
        if initial_state is None:
            print("Failed to read initial encoder position.")
            return

        initial_angle = initial_state["angle"]
        print(f"Initial angle: {initial_angle:.2f} deg")

        measured_times = []

        for target_angle in target_angles:
            print(f"\nTesting target angle {target_angle:.2f} deg")
            self.pid.reset()
            self.move_to_angle(initial_angle, SETTLING_MOVE_TIMEOUT, "[return]")
            time.sleep(0.2)
            self.pid.reset()

            settling_time = self.move_to_angle(
                target_angle,
                SETTLING_MOVE_TIMEOUT,
                "[measure]",
            )

            if settling_time is None:
                print(f"Target {target_angle:.2f} deg did not settle within timeout.")
                measured_times.append(float("nan"))
            else:
                print(f"Settling time for {target_angle:.2f} deg: {settling_time:.3f} s")
                measured_times.append(settling_time)

            self.yaw_motor.set_yaw_motor_speed(0)
            time.sleep(0.3)

        self.plot_settling_results(target_angles, measured_times)

    def run_elevator_until_pause_or_lower_limit(self):
        print("=== ELEVATOR RUN MODE ===")
        print("Elevator starts at speed +100.")
        print("When lower limit is pressed, it reverses to -100.")
        print("Press Enter, Space, or q to stop.")

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        last_print_time = 0.0
        reverse_mode = False
        seen_unpressed_state = False

        try:
            tty.setcbreak(fd)
            while True:
                lower_limit = self.elevator_motor.get_lower_limit()
                upper_limit = self.elevator_motor.get_upper_limit()

                if lower_limit == 1:
                    seen_unpressed_state = True

                if not reverse_mode and seen_unpressed_state and lower_limit == 0:
                    reverse_mode = True
                    print("\nLower limit pressed. Reversing elevator to -100.")

                current_speed = -ELEVATOR_RUN_SPEED if reverse_mode else ELEVATOR_RUN_SPEED
                self.elevator_motor.set_elevator_motor_speed(current_speed)
                self.elevator_current_speed = current_speed

                now = time.perf_counter()
                if now - last_print_time >= LIMIT_SIGNAL_PRINT_INTERVAL:
                    print(
                        f"Lower limit signal: {lower_limit} | "
                        f"Upper limit signal: {upper_limit} | "
                        f"elevator cmd: {current_speed}"
                    )
                    last_print_time = now

                ready, _, _ = select.select([sys.stdin], [], [], CONTROL_DT)
                if ready:
                    char = sys.stdin.read(1)
                    if char in ("\r", "\n", " ", "q", "Q"):
                        print("\nUser exited elevator mode.")
                        break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.stop_elevator_motion()

    def choose_mode(self):
        print("\n=== MODE MENU ===")
        print("1: Encoder calibration mode")
        print("2: Manual wind speed mode")
        print("3: 15-second sinusoidal wind tracking mode")
        print("4: Angle vs. settling time mode")
        print("5: Run elevator motor until pause or lower limit")
        print("q: Quit")
        return input("Select mode: ").strip().lower()

    def run(self):
        self.initialize_hardware()
        try:
            while True:
                selection = self.choose_mode()

                if selection == "1":
                    self.calibrate_encoder_zero()
                elif selection == "2":
                    self.run_manual_mode()
                elif selection == "3":
                    self.pid.reset()
                    self.run_sine_tracking_mode()
                elif selection == "4":
                    self.pid.reset()
                    self.run_settling_time_mode()
                elif selection == "5":
                    self.run_elevator_until_pause_or_lower_limit()
                elif selection in {"q", "quit", "exit"}:
                    break
                else:
                    print(f"Invalid selection: {selection}")
        finally:
            self.shutdown()


def main():
    controller = FinController()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\nStopped by user")


if __name__ == "__main__":
    main()
