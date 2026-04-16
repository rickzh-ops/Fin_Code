import os
import select
import subprocess
import sys
import termios
import tty


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DOWN_SCRIPT = os.path.join(BASE_DIR, "elevator_down_runner.py")
UP_SCRIPT = os.path.join(BASE_DIR, "elevator_up_runner.py")


def stop_process(process):
    if process is None:
        return None
    if process.poll() is not None:
        return None

    process.terminate()
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)
    return None


def start_process(process, script_path, label):
    if process is not None and process.poll() is None:
        current_script = getattr(process, "_script_path", None)
        if current_script == script_path:
            print(f"{label} runner is already active.")
            return process
        stop_process(process)

    new_process = subprocess.Popen([sys.executable, script_path], cwd=BASE_DIR)
    new_process._script_path = script_path
    print(f"Started {label} runner with PID {new_process.pid}.")
    return new_process


def main():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    active_process = None
    target_script = None
    target_label = None

    print("=== TESTMOTOR V3 ===")
    print("Down arrow: repeatedly run elevator down runner (speed +100).")
    print("Up arrow: repeatedly run elevator up runner (speed -100).")
    print("P: stop the current repeated runner.")
    print("Q: quit.")

    try:
        tty.setraw(fd)
        while True:
            if active_process is not None and active_process.poll() is not None:
                active_process = None

            if target_script is not None and active_process is None:
                active_process = start_process(active_process, target_script, target_label)

            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue

            char = sys.stdin.read(1)
            if char in ("p", "P"):
                target_script = None
                target_label = None
                active_process = stop_process(active_process)
                print("\nStopped current runner.")
                continue

            if char in ("q", "Q"):
                print("\nExiting testmotor_v3.")
                break

            if char != "\x1b":
                continue

            next_one = sys.stdin.read(1)
            next_two = sys.stdin.read(1)
            if next_one != "[":
                continue

            if next_two == "B":
                target_script = DOWN_SCRIPT
                target_label = "down"
                active_process = stop_process(active_process)
                print("\nDown runner armed for repeated 1s runs.")
            elif next_two == "A":
                target_script = UP_SCRIPT
                target_label = "up"
                active_process = stop_process(active_process)
                print("\nUp runner armed for repeated 1s runs.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        stop_process(active_process)


if __name__ == "__main__":
    main()
