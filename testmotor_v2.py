import spidev
import time
import RPi.GPIO as GPIO
from hal_plus_motordriver import Motor
from encoder import AMT22Encoder
from aero_logic import AeroLogic

# ==========================
# DEBUG
# ==========================
def print_status(motor):
    print("---- STATUS ----")
    print("Lower limit:", motor.get_lower_limit())
    print("Upper limit:", motor.get_upper_limit())
    print("----------------\n")


# ==========================
# INIT
# ==========================
print("=== INIT ===")

elevator_motor = Motor()
elevator_motor.set_elevator_motor_speed(0)
yaw_motor = Motor()
yaw_motor.set_yaw_motor_speed(0)
encoder = AMT22Encoder(yaw_motor.pi, yaw_motor.PINS["CS"])
time.sleep(1)
print_status(elevator_motor)
print_status(yaw_motor)

try:
    '''
    # ==========================
    # ELEVATOR MOTOR TEST
    # ==========================
    print("=== ELEVATOR MOTOR TEST ===")
    for speed in [100, 200, 300]:
        print(f"Forward speed: {speed}")
        elevator_motor.set_elevator_motor_speed(speed)
        time.sleep(1)
    '''
    '''
    for speed in [-200]:
        print(f"Reverse speed: {speed}")
        elevator_motor.set_elevator_motor_speed(speed)
        for _ in range(100):
            time.sleep(0.1)
            elevator_motor.set_elevator_motor_speed(speed)
        time.sleep(10)
    '''
    
    '''
    # ==========================
    # YAW MOTOR TEST
    # ==========================
    print("=== YAW MOTOR TEST ===")
    for speed in [400]:
        print(f"Forward speed: {speed}")
        yaw_motor.set_yaw_motor_speed(speed)
        time.sleep(5)

    for speed in [-400]:
        print(f"Reverse speed: {speed}")
        yaw_motor.set_yaw_motor_speed(speed)
        time.sleep(5)
    '''
    '''
    # ==========================
    # LIMIT SWITCH TEST
    # ==========================
    print("=== LIMIT SWITCH TEST ===")
    reverse_mode = False
    current_speed = -170
    elevator_motor.set_elevator_motor_speed(current_speed)
    print(f"Elevator command set to {current_speed}")
    while True:
        Lower = elevator_motor.get_upper_limit()
        if not reverse_mode and Lower == 0:
            reverse_mode = True
            current_speed = 0
            elevator_motor.set_elevator_motor_speed(current_speed)
            print("Lower limit pressed, reverse elevator")

        print(f"Lower limit: {Lower} | elevator cmd: {current_speed}")
        time.sleep(0.1)
    print("Stop motors")
    elevator_motor.set_elevator_motor_speed(0)
    yaw_motor.set_yaw_motor_speed(0)
    time.sleep(1)
    '''
    
    # ==========================
    # YAW TEST
    # ==========================
    try:
        while True:
            raw, pos, angle = encoder.read_data()
            print(f"raw=0x{raw:04X}, pos={pos:5d}, angle={angle:7.2f} deg")
            time.sleep(0.1)
    except KeyboardInterrupt:
        encoder.spi.close()
        GPIO.cleanup()
    '''
    print("=== YAW TEST ===")
    aero= AeroLogic()
    number= len(aero.wind_speeds)
    dis = [0]*number
    for test_speed in [0, 3, 8, 15, 20, 24, 30, 35, 41] :
        for i in range(1,number-1,1):
            dis[i]= abs(test_speed-aero.wind_speeds[i])
        for i in range(1,number,1):
            if dis[i] == min(dis):
                test_speed = aero.wind_speeds[i]
        target = aero.get_target_angle(test_speed)
        position = 0
        while position != 100:
            yaw_motor.set_yaw_motor_speed(100)
            position=encoder.get_position() 
            if position == target:
                print("Position reached")
            time.sleep(1)
            break
    print("Stop yaw")
    yaw_motor.set_yaw_motor_speed(0)
    time.sleep(1)
    # ==========================
    # STEPPER TEST
    # ==========================
    print("=== ELEVATOR TEST ===")

    print("Move UP")
    motor.move_elevator(100, direction=1, delay=0.002)
    time.sleep(1)

    print("Move DOWN")
    motor.move_elevator(100, direction=0, delay=0.002)
    time.sleep(1)
    '''
except KeyboardInterrupt:
    print("\nStopped by user")


finally:
    # ==========================
    # CLEANUP
    # ==========================
    print("=== CLEANUP ===")

    yaw_motor.set_yaw_motor_speed(0)
    elevator_motor.set_elevator_motor_speed(0)

    try:
        yaw_motor.stop_all()
        elevator_motor.stop_all()
    except:
        pass
