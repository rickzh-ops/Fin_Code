import time
from hal_plus_motordriver import Motor
# from amt22_encoder import AMT22Encoder
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

motor = Motor()
motor.set_motor_speed(0, 0)
motor.set_yaw(0)
# encoder=AMT22Encoder()
time.sleep(1)

print_status(motor)


try:
    # ==========================
    # MOTOR TEST
    # ==========================
    '''
    print("=== MOTOR TEST ===")
    
    for speed in [100, 200, 300]:
        print(f"Forward speed: {speed}")
        motor.set_motor_speed(speed, speed)
        time.sleep(1)

    for speed in [-100, -200, -300]:
        print(f"Reverse speed: {speed}")
        motor.set_motor_speed(speed, speed)
        time.sleep(1)

    print("Stop motors")
    motor.set_motor_speed(0, 0)
    time.sleep(1)
    # ==========================
    # YAW TEST
    # ==========================
    print("=== YAW TEST ===")

    for speed in [1, 2, 3]:
        print(f"CW speed: {speed}")
        motor.set_yaw(speed)
        time.sleep(10)

    for speed in [-1, -2, -3]:
        print(f"CCW speed: {speed}")
        motor.set_yaw(speed)
        time.sleep(10)
    aero= AeroLogic()
    dis=[]
    range= range(aero.wind_speeds)
    for test_speed in [0, 3, 8, 15, 20, 24, 30, 35, 41] :
        for i in range:
            dis[i]= abs(test_speed[i]-aero.wind_speeds[i])
        for i in range:
            if dis[i] == min(dis):
                test_speed = aero.wind_speeds[i]
        target = aero.get_target_angle(test_speed)
        position = 0
        while position != 100:
            motor.set_yaw(100)
            position=encoder.get_position() 
            if position == target:
                print("Position reached")
            time.sleep(1)
            break
    print("Stop yaw")
    motor.set_yaw(0)
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
    # ==========================
    # LIMIT SWITCH TEST
    # ==========================
    print("=== LIMIT SWITCH TEST ===")

    for _ in range(10):
        print(
            "LOWER:", motor.get_lower_limit(),
            "UPPER:", motor.get_upper_limit()
        )
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopped by user")


finally:
    # ==========================
    # CLEANUP
    # ==========================
    print("=== CLEANUP ===")

    motor.set_motor_speed(0, 0)
    motor.set_yaw(0)

    try:
        motor.stop_all()
    except:
        pass
