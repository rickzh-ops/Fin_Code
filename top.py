try:
    # Pi env
    import RPi.GPIO as GPIO
    import spidev
    from pololu_drv8835_rpi import motors
    IS_RPI = True
except (ImportError, RuntimeError):
    # Mac env
    IS_RPI = False
    print("Detected Mac/Non-Pi environment. Using Mock objects for development.")

    # Create an empty motor,preventing crash
    class MockMotors:
        class Motor:
            def setSpeed(self, speed): 
                print(f"[Mock] Motor speed set to: {speed}")
        motor1 = Motor()
        motor2 = Motor()
    
    motors = MockMotors()