import RPi.GPIO as gpio
from Motor_Control_Classes import *

# For interim demo, hardcode the robot's movement
if __name__ == "__main__":
    gpio.setmode(gpio.BOARD)
    
    # ultrasonic sensor
    echo_pin, trigger_pin = "", ""
    ultrasonic_sensor = UltrasonicSensor(echo_pin, trigger_pin)

    # wheels
    throttle = ""
    wheels = MecWheels(throttle)

    # pickup mechanism
    pickup = PickUpMechanism()
    
    # movement control #
    wheels.move_forward()
    
    # if object is detected
    for i in range(100):
        pickup.activateR() # roller
        pickup.activateC() # conveyor belt
    
    distance = ultrasonic_sensor.get_distance()
