import RPi.GPIO as gpio
from Motor_Control_Classes import *

# For interim demo, hardcode the robot's movement
# if __name__ == "__main__":

# wheels

kit = MotorKit(i2c=board.I2C())

for i in range(100):
    kit.stepper2.onestep()
    time.sleep(0.1)

    #wheels = MecWheels(throttle)
    
    # movement control #
    #@ wheels.move_forward()

 
    # ultrasonic sensor
   # echo_pin, trigger_pin = "", ""
    # ultrasonic_sensor = UltrasonicSensor(echo_pin, trigger_pin)

    
    # pickup mechanism
    #pickup = PickUpMechanism()
    
    
    
    # if object is detected
    #for i in range(100):
        #pickup.activateR() # roller
        #pickup.activateC() # conveyor belt
    
    #distance = ultrasonic_sensor.get_distance()
