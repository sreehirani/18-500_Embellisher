
############## Title: Motor_Control_Classes ############
###### Authors: Hirani Sattenapalli & Ella Lee #########
################## Date: 03 | 24 | 2024 ################


## Imported Libraries ## 
import time
from gpiozero import DistanceSensor
import RPi.GPIO as gpio
import board
from adafruit_motorkit import MotorKit

class UltrasonicSensor: 
    
    ## initializes class variables 
    def __init__(self, echo_pin, trigger_pin, threshold_distance=0.3):
        self.echo_pin = echo_pin
        self.trigger_pin = trigger_pin 
        gpio.setup(echo_pin, gpio.output)
        gpio.setup(trigger_pin, gpio.output)
        self.threshold_distance = threshold_distance
        self.sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin,
                                        threshold_distance=threshold_distance)    
    
    ## returns distance to obj
    def get_distance(self): 
        return self.sensor.distance
    
    # use self.sensor.when_in_range to define controls when obj < threshold_dist
    

class MecWheels: 
    
    ## initializes class variables 
    def __init__(self, throttle): 
        self.kit = MotorKit()
        self.throttle = throttle 
        gpio.setup(throttle, gpio.output)
        ## note:
            # M1 = bottom left motor
            # M2 = top left motor 
            # M3 = top right motor 
            # M4 = bottom right motor 
        
    
    ## change speed
    def change_speed(self, new_throttle): 
        self.throttle = new_throttle
    
    ## sets throttle to move forward 
    def move_forward(self): 
        self.kit.motor1.throttle = 0.9*self.throttle
        self.kit.motor2.throttle = 0.9*self.throttle
        self.kit.motor3.throttle = 0.9*self.throttle
        self.kit.motor4.throttle = 0.9*self.throttle
    
    ## turns off throttle of all 4 motors
    def stop(self): 
        self.kit.motor1.throttle = 0
        self.kit.motor2.throttle = 0
        self.kit.motor3.throttle = 0
        self.kit.motor4.throttle = 0
    
    ## rotates
        ## left wheels move backwards & right wheels move forwards = rotateleft
        ## right wheels move backwards & left wheels move forwards = rotateright
    def rotate(self, time_val): 
        self.kit.motor1.throttle = -self.throttle
        self.kit.motor2.throttle = -self.throttle
        self.kit.motor3.throttle = self.throttle
        self.kit.motor4.throttle = self.throttle
        time.sleep(time_val)
        MecWheels.stop(self) # check if this works
    
    ## Moves robot sideways left 
        # bottom left & top right moving forward
        # top left & bottom right moving backward
    def move_left(self, time_val): 
        self.kit.motor1.throttle = self.throttle
        self.kit.motor2.throttle = -self.throttle
        self.kit.motor3.throttle = self.throttle
        self.kit.motor4.throttle = -self.throttle
        time.sleep(time_val)
        MecWheels.stop(self) 
        
    ## Moves robot sideways right
        # bottom left & top right moving backwar
        # top left & bottom right moving forward
    def move_right(self, time_val): 
        self.kit.motor1.throttle = -self.throttle
        self.kit.motor2.throttle = self.throttle
        self.kit.motor3.throttle = -self.throttle
        self.kit.motor4.throttle = self.throttle
        time.sleep(time_val)
        MecWheels.stop(self) # check if this works
        
    
    ## Moves forward while turning right
        # all four motors moving forward, left motors moving 2x as fast as right
    def right_forward(self, time_val):
        self.kit.motor1.throttle = self.throttle
        self.kit.motor2.throttle = self.throttle
        self.kit.motor3.throttle = 0.5*self.throttle
        self.kit.motor4.throttle = 0.5*self.throttle
        time.sleep(time_val)
        MecWheels.stop(self) # check if this works
        
    ## Moves forward while turning left
        # all four motors moving forward, right motors moving 2x as fast as left
    def left_forward(self, time_val): 
        self.kit.motor1.throttle = 0.5*self.throttle
        self.kit.motor2.throttle = 0.5*self.throttle
        self.kit.motor3.throttle = self.throttle
        self.kit.motor4.throttle = self.throttle
        time.sleep(time_val)
        MecWheels.stop(self)
                       
    ## opposite two motors on & off 
    #def move_diagonal(self): 

class PickUpMechanism:
    def __init__(self):
        self.kit = MotorKit(i2c = board.I2C())
        
    def activateR(self):
        self.kit.stepper1.onestep()
    
    def activateC(self):
        self.kit.stepper2.onestep()
