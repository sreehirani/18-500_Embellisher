
############## Title: Base.py ############
###### Authors: Hirani Sattenapalli & Ella Lee #########
################## Date: 03 | 24 | 2024 ################


## This file defines the robot's movement & obstacle avoidance 


### Imported Libraries ### 
import time
from gpiozero import DistanceSensor
import RPi.GPIO as GPIO
import board
from adafruit_motorkit import MotorKit



### Constants ### 
TRIG_PIN = 2 # Pi GPIO pin connected to trig pin of ultrasonic sensor
ECHO_PIN = 3 # Pi GPIO pin connected to echo pin
SERVO_PIN = 18 # Pi GPIO pin connected to small servo motor 

distance_threshold = 50 # cm 
max_throttle = 0.6 # not used, but defined here 
                # Note: max throttle - 0.6 for now 
                # DO NOT GO ABOVE MAX THROTTLE !!!!! 


### Set up ###
# set up dc motors (using first hat): #
dcKit = MotorKit(i2c=board.I2C())     # maintains control over all 4 motors 
throttle = 0.2 

# set up mini servo: 
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz frequency
servo_pwm.start(0)  # Initialize servo position

# set up sensor & mini stepper # 
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
#GPIO.setup(SERVO_PIN, GPIO.OUT)



### FUNCTIONS ### 

# Functions for Distance Measurement #

## returns distance in cm 
def measure_distance():
    # Generate 10-microsecond pulse to TRIG pin
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Measure duration of pulse from ECHO pin
    pulse_start = time.time()
    pulse_end = pulse_start

    while GPIO.input(ECHO_PIN) == 0 and time.time() - pulse_start < 0.1:
        pulse_start = time.time()

    while GPIO.input(ECHO_PIN) == 1 and time.time() - pulse_end < 0.1:
        pulse_end = time.time()

    duration = pulse_end - pulse_start

    # Calculate the distance
    distance_cm = duration * 34300 / 2

    return distance_cm


# Functions for Controlling Wheels # 

## sets throttle to move forward 
def move_forward(): 
    dcKit.motor1.throttle = throttle
    dcKit.motor2.throttle = throttle
    dcKit.motor3.throttle = throttle
    dcKit.motor4.throttle = throttle

## sets throttle to move backward 
def move_backward(): 
    dcKit.motor1.throttle = -throttle
    dcKit.motor2.throttle = -throttle
    dcKit.motor3.throttle = -throttle
    dcKit.motor4.throttle = -throttle

## turns off throttle of all 4 motors
def stop(): 
    dcKit.motor1.throttle = 0
    dcKit.motor2.throttle = 0
    dcKit.motor3.throttle = 0
    dcKit.motor4.throttle = 0

## rotates
    ## left wheels move backwards & right wheels move forwards = rotateleft
    ## right wheels move backwards & left wheels move forwards = rotateright
def rotate(time_val): 
    dcKit.motor1.throttle = -throttle
    dcKit.motor2.throttle = -throttle
    dcKit.motor3.throttle = throttle
    dcKit.motor4.throttle = throttle
    time.sleep(time_val)
    stop() # check if this works

## Moves robot sideways left 
    # bottom left & top right moving forward
    # top left & bottom right moving backward
def move_left(time_val): 
    dcKit.motor1.throttle = throttle
    dcKit.motor2.throttle = -throttle
    dcKit.motor3.throttle = throttle
    dcKit.motor4.throttle = -throttle
    time.sleep(time_val)
    stop() 
    
## Moves robot sideways right
    # bottom left & top right moving backwar
    # top left & bottom right moving forward
def move_right(time_val): 
    dcKit.motor1.throttle = -throttle
    dcKit.motor2.throttle = throttle
    dcKit.motor3.throttle = -throttle
    dcKit.motor4.throttle = throttle
    time.sleep(time_val)
    stop() # check if this works
    

## Moves forward while turning right
    # all four motors moving forward, left motors moving 2x as fast as right
def right_forward(time_val):
    dcKit.motor1.throttle = throttle
    dcKit.motor2.throttle = throttle
    dcKit.motor3.throttle = 0.5*throttle
    dcKit.motor4.throttle = 0.5*throttle
    time.sleep(time_val)
    stop() # check if this works
    
## Moves forward while turning left
    # all four motors moving forward, right motors moving 2x as fast as left
def left_forward(time_val): 
    dcKit.motor1.throttle = 0.5*throttle
    dcKit.motor2.throttle = 0.5*throttle
    dcKit.motor3.throttle = throttle
    dcKit.motor4.throttle = throttle
    time.sleep(time_val)
    stop()
                    
## opposite two motors on & off 
#def move_diagonal(self): 


def get_direction(): 
    # check in front
    servo_pwm.ChangeDutyCycle(0)
    dist = measure_distance()
    if (dist < distance_threshold): 
        return "forward"
    
    # rotate left 
    servo_pwm.ChangeDutyCycle(7.5)
    dist = measure_distance()
    if (dist < distance_threshold): 
        return "left"
    
    # rotate right
    servo_pwm.ChangeDutyCycle(1.5)
    dist = measure_distance()
    if (dist < distance_threshold): 
        return "right"

    return "none"

def move_showcase(): 
    move_forward()
    time.sleep(5)
    stop()
    move_backward()
    time.sleep(5)
    stop()
    rotate(15)
    move_left(15)
    move_right(15)
    right_forward(10)
    left_forward(10)
    rotate(15)
    stop()
    
    

### CODE ### 
obj_detected = False
try:
    while (~obj_detected):
        # Measure distance
        dist = measure_distance()
        
        if (dist > distance_threshold): 
            move_forward()
            time.sleep(5)
            stop()
        else: 
            print("Object detected, moving away")
            stop()
            dir = get_direction()
            if (dir == "forward"): move_forward()
            elif (dir == "left"): move_left(10)
            elif (dir == "right"): move_right(10)
            else: rotate(10)

            # test run conveyor belt & roller 

        if (dist < 5): 
            move_showcase()

        time.sleep(0.5)

except KeyboardInterrupt:
    servo_pwm.stop()
    stop()
    GPIO.cleanup()
