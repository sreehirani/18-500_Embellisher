

# Reference: 
# This Raspberry Pi code was developed by newbiely.com
# This Raspberry Pi code is made available for public use without any restriction
# https://newbiely.com/tutorials/raspberry-pi/raspberry-pi-ultrasonic-sensor-servo-motor


import RPi.GPIO as GPIO
import time

# Constants
TRIG_PIN = 2   # Raspberry Pi GPIO pin connected to TRIG pin of ultrasonic sensor
ECHO_PIN = 3   # Raspberry Pi GPIO pin connected to ECHO pin of ultrasonic sensor
SERVO_PIN = 18  # Raspberry Pi GPIO pin connected to servo motor

DISTANCE_THRESHOLD = 50  # in centimeters

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(SERVO_PIN, GPIO.OUT)


# Create PWM instance for servo
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz frequency
servo_pwm.start(0)  # Initialize servo position


def measure_distance():
    # Generate 10-microsecond pulse to TRIG pin
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Measure duration of pulse from ECHO pin
    pulse_start = time.time()
    pulse_end = time.time()
    
    while (GPIO.input(ECHO_PIN) == 0):
        print("in here")
        pulse_start = time.time()

    while (GPIO.input(ECHO_PIN) == 1):
        print("also here")
        pulse_end = time.time()

    duration = pulse_end - pulse_start

    # Calculate the distance
    distance_cm = duration * 34300 / 2

    return distance_cm
print("wroking 3")
try:
    while True:
        # Measure distance
        print("here")
        #distance_cm = measure_distance()
        print("STARTING: \n")
        #if distance_cm < DISTANCE_THRESHOLD:
            # Rotate servo motor to 90 degrees
         #   servo_pwm.ChangeDutyCycle(7.5)
        #else:
            # Rotate servo motor to 0 degrees
         
        servo_pwm.ChangeDutyCycle(2.5)
        print("Rotaated servo")
        # Print the value
        #print(f"Distance: {distance_cm:.2f} cm")

        time.sleep(0.5)

except KeyboardInterrupt:
    servo_pwm.stop()
    GPIO.cleanup()
