import RPi.GPIO as gpio
import board
import time
from adafruit_motorkit import MotorKit

class PickUpMechanism:
    def __init__(self):
        self.kit = MotorKit(i2c = board.I2C())
        
    def activateR(self):
        self.kit.stepper1.onestep()
    
    def activateC(self):
        self.kit.stepper2.onestep()

# testing- reference: https://www.youtube.com/watch?v=ea6tSppgZlY
pickup = PickUpMechanism()
for i in range(100):
    pickup.activateR() # roller
    pickup.activateC() # conveyor belt
    time.sleep(0.01)


    