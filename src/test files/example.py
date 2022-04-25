
from __future__ import print_function
import time

from dual_g2_hpmd_rpi import motors, MAX_SPEED

# Define a custom exception to raise if a fault is detected.
class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num

def raiseIfFault():
    if motors.motor1.getFault():
        raise DriverFault(1)
    if motors.motor2.getFault():
        raise DriverFault(2)  

s = 20

try:
    motors.setSpeeds(0, 0)
    motors.motor1.setSpeed(s)
    raiseIfFault()

    # Disable the drivers for half a second.
    motors.disable()
    time.sleep(0.5)
    motors.enable()

    motors.motor2.setSpeed(s)
    raiseIfFault()
    
    time.sleep(5)
        
except DriverFault as e:
    print("Driver %s fault!" % e.driver_num)

finally:
  # Stop the motors, even if there is an exception
  # or the user presses Ctrl+C to kill the process.
    motors.forceStop()
