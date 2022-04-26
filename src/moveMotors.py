#!/usr/bin/env python

import rospy
import numpy as np
from dual_g2_hpmd_rpi import motors, MAX_SPEED

class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num

def raiseIfFault():
    if motors.motor1.getFault():
        raise DriverFault(1)
    if motors.motor2.getFault():
        raise DriverFault(2)

def reset_motors():
    print('Resetting Motors')
    try:
        motors.motor1.setSpeed(0)
        raiseIfFault()
        motors.motor2.setSpeed(0)
        raiseIfFault()
        # motors.forceStop()
    except DriverFault as e:
        print("Driver %s fault!" % e.driver_num)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        try:
            motors.motor1.setSpeed(200)
            # motors.motor1.setSpeed(100)
            raiseIfFault()
            motors.motor2.setSpeed(200)
            # motors.motor2.setSpeed(0)
            raiseIfFault()

        except DriverFault as e:
                print("Driver %s fault!" % e.driver_num)
    
    rospy.on_shutdown(reset_motors())