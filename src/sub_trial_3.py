#!/usr/bin/env python

# import rospy
import serial
import numpy as np
from std_msgs.msg import Float64, Int64, Float64MultiArray
from dual_g2_hpmd_rpi import motors, MAX_SPEED
import sys
from simple_pid import PID
import time
import csv

#object for  reading serial data
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.flushInput()
print("Connected...")

#Resetting serial communication
i=0
ts=0
flag = True
final_pos = 1

class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num

def raiseIfFault():
    if motors.motor1.getFault():
        raise DriverFault(1)
    if motors.motor2.getFault():
        raise DriverFault(2)

while i<5:
    if ser.inWaiting()>0:
        ser.write(str(1).encode('utf-8'))
        ser.flushInput()
        i=i+1
ser.flushInput()
print('Serial Reset.....')

rate = 1

if __name__ == "__main__":
    with open('data.csv','w') as file:
        writer = csv.writer(file)
        writer.writerow(['v','speed'])
        for v in range(13):
            time.sleep(3)
            for i in range(10):
                # print('ROS running....')
                try:
                    motors.motor1.setSpeed(5*480/12)
                    raiseIfFault()

                except DriverFault as e:
                    print("Driver %s fault!" % e.driver_num)

                if ser.inWaiting()>0:
                    print('Receiving serial data....')
                    read_serial = str(ser.readline())
                    b = read_serial.split(',')
                    # print(b)
                    if b[0] == "<":
                        try:
                            mpu_1 = float(b[1])
                            mpu_2 = float(b[2])
                            encoder_1 = int(b[4])
                            encoder_2 = int(b[3])
                            right_rpm = float(b[7])
                            left_rpm = float(b[6])
                            trans_vel = float(b[5])
                            writer.writerow([v,right_rpm])
                            print(b)
                            print(v,right_rpm)
                        except:
                            pass

                else:
                    pass

