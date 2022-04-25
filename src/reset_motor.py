#!/usr/bin/env python

import numpy as np
from std_msgs.msg import Float64, Int64
from dual_g2_hpmd_rpi import motors, MAX_SPEED
import serial
import pandas as pd
from scipy import signal
import time

#object for  reading serial data
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.flushInput()
print("Connected...")

#Resetting serial communication
i=0
ts=0
flag = True
final_pos = 1

while i<5:
    if ser.inWaiting()>0:
        ser.write(str(1).encode('utf-8'))
        ser.flushInput()
        i=i+1
ser.flushInput()
print('Serial Reset.....')
v=10
theta1 = []
theta2 = []
angvel1 = []
angvel2 = []
vol = []
ts = []
t0 = time.time()

tt = [signal.square(2 * np.pi * 5 * t) for t in range(100)]
# Define a custom exception to raise if a fault is detected.
class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num

def raiseIfFault():
    if motors.motor1.getFault():
        raise DriverFault(1)
    if motors.motor2.getFault():
        raise DriverFault(2)    
di=1
if __name__ == "__main__":
    while (time.time() - t0) <= 20:
        di = (-1)**((time.time() - t0)//2)

        if ser.inWaiting()>0:
        # print('Receiving serial data....')
            read_serial = str(ser.readline())
            b = read_serial.split(',')
            # print(b)
            if b[0] == "<":
                try:
                    # print(b)
                    # t1 = time.time()
                    motors.motor1.setSpeed(di*480*10/v)
                    # motors.motor1.setSpeed(20)
                    raiseIfFault()
                    mpu_1 = float(b[1])
                    mpu_2 = -float(b[2])
                    encoder_1 = int(b[4])
                    encoder_2 = int(b[3])
                    right_rpm = float(b[6])
                    left_rpm = float(b[7])
                    trans_vel = float(b[5])
                    lin_acc = float(b[8])
                    ang_vel = float(b[9])

                    theta1.append(encoder_1)
                    theta2.append(encoder_2)
                    angvel1.append(left_rpm)
                    angvel2.append(right_rpm)
                    vol.append(di*10)
                    print(di*480*10/v)
                    print(right_rpm)
                    ts.append(time.time() - t0)
                    # time.sleep(0.1)
                except:
                    print('pass')
                    pass

    while (time.time() - t0) <= 60:
        di = (-1)**((time.time() - t0)//1)

        if ser.inWaiting()>0:
            # print('Receiving serial data....')
            read_serial = str(ser.readline())
            b = read_serial.split(',')
            # print(b)
            if b[0] == "<":
                try:
                    # print(b)
                    # t1 = time.time()
                    motors.motor1.setSpeed(di*480*10/v)
                    # motors.motor1.setSpeed(20)
                    raiseIfFault()
                    mpu_1 = float(b[1])
                    mpu_2 = -float(b[2])
                    encoder_1 = int(b[4])
                    encoder_2 = int(b[3])
                    right_rpm = float(b[6])
                    left_rpm = float(b[7])
                    trans_vel = float(b[5])
                    lin_acc = float(b[8])
                    ang_vel = float(b[9])

                    theta1.append(encoder_1)
                    theta2.append(encoder_2)
                    angvel1.append(left_rpm)
                    angvel2.append(right_rpm)
                    vol.append(di*10)
                    print(di*480*10/v)
                    print(right_rpm)
                    ts.append(time.time() - t0)
                    # time.sleep(0.1)
                except:
                    print('pass')
                    pass

    while (time.time() - t0) <= 80:
        di = (-1)**((time.time() - t0)//2)

        if ser.inWaiting()>0:
            # print('Receiving serial data....')
            read_serial = str(ser.readline())
            b = read_serial.split(',')
            # print(b)
            if b[0] == "<":
                try:
                    # print(b)
                    # t1 = time.time()
                    motors.motor1.setSpeed(di*480*10/v)
                    # motors.motor1.setSpeed(20)
                    raiseIfFault()
                    mpu_1 = float(b[1])
                    mpu_2 = -float(b[2])
                    encoder_1 = int(b[4])
                    encoder_2 = int(b[3])
                    right_rpm = float(b[6])
                    left_rpm = float(b[7])
                    trans_vel = float(b[5])
                    lin_acc = float(b[8])
                    ang_vel = float(b[9])

                    theta1.append(encoder_1)
                    theta2.append(encoder_2)
                    angvel1.append(left_rpm)
                    angvel2.append(right_rpm)
                    vol.append(di*10)
                    print(di*480*10/v)
                    print(right_rpm)
                    ts.append(time.time() - t0)
                    # time.sleep(0.1)
                except:
                    print('pass')
                    pass

    while (time.time() - t0) <= 100:
        di = (-1)**((time.time() - t0)//1)

        if ser.inWaiting()>0:
            # print('Receiving serial data....')
            read_serial = str(ser.readline())
            b = read_serial.split(',')
            # print(b)
            if b[0] == "<":
                try:
                    # print(b)
                    # t1 = time.time()
                    motors.motor1.setSpeed(di*480*10/v)
                    # motors.motor1.setSpeed(20)
                    raiseIfFault()
                    mpu_1 = float(b[1])
                    mpu_2 = -float(b[2])
                    encoder_1 = int(b[4])
                    encoder_2 = int(b[3])
                    right_rpm = float(b[6])
                    left_rpm = float(b[7])
                    trans_vel = float(b[5])
                    lin_acc = float(b[8])
                    ang_vel = float(b[9])

                    theta1.append(encoder_1)
                    theta2.append(encoder_2)
                    angvel1.append(left_rpm)
                    angvel2.append(right_rpm)
                    vol.append(di*10)
                    print(di*480*10/v)
                    print(right_rpm)
                    ts.append(time.time() - t0)
                    # time.sleep(0.1)
                except:
                    print('pass')
                    pass

    df = pd.DataFrame(list(zip(theta1,theta2,angvel1,angvel2,vol,ts)),
                      columns =['theta1','theta2','angvel1','angvel2','vol','t'])
    df.to_csv('/home/pi/Pictures/motor_model_f.csv', index=False)