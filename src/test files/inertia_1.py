
#! /usr/bin/python
from __future__ import print_function
import serial
import csv
import time
import numpy as np
import signal

#keyboard interrupt function
def keyboardInterruptHandler(signal,frame):
    print("KeyboardInterrupt (ID: {}) hass been caught. Cleaning up..".
          format(signal))
    exit(0)
signal.signal(signal.SIGINT, keyboardInterruptHandler)

#initilization of sensor readings
mpu_1 = 0.0
mpu_2 = 0.0
encoder_1 = 0
encoder_2 = 0

#object for  reading serial data
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.flushInput()

theta_dot = 0
theta_pre = 0
theta_dot_pre = 0
t_pre = 0
t = 0
theta_dot = 0
theta_ddot = 0
inertia = 0

#master loop
with open('inertia.csv','w',newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['t','mpu2','dt','theta','theta_dot','theta_ddot','inertia'])
    while True:
        #serial reading from arduino
        if ser.inWaiting()>0:
            read_serial = str(ser.readline())
            b = read_serial.split(',')
            if b[0] == "b'<":
                try:
                    mpu_1 = float(b[1])
                    mpu_2 = float(b[2])
                    encoder_1 = float(b[3])
                    encoder_2 = float(b[4])
                    t = time.time()
                except:
                    pass


# calculation of x and theta
            theta = mpu_2*3.14/180
            dt = t - t_pre

            if dt != 0:
                d_theta = theta - theta_pre
                theta_dot = d_theta/dt
                dd_theta = theta_dot - theta_dot_pre
                theta_ddot = dd_theta/dt

                inertia = 0.05718*((9.81*np.sin(theta)/theta_ddot) - 0.05718)

            t_pre = t
            theta_pre = theta
            theta_dot_pre = theta_dot

            print ('dt:',dt,'mpu:',mpu_2,'theta : ', theta, 'theta_dot: ', theta_dot, 'theta_ddot: ', theta_ddot,'inertia: ',inertia)
            writer.writerow([t,mpu_2,theta,theta_dot,theta_ddot,inertia])
