#! /usr/bin/python
from __future__ import print_function
import serial
import csv
import time

import signal
import  matplotlib.pyplot as plt
from itertools import count
from matplotlib.animation import FuncAnimation

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

def keyboardInterruptHandler(signal,frame):
    print("KeyboardInterrupt (ID: {}) hass been caught. Cleaning up..".
          format(signal))
    motors.forceStop()
    exit(0)

signal.signal(signal.SIGINT, keyboardInterruptHandler)

plt.style.use('fivethirtyeight')

x_values = []
y_values = []

index = count()


Kp = 30
Kpe = 0.5
Ki = 0.5
Kie = 0
Kd = 0.01
Iterm = 0
Ieterm = 0
last_error = 0
mpu_1 = 0.0
mpu_2 = 0.0
encoder_1 = 0
encoder_2 = 0
data = []

ser = serial.Serial('/dev/ttyACM0', 115200)
ser.flushInput()

def animate(i):
    x_values.append(next(index))
    y_values.append(mpu_1)
    plt.cla()
    plt.plot(x_values, y_values)

ani = FuncAnimation(plt.gcf(), animate, 1000)

plt.tight_layout()
plt.show()

while True:
    if ser.inWaiting()>0:
        read_serial = str(ser.readline())
        b = read_serial.split(',')
        if b[0] == "b'<":
            try:
                mpu_1 = float(b[1])
                mpu_2 = float(b[2])
                encoder_1 = float(b[3])
                encoder_2 = float(b[4])
                print ("mpu_1:" ,mpu_1," mpu_2:",mpu_2," encoder_1:",encoder_1," encoder_2:",encoder_2)
            except:
                pass
        error = mpu_2 - 0
        Pterm = Kp*error
        Iterm += Ki*error
        Dterm = Kd*(error - last_error)
        last_error = error
        out = Pterm + Iterm + Dterm
        if out > 0:
            out = out + 10
        else:
            out = out - 10

        print (out)
        if out >= 480:
            newspeed = 480
        else:
            if out <= -480:
                newspeed = -480
            else:
       	        newspeed = out

        encoder_error = encoder_1 - encoder_2
        Ieterm += Kie*encoder_error
        newspeed1 = newspeed
        newspeed2 = newspeed + encoder_error*Kpe + Ieterm
        if newspeed2 >= 480:
            newspeed2 = 480
        else:
            if newspeed2 <= -480:
                newspeed2 = -480
            else:
                newspeed2 = newspeed2
                
        print ("newspeed1:" ,newspeed1," newspeed2:",newspeed2)
                
        try:
            motors.setSpeeds(0, 0)
            motors.motor1.setSpeed(newspeed1)
            raiseIfFault()
            motors.motor2.setSpeed(newspeed2)
            raiseIfFault()

        except DriverFault as e:
            print("Driver %s fault!" % e.driver_num)

        

