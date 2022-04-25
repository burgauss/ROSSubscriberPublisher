#! /usr/bin/python
from __future__ import print_function
import serial
import csv
import time

import signal

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

Kp =5
Kpe = 0.5
Ki = 0.5
Kie = 0
Kd = 0.5
Iterm = 0
Ieterm = 0
last_error = 0
mpu_1 = 0.0
mpu_2 = 0.0
encoder_1 = 0
encoder_2 = 0
data = []
ref_angle = 0
ref_pos = 0
K_ppos = 0
K_dpos = 0.00
K_ipos = 0
I_pos = 0.00
last_pos_error = 0
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.flushInput()
encoder_1_first = 0
encoder_2_first = 0

i=0
while i<2000:
    if ser.inWaiting()>0:
        read_first = str(ser.readline())
        b = read_first.split(',')
        if b[0] == "b'<":
            try:
                encoder_1_first = float(b[3])
                encoder_2_first = float(b[4])
            except:
                pass
    i=i+1

with open('data.csv','w',newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['encoder_1_first','encoder_2_first','gain','mpu_1','mpu_2','error','encoder_1','encoder_2','pos_error','pos_pd','pos_speed','out','newspeed','newspeed1','newspeed2','Pterm','Iterm','Dterm'])
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

#PD controller for position
            pos_error = ref_pos - encoder_1
            P_pos = K_ppos*pos_error
            I_pos += K_ipos*pos_error
            D_pos = K_dpos*(pos_error - last_pos_error)
            last_pos_error = pos_error
            pos_pd = P_pos + D_pos + I_pos
            #print ("pos_pd :", pos_pd)
            if pos_pd > 0:
                pos_pd = pos_pd + 10
            else:
                pos_pd = pos_pd - 10

            angle_error = ref_angle - mpu_2
            if abs(angle_error) < 5:
                gain = -abs(angle_error)/5 + 1
            else:
                gain = 0

#PID controller for both
            error = angle_error
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
                newspeed = -480
            else:
                if out <= -480:
                    newspeed = 480
                else:
       	            newspeed = -out

            encoder_error = encoder_1 - encoder_2
            Ieterm += Kie*encoder_error
            pos_speed = gain*pos_pd

            if pos_speed < -100:
                pos_speed = -100
            else:
                if pos_speed > 100:
                    pos_speed = 100
                else:
                    pos_speed = pos_speed

            newspeed1 = newspeed

            newspeed2 = newspeed1 + encoder_error*Kpe + Ieterm
            if newspeed2 >= 480:
                newspeed2 = 480
            else:
                if newspeed2 <= -480:
                    newspeed2 = -480
                else:
                    newspeed2 = newspeed2

            print ("newspeed1:" ,newspeed1," newspeed2:",newspeed2)
            writer.writerow([encoder_1_first,encoder_2_first,gain,mpu_1,mpu_2,error,encoder_1,encoder_2,pos_error,pos_pd,pos_speed,out,newspeed,newspeed1,newspeed2,Pterm,Iterm,Dterm])

            try:
                motors.motor1.setSpeed(newspeed1)
                raiseIfFault()
                motors.motor2.setSpeed(newspeed2)
                raiseIfFault()

            except DriverFault as e:
                print("Driver %s fault!" % e.driver_num)


