
#! /usr/bin/python
from __future__ import print_function
import serial
import csv
import time
import numpy as np
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

Kp = 18
Kpe = 0.5
Ki = 0.6
Kie = 0
Kd = 1.3
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
final_pos = 0.2

K_ppos = 0.01
K_dpos = 0.15
K_ipos  = 0.00001
K_iposs = K_ipos
I_pos = 0.00
last_pos_error = 0
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.flushInput()
encoder_1_first = 0
encoder_2_first = 0
x_pre = 0
theta_pre = 0
dia = 0.116
t = 0
t_pre = 0
x_dot = 0
theta_dot = 0
pre_x_error = 0
K_px = 45
K_ix = 4.5 
K_dx = 0
I_x = 0
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
    writer.writerow(['ref_pos','x','x_pid','theta','dx','d_theta','dt','x_dot','theta_dot','encoder_1_first','encoder_2_first','gain','mpu_1','mpu_2','error','encoder_1','encoder_2','pos_error','pos_pd','out','newspeed','newspeed1','newspeed2','P_pos','I_pos','D_pos'])
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
                    t = time.time()
                    print ("mpu_1:" ,mpu_1," mpu_2:",mpu_2," encoder_1:",encoder_1," encoder_2:",encoder_2)
                except:
                    pass
            x = 3.14*dia*encoder_1/1600
            theta = mpu_2
            dx = x - x_pre
            d_theta = theta - theta_pre
            dt = t - t_pre

            if dt != 0:
                x_dot = dx/dt
                theta_dot = d_theta/dt

            t_pre = t
            x_pre = x
            theta_pre = theta
#ref generation
            if ref_pos < final_pos:
                ref_pos = x + 0.00001

            ref_pos_enc = ref_pos*1600/(3.14*0.116) 

#PD controller for position
            pos_error = ref_pos_enc - encoder_1
            P_pos = K_ppos*pos_error
            I_pos += K_iposs*pos_error
            D_pos = K_dpos*(pos_error - last_pos_error)
            last_pos_error = pos_error
            pos_pd = P_pos + D_pos + I_pos
            print ("pos_pd :", pos_pd)

            x_error = ref_pos - x
            P_x = K_px*x_error
            I_x += K_ix*x_error
            D_x = K_dx*(x_error - pre_x_error)
            pre_x_error = x_error
            x_pid = P_x + I_x + D_x

            angle_error = pos_pd - mpu_2
            if abs(angle_error) < 2.5:
                gain = -abs(angle_error)/2.5 + 1
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
            writer.writerow([ref_pos,x,x_pid,theta,dx,d_theta,dt,x_dot,theta_dot,encoder_1_first,encoder_2_first,gain,mpu_1,mpu_2,error,encoder_1,encoder_2,pos_error,pos_pd,out,newspeed,newspeed1,newspeed2,P_pos,I_pos,D_pos])

            try:
                motors.motor1.setSpeed(newspeed1)
                raiseIfFault()
                motors.motor2.setSpeed(newspeed2)
                raiseIfFault()

            except DriverFault as e:
                print("Driver %s fault!" % e.driver_num)
