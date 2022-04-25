
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

#keyboard interrupt function
def keyboardInterruptHandler(signal,frame):
    print("KeyboardInterrupt (ID: {}) hass been caught. Cleaning up..".
          format(signal))
    motors.forceStop()
    exit(0)
signal.signal(signal.SIGINT, keyboardInterruptHandler)

#PID parameters for inner loop and encoder synchronization
Kp = 18
Kpe = 0.5
Ki = 0.6
Kie = 0
Kd = 1.3
Iterm = 0
Ieterm = 0
last_error = 0

#initilization of sensor readings
mpu_1 = 0.0
mpu_2 = 0.0
encoder_1 = 0
encoder_2 = 0

#set point values
ref_angle = 0
ref_pos = 0
final_pos = 0.5
start_pos = 0

#PID paarmeters for outer loop
K_ppos = 0.012
K_dpos = 0.18
K_ipos  = 0.00001
I_pos = 0.00
last_pos_error = 0

#object for  reading serial data
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.flushInput()

#for resetting serial communication
encoder_1_first = 0
encoder_2_first = 0

#for calculation of x and theta
x_pre = 0
theta_pre = 0
dia = 0.116
t = 0
t_pre = 0
x_dot = 0
theta_dot = 0

#Resetting serial communication
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

#master loop
j = 1
with open('data.csv','w',newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['ref_pos','x','theta','x_dot','theta_dot','encoder_1_first','encoder_2_first','mpu_1','mpu_2','error','encoder_1','encoder_2','pos_error','pos_pd','out','newspeed','newspeed1','newspeed2','P_pos','I_pos','D_pos'])
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
                    print ("mpu_1:" ,mpu_1," mpu_2:",mpu_2," encoder_1:",encoder_1," encoder_2:",encoder_2)
                except:
                    pass
            # calculation of x and theta
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

#ref pos generation
            if j > 2500:
                if ref_pos < final_pos:
                    ref_pos = ref_pos + 0.0004
            else:
                ref_pos = start_pos
            j = j + 1
            ref_pos_enc = ref_pos*1600/(3.14*0.116) 

#PID controller for position
            pos_error = ref_pos_enc - encoder_1
            P_pos = K_ppos*pos_error
            I_pos += K_ipos*pos_error
            D_pos = K_dpos*(pos_error - last_pos_error)
            last_pos_error = pos_error
            pos_pd = P_pos + D_pos + I_pos

            angle_error = pos_pd - mpu_2

#PID controller for inner loop
            error = angle_error
            Pterm = Kp*error
            Iterm += Ki*error
            Dterm = Kd*(error - last_error)
            last_error = error
            out = Pterm + Iterm + Dterm

            # addition of 10 for compensating motor inertia at start
            if out > 0:
                out = out + 10
            else:
                out = out - 10

            if out >= 480:
                newspeed = -480
            else:
                if out <= -480:
                    newspeed = 480
                else:
       	            newspeed = -out
            # encoder synchronization P controller
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
            writer.writerow([ref_pos,x,theta,x_dot,theta_dot,encoder_1_first,encoder_2_first,mpu_1,mpu_2,error,encoder_1,encoder_2,pos_error,pos_pd,out,newspeed,newspeed1,newspeed2,P_pos,I_pos,D_pos])
            #motor speed command
            try:
                motors.motor1.setSpeed(newspeed1)
                raiseIfFault()
                motors.motor2.setSpeed(newspeed2)
                raiseIfFault()

            except DriverFault as e:
                print("Driver %s fault!" % e.driver_num)
