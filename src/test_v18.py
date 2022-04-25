#! /usr/bin/python
from __future__ import print_function
import serial
import csv
import time
import numpy as np
import signal
from dual_g2_hpmd_rpi import motors, MAX_SPEED
from simple_pid import PID

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
    ser.close()
    exit(0)
signal.signal(signal.SIGINT, keyboardInterruptHandler)
#PID parameters for tilt control
Kp = 35
Ki = 0.5
Kd = 0.2

#PID parameters for encoder synchronization
Kpe = 0.25
Kie = 0
Kde = 0

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

#PID paarmeters for velocity control
K_pvel = 4
K_ivel = 0.02
K_dvel  = 0

#PID parameters for position control
K_ppos = 4
K_ipos = 0.2
K_dpos = 0.8

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

right_rpm = 0
left_rpm = 0
trans_vel = 0
ts = 0
tp = 0
e1=0
e2=0
i=0

#PID objects
pid_ang = PID(Kp,Ki,Kd,setpoint = 0)

pid_vel = PID(K_pvel,K_ivel,K_dvel,setpoint = 0)

pid_enc = PID(Kpe,Kie,Kde,setpoint = 0)

pid_pos = PID(K_ppos,K_ipos,K_dpos,setpoint = 0)
pid_pos.output_limits = (-0.75,0.75)

#Resetting serial communication
while i<5:
    if ser.inWaiting()>0:
        ser.write(str(1).encode('utf-8'))
        ser.flushInput()
        i=i+1

#master loop
j  = 1
ser.flushInput()
flag = True
with open('data.csv','w') as file:
    writer = csv.writer(file)
    writer.writerow(['t','pos_pid','vel_pid','r_rpm','l_rpm','v','ref_pos','x','theta','x_dot','theta_dot','mpu_1','mpu_2','encoder_1','encoder_2','out','newspeed','newspeed1','newspeed2'])
    while True:
        #serial reading from arduino
        if ser.inWaiting()>0:
            read_serial = str(ser.readline())
            b = read_serial.split(',')
            if b[0] == "b'<":
                try:
                    mpu_1 = float(b[1])
                    mpu_2 = float(b[2])
                    encoder_1 = float(b[4])
                    encoder_2 = float(b[3])
                    right_rpm = float(b[7])
                    left_rpm = float(b[6])
                    trans_vel = float(b[5])
                    t = time.time()
                    print ("v" ,trans_vel," mpu_2:",mpu_2," encoder_1:",encoder_1," encoder_2:",encoder_2,'r_rpm: ', right_rpm, 'l_rpm:', left_rpm)
                    if flag == True:
                        e1 = encoder_1
                        e2 = encoder_2
                        ts = time.time()
                        flag = False
                except:
                    pass
            # calculation of x and theta
            x = 3.14*dia*encoder_1/3200
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
            if ts != 0:
                tp = time.time() - ts
            if tp <= 5:
                ref_pos = 0
            if tp > 5:
                if ref_pos < final_pos:
                    #ref_pos = (tp - 5)*1.2*0.059 
                    ref_pos = 0

#PID controller for position and velocity
            pid_pos.sample_time = dt
            pid_pos.setpoint = ref_pos
            pos_pid = pid_pos(x)

            pid_vel.sample_time = dt
            pid_vel.setpoint = pos_pid
            vel_pid = pid_vel(trans_vel)


            pid_ang.sample_time = dt
            pid_ang.setpoint = vel_pid
            out = pid_ang(mpu_2)

            # addition of 5 for compensating motor inertia at start
            if out > 0:
                out = out + 5
            else:
                out = out - 5

            if out >= 480:
                newspeed = -480
            else:
                if out <= -480:
                    newspeed = 480
                else:
       	            newspeed = -out

            # encoder synchronization controller
            pid_enc.sample_time = dt
            pid_enc.setpoint = encoder_1
            enc_pid = pid_enc(encoder_2)

            newspeed1 = newspeed
            newspeed2 = newspeed1 + enc_pid
            if newspeed2 >= 480:
                newspeed2 = 480
            else:
                if newspeed2 <= -480:
                    newspeed2 = -480
                else:
                    newspeed2 = newspeed2

            print ("newspeed1:" ,newspeed1," newspeed2:",newspeed2)
            writer.writerow([tp,pos_pid,vel_pid,right_rpm,left_rpm,trans_vel,ref_pos,x,theta,x_dot,theta_dot,mpu_1,mpu_2,encoder_1,encoder_2,out,newspeed,newspeed1,newspeed2])
            #motor speed command
            try:
                motors.motor1.setSpeed(newspeed1)
                raiseIfFault()
                motors.motor2.setSpeed(newspeed2)
                raiseIfFault()

            except DriverFault as e:
                print("Driver %s fault!" % e.driver_num)
