#!/usr/bin/env python

from ast import Pass
from this import d
import rospy
import serial
import numpy as np
from std_msgs.msg import Float64, Int64, Float64MultiArray
from dual_g2_hpmd_rpi import motors, MAX_SPEED
import sys
from simple_pid import PID
import time
import csv
import pandas as pd
from datetime import datetime

##########Getting parameters#############
rate = 1
#PID parameters for tilt control
Kp = rospy.get_param("/tilt_controller/Kp")
Ki = rospy.get_param("/tilt_controller/Ki")
Kd = rospy.get_param("/tilt_controller/Kd")
#PID parameters for encoder synchronization
Kpe = rospy.get_param("/encoder_synch_controller/Kp")
Kie = rospy.get_param("/encoder_synch_controller/Ki")
Kde = rospy.get_param("/encoder_synch_controller/Kd")
#PID paarmeters for velocity control
K_pvel = rospy.get_param("/velocity_controller/Kp")
K_ivel = rospy.get_param("/velocity_controller/Ki")
K_dvel  = rospy.get_param("/velocity_controller/Kd")
#PID parameters for position control
K_ppos = rospy.get_param("/position_controller/Kp")
K_ipos = rospy.get_param("/position_controller/Ki")
K_dpos = rospy.get_param("/position_controller/Kd")

############################################
#########Motor related#####################
class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num

def raiseIfFault():
    if motors.motor1.getFault():
        raise DriverFault(1)
    if motors.motor2.getFault():
        raise DriverFault(2)
###############################################

class ControlNode:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.wheelDiameter = 0.116
        self.t_pre = 0
        self.pid_ang = PID(Kp,Ki,Kd,setpoint = 0)
        self.pid_vel = PID(K_pvel,K_ivel,K_dvel,setpoint = 0)
        self.pid_enc = PID(Kpe,Kie,Kde,setpoint = 0)
        self.pid_pos = PID(K_ppos,K_ipos,K_dpos,setpoint = 0)
        self.pid_pos.output_limits = (-10,10)
        self.speed_l = 0
        self.speed_r = 0
        self.ang_vel_lst = []
        self.position_lst =[]
        self.enc_l_lst = []
        self.enc_r_lst = []
        self.imu1_lst =[]
        self.imu2_lst =[]
        self.lin_acc_lst = []
        self.lin_vel_lst = []
        self.ref_angle_lst = []
        self.voltage_l_lst = []
        self.voltage_r_lst = []
        self.ref_pos_lst = []
        self.ref_vel_lst = []
        self.rpm_l_lst = []
        self.rpm_r_lst = []
        self.ang_acc_lst = []
        self.lin_acc_cal_lst = []
        self.ang_vel_cal_lst = []
        self.dt_lst = []
        self.u_list = []
        self.x1_list = []

        self.flag2AdjustParams = True
    
    def control_callback(self, arr):
        'The observer is called each time the subscriber receives new information'
        enc_l = arr.data[5]
        ref_pos = arr.data[7]
        lin_acc = arr.data[8]
        ang_vel = arr.data[9]
        theta = arr.data[1]
        lin_velocity = arr.data[4]
        enc_r = arr.data[6]

        t = time.time()
        dt = t - self.t_pre
        self.t_pre = t
        x = np.pi*self.wheelDiameter*enc_l/3200

        self.pid_pos.sampple_time = dt
        self.pid_pos.setpoint = ref_pos
        self.pos_pid_value = self.pid_pos(x)
    
        # Velocity controller
        self.pid_vel.sample_time = dt
        self.pid_vel.setpoint = self.pos_pid_value
        self.vel_pid_value = self.pid_vel(lin_velocity)

        # Angle controller

        self.pid_ang.sample_time = dt
        self.pid_ang.setpoint = -self.vel_pid_value
        self.ang_pid_value = self.pid_ang(theta)

        # Encoder sync

        self.pid_enc.sample_time = dt
        self.pid_enc.setpoint = enc_l
        self.enc_pid_value = self.pid_enc(enc_r)

        # Write to Motors
        self.speed_l = self.ang_pid_value
        self.speed_r = self.enc_pid_value + self.speed_l

        try:
            motors.motor1.setSpeed(self.speed_r)
            raiseIfFault()
            motors.motor2.setSpeed(self.speed_l)
            raiseIfFault()

        except:
            print("Driver %s fault!" % e.driver_num)
        finally:
            motors.forceStop()

        self.exportData()

    def subscribe(self):
        'Subscribe to the sensor_pub'
        rospy.Subscriber('/sensor_pub',Float64MultiArray, self.control_callback)

    def exportData(self):
            
            if self.flag2AdjustParams:
                ang_acc = (ang_vel - 0)/dt
                lin_acc_cal = (lin_velocity*diameter/2 - 0)/dt
                ang_vel_cal = (enc_l - 0)*2*3.14/(dt*3200)
                self.flag2AdjustParamsflag = False
            else:
                ang_acc = (ang_vel - ang_vel_lst[-1])/dt
                lin_acc_cal = (lin_velocity*diameter/2 - lin_vel_lst[-1])/dt
                ang_vel_cal = ((enc_l - enc_l_lst[-1])*2*3.14/3200)/dt
            
            self.dt_lst.append(dt)
            self.ang_vel_lst.append(ang_vel*4)
            self.ang_vel_cal_lst.append(ang_vel_cal)
            self.position_lst.append(x)
            self.enc_l_lst.append(enc_l)
            self.enc_r_lst.append(enc_r)
            self.imu1_lst.append(arr.data[0])
            self.imu2_lst.append(theta)
            self.lin_acc_lst.append(lin_acc)
            self.lin_vel_lst.append(lin_velocity)
            self.voltage_l_lst.append(self.speed_l*12/480)
            self.voltage_r_lst.append(self.speed_r*12/480)
            # voltage_r_lst.append(test_speed)
            self.ref_pos_lst.append(ref_pos)
            self.ref_vel_lst.append(self.pos_pid_value)
            self.ref_angle_lst.append(self.vel_pid_value)
            self.rpm_l_lst.append(arr.data[2])
            self.rpm_r_lst.append(arr.data[3])
            self.ang_acc_lst.append(ang_acc)
            self.lin_acc_cal_lst.append(lin_acc_cal)
            self.u_list.append(u)


    


if __name__ == '__main__':
    controlListener = ControlNode()
    while not rospy.is_shutdown():
        controlListener.subscribe()

    rospy.on_shutdown(motors.forceStop())

##########################################################
#############################Old Code#####################
##########################################################
# global A,B,C,L,K,x0,x1,y,u,Ar
# A = np.matrix([[1,0.0064,0,0],
#                [0,0.9241,0.0140,0.0001],
#                [0,0.003,1.0010,0.0067],
#                [0,0.0757,0.2989,1.0018]])

# B = np.matrix([[0],[0.0122],[0],[-0.0122]])
# C = np.matrix([[1,0,0.0887,0],
#                [0,0,1,0]])
# L = np.matrix([[1.8861,-0.16],
#               [127.7957,-10.292],
#               [0.0836,1.9584],
#               [17.6839,143.6917]])
# K = np.matrix([-174.1171,-89.7025,-893.4302,-121.7834])
# x0=np.zeros((4,1))
# x1 = np.zeros((4,1))
# y = np.zeros((2,1))
# u = np.zeros((1,1))
# Ar = (A-L*C)

"""
diameter = 0.116
# ref_pos = 0
t_pre = 0

global collect_data, flag
collect_data = True
flag = True
global ang_vel_pre
ang_vel_pre = 0
global lin_vel_pre
lin_vel_pre = 0
# global test_speed
# test_speed = 0

global dt_lst,ang_vel_lst,ang_vel_cal_lst, position_lst, enc_l_lst, enc_r_lst, imu1_lst, imu2_lst,lin_acc_cal_lst, lin_acc_lst,ang_acc_lst
global lin_vel_lst,x1_list,voltage_l_lst, u_list, voltage_r_lst, ref_pos_lst, ref_vel_lst, rpm_l_lst, rpm_r_lst
ang_vel_lst = []
position_lst =[]
enc_l_lst = []
enc_r_lst = []
imu1_lst =[]
imu2_lst =[]
lin_acc_lst = []
lin_vel_lst = []
ref_angle_lst = []
voltage_l_lst = []
voltage_r_lst = []
ref_pos_lst = []
ref_vel_lst = []
rpm_l_lst = []
rpm_r_lst = []
ang_acc_lst = []
lin_acc_cal_lst = []
ang_vel_cal_lst = []
dt_lst = []
u_list = []
x1_list = []

# class DriverFault(Exception):
#     def __init__(self, driver_num):
#         self.driver_num = driver_num

# def raiseIfFault():
#     if motors.motor1.getFault():
#         raise DriverFault(1)
#     if motors.motor2.getFault():
#         raise DriverFault(2)

class ControlNode:
    def __init__(self):
        rospy.init_node('controller',anonymous=True)
        self.r = rospy.Rate(0.05)
        self.vel_publisher = rospy.Publisher('motor_speed',Float64MultiArray,queue_size=10)
        # self.vel_publisher_r = rospy.Publisher('motor_speed_r',Float64,queue_size=10)
        self.pos_publisher = rospy.Publisher('position',Float64,queue_size=10)
        self.ref_angle_publisher = rospy.Publisher('ref_angle',Float64,queue_size=10)
        self.ref_vel_publisher = rospy.Publisher('ref_vel',Float64,queue_size=10)
        self.pid_ang = PID(Kp,Ki,Kd,setpoint = 0)
        self.pid_vel = PID(K_pvel,K_ivel,K_dvel,setpoint = 0)
        self.pid_enc = PID(Kpe,Kie,Kde,setpoint = 0)
        self.pid_pos = PID(K_ppos,K_ipos,K_dpos,setpoint = 0)
        self.pid_pos.output_limits = (-10,10)

    def control_callback(self,arr):
        #position controller
        global enc_l
        global x
        global t_pre
        global ref_pos


        ref_pos = arr.data[7]
        lin_acc = arr.data[8]
        ang_vel = arr.data[9]

        t = time.time()
        enc_l = arr.data[5]
        dt = t - t_pre
        t_pre = t
        x = np.pi*diameter*enc_l/3200



        self.pid_pos.sample_time = dt
        self.pid_pos.setpoint = ref_pos
        self.pos_pid_value = self.pid_pos(x)

        #velocity controller
        global lin_velocity
        lin_velocity = arr.data[4]

        self.pid_vel.sample_time = dt
        self.pid_vel.setpoint = self.pos_pid_value
        self.vel_pid_value = self.pid_vel(lin_velocity)




        #angle_controller
        global theta
        theta = arr.data[1]

        self.pid_ang.sample_time = dt
        self.pid_ang.setpoint = -self.vel_pid_value
        self.ang_pid_value = self.pid_ang(theta)

        # addition of 5 for compensating motor inertia at start
        if self.ang_pid_value > 0:
            out = self.ang_pid_value + 5
        else:
            out = self.ang_pid_value - 5

        if out >= 480:
            self.speed_wo_enc = 480
        else:
            if out <= -480:
                self.speed_wo_enc = -480
            else:
                self.speed_wo_enc = out

        # print('ang_pid_value:',ang_pid_value)
        #print('speed_wo_enc:',self.speed_wo_enc)
        #print('dt_theta:',dt)
        #print("theta:",theta)

        #encoder synchronisation
        global enc_r
        enc_r = arr.data[6]

        # encoder synchronization controller
        self.pid_enc.sample_time = dt
        self.pid_enc.setpoint = enc_l
        self.enc_pid_value = self.pid_enc(enc_r)

        self.speed_l = self.speed_wo_enc
        self.speed_r = self.speed_wo_enc + self.enc_pid_value
        if self.speed_r >= 480:
            self.speed_r = 480
        else:
            if self.speed_r <= -480:
                self.speed_r = -480
            else:
                self.speed_r = self.speed_r

        # global A,B,C,L,K,x0,x1,y,u
        #
        # y[0] = x
        # y[1] = theta
        # x1 = (Ar*x0) + (B*u) + (L*y)
        # u = (-K*x1)
        # x0 = x1

        try:
            motors.motor1.setSpeed(self.speed_r)
            # motors.motor1.setSpeed(100)
            raiseIfFault()
            motors.motor2.setSpeed(self.speed_l)
            # motors.motor2.setSpeed(0)
            raiseIfFault()

        except DriverFault as e:
                print("Driver %s fault!" % e.driver_num)


        speed_data = Float64MultiArray()

        speed_data.data = [self.speed_l,self.speed_r]

        self.vel_publisher.publish(speed_data)
        self.pos_publisher.publish(x)
        self.ref_angle_publisher.publish(self.vel_pid_value)
        self.ref_vel_publisher.publish(self.pos_pid_value)



        if collect_data:
            global flag
            if flag:
                ang_acc = (ang_vel - 0)/dt
                lin_acc_cal = (lin_velocity*diameter/2 - 0)/dt
                ang_vel_cal = (enc_l - 0)*2*3.14/(dt*3200)
                flag = False
            else:
                ang_acc = (ang_vel - ang_vel_lst[-1])/dt
                lin_acc_cal = (lin_velocity*diameter/2 - lin_vel_lst[-1])/dt
                ang_vel_cal = ((enc_l - enc_l_lst[-1])*2*3.14/3200)/dt
            # lin_vel_pre = lin_velocity*diameter/2
            # ang_vel_pre = ang_vel
            dt_lst.append(dt)
            ang_vel_lst.append(ang_vel*4)
            ang_vel_cal_lst.append(ang_vel_cal)
            position_lst.append(x)
            enc_l_lst.append(enc_l)
            enc_r_lst.append(enc_r)
            imu1_lst.append(arr.data[0])
            imu2_lst.append(theta)
            lin_acc_lst.append(lin_acc)
            lin_vel_lst.append(lin_velocity)
            voltage_l_lst.append(self.speed_l*12/480)
            voltage_r_lst.append(self.speed_r*12/480)
            # voltage_r_lst.append(test_speed)
            ref_pos_lst.append(ref_pos)
            ref_vel_lst.append(self.pos_pid_value)
            ref_angle_lst.append(self.vel_pid_value)
            rpm_l_lst.append(arr.data[2])
            rpm_r_lst.append(arr.data[3])
            ang_acc_lst.append(ang_acc)
            lin_acc_cal_lst.append(lin_acc_cal)
            u_list.append(u)

    def subscribe(self):

        rospy.Subscriber('/sensor_pub',Float64MultiArray,self.control_callback)

# def reset_motors():
#     print('Resetting Motors')
#     try:
#         motors.motor1.setSpeed(0)
#         raiseIfFault()
#         motors.motor2.setSpeed(0)
#         raiseIfFault()
#         # motors.forceStop()
#     except DriverFault as e:
#         print("Driver %s fault!" % e.driver_num)


if __name__ == "__main__":
    i = 0
    sub = ControlNode()
    now = datetime.now()
    nowWithFormat = now.strftime("%m%d%y%H%M")
    print('Publishing....')
    while not rospy.is_shutdown():
        sub.subscribe()
        sub.r.sleep()
        # print('Test Speed:',test_speed)
        # rospy.loginfo(test_speed)
        # if i<5:
        #     i = i +1
        # else:
        #     test_speed = test_speed + 1
        #     if test_speed >= 480:
        #         test_speed = 480
        #     i = 0

    if collect_data:
        df = pd.DataFrame(list(zip(lin_acc_cal_lst,ang_acc_lst,ang_vel_lst,position_lst,enc_l_lst,enc_r_lst,imu1_lst,imu2_lst,
                                   lin_acc_lst,lin_vel_lst,voltage_l_lst,voltage_r_lst,ref_pos_lst,
                                   ref_vel_lst,ref_angle_lst,rpm_l_lst,rpm_r_lst,ang_vel_cal_lst,dt_lst,u_list)),
                          columns =['lin_acc_cal','ang_acc','ang_vel','position','enc_l','enc_r','imu1','imu2','lin_acc',
                                    'lin_vel','voltage_l','voltage_r','ref_pos','ref_vel','ref_angle',
                                    'rpm_l','rpm_r','ang_vel_cal','dt','u'])
        df.to_csv('/home/pi/Data/weight_plate_low_'+nowWithFormat+'.csv', index=False)

    #rospy.on_shutdown(reset_motors())
    motors.forceStop()

"""