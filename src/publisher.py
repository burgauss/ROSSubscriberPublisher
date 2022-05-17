#!/usr/bin/env python
import rospy
import serial
import numpy as np
from std_msgs.msg import Float64, Int64, Float64MultiArray
import sys
from simple_pid import PID
import time

#object for  reading serial data
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.reset_input_buffer()

#Resetting serial communication###############
i=0
while i<5:
    #Get the number of bytes in the input buffer
    if ser.inWaiting() > 0:
        ser.write(str(1).encode('utf-8'))
        ser.reset_input_buffer()
        #print("ready")
        i=i+1
ser.reset_input_buffer()
#################################################

flag = True #flag to reset encoder count
final_pos = rospy.get_param("/pos/final_pos") #final position
ts=0 #starting time

class SensorNode_:
    def __init__(self):
        rospy.init_node('sensor', anonymous=True)
        self.data_to_send = Float64MultiArray()
        # self.rate = rospy.Rate(10)
        self.sensor_publisher = rospy.Publisher('sensor_pub',Float64MultiArray,queue_size=10)
        self.mpu_1 = 0.0
        self.mpu_2 = 0.0
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.trans_vel = 0.0
        self.encoder_1 = 0.0
        self.encoder_2 = 0.0
        self.ref_pos = 0.0
        self.lin_acc = 0.0
        self.ang_vel = 0.0



    def getFromArduino(self):
        if ser.in_waiting()>0:
            read_serial = str(ser.readline()) #receive data as string
            b = read_serial.split(',')
            if b[0] == "<":
                # convert data to respective data type
                self.mpu_1 = float(b[1])
                self.mpu_2 = -float(b[2])
                self.encoder_1 = int(b[4])
                self.encoder_2 = int(b[3])
                self.right_rpm = float(b[6])
                self.left_rpm = float(b[7])
                self.trans_vel = float(b[5])
                self.lin_acc = float(b[8])
                self.ang_vel = float(b[9])
                if flag ==True:
                    ts = time.time() 
                    flag=False
                tp = time.time() - ts
                self.ref_pos = self.posFunction(tp)
                
                return 1
            else:
                return 0

    def posFunction(self, tp):
        if tp <= 5:
            ref_pos = 0
        elif ref_pos < final_pos:
            ref_pos = (tp - 5)*3.2*0.059
        
        return ref_pos


    def publish2Subscriber(self):
        self.data_to_send.data = [self.mpu_1, self.mpu_2, self.left_rpm, self.right_rpm,
                    self.trans_vel, float(self.encoder_1), float(self.encoder_2),
                    self.ref_pos, self.lin_acc, self.ang_vel]
        
        self.sensor_publisher.publish(self.data_to_send)
   


if __name__ == "__main__":
    sensor = SensorNode_()
    
    try:
        while not rospy.is_shutdown():
            val = sensor.getFromArduino()
            if val == 1:
                sensor.publish2Subscriber()
    except rospy.ROSInterruptException:
        pass


#definition of Sensor Node Object
# class SensorNode:
#     def __init__(self):
#         #initialise node
#         rospy.init_node('sensor',anonymous=True)
#         # sleep rate
#         self.r = rospy.Rate(0.05)
#         #defining various publishers
#         self.control_speed_publisher = rospy.Publisher('control_speed',Float64,queue_size=10)
#         self.imu_publisher_1 = rospy.Publisher('imu1',Float64,queue_size=10)
#         self.imu_publisher_2 = rospy.Publisher('imu2',Float64,queue_size=10)
#         self.rpm_publisher_l = rospy.Publisher('rpm_l',Float64,queue_size=10)
#         self.rpm_publisher_r = rospy.Publisher('rpm_r',Float64,queue_size=10)
#         self.acc_publisher = rospy.Publisher('lin_acc',Float64,queue_size=10)
#         self.ang_vel_publisher = rospy.Publisher('ang_vel',Float64,queue_size=10)
#         self.lin_vel_publisher = rospy.Publisher('lin_vel',Float64,queue_size=10)
#         self.enc_publisher_l = rospy.Publisher('enc_l',Int64,queue_size=10)
#         self.enc_publisher_r = rospy.Publisher('enc_r',Int64,queue_size=10)
#         self.ref_pos_publisher = rospy.Publisher('ref_pos',Float64,queue_size=10)
#         self.sensor_publisher = rospy.Publisher('sensor_pub',Float64MultiArray,queue_size=10)

# if __name__ == "__main__":
#     sub = SensorNode() # sensor node object
#     data_to_send = Float64MultiArray()
#     print('Publishing......')

#     while not rospy.is_shutdown():
#         if ser.inWaiting()>0:
#             read_serial = str(ser.readline()) #receive data as string
#             b = read_serial.split(',')
#             if b[0] == "<":
#                 try:
#                     # convert data to respective data type
#                     mpu_1 = float(b[1])
#                     mpu_2 = -float(b[2])
#                     encoder_1 = int(b[4])
#                     encoder_2 = int(b[3])
#                     right_rpm = float(b[6])
#                     left_rpm = float(b[7])
#                     trans_vel = float(b[5])
#                     lin_acc = float(b[8])
#                     ang_vel = float(b[9])
#                     if flag ==True:
#                         ts = time.time()
#                         flag=False
#                     tp = time.time() - ts

#                     #Reference Pos generation
#                     if tp <= 5: #Wait for 5 seconds
#                         ref_pos = 0
#                     if tp > 5:
#                         if ref_pos < final_pos:
#                             if final_pos == 0:
#                                 ref_pos = 0
#                             else:
#                                 ref_pos = (tp - 5)*3.2*0.059

#                     # publish data to topics
#                     sub.imu_publisher_1.publish(mpu_1)
#                     sub.imu_publisher_2.publish(mpu_2)
#                     sub.rpm_publisher_l.publish(left_rpm)
#                     sub.rpm_publisher_r.publish(right_rpm)
#                     sub.lin_vel_publisher.publish(trans_vel)
#                     sub.enc_publisher_l.publish(encoder_1)
#                     sub.enc_publisher_r.publish(encoder_2)
#                     sub.ref_pos_publisher.publish(ref_pos)
#                     sub.acc_publisher.publish(lin_acc)
#                     sub.ang_vel_publisher.publish(ang_vel)
#                     data_to_send.data = [mpu_1,mpu_2,left_rpm,right_rpm,trans_vel,float(encoder_1),float(encoder_2),ref_pos,lin_acc,ang_vel]
#                     sub.sensor_publisher.publish(data_to_send)
#                 except:
#                     pass
#         else:
#             pass