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
ser.flushInput()
print("Connected...")

#Resetting serial communication
i=0
while i<5:
    if ser.inWaiting()>0:
        ser.write(str(1).encode('utf-8'))
        ser.flushInput()
        i=i+1
ser.flushInput()
print('Serial Reset.....')

flag = True #flag to reset encoder count
final_pos = rospy.get_param("/pos/final_pos") #final position
ts=0 #starting time

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


class SensorNode:
    def __init__(self):
        self.sensor_publisher = rospy.Publisher('sensor_pub', Float64MultiArray, queue_size=10)
        rospy.init_node('sensor', anonymous = True)
        # rate = rospy.Rate()       # Not used, based on the communication speed from the arduino
    



if __name__ == "__main__":
    sub = SensorNode() # sensor node object
    data_to_send = Float64MultiArray()
    print('Publishing......')
    ts = time.time()    #Getting time
    while not rospy.is_shutdown():
        # Getting serial Data from Arduino
        if ser.inWaiting() > 0:
            read_serial = str(ser.readline()) #receive data as string
            b = read_serial.split(',')
            if b[0] == "<":
                try:
                    # convert data to respective data type
                    mpu_1 = float(b[1])
                    mpu_2 = -float(b[2])
                    encoder_1 = int(b[4])
                    encoder_2 = int(b[3])
                    right_rpm = float(b[6])
                    left_rpm = float(b[7])
                    trans_vel = float(b[5])
                    lin_acc = float(b[8])
                    ang_vel = float(b[9])
                    
                    # Calculating the ammount of time that has passed since the first moment
                    # the Node was created
                    tp = time.time() - ts

                    #Reference Pos generation, theoretical a ramp function
                    # if tp <= 5: #Wait for 5 seconds
                    #     ref_pos = 0
                    # if tp > 5:
                    #     if ref_pos < final_pos:
                    #         if final_pos == 0:
                    #             ref_pos = 0
                    #         else:
                    #             ref_pos = (tp - 5)*3.2*0.059

                    if tp <= 5 or ref_pos == 0:
                        ref_pos = 0
                    elif tp > 5 and ref_pos < final_pos:
                        ref_pos = (tp - 5) * 3.2* 0.059
                    else:
                        ref_pos = final_pos

                    # publish data to topics
                    # sub.imu_publisher_1.publish(mpu_1)
                    # sub.imu_publisher_2.publish(mpu_2)
                    # sub.rpm_publisher_l.publish(left_rpm)
                    # sub.rpm_publisher_r.publish(right_rpm)
                    # sub.lin_vel_publisher.publish(trans_vel)
                    # sub.enc_publisher_l.publish(encoder_1)
                    # sub.enc_publisher_r.publish(encoder_2)
                    # sub.ref_pos_publisher.publish(ref_pos)
                    # sub.acc_publisher.publish(lin_acc)
                    # sub.ang_vel_publisher.publish(ang_vel)
                    data_to_send.data = [mpu_1,mpu_2,left_rpm,right_rpm,trans_vel,
                            float(encoder_1),float(encoder_2),ref_pos,lin_acc,ang_vel]
                    sub.sensor_publisher.publish(data_to_send)
                except:
                    raise Exception("Not possible to send data")
        else:
            pass