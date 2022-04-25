#!/usr/bin/env python

# import rospy
# import numpy as np
# from std_msgs.msg import Float64, Int64, Float64MultiArray
# from dual_g2_hpmd_rpi import motors, MAX_SPEED
#
#
# # Define a custom exception to raise if a fault is detected.
# class DriverFault(Exception):
#     def __init__(self, driver_num):
#         self.driver_num = driver_num
#
# def raiseIfFault():
#     if motors.motor1.getFault():
#         raise DriverFault(1)
#     if motors.motor2.getFault():
#         raise DriverFault(2)
#
# class MotorNode:
#     def __init__(self):
#         rospy.init_node('actuator',anonymous=True)
#         self.r = rospy.Rate(0.001)
#
#     # def motor_callback(self,speedl):
#     #     self.speed_l = speedl.data
#     #     print('speed_l',self.speed_l)
#
#     def motor_callback(self,speed):
#         speed_l = speed.data[0]
#         speed_r = speed.data[1]
#         print('speed_l',speed_l)
#         print('speed_r',speed_r)
#         try:
#             motors.motor1.setSpeed(speed_l)
#             raiseIfFault()
#             motors.motor2.setSpeed(speed_r)
#             raiseIfFault()
#
#         except DriverFault as e:
#                 print("Driver %s fault!" % e.driver_num)
#
#
#     def subscribe(self):
#         rospy.Subscriber('/motor_speed',Float64MultiArray,self.motor_callback)
#         # rospy.Subscriber('/motor_speed_r',Float64,self.motor_r_callback)
#
#
# if __name__ == "__main__":
#     sub = MotorNode()
#     while not rospy.is_shutdown()
#         sub.subscribe()
#         sub.r.sleep()

import rosbag
from geometry_msgs.msg import Point
import pandas as pd

# The bag file should be in the same directory as your terminal
bag = rosbag.Bag('/home/pi/.ros/2021-01-25-13-50-26.bag')
topics = ['/ang_vel','/position']
# /clock
# /enc_l
# /enc_r
# /imu1
# /imu2
# /lin_acc
# /lin_vel
# /motor_speed
# /position
# /ref_angle
# /ref_pos
# /ref_vel
# /rosout
# /rosout_agg
# /rpm_l
# /rpm_r
# /sensor_pub

# column_names = ['/ang_vel']
df = pd.DataFrame(columns=topics)
for topic in topics:
    for topic, msg, t in bag.read_messages(topics=topic):
        data = msg.data
        df = df.append(
            {topic : data},
            ignore_index=True
        )

df.to_csv('out.csv')