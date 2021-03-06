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

# flag = True #flag to reset encoder count
# final_pos = rospy.get_param("/pos/final_pos") #final position
# ts=0 #starting time

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

        self.flag = True #flag to reset encoder count
        self.final_pos = rospy.get_param("/pos/final_pos") #final position
        self.ts=0 #starting time



    def getFromArduino(self):
        if ser.inWaiting()>0:
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
                if self.flag ==True:
                    self.ts = time.time() 
                    self.flag=False
                tp = time.time() - self.ts
                self.posFunction(tp)        #update the self.ref_pos
                
                return 1
            else:
                return 0

    def posFunction(self, tp):
        if tp <= 5:
            self.ref_pos = 0
        elif self.ref_pos < self.final_pos:
            self.ref_pos = (tp - 5)*3.2*0.059
        

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


