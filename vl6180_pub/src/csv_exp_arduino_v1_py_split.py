#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug  6 20:13:54 2019

@author: muntahi
"""

import csv
import serial
import time
import re
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import  Imu


# Open com port
serial_port = '/dev/ttyACM1'
IMU_FRAME = '/imu'

rospy.init_node('bno085_sensor')
imu_pub = rospy.Publisher('imu/data', Imu)

imu_msg = Imu()
imu_msg.header.frame_id = IMU_FRAME

array=[]
DATASPLIT={}
count = 1
print("port")
with open("datafile2.csv", "w+") as new_file:
    csv_writer = csv.writer(new_file)
    t1= time.time()
    ser = serial.Serial(serial_port, 1000000)
    while True:
        # Strip whitespace on the left and right side
        # of the line
#        line = ser.readline().strip()
        line = ser.readline()
#        print('recieved',line)        
        

        count = count+1        
        if ((len(DATASPLIT) !=11) and (count <25)):
            continue
        DATASPLIT= line.decode().split(',')
        print('recieved:  Len:', len(DATASPLIT), 'data:', line)    
        #line =re.sub(r"\s+", "", line)
        #print('wite removed',line)
        
 #       for k in xrange(len(DATASPLIT)-1):
 #            print(float((DATASPLIT [k])), " ! ")
        
        imu_msg.linear_acceleration.x = float(DATASPLIT [0])
        imu_msg.linear_acceleration.y = float(DATASPLIT [1])
        imu_msg.linear_acceleration.z = float(DATASPLIT [2])
        
        imu_msg.angular_velocity.x = float(DATASPLIT [3])
        imu_msg.angular_velocity.y = float(DATASPLIT [4])
        imu_msg.angular_velocity.z = float(DATASPLIT [5])
        
        imu_msg.orientation.x = float(DATASPLIT [6])
        imu_msg.orientation.y = float(DATASPLIT [7])
        imu_msg.orientation.z = float(DATASPLIT [8])
        imu_msg.orientation.w = float(DATASPLIT [9])
        IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

        imu_msg.header.stamp = rospy.Time.now()
        imu_pub.publish(imu_msg)

        
        
#        print(float(lin_acc_x)+0," | " ,lin_acc_y ," | ",orientation_K)
        
        print(count)
        


        
        if (count==10000):
            t2= time.time()
            ser.close()
            print(t2-t1)
            break


