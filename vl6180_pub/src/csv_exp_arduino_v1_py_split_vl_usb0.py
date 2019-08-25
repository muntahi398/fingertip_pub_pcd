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
from sensor_msgs.msg import Range


# Open com port
serial_port = '/dev/ttyUSB0'
IMU_FRAME = '/imu'

rospy.init_node('vl_sensor_0')
imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
s0 = rospy.Publisher('range', Range, queue_size=10)
pub_range0=rospy.Publisher( "/finger0/s0", Range, queue_size=10);
pub_range1=rospy.Publisher( "/finger0/s1", Range, queue_size=10);
pub_range2=rospy.Publisher( "/finger0/s2", Range, queue_size=10);
pub_range3=rospy.Publisher( "/finger0/s3", Range, queue_size=10);
pub_range4=rospy.Publisher( "/finger0/s4", Range, queue_size=10);
pub_range5=rospy.Publisher( "/finger0/s5", Range, queue_size=10);
pub_range6=rospy.Publisher( "/finger0/s6", Range, queue_size=10);
pub_range7=rospy.Publisher( "/finger0/s7", Range, queue_size=10);
pub_range8=rospy.Publisher( "/finger0/s8", Range, queue_size=10);
pub_range9=rospy.Publisher( "/finger0/s9", Range, queue_size=10);
r = Range()

r.header.frame_id = "/vl_sensor"
r.radiation_type = 0
r.field_of_view = 0.1
r.min_range = 1 #min_range
r.max_range = 20#max_range


#imu_msg = Imu()
#imu_msg.header.frame_id = IMU_FRAME

array=[]
DATASPLIT={}
count = 1
#print("port")
with open("datafile2.csv", "w+") as new_file:
    csv_writer = csv.writer(new_file)
    t1= time.time()
    ser = serial.Serial(serial_port, 1000000)
    print("port opened")
    while True:
        # Strip whitespace on the left and right side
        # of the line
#        line = ser.readline().strip()
        line = ser.readline()
#        print('recieved',line)    
        r.header.stamp = rospy.Time.now()    
        
        print("cond")
        count = count+1        
        if ((len(DATASPLIT) !=2) and (count <25)):
            continue
        DATASPLIT= line.decode().split(',')
        print('recieved:  Len:', len(DATASPLIT), 'data:', line,'split_0',DATASPLIT [0],'split_1',DATASPLIT [1])    
        #line =re.sub(r"\s+", "", line)
        #print('wite removed',line)
        
 #       for k in xrange(len(DATASPLIT)-1):
 #            print(float((DATASPLIT [k])), " ! ")
 
        r.range = float(DATASPLIT [1])/1000  ## convert to mm

        if ((DATASPLIT [0])=="s0"):
            pub_range0.publish(r)
        elif ((DATASPLIT [0])=='s1'):
            pub_range1.publish(r)
        elif ((DATASPLIT [0])=='s2'):
            pub_range2.publish(r)
        elif ((DATASPLIT [0])=='s3'):
            pub_range3.publish(r)            
        elif ((DATASPLIT [0])=='s4'):
            pub_range4.publish(r)            
        elif ((DATASPLIT [0])=='s5'):
            pub_range5.publish(r)            
        elif ((DATASPLIT [0])=='s6'):
            pub_range6.publish(r)            
        elif ((DATASPLIT [0])=='s7'):
            pub_range7.publish(r)            
        elif ((DATASPLIT [0])=='s8'):
            pub_range8.publish(r)            
        elif ((DATASPLIT [0])=='s9'):
            pub_range9.publish(r)            
#        imu_msg.linear_acceleration.x = float(DATASPLIT [0])
#        imu_msg.linear_acceleration.y = float(DATASPLIT [1])
#        imu_msg.linear_acceleration.z = float(DATASPLIT [2])
        

#        IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

#        imu_msg.header.stamp = rospy.Time.now()
#        imu_pub.publish(imu_msg)

        
        
#        print(float(lin_acc_x)+0," | " ,lin_acc_y ," | ",orientation_K)
        
        print(count)
        


        
#        if (count==10000):
#            t2= time.time()
#            ser.close()
#            print(t2-t1)
#            break

#if __name__ == '__main__':
#    try:
#        talker()
#    except rospy.ROSInterruptException: pass
