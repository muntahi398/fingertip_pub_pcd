#!/usr/bin/env python

import rospy
from math import*
#from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
Fs=8000
f=500


rospy.init_node('laser_scan_publisher')

scan_pub_s0 = rospy.Publisher('scan_s0', Range, queue_size=50)
scan_pub_s1 = rospy.Publisher('scan_s1', Range, queue_size=50)

num_readings = 100
laser_frequency = 40
a=[0]*num_readings
count = 0
r = rospy.Rate(1.0)
scan = Range()
scan_s0=Range()
scan_s1=Range()

scan.header.frame_id = "/vl_sensor"
scan.radiation_type = 0
scan.field_of_view = 0.1
scan.min_range = 1 #min_range
scan.max_range = 20#max_range
scan_s0=scan    
scan_s1=scan    
while not rospy.is_shutdown():
#    scan = LaserScan()
    
    scan_s0.header.stamp = rospy.Time.now()  
    scan_s1.header.stamp = rospy.Time.now()  
    scan_s0.range=sin(2*pi*f*count/Fs)
    scan_s1.range=sin(4*pi*f*count/Fs)
   #     scan.ranges.append(1.0 * a[i])  # fake data
   #     scan.intensities.append(1)  # fake data

    scan_pub_s0.publish(scan_s0)
    scan_pub_s1.publish(scan_s1)
    count += 1
    r.sleep()
