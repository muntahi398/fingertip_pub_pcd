#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 26 19:29:30 2019

@author: muntahi
"""

import rospy
from tf import TransformListener

class myNode:

    def __init__(self, *args):
        self.tf = TransformListener()
#        rospy.Subscriber(...)
#        ...

    def some_method(self):
        if self.tf.frameExists("left_fingertip_sensor_s0") and self.tf.frameExists("world"):
            t = self.tf.getLatestCommonTime("left_fingertip_sensor_s0", "world")
            position, quaternion = self.tf.lookupTransform("left_fingertip_sensor_s0", "world", t)
            print position, quaternion
        else :
            print "not found frame"
            
    def fury(self):
#        now = rospy.Time.now()
        self.tf.waitForTransform("pbase_link", "world", rospy.Time(0), rospy.Duration(10.0))
        print "waited"
        (trans,rot) = self.tf.lookupTransform("left_fingertip_sensor_s0", "world",  rospy.Time(0))
        print trans,rot
        return trans
        
    def locations(self,frame1, frame2):
        self.tf.waitForTransform(frame1, frame2, rospy.Time(0), rospy.Duration(10.0))
        print "waited"
        (trans,rot) = self.tf.lookupTransform(frame1, frame2,  rospy.Time(0))
        print trans,rot
#       print(frame1, frame1)

sens_list =  ["left_fingertip_sensor_s0", "left_fingertip_sensor_s1", "left_fingertip_sensor_s2", "left_fingertip_sensor_s3", "left_fingertip_sensor_s4", "left_fingertip_sensor_s5", "left_fingertip_sensor_s6", "left_fingertip_sensor_s7", "left_fingertip_sensor_s8", "left_fingertip_sensor_s9" ]
if __name__ == '__main__':
    rospy.init_node('my_lookuptransform')
    tfb = myNode()
    alu = tfb.fury()
    tfb.locations(sens_list[2], "world")
    for x in range(len(sens_list)):
        tfb.locations(sens_list[x], "world")
    print alu[0], "|",alu[1]
    rospy.spin()