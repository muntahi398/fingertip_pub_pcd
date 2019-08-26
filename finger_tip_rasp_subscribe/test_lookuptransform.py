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
        now = rospy.Time.now()
        self.tf.waitForTransform("pbase_link", "world", now, rospy.Duration(10.0))
        print "waited"
        (trans,rot) = self.tf.lookupTransform("left_fingertip_sensor_s0", "world", now)
        print trans,rot


if __name__ == '__main__':
    rospy.init_node('my_lookuptransform')
    tfb = myNode()
    tfb.fury()
    rospy.spin()