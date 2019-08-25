#!/usr/bin/python

import rospy
import sys
from std_msgs.msg import Int32,Int32MultiArray,Float32MultiArray
import tf
from visualization_msgs.msg import Marker
import math
import time
from sensor_msgs.msg import Range




import numpy as np
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
#import matplotlib
#matplotlib.use('Agg')

ax=None




rospy.init_node('range_sensor_subscribe')
listener = tf.TransformListener()
br = tf.TransformBroadcaster()

sens_measurement  = rospy.Subscriber("/range_val_array", Float32MultiArray, fingertip_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4",Range,f0_s4_callback)

marker_publ = rospy.Publisher("ellipse", Marker, queue_size=10)
ellipse = Marker()
ellipse.type = Marker.CYLINDER
#ellipse.action = Marker.add
ellipse.scale.x = 2
ellipse.scale.y = 2
ellipse.scale.z = 1
ellipse.color.a = 1.0
ellipse.color.r = 1.0
ellipse.color.g = 1.0
ellipse.color.b = 1.0



rospy.spin()
    
#  
# =============================================================================
#
# 
# =============================================================================
