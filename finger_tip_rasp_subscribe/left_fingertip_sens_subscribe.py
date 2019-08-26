#!/usr/bin/python

import rospy
import sys
from std_msgs.msg import Int32,Int32MultiArray,Float32MultiArray
import tf
from visualization_msgs.msg import Marker
import math
import time
from sensor_msgs.msg import Range

import tf.msg
import geometry_msgs.msg
import math


import numpy as np
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
#import matplotlib
#matplotlib.use('Agg')

ax=None
p_x=0
p_y=0
theta=0
radius=1
change = 0

def fingertip_callback(msg):
    print "Here is lsit of sens_val", msg.data
    resp =[]
    resp = msg.data
    print resp  ##its float data now
    
    (trans_s0,rot) = listener.lookupTransform("f0_s0_meas", "base_link",  
                    rospy.Time(0))
    (trans_s1,rot) = listener.lookupTransform("f0_s1_meas", "base_link",  
                    rospy.Time(0))
    (trans_s2,rot) = listener.lookupTransform("f0_s2_meas", "base_link",  
                    rospy.Time(0))
    (trans_s3,rot) = listener.lookupTransform("f0_s3_meas", "base_link",  
                    rospy.Time(0))
    (trans_s4,rot) = listener.lookupTransform("f0_s4_meas", "base_link",  
                    rospy.Time(0))
    (trans_s5,rot) = listener.lookupTransform("f0_s5_meas", "base_link",  
                    rospy.Time(0))
    data =np.array([trans_s0,trans_s1,trans_s2,trans_s3,trans_s4,trans_s5])
    print "s9_tf_trans", data                       
 
    if order == 1:
    # best-fit linear plane
        A = np.c_[data[:,0], data[:,1], np.ones(data.shape[0])]
        C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])    # coefficients
    
    # evaluate it on grid
        Z = C[0]*X + C[1]*Y + C[2]
    
    # or expressed using matrix/vector product
    #Z = np.dot(np.c_[XX, YY, np.ones(XX.shape)], C).reshape(X.shape)

    elif order == 2:
    # best-fit quadratic curve
        A = np.c_[np.ones(data.shape[0]), data[:,:2], np.prod(data[:,:2], axis=1), data[:,:2]**2]
        C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])
    
    # evaluate it on a grid
        Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX*YY, XX**2, YY**2], C).reshape(X.shape)
        
    print "plotting"
    global ax
    ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
    ax.scatter(data[:,0], data[:,1], data[:,2], c='r', s=50)
    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_zlabel('Z')
    ax.axis('equal')
    ax.axis('tight')
    print "plotting"


    plt.draw()
##    plt.pause(.02)
    ax.cla()
#    plt.show()
#    fig.canvas.draw()

    

    br.sendTransform((0, 0, resp[0]),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      rospy.Time.now(),
                      "fingerdist_s0_meas",
                      "left_fingertip_sensor_s0")

    
    ## marker
    lns = [resp[0],resp[1],resp[4],resp[5]]
    ellipse = Marker()

    min_val = min(lns)
    min_val_index = lns.index(min_val)
    print (lns)
    print (min_val)
    print (min_val_index)
    del_x = 0.0382 -0.0028
    del_y = 0.0566 - 0.0307
    del_hx = resp[0]-resp[1]
    del_hy = resp[0]-resp[2] 
    roll = math.atan2((resp[0]-resp[1]) , del_x)
    print(roll, ' roll in radian')

    pitch = math.atan2((resp[0]-resp[2]) , del_y)
    print(pitch, ' pitch in radian')

    ellipse.header.frame_id = "fingertip_fingertip_normal"

    ellipse.pose.position.z = -0.0191
    ellipse.pose.position.y = 0.0435
    ellipse.pose.position.x = min_val+ math.sqrt (del_hx**2 +del_hy**2)
#    ellipse.pose.orientation = (roll,pitch,0,1)
    ellipse.pose.orientation.x = 0;
    ellipse.pose.orientation.y = pitch ; #- math.pi
    ellipse.pose.orientation.z = roll;
    ellipse.pose.orientation.w = 1;
    ellipse.scale.x = 0.02
    ellipse.scale.y = 0.02
    ellipse.scale.z = 0.01
    ellipse.color.a = 1.0
    ellipse.color.r = 1.0
    ellipse.color.g = 0.0
    ellipse.color.b = 0.0
    ellipse.lifetime.secs =1.0
    marker_publ.publish(ellipse)
 
class DynamicTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage)

        change = 0.0
        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "world" 
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "pbase_link"
            t.transform.translation.x = 0.50 * math.sin(change)
            t.transform.translation.y = 0.50 * math.cos(change)
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfm = tf.msg.tfMessage([t])
            self.pub_tf.publish(tfm)

            change += 0.1
            
def circular_broadcaster():
    rospy.sleep(0.1)

    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "world" 
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "pbase_link"
    t.transform.translation.x = 0.40 * math.sin(change)
    t.transform.translation.y = 0.40 * math.cos(change)
    t.transform.translation.z = 0.0

    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    tfm = tf.msg.tfMessage([t])
    self.pub_tf.publish(tfm)

    change += 0.1
            
def array_facing_marker():    
    (trans_s0,rot) = listener.lookupTransform("f0_s0_meas", "base_link",  
                    rospy.Time(0))
    (trans_s1,rot) = listener.lookupTransform("f0_s1_meas", "base_link",  
                    rospy.Time(0))
    (trans_s2,rot) = listener.lookupTransform("f0_s2_meas", "base_link",  
                    rospy.Time(0))
    (trans_s3,rot) = listener.lookupTransform("f0_s3_meas", "base_link",  
                    rospy.Time(0))
    (trans_s4,rot) = listener.lookupTransform("f0_s4_meas", "base_link",  
                    rospy.Time(0))
    (trans_s5,rot) = listener.lookupTransform("f0_s5_meas", "base_link",  
                    rospy.Time(0))    

def f0_s0_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s0_meas",
                      "left_fingertip_sensor_s0")
def f0_s1_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s1_meas",
                      "left_fingertip_sensor_s1")        
def f0_s2_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s2_meas",
                      "left_fingertip_sensor_s2")
def f0_s3_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s3_meas",
                      "left_fingertip_sensor_s3")
def f0_s4_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s4_meas",
                      "left_fingertip_sensor_s4")
def f0_s5_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s5_meas",
                      "left_fingertip_sensor_s5")
def f0_s6_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s6_meas",
                      "left_fingertip_sensor_s6")
def f0_s7_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s7_meas",
                      "left_fingertip_sensor_s7")
def f0_s8_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s8_meas",
                      "left_fingertip_sensor_s8")
def f0_s9_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s9_meas",
                      "left_fingertip_sensor_s9")

def f1_s0_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f1_s0_meas",
                      "right_fingertip_sensor_s0")
def f1_s1_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f1_s1_meas",
                      "right_fingertip_sensor_s1")        
def f1_s2_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f1_s2_meas",
                      "right_fingertip_sensor_s2")
def f1_s3_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f1_s3_meas",
                      "right_fingertip_sensor_s3")
def f1_s4_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f1_s4_meas",
                      "right_fingertip_sensor_s4")
def f1_s5_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f1_s5_meas",
                      "right_fingertip_sensor_s5")
def f1_s6_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f1_s6_meas",
                      "right_fingertip_sensor_s6")
def f1_s7_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f1_s7_meas",
                      "right_fingertip_sensor_s7")
def f1_s8_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f1_s8_meas",
                      "right_fingertip_sensor_s8")
def f1_s9_callback(msg):
    if(msg.range<0.250):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f1_s9_meas",
                      "right_fingertip_sensor_s9")                   
def fingertip_sensor_callback(msg):
    if(msg.range<255):
        br.sendTransform((0, 0, msg.range),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      msg.header.stamp,
                      "f0_s0_meas",
                      "left_fingertip_sensor_s0")  
        
def base_move(p_x,p_y):
    if(p_x<2):
        global change
#        transform.setOrigin(tf.Vector3(2.0*sin(change), 2.0*cos(change), 0.0) )
#        transform.setRotation( tf.Quaternion(change, 0, 0) );
        change += 0.1;
#        br.sendTransform(transform, rospy.Time(0), "pbase_link", "world");
        br.sendTransform(tf.Vector3(2.0*sin(change), 2.0*cos(change), 0.0),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time(0), "pbase_link", "world");

#        br.sendTransform((p_x, p_y, 0),
#                      tf.transformations.quaternion_from_euler(0, 0, 0),
#                      msg.header.stamp,
#                      "pbase_link",
#                      "base_link")         
                      
callback_finger_sens = lambda x: callback(x,dict_1,dict_2)
                                          

rospy.init_node('range_sensor_subscribe')
listener = tf.TransformListener()
br = tf.TransformBroadcaster()
#transform=tf.transformations()


#plot surface
X,Y = np.meshgrid(np.arange(-0.10, 0.10, 0.01), np.arange(-0.10, 0.10, 0.1))
XX = X.flatten()
YY = Y.flatten()
order = 2    # 1: linear, 2: quadratic
fig = plt.figure()
#plt.show()
#ax = fig.gca(projection='3d')
ax = fig.add_subplot(111, projection='3d')




#ax.plot_surface(100, 100,100,rstride=1, cstride=1, alpha=0.2)

sens_measurement  = rospy.Subscriber("/range_val_array", Float32MultiArray, fingertip_callback)
f0_s0_measurement  = rospy.Subscriber("/finger0/s0", Range, f0_s0_callback,)
f0_s1_measurement  = rospy.Subscriber("/finger0/s1", Range, f0_s1_callback)
f0_s2_measurement  = rospy.Subscriber("/finger0/s2", Range, f0_s2_callback)
f0_s3_measurement  = rospy.Subscriber("/finger0/s3", Range, f0_s3_callback)
f0_s4_measurement  = rospy.Subscriber("/finger0/s4", Range, f0_s4_callback)
f0_s5_measurement  = rospy.Subscriber("/finger0/s5", Range, f0_s5_callback)
f0_s6_measurement  = rospy.Subscriber("/finger0/s6", Range, f0_s6_callback)
f0_s7_measurement  = rospy.Subscriber("/finger0/s7", Range, f0_s7_callback)
f0_s8_measurement  = rospy.Subscriber("/finger0/s8", Range, f0_s8_callback)
f0_s9_measurement  = rospy.Subscriber("/finger0/s9", Range, f0_s9_callback)
#
f1_s0_measurement  = rospy.Subscriber("/finger1/s0", Range, f1_s0_callback)
f1_s1_measurement  = rospy.Subscriber("/finger1/s1", Range, f1_s1_callback)
f1_s2_measurement  = rospy.Subscriber("/finger1/s2", Range, f1_s2_callback)
f1_s3_measurement  = rospy.Subscriber("/finger1/s3", Range, f1_s3_callback)
f1_s4_measurement  = rospy.Subscriber("/finger1/s4", Range, f1_s4_callback)
f1_s5_measurement  = rospy.Subscriber("/finger1/s5", Range, f1_s5_callback)
f1_s6_measurement  = rospy.Subscriber("/finger1/s6", Range, f1_s6_callback)
f1_s7_measurement  = rospy.Subscriber("/finger1/s7", Range, f1_s7_callback)
f1_s8_measurement  = rospy.Subscriber("/finger1/s8", Range, f1_s8_callback)
f1_s9_measurement  = rospy.Subscriber("/finger1/s9", Range, f1_s9_callback)

#for p_x in range(0,2,.05):
p_x=p_x+0.05
#base_move(p_x,p_y)
#tfb = DynamicTFBroadcaster()
if p_x>1:
    p_x=0

#marker_publ = rospy.Publisher("ellipse", Marker, queue_size=10)
#ellipse = Marker()
#ellipse.type = Marker.CYLINDER
##ellipse.action = Marker.add
#ellipse.scale.x = 2
#ellipse.scale.y = 2
#ellipse.scale.z = 1
#ellipse.color.a = 1.0
#ellipse.color.r = 1.0
#ellipse.color.g = 1.0
#ellipse.color.b = 1.0

(trans,rot)=listener.lookupTransform('fingertip_base_link','left_fingertip_sensor_s8',rospy.Time(0))
print trans

rospy.spin()
    
#  
# =============================================================================
#
# 
# =============================================================================
