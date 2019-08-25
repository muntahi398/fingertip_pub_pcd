#!/usr/bin/python

import rospy
import sys
from std_msgs.msg import Int32,Int32MultiArray,Float32MultiArray
import tf
from visualization_msgs.msg import Marker
import math
import time





import numpy as np
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
#import matplotlib
#matplotlib.use('Agg')

ax=None

def fingertip_callback(msg):
    print "Here is lsit of sens_val", msg.data
    resp =[]
    resp = msg.data
    print resp  ##its float data now
    
    (trans_s0,rot) = listener.lookupTransform("fingerdist_s0_meas", "base_link",  
                    rospy.Time(0))
    (trans_s1,rot) = listener.lookupTransform("fingerdist_s1_meas", "base_link",  
                    rospy.Time(0))
    (trans_s2,rot) = listener.lookupTransform("fingerdist_s2_meas", "base_link",  
                    rospy.Time(0))
    (trans_s3,rot) = listener.lookupTransform("fingerdist_s3_meas", "base_link",  
                    rospy.Time(0))
    (trans_s4,rot) = listener.lookupTransform("fingerdist_s4_meas", "base_link",  
                    rospy.Time(0))
    (trans_s5,rot) = listener.lookupTransform("fingerdist_s5_meas", "base_link",  
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

    
#    br.sendTransform((0, 0, resp[9]),
#                      tf.transformations.quaternion_from_euler(0, 0, 0),
#                      rospy.Time.now(),
#                      "fingerdist_s9_meas",
#                      "fingertip_fingertip_sensor_s9")
    br.sendTransform((0, 0, resp[0]),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      rospy.Time.now(),
                      "fingerdist_s0_meas",
                      "fingertip_fingertip_sensor_s0")
#     br.sendTransform((0, 0, resp[7]),
#                      tf.transformations.quaternion_from_euler(0, 0, 0),
#                      rospy.Time.now(),
#                      "fingerdist_s7_meas",
#                      "fingertip_fingertip_sensor_s7")
#     br.sendTransform((0, 0, resp[5]),
#                      tf.transformations.quaternion_from_euler(0, 0, 0),
#                      rospy.Time.now(),
#                      "fingerdist_s5_meas",
#                      "fingertip_fingertip_sensor_s5")
#     br.sendTransform((0, 0, resp[4]),
#                      tf.transformations.quaternion_from_euler(0, 0, 0),
#                      rospy.Time.now(),
#                      "fingerdist_s4_meas",
#                      "fingertip_fingertip_sensor_s4")
#     br.sendTransform((0, 0, resp[3]),
#                      tf.transformations.quaternion_from_euler(0, 0, 0),
#                      rospy.Time.now(),
#                      "fingerdist_s3_meas",
#                      "fingertip_fingertip_sensor_s3")
#     br.sendTransform((0, 0, resp[2]),
#                      tf.transformations.quaternion_from_euler(0, 0, 0),
#                      rospy.Time.now(),
#                      "fingerdist_s2_meas",
#                      "fingertip_fingertip_sensor_s2")
#     br.sendTransform((0, 0, resp[1]),
#                      tf.transformations.quaternion_from_euler(0, 0, 0),
#                      rospy.Time.now(),
#                      "fingerdist_s1_meas",
#                      "fingertip_fingertip_sensor_s1")
#     br.sendTransform((0, 0, resp[8]),
#                      tf.transformations.quaternion_from_euler(0, 0, 0),
#                      rospy.Time.now(),
#                      "fingerdist_s8_meas",
#                      "fingertip_fingertip_sensor_s8")
    
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


rospy.init_node('range_sensor_subscribe')
listener = tf.TransformListener()
br = tf.TransformBroadcaster()


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
