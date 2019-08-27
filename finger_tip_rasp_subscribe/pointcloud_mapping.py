#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 26 14:00:12 2019

@author: muntahi
"""

import numpy as np
import pcl
import random

import rospy
import sys
from std_msgs.msg import Int32,Int32MultiArray,Float32MultiArray,String
from sensor_msgs.msg import Range

import tf
import tf.msg
import time
import pcl.pcl_visualization

#cloud = pcl.PointCloud_PointXYZRGB()

points = np.zeros((2001, 4), dtype=np.float32)
points_1 = np.zeros((2001, 4), dtype=np.float32)
points_z = np.zeros((1, 4), dtype=np.float32)
def main():
    # pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud = pcl.PointCloud_PointXYZRGB()

    # Fill in the cloud data
    # cloud.width  = 15;
    # cloud.height = 10;
    # cloud.points.resize (cloud.width * cloud.height)
    # cloud.resize (np.array([15, 10], dtype=np.float))
    # points = np.zeros((10, 15, 4), dtype=np.float32)
    points = np.zeros((150, 4), dtype=np.float32)
    RAND_MAX = 1.0
    # Generate the data
    for i in range(0, 75):
        # set Point Plane
        points[i][0] = 1024 * random.random() / (RAND_MAX + 1.0)
        points[i][1] = 1024 * random.random() / (RAND_MAX + 1.0)
        points[i][2] = 0.1 * random.random() / (RAND_MAX + 1.0)
        points[i][3] = 255 << 16 | 255 << 8 | 255

    for i in range(75, 150):
        # set Point Randomize
        points[i][0] = 1024 * random.random() / (RAND_MAX + 1.0)
        points[i][1] = 1024 * random.random() / (RAND_MAX + 1.0)
        points[i][2] = 1024 * random.random() / (RAND_MAX + 1.0)
        points[i][3] = 255 << 16 | 255 << 8 | 255

    # Set a few outliers
    points[0][2] = 2.0
    points[3][2] = -2.0
    points[6][2] = 4.0

    print(cloud)

    for i in range(0, 150):
        print(points[i][0], points[i][1], points[i][2], points[i][3])

    cloud.from_array(points)

    # Create the segmentation object
    # pcl::SACSegmentation<pcl::PointXYZRGB> seg
    seg = cloud.make_segmenter()
    # Optional
    seg.set_optimize_coefficients(True)
    # Mandatory
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.1)

    # pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients)
    # pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    inliers, model = seg.segment()

    # if inliers.size
    #   return
    # end

    print(model)
    # std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    # << coefficients->values[1] << " "
    # << coefficients->values[2] << " "
    # << coefficients->values[3] << std::endl;
    #
    # std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    # for (size_t i = 0; i < inliers->indices.size (); ++i)
    # {
    #   std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
    #   << cloud.points[inliers->indices[i]].y << " "
    #   << cloud.points[inliers->indices[i]].z << std::endl;
    #   cloud.points[inliers->indices[i]].r = 255;
    #   cloud.points[inliers->indices[i]].g = 0;
    #   cloud.points[inliers->indices[i]].b = 0;
    # }
    for i in inliers:
        points[i][3] = 255 << 16 | 255 << 8 | 0

    cloud.from_array(points)

    #
    # pcl::visualization::CloudViewer viewer("Cloud Viewer");
    # viewer.showCloud(cloud.makeShared());
    # while (!viewer.wasStopped ())
    visual = pcl.pcl_visualization.CloudViewing()
    visual.ShowColorCloud(cloud)

    v = True
    while v:
        v = not(visual.WasStopped())
from tf import TransformListener

def main3():
    rospy.init_node('pcl_publish')

    cloud_0 = pcl.PointCloud_PointXYZRGB()
    cloud_1 = pcl.PointCloud_PointXYZRGB()
    visual = pcl.pcl_visualization.CloudViewing()
#    visual_1 = pcl.pcl_visualization.CloudViewing()
    global points,points_1
    i=0
    listener = tf.TransformListener()
    tfb = myNode()
    alu =tfb.locations("left_fingertip_sensor_s0", "world")
    alu_1 =tfb.locations("world","pbase_link")
    if (len(points_z[:,1])==50):
        cloud_0.from_array(points_z)
    print "total size",points_z.shape

    for i in range(0, 200):
        alu =tfb.locations( "world","pbase_link")
        points[i][0] =alu[0]
        points[i][1] =alu[1]
        points[i][2] =alu[2]
        points[i][3] =255 << 16 | 255 << 8 | 255
        i=i+1
        alu_1 =tfb.locations("world","left_fingertip_sensor_s0")
        points[i][0] =alu_1[0]
        points[i][1] =alu_1[1]
        points[i][2] =alu_1[2]
        points[i][3] =255 << 16 | 255 << 8 | 255
        time.sleep(.1)

    cloud_0.from_array(points)
#    cloud_1.from_array(points_1)
    print "cloud is "
    print(cloud_0)
#    print(cloud_1)

#    pcl.visualization.CloudViewer viewer("Cloud Viewer");
#    viewer.showCloud(cloud.makeShared());
    # while (!viewer.wasStopped ())
    visual.ShowColorCloud(cloud_0)
#    visual_1.ShowColorCloud(cloud_1)

    print alu[0], "|",alu[1]
    rospy.spin()

class myNode:

    def __init__(self, *args):
        self.tf = TransformListener()
#        rospy.Subscriber(...)
#        ...      
    def locations(self,frame1, frame2):
        self.tf.waitForTransform(frame1, frame2, rospy.Time(0), rospy.Duration(0.2))
#        print "waited"
        (trans,rot) = self.tf.lookupTransform(frame1, frame2,  rospy.Time(0))
#        print trans,rot
        return trans
#       print(frame1, frame1)
    def add_point(self,frame1, frame2):
        global points_z
        self.tf.waitForTransform(frame1, frame2, rospy.Time(0), rospy.Duration(1))
        (alu,rot) = self.tf.lookupTransform(frame1, frame2,  rospy.Time(0))
#        print trans,rot
        points[0] =alu[0]
        points[1] =alu[1]
        points[2] =alu[2]
        points[3] =255 << 16 | 255 << 8 | 255
        points_z = np.vstack([points_z, points])
    def sub_command(self,sub_command):
        print sub_command
    
        
def main2():
    rospy.init_node('pcl_publish')

    cloud_0 = pcl.PointCloud_PointXYZRGB()
    cloud_1 = pcl.PointCloud_PointXYZRGB()
    visual = pcl.pcl_visualization.CloudViewing()
#    visual_1 = pcl.pcl_visualization.CloudViewing()
    global points_z
    i=0 
    points_z=np.zeros((1, 4), dtype=np.float32)
    listener = tf.TransformListener()
    tfb = myNode()
    rospy.Subscriber("/finger0/s0", Range, tfb.add_point("left_fingertip_sensor_s0", "world"))
    sub_command = rospy.Subscriber("fingertip/command", String, tfb.sub_command())


    alu =tfb.locations("left_fingertip_sensor_s0", "world")
    alu_1 =tfb.locations("world","pbase_link")
    if (len(points_z[:,1])==50):
        
        print "total size",points_z.shape,  len(points_z[:,2])
        cloud_0.from_array(points_z)
        

    print "cloud is "
    print(cloud_0)
#    print(cloud_1)

    #
    # pcl::visualization::CloudViewer viewer("Cloud Viewer");
    # viewer.showCloud(cloud.makeShared());
    # while (!viewer.wasStopped ())
    visual.ShowColorCloud(cloud_0)
#    visual_1.ShowColorCloud(cloud_1)

    print alu[0], "|",alu[1]
    
    
    
    rospy.spin()


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main2()