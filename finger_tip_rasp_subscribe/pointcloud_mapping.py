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
from std_msgs.msg import Int32,Int32MultiArray,Float32MultiArray
import tf
import tf.msg

import pcl.pcl_visualization

#cloud = pcl.PointCloud_PointXYZRGB()

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

def main2():
    rospy.init_node('pcl_publish')
    cloud_0 = pcl.PointCloud_PointXYZRGB()
    listener = tf.TransformListener()
    now = rospy.Time.now()
    while not rospy.is_shutdown():
            (trans,rot)=listener.lookupTransform('pbase_link','world',rospy.Time(0))
            (trans,rot)=listener.lookupTransformFull('fingertip_base_link','left_fingertip_sensor_s8',rospy.Time(0))
            print trans
    t = tf.Transformer(True, rospy.Duration(10.0))
    t.getFrameStrings()
    i=0
    t.lookupTransform('world', 'left_fingertip_sensor_s0', rospy.Time(0))
#    (trans_s0,rot_s0) =listener.lookupTransform("leftfingertip_frame_in", "left_fingertip_sensor_s8", rospy.Time(0))
#    print trans_s0
    points = np.zeros((150, 4), dtype=np.float32)
    rospy.spin()


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main2()