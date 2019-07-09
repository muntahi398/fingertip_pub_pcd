
cmake_minimum_required(VERSION 2.8.3)
project(fingertip_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  pcl_ros
  pcl_conversions
  fingertip_msg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fingercloudpub src/fingercloud_pub.cpp src/pointcloud.cpp)
add_executable(ellipse src/ellipse.cpp)
#add_library(fingercloudpub src/fingercloud_pub.cpp src/pointcloud.cpp)

