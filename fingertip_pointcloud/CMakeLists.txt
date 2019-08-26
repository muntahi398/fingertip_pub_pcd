
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
  tf
)

catkin_package(
  CATKIN_DEPENDS fingertip_msg tf 
)
#catkin_package( CATKIN_DEPENDS )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(ellipse src/ellipse.cpp)
target_link_libraries(ellipse ${catkin_LIBRARIES})

add_executable(fingercloudpub src/fingercloud_pub_mod.cpp src/pointcloud.cpp)
target_link_libraries(fingercloudpub ${catkin_LIBRARIES})

#add_library(fingercloudpub src/fingercloud_pub.cpp src/pointcloud.cpp)


