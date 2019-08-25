cmake_minimum_required(VERSION 2.8.3)
project(iai_fingertip_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iai_kms_40_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime


)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


