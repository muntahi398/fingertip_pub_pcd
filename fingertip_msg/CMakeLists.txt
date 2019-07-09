cmake_minimum_required(VERSION 2.8.3)
project(fingertip_msg)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation

)
add_message_files(FILES adns.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})



