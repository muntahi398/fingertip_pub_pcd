cmake_minimum_required(VERSION 2.8.3)
project(fingertip_laser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  pcl_ros
  pcl_conversions
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(fingeraux source/Buffer.cpp source/AdnsRegisters.cpp)

add_executable(fingerpub source/finger_pub.cpp source/AdnsReader.cpp source/FtdiUsbDevice.cpp 
					  source/FrameReader.cpp source/FtdiContext.cpp source/AdnsMotionBurstReport.cpp)
target_link_libraries(fingerpub ftdi fingeraux)

#rosbuild_init()

include_directories("/usr/include")

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} ftdi)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_library(fingeraux source/Buffer.cpp source/AdnsRegisters.cpp)

#rosbuild_add_executable(fingerpub source/finger_pub.cpp source/AdnsReader.cpp source/FtdiUsbDevice.cpp 
#					  source/FrameReader.cpp source/FtdiContext.cpp source/#AdnsMotionBurstReport.cpp)
#target_link_libraries(fingerpub ftdi fingeraux)
