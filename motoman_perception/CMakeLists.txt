cmake_minimum_required(VERSION 2.8.3)
project(motoman_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2_sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motoman_perception
#  CATKIN_DEPENDS roscpp rospy std_msg
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

