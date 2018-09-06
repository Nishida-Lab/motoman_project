cmake_minimum_required(VERSION 2.8.3)
project(motoman_calib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf_conversions
  )

find_package(VTK REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
${catkin_INCLUDE_DIRS})

## Declare a C++ executable
##add_executable(motoman_calib src/calib.cpp)
add_executable(motoman_calib src/calib_yuji.cpp)



## Specify libraries to link a library or executable target against
target_link_libraries(motoman_calib pcl_common pcl_io
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES}
)

