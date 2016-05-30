cmake_minimum_required(VERSION 2.8.3)
project(motoman_point_cloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  cmake_modules
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  cmake_modules
  )

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros pcl_msgs tf tf_conversions
  )

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_COMMON_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(
  kinect_downsampler_node
  src/kinect_downsampler.cpp
  src/kinect_downsampler_node.cpp
  )

target_link_libraries(kinect_downsampler_node
  ${catkin_LIBRARIES}
)

add_executable(point_cloud_merger src/point_cloud_merger.cpp)
target_link_libraries(point_cloud_merger ${catkin_LIBRARIES})