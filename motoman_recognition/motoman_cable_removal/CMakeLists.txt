cmake_minimum_required(VERSION 2.8.3)
project(motoman_cable_removal)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  tf
  tf_conversions
  eigen_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motoman_cable_removal
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  cable_point_cloud_removal_node
  src/cable_point_cloud_removal.cpp
  src/cable_point_cloud_removal_node.cpp
  )

target_link_libraries(cable_point_cloud_removal_node
  ${catkin_LIBRARIES}
)

add_executable(
  cable_removal_draw_node
  src/cable_removal_draw.cpp
  src/cable_removal_draw_node.cpp
  )

target_link_libraries(cable_removal_draw_node
  ${catkin_LIBRARIES}
)

add_executable(
  cable_point_cloud_removal_downsample_node
  src/cable_point_cloud_removal_downsample.cpp
  src/cable_point_cloud_removal_downsample_node.cpp
  )

target_link_libraries(cable_point_cloud_removal_downsample_node
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_motoman_cable_removal.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
