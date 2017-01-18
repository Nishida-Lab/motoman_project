cmake_minimum_required(VERSION 2.8.3)
project(motoman_viz)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_move_group
  moveit_msgs
  motoman_viz_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  )

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
add_definitions(-std=c++11)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

## Declare a C++ executable
add_executable(forward_kinematics src/forward_kinematics.cpp)

add_dependencies(forward_kinematics forward_kinematics_generate_messages_cpp)
add_dependencies(forward_kinematics ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(forward_kinematics
  ${catkin_LIBRARIES}
  )
