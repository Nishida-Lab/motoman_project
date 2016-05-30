cmake_minimum_required(VERSION 2.8.3)
project(motoman_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_msgs
  moveit_commander
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  )

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  )

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(move_group_plan_single_target
  src/move_group_plan_single_target.cpp
  )

target_link_libraries(move_group_plan_single_target
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN_LIBRARIES}
  )

find_package(roslaunch)
roslaunch_add_file_check(test/sia5_moveit_planning_execution_launch_test.xml)
roslaunch_add_file_check(test/sia5_with_jamming_moveit_planning_execution_launch_test.xml)
roslaunch_add_file_check(test/sia5_with_jamming_and_kinectv2_moveit_planning_execution_launch_test.xml)
roslaunch_add_file_check(test/sia5_with_kinectv2_moveit_planning_execution_launch_test.xml)
roslaunch_add_file_check(test/sia5_with_multi_kinectv2_moveit_planning_execution_launch_test.xml)
roslaunch_add_file_check(test/sia5_gazebo_nishida_lab_moveit_planning_execution_launch_test.xml)
