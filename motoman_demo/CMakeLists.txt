cmake_minimum_required(VERSION 2.8.3)
project(motoman_demo)
find_package(catkin REQUIRED
  rospy
  moveit_msgs
  moveit_commander
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  )


catkin_package(
  CATKIN_DEPENDS other_catkin_pkg
  )
