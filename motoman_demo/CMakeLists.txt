cmake_minimum_required(VERSION 2.8.3)
project(motoman_demo)
find_package(catkin REQUIRED
  rospy
  moveit_commander
  tf
  )


catkin_package(
  CATKIN_DEPENDS
  )
