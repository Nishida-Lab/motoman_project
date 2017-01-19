cmake_minimum_required(VERSION 2.8.3)
project(motoman_viz_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  )

add_message_files(
  FILES 
  EuclideanLinkTrajectory.msg
  EuclideanLinkTrajectoryPoint.msg
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
