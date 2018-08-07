cmake_minimum_required(VERSION 2.8.3)
project(motoman_moveit)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  moveit_msgs
  #moveit_commander
  moveit_core
  #moveit_ros_planning
  moveit_ros_planning_interface
  motoman_description
  pluginlib
  roscpp
  roslaunch
  )

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  )

# Build
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# Install
foreach(dir config launch scene)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

#roslaunch_add_file_check(test/sia5_moveit_planning_execution_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_jamming_moveit_planning_execution_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_jamming_and_kinectv2_moveit_planning_execution_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_kinectv2_moveit_planning_execution_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_multi_kinectv2_moveit_planning_execution_launch_test.xml)
#roslaunch_add_file_check(test/sia5_gazebo_nishida_lab_moveit_planning_execution_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_dhand_moveit_planning_execution_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_dhand_and_multi_kinect_moveit_planning_execution_launch_test.xml)
