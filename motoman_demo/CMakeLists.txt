cmake_minimum_required(VERSION 2.8.3)
project(motoman_demo)
find_package(catkin REQUIRED
  #ar_pose
  image_proc
  motoman_demo_msgs
  moveit_commander
  tf
  sensor_msgs
  rosbridge_server
  roslaunch
  rospy
  )

catkin_package(
  CATKIN_DEPENDS
  )

# Install
foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/ar_pose_multip.launch)
  roslaunch_add_file_check(launch/ar_pose_single.launch)
  roslaunch_add_file_check(launch/handring_parallel.launch)
  roslaunch_add_file_check(launch/pepper_websocket.launch)
endif()
