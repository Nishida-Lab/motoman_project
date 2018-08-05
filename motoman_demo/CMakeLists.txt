cmake_minimum_required(VERSION 2.8.3)
project(motoman_demo)
find_package(catkin REQUIRED
  rospy
  moveit_commander
  tf
  motoman_demo_msgs
  sensor_msgs
  )


catkin_package(
  CATKIN_DEPENDS
  )

find_package(roslaunch)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
