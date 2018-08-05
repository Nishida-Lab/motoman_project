cmake_minimum_required(VERSION 2.8.3)
project(motoman_description)
find_package(catkin REQUIRED
  joint_state_publisher
  robot_state_publisher
  rviz
  xacro
  roslaunch
  )

catkin_package()

# Install
foreach(dir launch meshes urdf robots robots/sia5)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
