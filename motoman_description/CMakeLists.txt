cmake_minimum_required(VERSION 2.8.3)
project(motoman_description)
find_package(catkin REQUIRED)

foreach(dir launch meshes urdf robots robots/sia5)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

find_package(roslaunch)
roslaunch_add_file_check(test/sia5/sia5_launch_test.xml)
roslaunch_add_file_check(test/sia5/sia5_with_jamming_launch_test.xml)
roslaunch_add_file_check(test/sia5/sia5_with_jamming_and_kinectv2_launch_test.xml)
roslaunch_add_file_check(test/sia5/sia5_with_kinectv2_launch_test.xml)
roslaunch_add_file_check(test/sia5/sia5_gazebo_nishida_lab_launch_test.xml)
roslaunch_add_file_check(test/sia5/sia5_with_dhand_launch_test.xml)
roslaunch_add_file_check(test/sia5/sia5_with_dhand_and_multi_kinect_launch_test.xml)
