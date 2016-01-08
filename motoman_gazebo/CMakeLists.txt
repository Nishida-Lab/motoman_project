cmake_minimum_required(VERSION 2.8.3)
project(motoman_gazebo)
find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)
roslaunch_add_file_check(test/sia5_empty_world_launch_test.xml)
roslaunch_add_file_check(test/sia5_with_table_launch_test.xml)
roslaunch_add_file_check(test/sia5_with_jamming_launch_test.xml)
roslaunch_add_file_check(test/sia5_nishida_lab_launch_test.xml)
