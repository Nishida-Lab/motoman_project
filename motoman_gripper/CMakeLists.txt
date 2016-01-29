cmake_minimum_required(VERSION 2.8.3)
project(motoman_gripper)
find_package(catkin REQUIRED)

find_package(roslaunch)
roslaunch_add_file_check(test/gripper_launch_test.xml)
