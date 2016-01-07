cmake_minimum_required(VERSION 2.8.3)
project(motoman_description)
find_package(catkin REQUIRED)

find_package(roslaunch)
roslaunch_add_file_check(
  test/sia5/test_sia5.xml
  test/sia5/test_sia5_with_table.xml
  test/sia5/test_sia5_with_jamming.xml
  test/sia5/test_sia5_nishida_lab.xml
  )
