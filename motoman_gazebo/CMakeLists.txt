cmake_minimum_required(VERSION 2.8.3)
project(motoman_gazebo)
find_package(catkin REQUIRED COMPONENTS
    #actionlib_msg
    motoman_description
    motoman_point_cloud
    roslaunch
    )

catkin_package(
    CATKIN_DEPENDS
    #actionlib_msgs
    motoman_description
    motoman_point_cloud
    )

# Install
foreach(dir launch models worlds)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
#roslaunch_add_file_check(test/sia5_empty_world_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_jamming_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_jamming_and_kinectv2_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_kinectv2_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_multi_kinectv2_launch_test.xml)
#roslaunch_add_file_check(test/sia5_gazebo_nishida_lab_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_dhand_launch_test.xml)
#roslaunch_add_file_check(test/sia5_with_dhand_and_multi_kinect_launch_test.xml)
