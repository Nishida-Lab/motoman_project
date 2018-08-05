cmake_minimum_required(VERSION 2.8.3)
project(motoman_gripper)
find_package(catkin REQUIRED
	roslaunch
	)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

roslaunch_add_file_check(test/gripper_launch_test.xml)
