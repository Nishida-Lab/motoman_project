#!/bin/bash
echo "##############################"
echo "     Starting the System      "
echo "##############################"
rosaddress server &&
roslaunch motoman_control sia5_with_dhand_and_multi_kinect_streaming.launch&
sleep 5s;
roslaunch motoman_control sia5_real_control.launch&
sleep 5s;
echo nishidalab123 | sudo -S chmod 777 /dev/ttyUSB0 &&
rosrun dhand_driver listener.py&
rosrun motoman_viz forward_kinematics_solver&
roslaunch motoman_moveit sia5_with_dhand_moveit_planning_execution.launch&
sleep 30s
roslaunch motoman_demo handring_parallel.launch&
echo "##############################"
echo " Complete to start the System "
echo "##############################"

