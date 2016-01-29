#!/usr/bin/env python

#############################################################################################
 # Copyright (c) 2014 Daiki Maekawa and ROS JAPAN Users Group All Rights Reserved.         #
 #                                                                                         #
 # @file moveit_command_sender.py                                                          #
 # @brief This program will run you through using python interface to the move_group node. #
 # @author Daiki Maekawa                                                                   #
 # @date 2014-06-08                                                                        #
#############################################################################################

import moveit_commander
import rospy
import geometry_msgs.msg
import copy

def main():
    rospy.init_node("moveit_command_sender")

    robot = moveit_commander.RobotCommander()
    
    print "=" * 10, " Robot Groups:"
    print robot.get_group_names()

    print "=" * 10, " Printing robot state"
    print robot.get_current_state()
    print "=" * 10 

    arm = moveit_commander.MoveGroupCommander("arm")
    
    print "=" * 15, " Arm ", "=" * 15
    print "=" * 10, " Reference frame: %s" % arm.get_planning_frame()
    
    print "=" * 10, " Reference frame: %s" % arm.get_end_effector_link()
    
    arm_initial_pose = arm.get_current_pose().pose
    print "=" * 10, " Printing initial pose: "
    print arm_initial_pose
    
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2035
    target_pose.position.y = -0.5399
    target_pose.position.z = 0.0709
    target_pose.orientation.x = 0.000427
    target_pose.orientation.y = 0.000317
    target_pose.orientation.z = -0.000384
    target_pose.orientation.w = 0.999999
    arm.set_pose_target(target_pose)
    
    print "=" * 10, " plan1..."
    arm.go()
    rospy.sleep(1)

    print "=" * 10, " Planning to a joint-space goal"
    arm.clear_pose_targets()
    print "=" * 10, " Joint values: ", arm.get_current_joint_values()
    
    arm_variable_values = [
        1.4377544509919726, 
        -1.3161643133168621, 
        -2.126307271452489, 
        1.4335761224859305, 
        0.02359653211486051, 
        0.55989121526186
    ]
    
    arm.set_joint_value_target(arm_variable_values)
    
    print "=" * 10, " plan3..."
    arm.go()
    rospy.sleep(1)
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
