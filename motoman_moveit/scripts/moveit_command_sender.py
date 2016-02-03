#!/usr/bin/env python
# -*- coding: utf-8 -*-

import moveit_commander
import rospy
import geometry_msgs.msg
# import copy

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
    target_pose.position.x = 0.5
    target_pose.position.y = 0.0
    target_pose.position.z = 0.5
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 1.0
    arm.set_pose_target(target_pose)

    print "=" * 10, " plan..."
    # plan()はMoveit!プラグインでいう「Plan」実機は動かない
    # プランニングだけ
    # plan = arm.plan()
    # go()はMoveit!プラグインでいう「Plan and Exucute」
    arm.go()
    rospy.sleep(1)

    print "=" * 10, " Planning to a joint-space goal"
    arm.clear_pose_targets()
    print "=" * 10, " Joint values: ", arm.get_current_joint_values()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "ROSInterruptException!!!!"
        pass