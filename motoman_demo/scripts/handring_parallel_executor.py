#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Moveit
import moveit_commander
import geometry_msgs.msg
from motoman_msgs.msg import DisplayTrajectory
# Octomap Service
from std_srvs.srv import Empty
# ROS
import rospy
# D-Hand
from dhand.msg import Servo_move
from std_msgs.msg import String
# Basic
import sys
import copy
from math import *



class HandringExecutor(object):

    def __init__(self):
        # ========== publisher to jamming gripper ========== #
        self.grasp_pub = rospy.Publisher('/dhand_grasp', Servo_move, queue_size=1)
        self.grasp_msg = Servo_move()
        self.grasp_msg.position = 0.0
        self.grasp_msg.speed = 20
        self.grasp_msg.acceleration = 0.2
        self.grasp_msg.current_limit = 0.5
        self.grasp_pub.publish(self.grasp_msg)

        # ========== Moveit init ========== #
        # moveit_commander init
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")

        # ======== Subscriber ======== #
        plan_sub = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.executeCallback

    # -------- Clear Octomap -------- #
    def clear_octomap(self):
        rospy.wait_for_service('clear_octomap')
        try:
            result = rospy.ServiceProxy('clear_octomap', Empty)
            result()
        except rospy.ServiceException, e:
            rospy.logwarn("Couldn't Clear Octomap")

    # -------- Plannning & Execution -------- #
    def executeCallback(self, plan):
        self.arm.execute(plan.trajectory.joint_trajectory)
        rospy.loginfo("!! Execute !!")
        self.arm.clear_pose_targets()

if __name__ == '__main__':
    rospy.init_node("handring_parallel_executor")
    handring_executor = HandringExecutor()
    rospy.spin()
