#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Basic
import sys
import copy
from math import *
# ROS
import rospy
# Moveit
import moveit_commander
# == Messages ==
# for execution
from motoman_demo_msgs.msg import HandringPlan
# for D-Hand
from dhand.msg import Servo_move

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
        self.arm = moveit_commander.MoveGroupCommander("arm")

        # ======== Subscriber ======== #
        plan_sub = rospy.Subscriber('/handring_parallel_planner/handring_plan', HandringPlan, self.planCallback)

        # task queue
        self.task_q = []

    # -------- Plannning & Execution -------- #
    def planCallback(self, plan):
        self.task_q.append(plan)

    def execute(self):
        self.arm.execute(self.task_q.trajectory)
        self.task_q.pop(0)

    def isTask(self):
        if not self.task_q:
            return False
        return True

    def shutdown(self):
        self.arm.stop()
        rospy.logwarn("(xOx) Aborted (xOx)")
    
                                    
if __name__ == '__main__':
    rospy.init_node("handring_parallel_executor")
    rate = rospy.Rate(10)
    handring_executor = HandringExecutor()
    rospy.spin()
    while not rospy.is_shutdown():
        if handring_executor.isTask():
            handring_executor.execute()
        rospy.spin()
        rate.sleep()
    
    rospy.on_shutdown(handring_executor.shutdown)
