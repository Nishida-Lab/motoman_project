#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Basic
import sys
import copy
from math import *
# ROS
import rospy
# == Action lib client ==
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
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

        # ========== Action lib client init ========== #
        self.client = actionlib.SimpleActionClient('sia5_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()


        # ======== Subscriber ======== #
        plan_sub = rospy.Subscriber('/handring_parallel_planner/handring_plan', HandringPlan, self.planCallback)

        # task queue
        self.task_q = []
        
        rospy.loginfo("(^O^) Ready (^O^)")
        

    # -------- Plannning & Execution -------- #
    def planCallback(self, plan):
        self.task_q.append(plan)

    def execute(self):
        plan = self.task_q[0]
        goal = FollowJointTrajectoryGoal(trajectory=plan.trajectory.joint_trajectory)
        self.client.send_goal(goal)
        self.client.wait_for_result()
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
    rate = rospy.Rate(5)
    handring_executor = HandringExecutor()
    while not rospy.is_shutdown():
        if handring_executor.isTask():
            handring_executor.execute()
    
    rospy.on_shutdown(handring_executor.shutdown)
