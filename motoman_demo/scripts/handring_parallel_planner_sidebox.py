#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Basic
import sys
import copy
from math import *
# ROS
import rospy
import rosparam
# Moveit
import moveit_commander
# TF
import tf2_ros
import tf
# == Messages ==
# for Cartesian path
from geometry_msgs.msg import Pose
# for Pepper message
from std_msgs.msg import String
# for Start state
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import RobotTrajectory
# for Planned path
from motoman_demo_msgs.msg import HandringPlan
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
# for object bounding box
from motoman_viz_msgs.msg import BoundingBoxArray
from motoman_viz_msgs.msg import BoundingBox
# == Service ==
# for cleaning the Octomap
from std_srvs.srv import Empty

class HandringPlanner(object):

    def __init__(self):
        # ========= Subscriber ======== #
        # self.speech_sub_topic = rospy.get_param('~speech')
        self.speech_sub = rospy.Subscriber('/speech', String, self.speechCallback)

        # ========== Moveit init ========== #
        # moveit_commander init
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.target_pose = Pose()
        # Set the planning time
        self.arm.set_planner_id('RRTConnectkConfigDefault')
        self.planning_limitation_time = 5.0
        self.arm.set_planning_time(self.planning_limitation_time)

        # ========== TF ======== #
        # TF Listner #
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)

        # ========== Handring Plan publisher ======== #
        self.hp_pub = rospy.Publisher('~handring_plan', HandringPlan, queue_size=6)
        
        # ========= Box Poses ======== #
        self.box_pose = [{}, {}]
        # Box 0 pose
        self.box_pose[0]["joint_s"] = -0.6041074967384338
        self.box_pose[0]["joint_l"] = 0.576959013938903
        self.box_pose[0]["joint_e"] = 1.396013617515564
        self.box_pose[0]["joint_u"] = -0.6198193430900574
        self.box_pose[0]["joint_r"] = 0.0 
        self.box_pose[0]["joint_b"] = -0.4 
        self.box_pose[0]["joint_t"] = 0.0
        # Box 1 pose
        self.box_pose[1]["joint_s"] = 0.6041074967384338
        self.box_pose[1]["joint_l"] = 0.5769590139389038
        self.box_pose[1]["joint_e"] = -1.396013617515564
        self.box_pose[1]["joint_u"] = -0.6198193430900574
        self.box_pose[1]["joint_r"] = 0.0 
        self.box_pose[1]["joint_b"] = -0.4 
        self.box_pose[1]["joint_t"] = 0.0

        # ======== Object Info ======== #
        self.diff = rospy.get_param('~diff_from_object', 0.03)     # diff from offset to grasp the object
        self.offset = rospy.get_param('~offset', 0.45)    # offset from top of the object
        self.box_sub = rospy.Subscriber('/clustering_result', BoundingBoxArray, self.bbArrayCallback)
        self.initial_box_num = 0

        rospy.loginfo("HPP Initialized")
        
    # -------- Bounding Box Array call back -------- #
    def bbArrayCallback(self, message):
        self.initial_box_num = len(message.boxes)

    # -------- Get TF -------- #
    def get_object_tf_data(self, num):
        tf_time = rospy.Time(0)
        target = "object_" + str(num)
        get_tf_flg = False
        while not get_tf_flg :
            try :
                trans = self.tf_buffer.lookup_transform('world', target, tf_time, rospy.Duration(10))
                get_tf_flg = True

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
