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
    def get_tf_data(self, num):
        tf_time = rospy.Time(0)
        target = "object_" + str(num)
        get_tf_flg = False
        while not get_tf_flg :
            try :
                trans = self.tf_buffer.lookup_transform('world', target, tf_time, rospy.Duration(10))
                get_tf_flg = True

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
                continue
        return trans 

    # -------- Clear Octomap -------- #
    def clear_octomap(self):
        rospy.wait_for_service('clear_octomap')
        try:
            result = rospy.ServiceProxy('clear_octomap', Empty)
            result()
        except rospy.ServiceException, e:
            rospy.logwarn("Couldn't Clear Octomap")
            
    # -------- Plannning & Execution -------- #
    def get_plan(self, trans, z_offset, start_state, grasp):
        # Set argument start state
        moveit_start_state = RobotState()
        moveit_start_state.joint_state = start_state
        self.arm.set_start_state(moveit_start_state)
        # Calculate goal pose
        self.target_pose.position.x = trans.transform.translation.x
        self.target_pose.position.y = trans.transform.translation.y
        self.target_pose.position.z = trans.transform.translation.z + z_offset
        q = (trans.transform.rotation.x,
             trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w)
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(q)
        pitch += pi/2.0
        tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.target_pose.orientation.x = tar_q[0]
        self.target_pose.orientation.y = tar_q[1]
        self.target_pose.orientation.z = tar_q[2]
        self.target_pose.orientation.w = tar_q[3]
        self.arm.set_pose_target(self.target_pose)
        # plan
        plan = RobotTrajectory()
        counter = 0
        while len(plan.joint_trajectory.points) == 0 :
            plan = self.arm.plan()
            counter+=1
            self.arm.set_planning_time(self.planning_limitation_time+counter*5.0)
            if counter > 1 :
                return (False, start_state)
        self.arm.set_planning_time(self.planning_limitation_time)
                
        rospy.loginfo("!! Got a plan !!")
        # publish the plan
        pub_msg = HandringPlan()
        pub_msg.grasp = grasp
        pub_msg.trajectory = plan
        self.hp_pub.publish(pub_msg)
        self.arm.clear_pose_targets()
        # return goal state from generated trajectory
        goal_state = JointState()
        goal_state.header = Header()
        goal_state.header.stamp = rospy.Time.now()
        goal_state.name = plan.joint_trajectory.joint_names[:]
        goal_state.position = plan.joint_trajectory.points[-1].positions[:]
        return (True, goal_state)

    def get_cartesian_plan(self, trans, z_offset, start_state, grasp):
        # set argument start state
        moveit_start_state = RobotState()
        moveit_start_state.joint_state = start_state
        self.arm.set_start_state(moveit_start_state)
        # set waypoints
        waypoints = []
        self.target_pose.position.x = trans.transform.translation.x
        self.target_pose.position.y = trans.transform.translation.y
        self.target_pose.position.z = trans.transform.translation.z
        q = (trans.transform.rotation.x,
             trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w)
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(q)
        pitch += pi/2.0
        tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.target_pose.orientation.x = tar_q[0]
        self.target_pose.orientation.y = tar_q[1]
        self.target_pose.orientation.z = tar_q[2]
        self.target_pose.orientation.w = tar_q[3]
        wpose = Pose()
        wpose.position = copy.deepcopy(self.target_pose.position)
        wpose.orientation = copy.deepcopy(self.target_pose.orientation)
        wpose.position.z = copy.deepcopy(self.target_pose.position.z + z_offset)
        waypoints.append(wpose)
        # plan
        plan = RobotTrajectory()
        counter = 0
        while len(plan.joint_trajectory.points) == 0 :
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0)         # jump_threshold
            counter+=1
            self.arm.set_planning_time(self.planning_limitation_time+counter*5.0)
            if counter > 1 :
                return (False, start_state)
        self.arm.set_planning_time(self.planning_limitation_time)

        rospy.loginfo("!! Got a cartesian plan !!")
        # publish the plan
        pub_msg = HandringPlan()
        pub_msg.grasp = grasp
        pub_msg.trajectory = plan
        self.hp_pub.publish(pub_msg)
        self.arm.clear_pose_targets()
        # return goal state from generated trajectory
        goal_state = JointState()
        goal_state.header = Header()
        goal_state.header.stamp = rospy.Time.now()
        goal_state.name = plan.joint_trajectory.joint_names[:]
        goal_state.position = plan.joint_trajectory.points[-1].positions[:]
        return (True, goal_state)

        
    # -------- Go to Home Position -------- #
    def get_home_plan(self, start_state, grasp):
        # Set argument start state
        moveit_start_state = RobotState()
        moveit_start_state.joint_state = start_state
        self.arm.set_start_state(moveit_start_state)
        # Calculate goal pose
        init_pose = self.arm.get_current_joint_values()
        init_pose[0] = 0.0
        init_pose[1] = 0.0
        init_pose[2] = 0.0
        init_pose[3] = 0.0
        init_pose[4] = 0.0
        init_pose[5] = 0.0
        init_pose[6] = 0.0
        self.arm.set_joint_value_target(init_pose)
        # plan
        plan = RobotTrajectory()
        counter = 0
        while len(plan.joint_trajectory.points) == 0 :
            plan = self.arm.plan()
            counter+=1
            self.arm.set_planning_time(self.planning_limitation_time+counter*5.0)
            if counter > 1 :
                return (False, start_state)
        self.arm.set_planning_time(self.planning_limitation_time)
        
        rospy.loginfo("!! Got a home plan !!")
        # publish the plan
        pub_msg = HandringPlan()
        pub_msg.grasp = grasp
        pub_msg.trajectory = plan
        self.hp_pub.publish(pub_msg)
        self.arm.clear_pose_targets()
        # return goal state from generated trajectory
        goal_state = JointState()
        goal_state.header = Header()
        goal_state.header.stamp = rospy.Time.now()
        goal_state.name = plan.joint_trajectory.joint_names[:]
        goal_state.position = plan.joint_trajectory.points[-1].positions[:]
        return (True, goal_state)
    
    # -------- Go to Box Position -------- #
    def get_box_plan(self, num, start_state, grasp):
        # Set argument start state
        moveit_start_state = RobotState()
        moveit_start_state.joint_state = start_state
        self.arm.set_start_state(moveit_start_state)
        # Calculate goal pose
        self.arm.set_joint_value_target(self.box_pose[num])
        # plan
        plan = RobotTrajectory()
        counter = 0
        while len(plan.joint_trajectory.points) == 0 :
            plan = self.arm.plan()
            counter+=1
            self.arm.set_planning_time(self.planning_limitation_time+counter*5.0)
            if counter > 1 :
                return (False, start_state)
        self.arm.set_planning_time(self.planning_limitation_time)
        rospy.loginfo("!! Got a box plan !!")
        # publish the plan
        pub_msg = HandringPlan()
        pub_msg.grasp = grasp
        pub_msg.trajectory = plan
        self.hp_pub.publish(pub_msg)
        self.arm.clear_pose_targets()

        # return goal state from generated trajectory
        goal_state = JointState()
        goal_state.header = Header()
        goal_state.header.stamp = rospy.Time.now()
        goal_state.name = plan.joint_trajectory.joint_names[:]
        goal_state.position = plan.joint_trajectory.points[-1].positions[:]
        return (True, goal_state, plan)

    # def set_plan(self, plan, grasp):
    
    # -------- Get message from pepper -------- #
    def speechCallback(self, message):
        rospy.loginfo("(-O-) Task start (-O-)")
        # initialize
        start_state = JointState()
        start_state.header = Header()
        start_state.header.stamp = rospy.Time.now()
        start_state.name = rosparam.get_param("/controller_joint_names")
        # start_state.name = rosparam.get_param("/sia5_controller/joints")
        for i in range(len(start_state.name)):
            start_state.position.append(0.)
        get_num_from_pepper = int(message.data)

        # do the planning depending on the order number
        if get_num_from_pepper == 99 :
            rospy.loginfo("Called order 99")
            trans = []
            box_num = self.initial_box_num
            rospy.loginfo("%d objects detected...", box_num)
            for x in xrange(1, box_num+1):
                trans.append(self.get_tf_data(x))
            for x in xrange(0, box_num):
                state = self.run(1, (x+1)%2, start_state, trans[x])
                if rospy.is_shutdown():
                    rospy.on_shutdown(self.shutdown)
                    break
                rospy.loginfo("No.%i task finished.", x+1)
                start_state = state
            rospy.loginfo("(^O^) All task finished (^O^)")
            
        else :
            object_num = get_num_from_pepper / 10
            box_num = get_num_from_pepper % 10 - 1
            trans = self.get_tf_data(object_num)
            self.run(object_num, box_num, start_state, trans)
            if rospy.is_shutdown():
                rospy.on_shutdown(self.shutdown)
            rospy.loginfo("(^O^) All task finished (^O^)")

    # -------- Shutdown -------- #
    def shutdown(self):
        rospy.logwarn("(xOx) Aborted (xOx)")
            
    # -------- Run the Program -------- #
    def run(self, obj_num, box_num, start_state, trans):
        # Go to Grasp
        (result, state) = self.get_plan(trans, self.offset, start_state, False)
        if rospy.is_shutdown():
            return
        (result, state) = self.get_cartesian_plan(trans, 0.3 + self.diff, state, True)
        if rospy.is_shutdown():
            return
        # Back to upper side
        (result, state) = self.get_cartesian_plan(trans, self.offset + 0.05, state, True)
        if rospy.is_shutdown():
            return
        # Back to home
        (result, state) = self.get_home_plan(state, True)
        if rospy.is_shutdown():
            return
        # Go to Box
        (result, state, plan) = self.get_box_plan(box_num, state, False)
        if rospy.is_shutdown():
            return
        # Go to home
        (result, state) = self.get_home_plan(state, False)
        if rospy.is_shutdown():
            return

        return state
        

if __name__ == '__main__':
    rospy.init_node("handring_parallel_planner")
    handring_planner = HandringPlanner()
    rospy.spin()
