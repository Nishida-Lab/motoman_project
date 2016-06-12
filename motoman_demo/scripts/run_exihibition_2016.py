#!/usr/bin/env python
# -*- coding: utf-8 -*-


from math import *
# Moveit 
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import tf
# Octomap Service
from std_srvs.srv import Empty

# ROS
import rospy
# D-Hand
from dhand.msg import Servo_move



class Handring(object):

    def __init__(self):
        # ========== publisher to jamming gripper ========== #
        self.grasp_pub = rospy.Publisher('/dhand_grasp', Servo_move, queue_size=1)
        self.grasp_msg = Servo_move()
        self.grasp_msg.position = 0.0
        self.grasp_msg.speed = 15
        self.grasp_msg.acceleration = 0.2
        self.grasp_msg.current_limit = 0.5
        
        # ========== Moveit init ========== #
        # moveit_commander init
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm_initial_pose = self.arm.get_current_pose().pose
        self.target_pose = geometry_msgs.msg.Pose()
        # Set the planning time
        self.arm.set_planning_time(10.0)
 
        # ========== TF ======== #
        # TF Listner #
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)

        # ========= Box Poses ======== #
        self.box_pose = [geometry_msgs.msg.Pose(), geometry_msgs.msg.Pose()]
        self.box_pose[0].position.x = 0.347605
        self.box_pose[0].position.y = -0.337682
        self.box_pose[0].position.z = 0.605245
        self.box_pose[0].orientation.x = -0.189665
        self.box_pose[0].orientation.y = 0.658327
        self.box_pose[0].orientation.z = -0.181423
        self.box_pose[0].orientation.w = 0.705491
        self.box_pose[1].position.x = 0.596435
        self.box_pose[1].position.y = 0.237905
        self.box_pose[1].position.z = 0.506382
        self.box_pose[1].orientation.x = 0.0204865
        self.box_pose[1].orientation.y = 0.424285
        self.box_pose[1].orientation.z = 0.155347
        self.box_pose[1].orientation.w = 0.891869


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
    def set_plan(self, trans, z_offset):
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
        print "Move !!"
        self.arm.go()
        rospy.sleep(0.5)
        self.arm.clear_pose_targets()

    # -------- Go to Home Position -------- #
    def go_home(self):
        # Go to Initial Pose
        init_pose = self.arm.get_current_joint_values()
        init_pose[0] = 0.0
        init_pose[1] = 0.0
        init_pose[2] = 0.0
        init_pose[3] = 0.0
        init_pose[4] = 0.0
        init_pose[5] = 0.0
        init_pose[6] = 0.0
        self.arm.set_joint_value_target(init_pose)
        self.arm.go()
        rospy.sleep(0.5)
        self.arm.clear_pose_targets()

    # -------- Go to Box Position -------- #
    def go_box(self, num):
        self.arm.set_pose_target(self.box_pose[num])
        self.arm.go()
        rospy.sleep(0.5)
        self.arm.clear_pose_targets()
        
    # -------- Run the Program -------- #
    def run(self, obj_num, box_num):
        trans = self.get_tf_data(obj_num)
        print "world -> object_" + str(obj_num)
        print trans.transform

        print "Go to Grasp."
        self.set_plan(trans, 0.4)
        self.set_plan(trans, 0.33)
                
        # Grasp
        print "!! Grasping !!"
        self.grasp_msg.position = 7
        # self.grasp_pub.publish(self.grasp_msg)
        rospy.sleep(0.5)

        print "Going up"
        self.set_plan(trans, 0.5)

        print "Go to Box"
        self.go_box(box_num)

        # Release
        print "!! Release !!"
        self.grasp_msg.position = 0.0
        # self.grasp_pub.publish(self.grasp_msg)
        rospy.sleep(0.5)

        print "Go to Home Position"
        self.go_home()
            
if __name__ == '__main__':
    rospy.init_node("run_exihibition_2016")
    handring = Handring()
    handring.run(0)
    
