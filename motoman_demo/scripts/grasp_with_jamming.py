#!/usr/bin/env python

import moveit_commander
import rospy
import geometry_msgs.msg
import copy
import tf
from std_msgs.msg import Int32

def main():
    # init node
    rospy.init_node("grasp_commander")

    # ========== publisher to jamming gripper ========== #
    grasp_pub = rospy.Publisher('/jamming_grasp', Int32, queue_size=1)
    
    # ========== Moveit init ========== #
    # moveit_commander init
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm_initial_pose = arm.get_current_pose().pose
    target_pose = geometry_msgs.msg.Pose()

    # ========== TF Lister ========== #
    tf_listner = tf.TransformListner()
    # Get target TF
    try:
        (tgt_trans,tgt_pose) = tf_listner.lookupTransform('/world', '/target', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    print "Target Place & Pose"
    print "( ", tgt_trans.[0] ", ", tgt_trans[1], ", ", tgt_trans[2], " )"
    print "( ", tgt_pose.[0] ", ", tgt_pose[1], ", ", tgt_pose[2], tgt_pose[3], " )"

    # Go to up from target
    target_pose.position.x = tgt_trans.[0]
    target_pose.position.y = tgt_trans.[1]
    target_pose.position.z = tgt_trans.[2] + 0.2
    target_pose.orientation.x = tgt_pose.[0]
    target_pose.orientation.y = tgt_pose.[1]
    target_pose.orientation.z = tgt_pose.[2]
    target_pose.orientation.w = tgt_pose.[3]
    arm.set_pose_target(target_pose)
    arm.go()

    # Get Grasp
    target_pose.position.x = tgt_trans.[0]
    target_pose.position.y = tgt_trans.[1]
    target_pose.position.z = tgt_trans.[2] + 0.1
    target_pose.orientation.x = tgt_pose.[0]
    target_pose.orientation.y = tgt_pose.[1]
    target_pose.orientation.z = tgt_pose.[2]
    target_pose.orientation.w = tgt_pose.[3]
    arm.set_pose_target(target_pose)
    arm.go()

    # Grasp
    grasp_pub.publish(1)
    print "!! Grasping !!"
    rospy.sleep(1.0)
    
    # Go to Home Position
    target_pose.position.x = 0
    target_pose.position.y = 0
    target_pose.position.z = 0
    target_pose.orientation.x = 0
    target_pose.orientation.y = 0
    target_pose.orientation.z = 0
    target_pose.orientation.w = 1
    arm.set_pose_target(target_pose)
    arm.go()

    # Release
    print " !! Release !!"
    grasp_pub.publish(0)
    rospy.sleep(2)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
