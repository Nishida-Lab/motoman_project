#!/usr/bin/env python

import math
import moveit_commander
import move_group
import rospy
import geometry_msgs.msg
import copy
import tf2_ros
import tf
from std_msgs.msg import Int32
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox


def get_tf_data(num):
    print"tf here"
    # ========== TF Lister ========== #
    tf_buffer = tf2_ros.Buffer()
    tf_listner = tf2_ros.TransformListener(tf_buffer)

    tf_time = rospy.Time(0)
    target = "object_" + str(num)
    get_tf_flg = False
    while not get_tf_flg :
        try :
            trans = tf_buffer.lookup_transform('world', target, tf_time, rospy.Duration(10)) 
            get_tf_flg = True
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
            continue
    return trans

def run():
    print "here"
    # ========== publisher to jamming gripper ========== #
    grasp_pub = rospy.Publisher('/jamming_grasp', Int32, queue_size=1)

    # ========== Moveit init ========== #
    # moveit_commander init
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm_initial_pose = arm.get_current_pose().pose
    target_pose = geometry_msgs.msg.Pose()
    # Set the planning time
    arm.set_planning_time(10.0)

    num_str = raw_input()
    num = int(num_str)
    trans = get_tf_data(num)

    print "world -> object[" + str(num) + "]"
    print trans.transform
    target_pose.position.x = trans.transform.translation.x
    target_pose.position.y = trans.transform.translation.y
    target_pose.position.z = trans.transform.translation.z + 0.40
    q = (trans.transform.rotation.x,
         trans.transform.rotation.y,
         trans.transform.rotation.z,
         trans.transform.rotation.w)
    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(q)
    # roll -= math.pi/6.0
    pitch += math.pi/2.0
    # yaw += math.pi/4.0
    tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    target_pose.orientation.x = tar_q[0]
    target_pose.orientation.y = tar_q[1]
    target_pose.orientation.z = tar_q[2]
    target_pose.orientation.w = tar_q[3]
    target_pose.orientation = target_pose.orientation
    arm.set_pose_target(target_pose)
    plan = arm.plan()
    print "Plan OK?"
    res = raw_input()
    if res == 'y':
        arm.execute(plan)
    arm.clear_pose_targets()

    
    target_pose.position.x = trans.transform.translation.x
    target_pose.position.y = trans.transform.translation.y
    target_pose.position.z = trans.transform.translation.z + 0.37
    q = (trans.transform.rotation.x,
         trans.transform.rotation.y,
         trans.transform.rotation.z,
         trans.transform.rotation.w)
    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(q)
    # roll -= math.pi/6.0
    pitch += math.pi/2.0
    # yaw += math.pi/4.0
    tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    target_pose.orientation.x = tar_q[0]
    target_pose.orientation.y = tar_q[1]
    target_pose.orientation.z = tar_q[2]
    target_pose.orientation.w = tar_q[3]
    target_pose.orientation = target_pose.orientation
    arm.set_pose_target(target_pose)
    plan = arm.plan()
    print "Plan OK?"
    res = raw_input()
    if res == 'y':
        arm.execute(plan)
    arm.clear_pose_targets()
    

    target_pose.position.x = trans.transform.translation.x
    target_pose.position.y = trans.transform.translation.y
    target_pose.position.z = trans.transform.translation.z + 0.45
    q = (trans.transform.rotation.x,
         trans.transform.rotation.y,
         trans.transform.rotation.z,
         trans.transform.rotation.w)
    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(q)
    # roll -= math.pi/6.0
    pitch += math.pi/2.0
    # yaw += math.pi/4.0
    tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    target_pose.orientation.x = tar_q[0]
    target_pose.orientation.y = tar_q[1]
    target_pose.orientation.z = tar_q[2]
    target_pose.orientation.w = tar_q[3]
    target_pose.orientation = target_pose.orientation
    arm.set_pose_target(target_pose)
    plan = arm.plan()
    print "Plan OK?"
    res = raw_input()
    if res == 'y':
        arm.execute(plan)
    arm.clear_pose_targets()
    
    # Grasp
    grasp_pub.publish(1)
    print "!! Grasping !!"
    rospy.sleep(0.5)
    
    print "Home Position"
    # Go to Set Position
    # target_pose.position.x = 0.26439
    # target_pose.position.y = -0.401881
    # target_pose.position.z = 0.361824
    # target_pose.orientation.x = 0.183269
    # target_pose.orientation.y = 0.296633
    # target_pose.orientation.z = -0.6153
    # target_pose.orientation.w = 0.706985
    # print target_pose
    # arm.set_pose_target(target_pose)
    # arm.plan()
    #  arm.go()
    # arm.clear_pose_targets()
    
    # Release
    # print " !! Release !!"
    # grasp_pub.publish(0)
    # rospy.sleep(1)
    
    # arm.set_pose_target(arm_initial_pose)
    # arm.plan()
    # arm.clear_pose_targets
    
if __name__ == '__main__':
    rospy.init_node("ex_2016")
    run()
