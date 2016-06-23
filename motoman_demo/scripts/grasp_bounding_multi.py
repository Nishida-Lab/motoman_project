#!/usr/bin/env python

import math
import moveit_commander
import rospy
import geometry_msgs.msg
import copy
import tf2_ros
import tf
from std_msgs.msg import Int32
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox


def get_tf_data(times):
    print"tf here"
    # ========== TF Lister ========== #
    tf_buffer = tf2_ros.Buffer()
    tf_listner = tf2_ros.TransformListener(tf_buffer)

    tf_time = rospy.Time(0)
    trans = []
    for i in range(times):
        target = "object_" + str(i)
        get_tf_flg = False
        while not get_tf_flg :
            try :
                trans.append( tf_buffer.lookup_transform('world', target, tf_time, rospy.Duration(10)) )
                get_tf_flg = True

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
                continue
    return trans

def cb_once(bbox_array_data):
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

    trans = []
    trans = get_tf_data(len(bbox_array_data.boxes))

    for i in range(len(trans)):
        print "world -> object[" + str(i) + "]"
        print trans[i].transform
        target_pose.position.x = trans[i].transform.translation.x
        target_pose.position.y = trans[i].transform.translation.y
        target_pose.position.z = trans[i].transform.translation.z + 0.4
        q = (trans[i].transform.rotation.x,
             trans[i].transform.rotation.y,
             trans[i].transform.rotation.z,
             trans[i].transform.rotation.w)
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
        arm.go()
        arm.clear_pose_targets()
        # Grasp
        grasp_pub.publish(1)
        print "!! Grasping !!"
        rospy.sleep(0.5)

        # Go to Set Position
        # target_pose.position.x = -0.13683 + (i-1)*0.08
        # target_pose.position.y = -0.22166
        # target_pose.position.z = 0.40554
        # target_pose.orientation.x = 0.00021057
        # target_pose.orientation.y = 0.70092
        # target_pose.orientation.z = 0.00030867
        # target_pose.orientation.w = 0.71324
        # arm.set_pose_target(target_pose)
        # arm.go()
        # arm.clear_pose_targets()

        # # Release
        # print " !! Release !!"
        # grasp_pub.publish(0)
        # rospy.sleep(1)

    arm.set_pose_target(arm_initial_pose)
    arm.go()
    arm.clear_pose_targets

    subscriber.unregister()

def main():
    # init node
    rospy.init_node("grasp_commander")

    # ========= Call Back Function ========
    sub_once = None
    sub_once = rospy.Subscriber('/clustering_result', BoundingBoxArray, cb_once, sub_once)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
