#!/usr/bin/env python

import math
import moveit_commander
import rospy
import geometry_msgs.msg
import copy
import tf2_ros
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

    # Set the planning time
    arm.set_planning_time(10.0)

    # ========== TF Lister ========== #
    tf_buffer = tf2_ros.Buffer()
    tf_listner = tf2_ros.TransformListener(tf_buffer)

    for i in [0, 1]:
        target = "object_" + str(i)
        get_tf_flg = False
        # Get target TF
        while not get_tf_flg :
            try :
                trans = tf_buffer.lookup_transform('world', target, rospy.Time(0),rospy.Duration(10))
                print "world -> ar_marker"
                print trans.transform
                print "Target Place & Pose"
                # Go to up from target
                target_pose.position.x = trans.transform.translation.x
                target_pose.position.y = trans.transform.translation.y
                target_pose.position.z = trans.transform.translation.z + 0.10
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
                print target_pose
                arm.set_pose_target(target_pose)
                arm.go()
                arm.clear_pose_targets()
                #rospy.sleep(2)
                # Get Grasp
                # waypoints = []
            
                # waypoints.append(arm.get_current_pose().pose)

                # wpose = geometry_msgs.msg.Pose()
                # wpose.orientation.w = 1.0
                # wpose.position.x = waypoints[0].position.x
                # wpose.position.y = waypoints[0].position.y - 0.085
                # wpose.position.z = waypoints[0].position.z
                # waypoints.append(copy.deepcopy(wpose))
                # (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.005, 0.0, False)
            
                # target_pose.position.x = trans.transform.translation.x
                # target_pose.position.y = trans.transform.translation.y + 0.04
                # target_pose.position.z = trans.transform.translation.z + 0.243
                # target_pose.orientation = target_pose.orientation
                # print target_pose
                # arm.set_pose_target(target_pose)
                # arm.go()
                # arm.clear_pose_targets()
                # # Grasp
                # grasp_pub.publish(1)
                # print "!! Grasping !!"
                # rospy.sleep(2.0)

                # # Start Return
                # # waypoints = []
            
                # # waypoints.append(arm.get_current_pose().pose)
            
                # # wpose = geometry_msgs.msg.Pose()
                # # wpose.orientation.w = 1.0
                # # wpose.position.x = waypoints[0].position.x
                # # wpose.position.y = waypoints[0].position.y + 0.085
                # # wpose.position.z = waypoints[0].position.z
                # # waypoints.append(copy.deepcopy(wpose))
                # # (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.005, 0.0, False)
                # target_pose.position.x = trans.transform.translation.x
                # target_pose.position.y = trans.transform.translation.y + 0.04
                # target_pose.position.z = trans.transform.translation.z + 0.32
                # target_pose.orientation = target_pose.orientation
                # print target_pose
                # arm.set_pose_target(target_pose)
                # arm.go()
                # arm.clear_pose_targets()
                # #rospy.sleep(2)

                # # Go to Home Position
                # target_pose.position.x = -0.13683 + (i-1)*0.08
                # target_pose.position.y = -0.22166
                # target_pose.position.z = 0.50554
                # target_pose.orientation.x = 0.00021057
                # target_pose.orientation.y = 0.70092
                # target_pose.orientation.z = 0.00030867
                # target_pose.orientation.w = 0.71324
                # arm.set_pose_target(target_pose)
                # arm.go()
                # arm.clear_pose_targets()
                # #rospy.sleep(2)

                # target_pose.position.x = target_pose.position.x
                # target_pose.position.y = target_pose.position.y
                # target_pose.position.z -= 0.10
                # target_pose.orientation = target_pose.orientation
                # arm.set_pose_target(target_pose)
                # arm.go()
                # arm.clear_pose_targets()
                
                # # Release
                # print " !! Release !!"
                # grasp_pub.publish(0)
                # rospy.sleep(1)
                get_tf_flg = True
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
                continue


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
