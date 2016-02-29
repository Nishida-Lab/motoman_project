#!/usr/bin/env python
# -*- coding: utf-8 -*-

import moveit_commander
import rospy
import geometry_msgs.msg
import copy

def main():
    rospy.init_node("moveit_command_sender")

    robot = moveit_commander.RobotCommander()

    print "=" * 10, " Robot Groups:"
    print robot.get_group_names()

    print "=" * 10, " Printing robot state"
    print robot.get_current_state()
    print "=" * 10

    arm = moveit_commander.MoveGroupCommander("arm")

    print "=" * 15, " Arm ", "=" * 15
    print "=" * 10, " Reference frame: %s" % arm.get_planning_frame()

    print "=" * 10, " Reference frame: %s" % arm.get_end_effector_link()

    arm.clear_pose_targets()
    arm_initial_pose = arm.get_current_pose().pose
    print "=" * 10, " Printing initial pose: "
    print arm_initial_pose

    print "=" * 10, " Cartesian Paths"
    waypoints = []

    waypoints.append(arm.get_current_pose().pose)

    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z - 0.1
    waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= 0.2
    # wpose.position.z -= 0.2
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.z += 0.2
    # waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0, avoid_collisions=False)

    print "=" * 10, " plan1..."
    arm.execute(plan)
    rospy.sleep(5)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "ROSInterruptException!!!!"
        pass