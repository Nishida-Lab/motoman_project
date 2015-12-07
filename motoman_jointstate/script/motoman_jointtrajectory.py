#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def jointtrajectory_publisher():
    pub = rospy.Publisher('set_joint_trajectory', JointTrajectory, queue_size=10)
    rospy.init_node('set_joint_trajectory_test', anonymous=True)
    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        now = rospy.get_time()
        msg = JointTrajectory()
        msg.joint_names = ["joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"]
        msg.points = []

        p = JointTrajectoryPoint()
        p.time_from_start = rospy.rostime.Duration(2)
        p.positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        msg.points.append(p)

        rospy.loginfo("hello world %s" % now)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        jointtrajectory_publisher()
    except rospy.ROSInterruptException:
        pass