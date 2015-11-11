#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from sensor_msgs.msg import JointState

if __name__ == "__main__":
  rospy.init_node("tf_listener_py")

  tf_buffer = tf2_ros.Buffer()
  tf_listener = tf2_ros.TransformListener(tf_buffer)

  tf_name = ["base_link", "link_b", "link_e", "link_l", "link_r", "link_s", "link_t", "link_u"]

  rate = rospy.Rate(10.0)
  while not rospy.is_shutdown():
    try:
      for i in range(1,7):
        trans = tf_buffer.lookup_transform(tf_name[0], tf_name[i], rospy.Time())
        print tf_name[0] + "から見た" + tf_name[i]
        print trans.transform.translation
        print ""
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.logwarn("tf not found")
    rate.sleep()