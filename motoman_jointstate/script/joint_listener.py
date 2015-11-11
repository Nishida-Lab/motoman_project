#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def callback(message):
  joint_name = ["s", "l", "e", "u", "r", "b", "t"]
  for i in range(0, 7):
    name = "joint_" + joint_name[i] + " : " + str(message.position[i])
    rospy.loginfo("%s", name)
  print("")

rospy.init_node("joint_listener_py")
sub = rospy.Subscriber("/joint_states", JointState, callback)
rospy.spin()