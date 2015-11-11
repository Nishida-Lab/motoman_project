#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("joint_publisher_py")
jointstate = JointState()

joint_name =["joint_s", "joint_l", "joint_e", "joint_u", "joint_r", "joint_b", "joint_t"]
joint_tmp = [0] * 7
cnt = 0

try:
  pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
  rate_mgr = rospy.Rate(10)  # 10hz
  print "Run! joint_publisher_py"
  while not rospy.is_shutdown():
    jointstate.header.stamp = rospy.Time.now()

    for i in range(0, 7):
      joint_tmp[i] = cnt/100.0

    jointstate.name = joint_name
    jointstate.position = joint_tmp

    jointstate.velocity = []
    jointstate.effort = []

    pub.publish(jointstate)
    rate_mgr.sleep()  # this has rospy.spin()
    cnt += 1
except rospy.ROSInterruptException:
  pass