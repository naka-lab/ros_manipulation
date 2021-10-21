#!/usr/bin/env python
from __future__ import print_function, unicode_literals
import rospy
import dynamic_reconfigure.client

if __name__ == '__main__':
    rospy.init_node("dynamic_reconfigure_client")
    client = dynamic_reconfigure.client.Client("/crane_x7/crane_x7_control/crane_x7_gripper_finger_a_joint/")
    client.update_configuration({"current_limit": 70})
    rospy.spin()