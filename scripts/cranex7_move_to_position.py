#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import math
import time
import tf

PI = math.pi

def set_pose(x, y, z, rz, ry, rx):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    x,y,z,w = tf.transformations.quaternion_from_euler(rz, ry, rx)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w

    arm.set_start_state_to_current_state()
    arm.set_pose_target(pose)

    print(arm.get_current_pose())

    ret = arm.go()

    return ret

def open_gripper( _open=True ):
    gripper.set_start_state_to_current_state()
    if _open:
        gripper.set_joint_value_target([0.7, 0.7])
    else:
        gripper.set_joint_value_target([0.1, 0.1])
    gripper.go()

def main():
    # 初期姿勢
    set_pose( 0.2, 0, 0.26, PI/2, 0,  PI/2  )
    open_gripper()

    # 正面のものを掴む
    set_pose( 0.45, 0, 0.1, PI/2, 0,  PI/2  )
    open_gripper(False)

    # 初期姿勢
    set_pose( 0.2, 0, 0.26, PI/2, 0,  PI/2  )

    # 左前45度に置く
    set_pose( 0.3, 0.3, 0.1, PI/2, 0,  PI/2+PI/4  )
    open_gripper()

    # 初期姿勢
    set_pose( 0.2, 0, 0.26, PI/2, 0,  PI/2  )

if __name__ == '__main__':
    rospy.init_node("move_to_position")

    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    main()
