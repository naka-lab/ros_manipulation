#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import moveit_commander
from geometry_msgs.msg import PoseStamped 
from dynamixel_controllers.srv import *
import tf
import math
import time

PI = math.pi

def set_pose(x, y, z, rz, ry, rx):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    x,y,z,w = tf.transformations.quaternion_from_euler(rz, ry, rx)
    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w

    arm.set_start_state_to_current_state()
    arm.set_pose_target(pose)

    print(arm.get_current_pose())

    ret = arm.go()
    time.sleep(0.5)

    return ret

def open_gripper( _open=True ):
    gripper.set_start_state_to_current_state()
    if _open:
        gripper.set_joint_value_target([0.1])
    else:
        gripper.set_joint_value_target([0.01])
    gripper.go()
    time.sleep(0.5)

def main():
    # 初期位置・グリッパを開く
    set_pose( 0, 0, 0.3, 0, PI/4, 0  )
    open_gripper()

    # アーム移動、グリッパを閉じる
    set_pose( 0.2, 0, 0.05, 0, PI/2, 0  )
    open_gripper( False )

    # 初期位置
    set_pose( 0, 0, 0.3, 0, PI/4, 0  )

    # アーム移動、グリッパを閉じる
    set_pose( 0, -0.2, 0.05, PI/2, PI/2, 0  )
    open_gripper()


if __name__ == '__main__':
    rospy.init_node("move_to_position")

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_pose_reference_frame("base_link")
    arm.set_goal_position_tolerance( 0.02 )
    arm.set_planning_time( 10 )
    gripper = moveit_commander.MoveGroupCommander("gripper")

    main()
