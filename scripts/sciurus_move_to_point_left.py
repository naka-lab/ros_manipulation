#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import math
import time
import actionlib
import tf
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

PI = math.pi

def set_pose(x, y, z, rx, ry, rz, left_right="left" ):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    x,y,z,w = tf.transformations.quaternion_from_euler(rx, ry, rz)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w

    if left_right=="left":
        arm_l.set_start_state_to_current_state()
        arm_l.set_pose_target(pose)
        ret = arm_l.go()
    elif left_right=="right":
        arm_r.set_start_state_to_current_state()
        arm_r.set_pose_target(pose)
        ret = arm_r.go()
    else:
        print(left_right, ":left_rightの指定が不正です")

    return ret

def open_gripper( _open=True, left_right="left" ):
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 2.0
    
    if left_right=="left":
        if _open:
            gripper_goal.command.position = -0.9
        else:
            gripper_goal.command.position = -0.1
        gripper_l.send_goal(gripper_goal)
        gripper_l.wait_for_result(rospy.Duration(1.0))
    elif left_right=="right":
        if _open:
            gripper_goal.command.position = 0.9
        else:
            gripper_goal.command.position = 0.1

        gripper_r.send_goal(gripper_goal)
        gripper_r.wait_for_result(rospy.Duration(1.0))
    else:
        print(left_right, ":left_rightの指定が不正です")

def set_init_pose():
    arm_l.set_joint_value_target( [0, PI/2, 0, -PI*3/4, 0, PI/3, PI/2] )
    arm_r.set_joint_value_target( [0, -PI/2, 0, PI*3/4, 0, -PI/3, -PI/2] )
    arm_l.go()
    arm_r.go()


def grasp_left():
    # 初期姿勢
    set_init_pose()

    # ハンドを開く
    open_gripper(True, "left")

    # 把持位置へ移動
    set_pose( 0.4, 0.0, 0.1, 0, 0.0, -PI/2, "left" )

    # ハンドを閉じる
    open_gripper(False, "left")

    # 初期位置へ戻る
    set_init_pose()

    # 置く位置へ移動
    set_pose( 0.4, 0.4, 0.1, 0, 0.0, -PI/2+PI/4, "left" )

    # ハンドを開く
    open_gripper(True, "left")

    # 初期位置へ戻る
    set_init_pose()

def grasp_right():
    set_init_pose()

    # ハンドを開く
    open_gripper(True, "right")

    # 把持位置へ移動
    set_pose( 0.4, 0.0, 0.1, 0, 0, PI/2, "right" )

    # ハンドを閉じる
    open_gripper(False, "right")

    # 初期位置へ戻る
    set_init_pose()

    # 置く位置へ移動
    set_pose( 0.4, -0.4, 0.1, 0, 0.0, PI/2-PI/4, "right" )

    # ハンドを開く
    open_gripper(True, "right")

    # 初期位置へ戻る
    set_init_pose()


def main():
    grasp_left()
    grasp_right()

if __name__ == '__main__':
    rospy.init_node("move_to_position")

    robot = moveit_commander.RobotCommander()
    arm_l = moveit_commander.MoveGroupCommander("l_arm_group")
    arm_r = moveit_commander.MoveGroupCommander("r_arm_group")
    arm_l.set_max_velocity_scaling_factor(0.1)
    arm_r.set_max_velocity_scaling_factor(0.1)
    gripper_l = actionlib.SimpleActionClient("/sciurus17/controller2/left_hand_controller/gripper_cmd", GripperCommandAction)
    gripper_r = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
    gripper_l.wait_for_server()
    gripper_r.wait_for_server()

    main()
