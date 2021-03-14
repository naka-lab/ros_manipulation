#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import math
import time
import actionlib
from std_msgs.msg import String
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PointStamped
import yaml
import tf

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

def set_neck_pose(yaw, pitch):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["neck_yaw_joint", "neck_pitch_joint"]

    point = JointTrajectoryPoint( positions=[yaw, pitch], time_from_start=rospy.Duration(nsecs=1))
    goal.trajectory.points.append(point)

    neck.send_goal(goal)
    neck.wait_for_result()
    return neck.get_result()

def transform( camx, camy, camz ):
    tf_listener.waitForTransform( "base_link" , "camera_link", rospy.Time(0), rospy.Duration(5)  )

    pos_from_cam = PointStamped()
    pos_from_cam.header.frame_id = "camera_link"
    pos_from_cam.point.x = camx
    pos_from_cam.point.y = camy
    pos_from_cam.point.z = camz

    pos_trans = tf_listener.transformPoint( "base_link", pos_from_cam )

    return pos_trans

"""
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
"""

def grasp_left(x, y, z):

    # 初期姿勢
    set_init_pose()

    # ハンドを開く
    open_gripper(True, "left")

    # 把持位置へ移動
    if z<0.1:
        z = 0.1
    theta = math.atan2( y-0.3, x )

    # 少し手前に移動
    set_pose( x-0.1*math.cos(theta), y-0.1*math.sin(theta), z, 0, 0, -PI/2+theta, "left" )

    # 物体位置へ移動
    set_pose( x-0.05*math.cos(theta), y-0.05*math.sin(theta), z, 0, 0, -PI/2+theta, "left" )

    # ハンドを閉じる
    open_gripper(False, "left")

    # 持ち上げる
    set_pose( x-0.05*math.cos(theta), y-0.05*math.sin(theta), z+0.1, 0, 0, -PI/2+theta, "left" )

    # 初期姿勢
    set_init_pose()

def grasp_right(x, y, z):
    # 初期姿勢
    set_init_pose()

    # ハンドを開く
    open_gripper(True, "right")

    # 把持位置へ移動
    if z<0.1:
        z = 0.1
    theta = math.atan2( y+0.3, x )

    # 少し手前に移動
    set_pose( x-0.1*math.cos(theta), y-0.1*math.sin(theta), z, 0, 0, PI/2+theta, "right" )

    # 物体位置へ移動
    set_pose( x-0.05*math.cos(theta), y-0.05*math.sin(theta), z, 0, 0, PI/2+theta, "right" )

    # ハンドを閉じる
    open_gripper(False, "right")

    # 持ち上げる
    set_pose( x-0.05*math.cos(theta), y-0.05*math.sin(theta), z+0.1, 0, 0, PI/2+theta, "right" )

    # 初期姿勢
    set_init_pose()


def main():
    # 下を向く
    set_neck_pose( 0, -45/180*PI )

    # 初期姿勢に移動
    set_init_pose()

    # ターゲットとなる物体がカメラに映るのを待つ
    target = None
    while not rospy.is_shutdown():
        data = rospy.wait_for_message( "/ar_marker_rec/object_info", String )
        obj_info = yaml.safe_load(data.data)

        if len(obj_info)==0:
            print("物体なし")
            continue
        elif obj_info:
            for o in obj_info:
                if o["label"]==0:
                    print("物体発見")
                    target = o["position"]
                    break

        if target!=None:
            break

    # 座標変換
    p = transform( target[0], target[1], target[2] )
    print( p.point.x, p.point.y, p.point.z )

    # 左腕
    grasp_left(p.point.x, p.point.y, p.point.z)

    # 右腕
    grasp_right(p.point.x, p.point.y, p.point.z)


if __name__ == '__main__':
    rospy.init_node("grasp_object")

    robot = moveit_commander.RobotCommander()
    arm_l = moveit_commander.MoveGroupCommander("l_arm_group")
    arm_r = moveit_commander.MoveGroupCommander("r_arm_group")
    arm_l.set_max_velocity_scaling_factor(0.1)
    arm_r.set_max_velocity_scaling_factor(0.1)
    gripper_l = actionlib.SimpleActionClient("/sciurus17/controller2/left_hand_controller/gripper_cmd", GripperCommandAction)
    gripper_r = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
    gripper_l.wait_for_server()
    gripper_r.wait_for_server()

    neck = actionlib.SimpleActionClient("/sciurus17/controller3/neck_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    neck.wait_for_server()

    tf_listener = tf.TransformListener() 
    
    main()
