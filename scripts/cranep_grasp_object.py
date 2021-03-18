#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String
import tf
import math
import time
import yaml

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

def grasp(x, y, z):
    # 初期位置・グリッパを開く
    set_pose( 0, 0, 0.3, 0, PI/4, 0  )
    open_gripper()

    # アーム移動、グリッパを閉じる
    if z<0.05:
        z = 0.05
    theta = math.atan2( y, x )
    set_pose( x, y, z, theta, PI/2, 0  )
    open_gripper( False )

     # 初期位置
    set_pose( 0, 0, 0.3, 0, PI/4, 0  )


def transform( camx, camy, camz ):
    tf_listener.waitForTransform( "base_link" , "camera_depth_optical_frame", rospy.Time(0), rospy.Duration(5)  )

    pos_from_cam = PointStamped()
    pos_from_cam.header.frame_id = "camera_depth_optical_frame"
    pos_from_cam.point.x = camx
    pos_from_cam.point.y = camy
    pos_from_cam.point.z = camz

    pos_trans = tf_listener.transformPoint( "base_link", pos_from_cam )

    return pos_trans

def main():
    while not rospy.is_shutdown():
        data = rospy.wait_for_message( "/ar_marker_rec/object_info", String )
        obj_info = yaml.safe_load(data.data)

        if len(obj_info)==0:
            print("物体なし")
            continue
        else:
            print("物体発見")
            break

    # とりあえず0番目の物体を選択
    p = obj_info[0]["position"]

    # 座標変換
    #p = transform( -0.1, 0.0, 0.2 )
    p = transform( p[0], p[1], p[2] )

    # 使える範囲か確認
    d = math.sqrt( p.point.x**2 + p.point.y**2 + p.point.z**2 )
    if d>0.26:
        print("遠すぎてつかめない！！")
        return

    # 把持
    grasp( p.point.x, p.point.y, p.point.z )
   

if __name__ == '__main__':
    rospy.init_node("grasp_object")

    tf_listener = tf.TransformListener()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_pose_reference_frame("base_link")
    arm.set_goal_position_tolerance( 0.02 )
    arm.set_planning_time( 10 )
    gripper = moveit_commander.MoveGroupCommander("gripper")

    main()
