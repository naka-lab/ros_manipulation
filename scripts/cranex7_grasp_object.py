#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import tf
import math
import time
import yaml

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

def set_init_pose():
    arm.set_joint_value_target( [0, -0.3, 0, -2.5, 0, 1.2, 1.57] )
    arm.go()


def grasp(x, y, z):
    # 初期位置・グリッパを開く
    set_init_pose()
    open_gripper()

    # アーム移動、グリッパを閉じる
    if z<0.1:
        z = 0.1
    theta = math.atan2( y, x )

    # 一旦手前にハンドを持っていく
    set_pose( x-0.1*math.cos(theta), y-0.1*math.sin(theta), z, PI/2, 0,  PI/2+theta  )

    # 物体位置へ移動
    set_pose( x, y, z, PI/2, 0,  PI/2+theta  )
    open_gripper( False )

    # 持ち上げる
    set_pose( x, y, z+0.1, PI/2, 0,  PI/2+theta  )

     # 初期位置
    set_init_pose()


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
                if o["label"]==０:
                    print("物体発見")
                    target = o["position"]
                    break

        if target!=None:
            break

    # 座標変換
    p = transform( target[0], target[1], target[2] )

    # 少し手前にする
    p.point.x -= 0.03

    # 使える範囲か確認
    d = math.sqrt( p.point.x**2 + p.point.y**2 + p.point.z**2 )
    if d>0.6:
        print("遠すぎてつかめない！！")
        return

    # 把持
    grasp( p.point.x, p.point.y, p.point.z )
   

if __name__ == '__main__':
    rospy.init_node("grasp_object")

    tf_listener = tf.TransformListener()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    main()
