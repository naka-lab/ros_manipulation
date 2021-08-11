#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from geometry_msgs.msg import PoseStamped
import moveit_commander
import tf
import math

def compute_ik( x, y, z, rz, ry, rx ):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    x,y,z,w = tf.transformations.quaternion_from_euler(rz, ry, rx)
    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w

    req = GetPositionIKRequest()
    req.ik_request.group_name = "arm"
    req.ik_request.robot_state = robot_commander.get_current_state()
    req.ik_request.pose_stamped = pose
    ik = get_ik(req)

    if ik.error_code.val==1:
        print("IK成功")
        print(ik.solution.joint_state.name)
        print(ik.solution.joint_state.position)
        return ik.solution.joint_state.position[:7]
    else:
        print("IK失敗")
        return None

def main():
    # 初期位置へ移動
    arm.set_joint_value_target( [0, -0.3, 0, -2.5, 0, 1.2, 1.57] )
    arm.go()

    # IKを解く
    joints = compute_ik( 0.45, 0, 0.2, math.pi/2, 0, math.pi/2 )

    # 移動する
    if joints!=None:
        arm.set_joint_value_target( joints )
        arm.go()

if __name__=="__main__":
    rospy.init_node("ik_test")

    # IKを計算するサービス
    get_ik = rospy.ServiceProxy("compute_ik", GetPositionIK)
    rospy.wait_for_service("/compute_ik")

    robot_commander = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")

    main()
