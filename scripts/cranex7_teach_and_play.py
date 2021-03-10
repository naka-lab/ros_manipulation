#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
import time
import dynamic_reconfigure.client
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib


def enable_trque( enable=True ):
    def send_gain( pgain ):
        for client in param_clients:
            client.update_configuration( {"position_p_gain":pgain, "position_i_gain":0,"position_d_gain":0} )

    if enable:
        # いきなり動きだすので，今の姿勢を目標姿勢にする
        arm.set_joint_value_target( arm.get_current_joint_values() )
        arm.go()
        gripper.set_joint_value_target(gripper.get_current_joint_values())
        gripper.go()

        # 徐々にトルクをもとに戻す
        send_gain(100)
        time.sleep(1)
        send_gain(400)
        time.sleep(1)
        send_gain(800)
    else:
        send_gain(1)

    time.sleep(1)

def teach():
    # JointStateを10Hzで10秒間記録
    r = rospy.Rate(10) 
    joint_states = []
    for i in range(100):
        js = rospy.wait_for_message( "/crane_x7/joint_states", JointState )
        print(i ,":" ,js.position)
        joint_states.append( js )
        r.sleep()
    return joint_states


def play( joint_states ):
    action_client = actionlib.SimpleActionClient( "/crane_x7/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction )
    action_client.wait_for_server()

    # JointStateをFollowJointTrajectoryGoalに変換して送信
    N = len(joint_states)
    joint_goal = FollowJointTrajectoryGoal()
    joint_goal.trajectory.joint_names = joint_states[0].name[1:]

    for i in range(N):
        jp = JointTrajectoryPoint( positions=joint_states[i].position[1:], time_from_start=rospy.Duration((i+1)*0.1))
        joint_goal.trajectory.points.append( jp )
    
    action_client.send_goal( joint_goal )
    action_client.wait_for_result()

def set_init_pose():
    arm.set_joint_value_target( [0, -0.5, 0, -2.0, 0, 0.9, -1.57] )
    arm.go()

def main():
    # 初期姿勢に移動
    set_init_pose()

    input("Enterを押すと脱力し，10秒間動作を記録します．")

    # トルクをオフ
    enable_trque( False )

    # 関節角を取得
    joint_states = teach()

    # トルクをオン
    enable_trque( True )

    input("Enterを押すと記録した動作を再生します．")

    # 初期姿勢
    set_init_pose()

    # 動作再生
    play( joint_states )

if __name__ == '__main__':
    rospy.init_node("teach_and_play")

    joint_names = [
        "/crane_x7/crane_x7_control/crane_x7_shoulder_fixed_part_pan_joint", 
        "/crane_x7/crane_x7_control/crane_x7_shoulder_revolute_part_tilt_joint", 
        "/crane_x7/crane_x7_control/crane_x7_upper_arm_revolute_part_twist_joint", 
        "/crane_x7/crane_x7_control/crane_x7_upper_arm_revolute_part_rotate_joint", 
        "/crane_x7/crane_x7_control/crane_x7_lower_arm_fixed_part_joint", 
        "/crane_x7/crane_x7_control/crane_x7_lower_arm_revolute_part_joint", 
        "/crane_x7/crane_x7_control/crane_x7_wrist_joint", 
        "/crane_x7/crane_x7_control/crane_x7_gripper_finger_a_joint"
    ]

    # gain設定用のdynamic reconfigure
    param_clients = [ dynamic_reconfigure.client.Client(name, timeout=10 ) for name in joint_names ]

    # robotの準備
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    main()

    


