#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
import time
import dynamic_reconfigure.client
import sys


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

def show_help():
    print("エラー ------------------------")
    print("＊トルクをOnにする場合")
    print("  rosrun ros_manipulation cranex7_enable_torque.py 1")
    print("＊トルクをOffにする場合")
    print("  rosrun ros_manipulation cranex7_enable_torque.py 0")
    print("------------------------------")

def main():
    if len(sys.argv)!=2:
        show_help()
        return

    if sys.argv[1]=="1":
        input("Enterを押すとトルクをOnにします")
        # トルクをオン
        enable_trque( True )
    elif sys.argv[1]=="0":
        input("Enterを押すとトルクをOffにします")
        # トルクをオン
        enable_trque( False )
    else:
        show_help()

    while not rospy.is_shutdown():
        j = arm.get_current_joint_values()
        p = arm.get_current_pose()
        print( "関節角： %.2f %.2f %.2f %.2f %.2f %.2f %.2f"% tuple(j) )
        print( "位置姿勢：%.2f %.2f %.2f %.2f %.2f %.2f %.2f" % (p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w) )
        print("---------------")
        time.sleep(0.1)

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

    


