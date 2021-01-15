#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import JointState 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_controllers.srv import TorqueEnable
from control_msgs.msg import GripperCommandActionGoal
import time

def set_pose( j0, j1, j2, j3, j4, duration):
    # アームの関節角を送信
    jt = JointTrajectory()
    jt.joint_names = joint_names[:4]
    jp = JointTrajectoryPoint( positions=[j0, j1, j2, j3], 
    time_from_start=rospy.Duration(duration),
    #velocities=[0.5, 0.5, 0.5, 0.5],
    #accelerations=[0.5, 0.5, 0.5, 0.5]
    )
    jt.points.append( jp )
    pub_arm.publish( jt )

    # グリッパーの角度を送信
    gg = GripperCommandActionGoal()
    gg.goal.command.position = j4
    pub_gripper.publish( gg )


def teach():
    set_torque_srv_names = [
        "shoulder_flex_servo_controller/torque_enable",
        "shoulder_revolute_servo_controller/torque_enable",
        "elbow_servo_controller/torque_enable", 
        "finger_servo_controller/torque_enable",
        "wrist_servo_controller/torque_enable"
    ]

    set_trque_srvs = []
    for srv in set_torque_srv_names:
        rospy.wait_for_service( srv )
        set_trque_srvs.append( rospy.ServiceProxy(srv, TorqueEnable) )


    # トルクをOFF
    for i in range(5):
        set_trque_srvs[i](False )


    # joint_statesを2Hz, 10秒間記録
    joint_states = { n:0 for n in joint_names }
    sub = rospy.Subscriber( "/joint_states", JointState, lambda x: joint_states.update( {x.name[0]:x.position[0]} ))

    joint_states_samples = []
    r = rospy.Rate(2) 
    for i in range( 20 ):
        joint_states_samples.append( joint_states.copy() )
        r.sleep()

    sub.unregister()

    # トルクをOn
    for i in range(5):
        set_trque_srvs[i](True )

    return joint_states_samples

def play( joint_states_samples ):
    r = rospy.Rate(2)
    for i in range( 20 ):
        positions = [ joint_states_samples[i][n] for n in joint_names ] 
        """
        j0 = js["crane_plus_shoulder_revolute_joint"]
        j1 = js["crane_plus_shoulder_flex_joint"]
        j2 = js["crane_plus_elbow_joint"]
        j3 = js["crane_plus_wrist_joint"]
        j4 = js["crane_plus_moving_finger_joint"]
        j4 = -j4/6.5
        """
        positions[4] = -positions[4]/6.5
        set_pose( *positions, 0.5)
        r.sleep()
    

def main():
    print("初期姿勢へ移動")
    set_pose(0, 0, 0, 0, 0, 3)
    time.sleep(3)

    print("teacing開始")
    joint_state_samples = teach()

    print("play")
    play( joint_state_samples )


if __name__ == '__main__':

    joint_names = [
        "crane_plus_shoulder_revolute_joint",
        "crane_plus_shoulder_flex_joint",
        "crane_plus_elbow_joint", 
        "crane_plus_wrist_joint",
        "crane_plus_moving_finger_joint"
    ]

    rospy.init_node("teach_and_play")
    pub_arm = rospy.Publisher( "/crane_plus/command", JointTrajectory, queue_size=10 )
    pub_gripper = rospy.Publisher( "/crane_plus_gripper/gripper_command/goal", GripperCommandActionGoal, queue_size=10 )
    time.sleep(0.5)

    main()