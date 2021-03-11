#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time
import yaml
from math import pi
import sys
import torch
import numpy as np
import matplotlib.pyplot as plt


    
def recieve_ar_makers():
    # 4つマーカーを取得
    while not rospy.is_shutdown():
        data = rospy.wait_for_message( "/ar_marker_rec/object_info", String )
        #print( data.data )

        data = yaml.load(data.data)

        print(len(data))

        if len(data)>=4:
            marker_pos = []

            for l in range(4):
                for d in data:
                    if d["label"]==l:
                        marker_pos.append( d["position"] )
            
            if len(marker_pos)==4:
                break

    return marker_pos

def estimate_transform( pos_from_cam, pos_from_arm ):
    pos_from_cam = torch.tensor( pos_from_cam ).reshape( -1, 3, 1 )
    pos_from_arm = torch.tensor( pos_from_arm ).reshape( -1, 3, 1 )
    N = pos_from_arm.shape[0]

    #print(pos_from_cam)
    #print(pos_from_arm)
    #return

    rx = torch.tensor([ -1.57 ], requires_grad=True)
    ry = torch.tensor([ 3.14 ], requires_grad=True)
    rz = torch.tensor([ -1.66 ], requires_grad=True)

    x = torch.tensor([ 0.0 ], requires_grad=True)
    y = torch.tensor([ 0.0 ], requires_grad=True)
    z = torch.tensor([ 0.0 ], requires_grad=True)

    optimizer = torch.optim.Adam( [rx, ry, rz, x, y, z] )

    loss_list = []
    for i in range(8000):
        Rx = torch.eye( 3, 3 )
        Rx[1,1] = torch.cos(-rx)
        Rx[1,2] = -torch.sin(-rx)
        Rx[2,1] = torch.sin(-rx) 
        Rx[2,2] = torch.cos(-rx)

        Ry = torch.eye( 3, 3 )
        Ry[0,0] = torch.cos(-ry)
        Ry[0,2] = torch.sin(-ry)
        Ry[2,0] = -torch.sin(-ry) 
        Ry[2,2] = torch.cos(-ry)

        Rz = torch.eye( 3, 3 )
        Rz[0,0] = torch.cos(-rz)
        Rz[0,1] = -torch.sin(-rz)
        Rz[1,0] = torch.sin(-rz) 
        Rz[1,1] = torch.cos(-rz)

        T = torch.zeros(3,1)
        T[0,0] = x
        T[1,0] = y
        T[2,0] = z

        pos_pred = Rx.matmul(Ry.matmul(Rz.matmul(pos_from_arm-T)))
        loss = torch.sqrt( 1/N * torch.sum( (pos_from_cam-pos_pred)**2 ) )

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        loss_list.append(loss)

        if i%200==0:
            print(loss)


    return [ float(v) for v in (x, y, z, rx, ry, rz)]

def main():
    rospy.init_node("camera_calibration")

    if len(sys.argv)<3:
        print( "引数に4点マーカーの中心位置のxy座標を指定" )
        print( "例：python camera_calibration.py x y" )
        return 
    else:
        cx = float(sys.argv[1])
        cy = float(sys.argv[2])
        pos_from_arm = torch.Tensor([
            [cx+0.05, cy+0.05, 0],
            [cx+0.05, cy-0.05, 0],
            [cx-0.05, cy+0.05, 0],
            [cx-0.05, cy-0.05, 0]
        ])

    pos_from_cam = recieve_ar_makers()
    print( pos_from_cam )
    x,y,z,rx,ry,rz = estimate_transform( pos_from_cam, pos_from_arm )
    print("****** ターミナルで以下のコマンドを実行 ******")
    print("rosrun tf static_transform_publisher %.4f %.4f %.4f %.4f %.4f %.4f /base_link /camera_link 100"%(x,y,z,rz,ry,rx) )
    print("**********************************************")

 
        
if __name__ == '__main__':
    main()
