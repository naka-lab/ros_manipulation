# CRANE+

[CRANE+](https://rt-net.jp/products/cranev2/)をROS noeticを動かす例

## 準備
- ROSパッケージ
```
sudo apt-get install ros-noetic-ros-controllers
sudo apt-get install ros-noetic-moveit*
pip install pyassimp pyserial

cd ~/catkin_ws/src
git clone https://github.com/naka-tomo/crane_plus_arm.git
git clone https://github.com/naka-tomo/dynamixel_motor.git
cd ~/catkin_ws/
catkin_make
```

- [DynamixelのGUIツール](http://www.robotis.com/service/download.php?no=1671)：サーボモータの接続確認，パラメータ確認に使用
```
sudo chmod 775 DynamixelWizard2Setup_x64
./DynamixelWizard2Setup_x64
sudo usermod -aG dialout <your_account_id>
reboot
```

## ノード実行
- 実機の場合
```
roslaunch crane_plus_hardware start_arm_standalone.launch
roslaunch crane_plus_moveit_config robot.launch config:=true
```

- シミュレーターの場合
```
roslaunch crane_plus_moveit_config sim.launch
```

## サンプル
- [位置指定](scripts/cranep_move_to_position.py)
  - 前に置かれたものを掴んで，右側へ移動
  
- カメラに映った物体の把持
  - [カメラキャリブレーション](README.md)を実行
  - ARマーカー認識を実行
  ```
  rosrun ros_vision ar_marker_recognition.py 
  ```
  - [ARマーカー](https://github.com/naka-lab/ros_vision/tree/master/scripts/ARMarker)を貼り付けた物体をアームの届く範囲に置く
  - [物体把持プログラム](scripts/cranep_grasp_object.py)を実行
