# Crane X7

[Crane X7](https://rt-net.jp/products/crane-x7/)を動かす例

## 準備
- Crane X7のパッケージをダウンロード
```
cd ~/catkin_ws/src
git clone https://github.com/rt-net/crane_x7_ros.git
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
git clone https://github.com/rt-net/crane_x7_description
rosdep install -r -y -i --from-paths .
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
sudo chmod 666 /dev/ttyUSB0
pip install pyassimp
```

- catkin_makeで`AttributeError: module 'em' has no attribute 'RAW_OPT'`というエラーが出る場合，emptyをダウングレードする
  ```
  pip install empy==3.3.4
  ```

- サンプル類をダウンロード
```
cd ~/catkin_ws/src
git clone https://github.com/naka-lab/ros_manipulation.git
```

## 実行
- 実機の場合
  ```
  roslaunch crane_x7_bringup demo.launch fake_execution:=false
  ```
  ※ロボットが動かない場合は，`sudo chmod 666 /dev/ttyUSB0`を実行する

## サンプル
- [位置指定](scripts/cranex7_move_to_position.py)
  - 前に置いてあるものを掴んで左に置く

- [動作の記録と再生](scripts/cranex7_teach_and_play.py)
  - 10秒間動作を記録して，そのまま再生

- [トルクのオン・オフと関節角度・手先位置確認プログラム](scripts/cranex7_enable_torque.py)
  - トルクオン：`rosrun ros_manipulation cranex7_enable_torque.py 1`
  - トルクオフ：`rosrun ros_manipulation cranex7_enable_torque.py 0`

- カメラに映った物体の把持
  - [カメラキャリブレーション](README_CAMERACALIB.md)を実行
  - ARマーカー認識を実行
  ```
  rosrun ros_vision ar_marker_recognition.py 
  ```
  - [ARマーカー](https://github.com/naka-lab/ros_vision/tree/master/scripts/ARMarker)を貼り付けた物体をアームの届く範囲に置く
  - [物体把持プログラム](scripts/cranex7_grasp_object.py)を実行（初期状態では0番のマーカーを掴むようになっています）

## トラブルシューティング
- ハンドが高負荷で落ちる場合
  - Craneのノードを立ち上げた状態で`rosrun rqt_reconfigure rqt_reconfigure`を実行
  - `crane_x7/crane_x7_control/crane_x7_gripper_finger_a_joint`にある`current_limit`の値を小さくする
  - [プログラム](scripts/cranex7_param_reconfigure.py)からも変更可能
