# Sciurus17

Sciurus17をros noeticで動かす例

## 準備
- インストール
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/sciurus17_ros.git
  git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
  sudo atp-get update
  rosdep install -r -y --from-paths . --ignore-src
  cd ~/catkin_ws
  catkin_make
  source ~/catkin_ws/devel/setup.bash

  - roscd sciurus17_tools/scripts/
  - ./create_udev_rules
  ```
  インストール後PCを再起動する

- そのままだとros noeticではエラーになるのでURDFを入れ替える
  ```
  roscd sciurus17_description/urdf/
  mv sciurus17.urdf.xacro sciurus17.urdf.xacro.bak
  wget https://github.com/naka-lab/ros_manipulation/raw/main/sciurus17/sciurus17.urdf.xacro
  ```

- [画像認識](https://github.com/naka-lab/ros_vision)を利用するために，relasenseのlaunchファイルを入れかえる
  ```
  roscd sciurus17_vision/launch/
  mv realsense.launch realsense.launch.bak
  wget https://github.com/naka-lab/ros_manipulation/raw/main/sciurus17/realsense.launch
  ```

## 実行
- rvizのみ
```
roslaunch sciurus17_bringup sciurus17_bringup.launch
```

- gazebo
```
roslaunch sciurus17_gazebo sciurus17_with_table.launch
```

## サンプル
- [座標指定移動](scripts/sciurus_move_to_point_left.py)
- [物体把持](scripts/sciurus_grasp_object.py)
  - ARマーカー認識を実行
  ```
  rosrun ros_vision ar_marker_recognition.py /camera/depth_registered/points:=/sciurus17/camera/depth_registered/points
  ```
  （SciurusではRealsenseのトピック名が変わっているのでトピック名を変更している．他の物体認識系のプログラムも上記のようにトピック名を変えると使用可能．）
  - [ARマーカー](https://github.com/naka-lab/ros_vision/tree/master/scripts/ARMarker)を貼り付けた物体をアームの届く範囲に置く
  - [物体把持プログラム](scripts/sciurus_grasp_object.py)を実行（初期状態では0番のマーカーを掴むようになっています）
