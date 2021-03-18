# Sciurus17

Sciurus17をros noeticで動かす例（実機では未検証）

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
```

- そのままだとros noeticではエラーになるのでURDFを入れ替える
```
roscd sciurus17_description/urdf/
mv sciurus17.urdf.xacro sciurus17.urdf.xacro.bak
wget https://github.com/naka-lab/ros_manipulation/raw/main/noetic/sciurus17.urdf.xacro
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
