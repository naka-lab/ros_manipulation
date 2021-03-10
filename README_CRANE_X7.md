# Crane X7

[Crane X7](https://rt-net.jp/products/crane-x7/)を動かす例

## 準備
- Crane X7のパッケージをダウンロード
```
cd ~/catkin_ws/src
git clone https://github.com/rt-net/crane_x7_ros.git
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
rosdep install -r -y --from-paths --ignore-src crane_x7_ros
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
sudo chmod 666 /dev/ttyUSB0
pip install pyassimp
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

## サンプル
- [位置指定](scripts/cranex7_move_to_position.py)
  - 前に置いてあるものを掴んで左に置く

- [動作の記録と再生](scripts/cranex7_teach_and_play.py)
  - 10秒間動作を記録して，そのまま再生
