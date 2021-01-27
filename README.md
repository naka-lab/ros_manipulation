# ros_manipulation

## [CRANE+の場合](READE_CRANE+.md)

## カメラとアームのキャリブレーション
### インストール
```
conda install pytorch
cd ~/catkin_ws/src
git clone https://github.com/naka-lab/ros_vision.git
git clone https://github.com/naka-lab/ros_manipulation.git
```

### キャリブレーション手順
1. [キャリブレーション用ARマーカー](scripts/cabration_marker.pdf)を設置して，アームの基準座標からのxy座標を確認する
2. マーカー認識を実行
```
rosrun ros_vision ar_marker_recognition.py 
```
3. キャリブレーションプログラムを実行．引数は，手順1で確認したxy座標．
```
rosrun ros_manipulation camera_calibration.py 0.37 0
```
4. 実行後表示されたコマンドをコピーして実行．
```
rosrun tf static_transform_publisher 0.0354 -0.1414 0.1211 -1.3768 0.1132 -1.9365 /base_link /camera_link 100
```
