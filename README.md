# pingpong_gunner

# 環境構築
### ビルドに必要なパッケージの取得
```
pip install catkin_pkg cython
```

## ビルド＆インストール
```
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
~/pingpong_ws/venv/bin/colcon build --symlink-install
source $HOME/pingpong_ws/install/setup.bash
```

## micro-ROS Agent のビルド（必須）
```
cd ~/pingpong_ws
ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup build_firmware.sh
source $HOME/pingpong_ws/install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source $HOME/pingpong_ws/install/local_setup.bash
```

# 実行
## 1. `micro_ros_agent` を起動

```
ros2 launch pingpong_gunner agent.launch.py serial:=/dev/ttyACM0
```

`serial:=/dev/ttyACM0` は接続する Raspberry Pi Pico のデバイス

## 2. `pingpong_gunner` を起動
```
ros2 run pingpong_gunner pingpong_gunner_exe
```

※ 本当は launch ファイルにまとめたかったけどまとめると何故か通信できない

## 3. 卓球ロボットの回路 （ Raspberry Pi Pico ） に接続
必ず `micro_ros_agent` を起動したあとに接続すること！

# メモ
build_depend をいくつか追加したほうが良い？

