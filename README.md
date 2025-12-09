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
## `micro_ros_agent`, `gunner`, `cpp_pingpong` 全部実行

```
ros2 launch pingpong_gunner gunner.launch.py
```

## 一つづつ実行
### `micro_ros_agent`
```
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyACM0 -v6
```

### `gunner`
```
ros2 run pingpong_gunner pingpong_gunner
```

# メモ
build_depend をいくつか追加したほうが良い？

