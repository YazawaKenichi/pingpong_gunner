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

# 実行
```
ros2 launch pingpong_gunner gunner.launch.py
```

これで `micro_ros_agent`, `gunner`, `cpp_pingpong` が起動

# メモ
build_depend をいくつか追加したほうが良い？

