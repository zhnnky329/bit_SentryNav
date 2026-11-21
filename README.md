# IRobot-Nav27

## 实车定位与导航

在车机的 ROS 2 Jazzy 环境下，将同一场地的地图 YAML（及其引用的栅格图）和 PCD 路径填入 [site.yaml](src/bringup/config/site.yaml)。默认场地目录为 `~/robot_maps/training/`。相对路径从场地配置所在目录解析。

```zsh
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-up-to bringup
source install/setup.zsh
ros2 launch bringup start.launch.py
```

入口启动 MID-360、滤波、Super-LIO、TF 桥接和重定位。地图和 RViz 随入口启动，TF 就绪后拉起 Nav2 导航服务。底盘通信由车机单独管理。Nav2 参数、雷达外参和 RViz 配置位于 [bringup/config](src/bringup/config)。

调试时可用 `use_relocalizer:=false` 进入手动初始位姿模式，只加载场地的二维地图。也可以用 `site:=/绝对路径/site.yaml` 临时选择另一份配置。
