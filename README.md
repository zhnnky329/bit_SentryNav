# IRobot-Nav27

## 实车定位与导航

在 [bringup.yaml](src/bringup/config/bringup.yaml) 中设置二维地图路径 `grid_map_yaml`、点云地图路径 `pcd_map`、重定位开关 `use_relocalizer` 和 RViz 开关 `use_rviz`。两个开关默认开启。

```zsh
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-up-to bringup
source install/setup.zsh
ros2 launch bringup start.launch.py
```

入口启动 MID-360、滤波、Super-LIO、TF 桥接和重定位。地图和 RViz 随入口启动，TF 就绪后拉起 Nav2 导航服务。底盘通信由车机单独管理。Nav2 参数、雷达外参和 RViz 配置位于 [bringup/config](src/bringup/config)。

调试时可在 YAML 中设置 `use_relocalizer: false` 使用纯里程计模式，此时只需提供二维地图，但需要手动指定初始位姿。
