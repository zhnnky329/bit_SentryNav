# 启动指南

在 NUC 上使用 Ubuntu 24.04 / ROS 2 Jazzy。以下命令均在 `bit_SentryNav` 仓库根目录执行。依赖和配置见[独立适配说明](src/standalone/README.md)。

## 1. 编译

```bash
bash build.bash
bash standalone.bash check
```

## 2. 在线导航预览

地图已随 `bit_standalone` 安装：`src/standalone/bit_standalone/map/{map.yaml,map.pgm,map.pcd}`，启动时无需再传路径。`relocalizer` 会用 PGM/YAML 与 PCD 自动全局定位，第一次验证成功后将结果锁存为 `map -> odom`，之后停止匹配并依靠里程计递推。启动后观察状态，`state` 变为 `TRACKING` 或 `FROZEN` 即表示已输出定位结果：

```bash
bash start.bash mode:=live enable_can:=false rviz:=true
```

```bash
bash standalone.bash shell
ros2 run tf2_ros tf2_echo map odom
```

`/relocalizer/status` 只在定位状态变化时发布；若启动后才订阅可能错过唯一的 `FROZEN` 消息。需要重新定位时释放锁存并再搜一次：

```bash
ros2 service call /relocalizer/relocalize std_srvs/srv/Trigger "{}"
```

换场地时仍可用 `map:=/绝对路径/map.yaml pcd:=/绝对路径/map.pcd` 覆盖内置地图。默认雷达 IP 为 `192.168.1.114`，NUC 雷达网口 IP 为 `192.168.1.50`。等 RViz 中点云与地图对齐，再用 **Nav2 Goal** 发目标。此命令关闭 CAN 发送，速度预览可在另一终端查看：

```bash
bash standalone.bash shell
ros2 topic echo /bit/cmd_vel_base
```

## 3. 录包回放

将上面的 `mode:=live` 改为 `mode:=replay`，另开一个终端播放录包：

```bash
bash standalone.bash bag /绝对路径/录包目录
```

回放包中的 `/livox/lidar` 需为 `livox_ros_driver2/msg/CustomMsg`。若录包只有 `sensor_msgs/msg/PointCloud2` 点云（例如旧系统的 `/livox/lidar/pointcloud`），给导航 launch 追加：

```bash
relocalizer_input_topic:=/livox/lidar/pointcloud relocalizer_input_format:=pointcloud2
```

此时 Point-LIO 本身也需要能接收该点云话题，否则仅重定位链路可用、里程计不会更新。
