# 启动指南

在 NUC 上使用 Ubuntu 24.04 / ROS 2 Jazzy。以下命令均在 `bit_SentryNav` 仓库根目录执行。依赖和配置见[独立适配说明](src/standalone/README.md)。

## 1. 编译

```bash
bash build.bash
bash standalone.bash check
```

## 2. 在线导航预览

将 `MAP_DIR` 改为实际地图目录，同一场地的 `map.yaml`、`map.pgm` 和 `map.pcd` 放在一起。`initial_pose` 输入 `odom` 在 `map` 中的初始猜测，格式为 `[x,y,z,roll,pitch,yaw]`，位置用米，角度用弧度。

```bash
MAP_DIR="$HOME/Desktop/IRobot-SentryNav27/src/bringup/map"
read -r -p '输入初始位姿 [x,y,z,roll,pitch,yaw]: ' INITIAL_POSE

bash start.bash mode:=live enable_can:=false rviz:=true \
  map:="$MAP_DIR/map.yaml" pcd:="$MAP_DIR/map.pcd" \
  initial_pose:="$INITIAL_POSE"
```

默认雷达 IP 为 `192.168.1.114`，NUC 雷达网口 IP 为 `192.168.1.50`。等 RViz 中点云与地图对齐，再用 **Nav2 Goal** 发目标。此命令关闭 CAN 发送，速度预览可在另一终端查看：

```bash
bash standalone.bash shell
ros2 topic echo /bit/cmd_vel_base
```

## 3. 录包回放

将上面的 `mode:=live` 改为 `mode:=replay`，另开一个终端播放录包：

```bash
bash standalone.bash bag /绝对路径/录包目录
```
