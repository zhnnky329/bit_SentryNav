# IRobot_sentrynav27

从 SentryNav2026_XDU 提取的平地训练导航工作区。已完成静态迁移检查及 bringup 配置包构建；完整导航构建和实车验证尚未完成。

## 迁移范围

- 感知：Livox 驱动、Super-LIO（包括 basic）、cpp_lidar_filter。
- 导航：sentry_navigation 的 LIO TF 桥接和 Nav2 编排、goal_approach_controller。
- 配置：精简后的 bringup。
- 不包含 Lightning-LM、D435、Odin、PriAdaptiveMppi、目标追踪、越线层、队友层、旧决策和 CAN 通信。

全局规划为 SmacPlanner2D。局部控制为 GoalApproachController 包装官方 MPPI，参数来自旧 AdaptiveMppi 的 normal 配置，包含接近目标限速和末端直驱。没有上下坡参数切换。实时障碍物继续使用 STVL，仅观察 Livox 点云。

## 环境与构建

源项目面向 Ubuntu 22.04 + ROS 2 Humble，当前整理环境为 Ubuntu 24.04 + Jazzy。源码迁移不代表跨版本兼容验证。按部署环境选择发行版，不要混用旧 install 或 build。

需要对应发行版的 Nav2、STVL、PCL、Eigen、glog 和 Livox SDK2。Livox SDK2 是额外系统依赖，rosdep 不一定能安装。

```bash
# Ubuntu 24.04 使用 jazzy；Ubuntu 22.04 使用 humble。
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch sentry_navigation start.launch.py map:=/绝对路径/训练场/map.yaml
```

不附带旧比赛场地图。map.yaml 中的 image 应引用同目录地图图像或正确绝对路径。

## 上车前必须核对

- `src/bringup/config/lidar.yaml` 中的安装外参、点云过滤包围盒、Livox 网络配置来自旧车，必须按新车标定。base_link 约定为大 yaw 系。
- `src/bringup/config/nav2_params.yaml` 中的车体半径/轮廓同样是旧值，不可当作实车尺寸。需覆盖大 yaw 转动时的真实外形。
- 当前是纯 LIO 里程计：map 与 odom 静态重合，首帧归零。启动位姿必须与地图约定一致；未实现任意起点自动重定位或持续地图漂移修正。
- 保留 Super-LIO 自带 relocation 源码，但它不是当前启动路径，也没有验证为可用的地图定位替代方案。
- 当前控制参数沿用原 normal 的角速度配置。后续 CAN 仅执行 vx/vy 时，必须先验证控制器的预测与实际大 yaw 运动一致，不可仅丢弃 angular.z 就认为控制闭环匹配。禁止在未完成这一项和底盘急停验证前接通自主运动。
- 默认不启动 CAN 和旧决策，不产生底盘硬件下发。
- 若出现近点绕圈或障碍清除异常，分别检查控制器和实时障碍层，不同时替换多条链路。

## 原文件与变更

除下列迁移适配，所选包的算法源码按原样保留：sentry_navigation 去掉 Odin/Lightning-LM 桥接节点及裁判依赖；TF monitor 仅检查 Livox 链；导航地图改为必填参数；新增 sentry_navigation 基础入口，以及 bringup 参数和固定插件执行树。GoalApproachController 的 package.xml 补充其实际动态加载的 MPPI 依赖；RViz 移除旧自适应面板、越线 Marker 及未提供的 RobotModel/LaserScan 显示。

原仓库保留不动，第三方包原有许可证一并保留。尚未创建 Git 仓库或提交。
