# bit SentryNav 独立 Jazzy 适配

本工作区来自用户 fork `zhnnky329/bit_SentryNav`，基线 `8288b7520ecb635bb27f16b0d49e3cf304f37a72`，目标 Ubuntu 24.04 / ROS 2 Jazzy。已写入独立感知、重定位、MINCO/Nav2 和可选 SocketCAN 接口；尚未构建、运行 ROS 或完成实车验收。旧导航工程未改动。

## 本车 TF 与速度定义

```text
map → odom → base_link → livox_frame
```

- `map → odom`：GICP 重定位提供地图修正，首次收敛前不发布。
- `odom → base_link`：Point-LIO 原始 IMU 位姿经内部外参与安装外参组合，补偿杆臂后得到。
- `base_link → livox_frame`：固定安装外参。base_link 原点在大云台旋转轴，与底盘运动中心重合，朝向随大云台。

base_link 不表示底盘独立朝向。用户确认电控负责大云台到底盘转换，上位机只提供 base_link 的 vx/vy。独立运行不再发布 camera_init、body、nav_base、gimbal_link 或 gimbal_heading 的 TF。

Point-LIO 的世界点云、路径和原始 odom 使用 `odom`；内部原始 odom 的 child 为 `livox_imu`，只供数值转换及 ROGMap 传感器位置使用，不单独发布该 TF。IMU 系点云关闭，避免把它错误标记为雷达系。Point-LIO 原生 TF 关闭，四级链由 GICP 与 bit_base_odometry 唯一发布。

`/odom` 的 pose 在 odom 系，twist 在 base_link 系；保留真实云台角速度，不把它伪造为零。转换后的位姿与 TF 使用原始测量时间，不额外重置启动位置，确保与 Point-LIO 世界点云一致。

| 话题 | 含义 |
| --- | --- |
| /aft_mapped_to_init | 内部 IMU 原始里程计，供转换和 ROGMap 使用 |
| /odom | 运动中心位姿及 base_link 系速度，供规划与控制使用 |
| /bit/cmd_vel_world | ControllerServer 输出，线速度沿 odom 坐标轴 |
| /bit/cmd_vel_base | TwistStamped，按完整 TF 旋转得到的 base_link 速度；CAN 只取 x/y，角速度为零 |
| /bit/cmd_vel_mpc_preview | MPC 私有调试输出，不能替代 ControllerServer 最终输出验收 |
| /bit/can_0x190_payload_preview | 实际发送所用编码的 8 字节预览 |

MPC 原有按角速度进行的输出延时旋转在独立配置中设为零，避免对世界速度额外偏转。世界到 base_link 只在适配器转换一次，底盘旋转留给电控。完整旋转使消息与 TF 一致，但二维规划与 CAN x/y 接口的坡面行为尚未验证。

速度适配器采用最近有效姿态，输入/里程计超过 0.1 s 或出现非有限值时输出零。输出时间戳为生成时间，不是姿态采样时间；输入 Twist 无时间戳，尚非严格同刻同步，转动时的时延影响需实测。

## 车辆配置

| 项目 | 当前值 |
| --- | --- |
| MID360 / NUC 雷达网口 | 192.168.1.114 / 192.168.1.50 |
| 雷达安装 | 旋转轴前方 0.22 m；俯仰 45°、高度及零向沿用旧配置 |
| 轮廓 | 沿用旧 ±0.10 m 方形的外接圆，半径 0.1414213562 m；不依赖未知底盘朝向 |
| 速度比例 | 用户指定暂按 ±127 ↔ ±1 m/s，未标定 |
| 预置运动参数 | MPC x/y 各限 ±0.3 m/s，MINCO 最大速度 0.3 m/s；不控制 yaw |

安装外参见 `bit_standalone/config/robot.yaml`；网络地址见 `config/MID360_config.json`；轮廓/控制参数见 `config/nav2_jazzy.yaml`；发送参数见 `config/communication.yaml`。程序不修改网口配置。

0.1414 m 半径小于雷达离中心的 0.22 m，不能代表包含雷达/支架的真实整车外包络。实车前须核对尺寸，不能用沿用旧配置代替测量。

## CAN 接口与电控待处理项

已实现标准 CAN ID `0x190`、DLC 8：byte0/1 分别为 `trunc(clamp(vx或vy / speed_at_full_scale_mps, -1, 1) × 127)` 的 int8；其余六字节为零。当前固件导航分支不使用 byte2/3 的 yaw 字段，不在这里传角速度、自动模式或虚假的其他模块心跳。

发送默认关闭，只有 `mode:=live enable_can:=true` 才启动。回放模式禁止开启。SocketCAN 使用现有 can0，默认 500 Hz 沿用原工程；需要系统事先配置正确 CAN 波特率并启用网口。输入缺失、超时或无效时连续发零；发送错误会报错退出，退出尝试发一帧零。未实际打开 CAN 或发送过任何报文。

**按提供的电控源码，需要核对以下两项；这些是代码分支分析，尚未确认实际测试模式和在线检测行为：**

1. `app/targets/sentry/BigGimbal.cc:231–271` 的导航分支在 NUC 离线时先清零 source，随后仍用缓存导航 vx/vy 覆盖。电控应按导航 0x190 自身在线状态分支，超时后直接保持零，不再读取旧速度；建议约定接收超时 100 ms 后清零并交给底盘停车。实际已有超时阈值位于未包含的 librm 子模块，须电控确认。
2. `BigGimbal.cc:132–142,306–307` 与 `main.cc:129`：device_nuc 包含自瞄与导航通信对象；在 Match 模式下，all_device_ok() 为假会切入慢速自旋。未启动自瞄算法不必然等于自瞄通信离线，其他模式也不能直接套用此结论。需电控确认纯导航测试使用的模式、实际在线状态，以及是否需要调整此分支；这尚未被证实为先前绕终点的原因。

无需新增云台角度反馈即可完成本次速度接口。可选增加反馈：当前手动/自动模式、导航在线标志、实际底盘 vx/vy，便于后续核对接管和速度比例；这些不是当前软件所依赖的已有字段。

## 独立启动

`standalone.bash` 清除继承环境，只加载 `/opt/ros/jazzy/setup.bash` 和本仓 install_standalone。默认 ROS_DOMAIN_ID=73，可用 BIT_ROS_DOMAIN_ID 指定；同一次运行的终端应使用相同域。build/install/log 均使用 `_standalone` 后缀。

构建选择 bit_standalone、navi2、point_lio、icp_relocalization、minco_planner、minco_controller 及依赖；不加入上游通信、决策或 Humble/Fortress 仿真。系统需提供 Nav2/STVL/PCL/Eigen/yaml-cpp/OpenMP、Python numpy/scipy/yaml、Livox-SDK2 和 small_gicp。原仓依赖清单不完整，rosdep 成功不代表构建成功。

在 Ubuntu/Jazzy 独立目录执行：

```bash
bash build.bash
bash standalone.bash check
```

录包感知验证（两终端）：

```bash
bash standalone.bash perception mode:=replay rviz:=true
bash standalone.bash bag /absolute/path/to/rosbag_directory
```

只播放 /livox/lidar、/livox/imu 和 /clock，不播放旧 TF/odom/命令。在线感知可使用 mode:=live。

导航需显式指定同场地的 map YAML、PCD 和 `initial_pose`。initial_pose 为 **odom 在 map 中的初始猜测** `[x,y,z,roll,pitch,yaw]`，角度单位 rad，不是 base_link 目标点。本机 `local_maps/20261124_snapshot/` 为独立地图副本，不纳入 Git，换机器须复制或另行指定。

```bash
bash start.bash mode:=live map:="$SCENE_MAP" pcd:="$SCENE_PCD" initial_pose:="$INITIAL_POSE"
```

该命令仅启动传感器、定位、导航及预览；电控上述条件核对并处理、Jazzy 验证通过并具备现场手动接管条件后，才在同一命令增加 `enable_can:=true`。独立入口只支持 RViz 单目标 NavigateToPose，不包含原比赛决策、多目标巡航或运动恢复。

## 验证与未完成项

离线检查覆盖外参组合、杆臂补偿、纯云台转动中心不动、边平移边转动、四级 TF 重建雷达位置、世界/机体系速度、CAN 有符号编码及超时置零。静态检查覆盖 Python/XML/JSON/YAML/RViz、启动资源与配置、Bash 和 diff。

尚未执行 Jazzy 构建、插件加载、录包算法回放、SocketCAN 实发或实车导航。当前 macOS 没有 ROS，最近一次 NUC 连接超时。GICP 首次收敛后的 TF 保持不等于持续定位质量保证；MINCO 登记前的 cancel_checker 也不能证明异步轨迹已取消。

验收需实际确认：四级 TF 单一发布、地图点云对齐、云台原地转动时中心不漂移、到达/取消后 ControllerServer/最终速度/CAN 同时归零、手动接管有效、导航断流停车、纯导航模式不自旋及速度比例正确。仅 MPC 私有调试话题归零不足以证明停车。
