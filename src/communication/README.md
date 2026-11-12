# rm_referee_ros2

ROS 2 裁判系统通信工具包，提供串口接收与发送、原始数据录制与回放、ROS 消息、裁判位置 TF 和 rqt 测试界面。

本分支适配 **《RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0（20261029）》**。代码基于 XDU-IRobot/rm_referee_ros2，按本地 `~/rm_referee_ros2` 工作副本同步通信实现；mock 同步自 `~/integrate/SentryNav2026_XDU/src/rm_referee_mock`。历史协议定义保留，运行节点与回放扫描器均使用 `kRm2027V100`。

## 包与数据流

| 包 | 功能 |
| --- | --- |
| `rm_referee_msgs` | 裁判 ROS 消息及 `Tx` 服务定义 |
| `rm_referee` | 常规/图传链路解帧、话题发布、常规串口发送、录制回放、位置 TF |
| `rm_referee_mock` | Match Control 等 rqt 插件，直接构造 ROS 消息供离线测试 |
| `rm_referee/libs/librm` | Git 子模块；字节流组帧、CRC 校验、命令映射和协议数据结构 |

```mermaid
flowchart LR
  A[裁判系统串口] --> B[RefereeNode 读线程]
  B --> C[Referee 字节流解帧]
  C --> D[PublishMsg]
  D --> E[/rm_referee/*]
  R[原始录包回放] --> C
  M[Match Control] --> E
  E --> F[外部导航包 TopicListener / 黑板]
  F --> G[上层行为树]
  G --> H[Nav2 NavigateToPose]
  G --> I[/rm_referee/tx]
  I --> A
```

mock 直接发布消息，不验证串口和解帧。行为树、Nav2、PB Gazebo 是外部包，不由本仓库启动或实现。完整接线与占点改造方案见 [2027 全链路与行为树方案](docs/2027全链路与行为树方案.md)。

## 2027 主要接口变化

| 命令 | 本版处理 |
| --- | --- |
| `0x0003` | 24 字节；增加双方累计控制度，使用 R/B 前哨原始血量；2 号位保留 |
| `0x0101` | 8 字节；`event_data`、`event_data_2` 两个 `uint32`，包含阵地控制状态 |
| `0x0204` | 7 字节，移除旧 `remaining_energy` 字段 |
| `0x0206` | 7 字节，增加命中序号和有符号命中坐标 |
| `0x0209` | 6 字节，第二个 RFID 字段为 `uint16` |
| `0x020B` | 保持 40 字节；重装工程与保留位命名更新，保留位不解释成机器人位置 |
| `0x020E` | 8 字节；增加第二状态字节、强化区域中心与半径 |
| `0x0311` | 图传自定义控制数据扩为 60 字节 |
| `0x0301` | 支持 6～118 字节交互数据；新增 `RobotInteraction` 入站话题 |

本次还修复了静默串口退出卡住的问题：读取最多等待 100 ms 返回检查停止标志，析构先等待读线程结束，再关闭设备。

固定长度命令必须精确匹配长度；CRC 错、未知命令、长度错误的帧不发布。解码缓冲区为 512 字节，可容纳 `0x0310` 的 309 字节整帧。旧 ROS 消息类型可能为兼容而保留，不能据此认为旧命令仍在 2027 节点中发布。

`ally_outpost_hp` / `enemy_outpost_hp` 是根据本机 `robot_id` 从 R/B 血量派生的兼容字段；收到本机 ID 前为 `65535`。**前哨血量、阵地控制状态、累计控制度是三种不同信息。**

## 构建与校验

本次使用 Ubuntu 22.04、ROS 2 Humble 本地验证。其他 ROS 发行版未在本次验证。

```bash
cd ~/rm_referee_ros2_2027
source /opt/ros/humble/setup.bash
git submodule update --init --recursive
colcon build --base-paths rm_referee_msgs rm_referee rm_referee_mock \
  --packages-up-to rm_referee rm_referee_mock \
  --build-base build_2027 --install-base install_2027
source install_2027/setup.bash
ctest --test-dir build_2027/rm_referee --output-on-failure
ctest --test-dir build_2027/rm_referee_mock --output-on-failure
ros2 interface show rm_referee_msgs/msg/EventData
ros2 interface show rm_referee_msgs/msg/RobotInteraction
# 选择一个没有其他机器人/测试发布器的 DDS 域
ROS_DOMAIN_ID=186 python3 rm_referee/test/serial_ros_smoke.py
```

`ctest` 的 Python 测试依赖 `pytest`；缺少时应先安装测试依赖并重新配置，不能把“没有测试”当通过。本次结果与逐文件清单见 [验证记录](docs/2027更新验证记录.md)。

新建构建目录是为了避免移动路径后的 CMake 缓存问题。源码仍按链路分目录保存消息，安装时同时放到标准 `share/rm_referee_msgs/msg` 路径，支持 `ros2 interface show`。

`librm` 的协议改动单独提交，主仓库记录其提交号。当前为本地提交；向远端分享时，需要先发布子模块提交并确保接收方能取得它，再发布主仓库引用。

## 串口驱动

先确认设备与权限，再启动。节点默认参数与示例 launch 的参数可能不同，下面列出 **当前示例 launch**：

| 参数 | 示例值 | 说明 |
| --- | --- | --- |
| `enable_normal` | `True` | 常规链路，115200 波特率 |
| `enable_vt` | `False` | 图传链路；需要时单独启用 |
| `new_vt` | `True` | 图传启用时，VT03 使用 921600；旧图传 115200 |
| `normal_tty_device` | `/dev/ttyUSB0` | 常规链路设备 |
| `vt_tty_device` | `/dev/ttyUSB0` | 图传设备；双链路时必须指定不同的实际设备 |
| `record_raw_data` | `True` | 当前示例启用原始字节录制 |
| `raw_data_path` | `/tmp/rm_referee_data` | 自动创建；长期保存请使用持久目录 |

```bash
ros2 launch rm_referee referee_node.launch.py
ros2 topic info -v /rm_referee/event_data
ros2 topic echo /rm_referee/event_data --qos-reliability best_effort --once
```

本 launch 在 Python 中配置上述参数，修改 launch 后需要重新安装；也可直接运行节点并用 `--ros-args -p 参数:=值` 覆盖。设备访问权限优先通过 `dialout` 组或设备规则配置；每次插拔后重新 `chmod 777` 不是稳定方案。记录器保存接收字节块、长度与时间，不记录波特率等设备配置。

### 原始数据回放

```bash
ros2 launch rm_referee referee_replay.launch.py \
  normal_data_file:=/实际路径/normal_raw_data.bin \
  replay_rate:=1.0 start_game_progress:=4
```

`normal_data_file` / `vt_data_file` 选择链路；`replay_rate=0` 不等待时间间隔；`start_game_progress=-1` 从开头回放，`0～5` 定位比赛阶段。回放复用 2027 译码与发布路径，不打开串口，也不提供 `/rm_referee/tx`。保留了原仓库录包末尾零填充兼容处理。

### 话题

发布 QoS 使用 `SensorDataQoS`；订阅者应检查 Best Effort 兼容性。`header.stamp` 是本机发布时间，`GameStatus.sync_timestamp` 是协议中的同步时间。

| 话题前缀均为 `/rm_referee/` | `rm_referee_msgs/msg/` 类型 | 用途 |
| --- | --- | --- |
| `game_status`、`game_result` | `GameStatus`、`GameResult` | 比赛阶段、剩余时间、结果 |
| `game_robot_hp` | `GameRobotHP` | 血量、R/B 前哨血量、累计控制度 |
| `event_data` | `EventData` | 阵地状态、增益与其他场地事件 |
| `robot_status` | `RobotStatus` | 本机 ID、HP、功率与供电状态 |
| `power_heat_data` | `PowerHeatData` | 缓冲能量、枪口热量 |
| `robot_pos`、`ground_robot_position` | `RobotPos`、`GroundRobotPosition` | 裁判提供的位置数据 |
| `buff`、`hurt_data`、`shoot_data` | `Buff`、`HurtData`、`ShootData` | 增益、受击与射击 |
| `projectile_allowance` | `ProjectileAllowance` | 弹量、金币 |
| `rfid_status` | `RFIDStatus` | 地面交互区域、隧道检测位 |
| `sentry_info` | `SentryInfo` | 哨兵状态回显 |
| `radar_info`、`radar_mark_data` | `RadarInfo`、`RadarMarkData` | 雷达状态 |
| `referee_warning` | `RefereeWarning` | 判罚 |
| `dart_info`、`dart_client_cmd` | `DartInfo`、`DartClientCmd` | 飞镖信息 |
| `map_command` | `MapCommand` | 小地图指令 |
| `robot_interaction` | `RobotInteraction` | 发给本机的同队自定义数据 |
| `robot_custom_data_2`、`custom_control` | `RobotCustomData2`、`CustomControl` | 图传链路自定义数据 |

`robot_interaction` 只暴露子内容 ID 为 `0x0200～0x02FF`、同队且接收者等于本机 ID 的消息；必须先接收 `0x0201` 确认本机 ID。`payload` 不含前面的 6 字节交互头。雷达与哨兵之间的应用字段、坐标系、版本号、过期时间仍需双方约定。

### 发送服务

`/rm_referee/tx` 类型为 `rm_referee_msgs/srv/Tx`。请求 `data` 是**已经组成完整帧的字节数组**。服务将其写入常规串口，不代替调用者组帧、校验业务合法性或限制发送频率。`ok=true` 只表示本机写入成功。

哨兵自主决策请求使用 `0x0301` 外层命令与 `0x0120` 子内容；机器人自定义协同消息使用 `0x0200～0x02FF` 子内容。业务完成要等待后续裁判状态，不能仅凭 `Tx.ok` 判断姿态切换、复活或占点完成。

## TF Publisher

独立的 `tf_publisher` 将裁判位置发布为 `self`、`heavyengineer`、`standard_3`、`standard_4` 等可视化坐标。它不建立 LiDAR 里程计，也不维护导航的 `map→odom`。不要把其裁判位置 TF 与定位节点同时接到同一个机器人 TF 子坐标系。

![位置可视化](docs/tf_publisher.jpg)

## rqt Mock

```bash
ros2 run rm_referee_mock rqt_clean_start
```

默认打开 Match Control；也支持 `fake_location`、`keyboard`、`dart_client`。统一各终端的 `ROS_DOMAIN_ID` 和 `RMW_IMPLEMENTATION`，测试时不要同时启动真实串口和同名前缀的 mock 发布器。

![2027 Match Control](rm_referee_mock/2027_match_control_preview.png)

2027 界面提供 R/B 前哨及基地状态、累计控制度与场景按钮。先设 Robot ID 7/107，在非比赛阶段填好时间，再切到阶段 4；默认 Topic prefix 为 `/rm_referee`。界面“预计任务”不是行为树实际运行结果。

mock 的旧姿态、补给、复活规则模拟仍有简化，不能当作完整 2027 比赛服务器。详见 [mock 说明](rm_referee_mock/README.md)。

## 验收边界

自动测试覆盖协议字节布局、坏 CRC、未知命令、固定/可变长度、红蓝事件编码与 mock 规则回归。通过这些检查不等于真实串口、服务器接收、机器人过洞或 Nav2 抢占已验收。原始录包、ROS bag、黑板日志和 Gazebo 运动应分层验证。

场地侧的修复与分层测试见 [RM2027 场地仿真验证](docs/2027场地仿真验证.md)，行为树改造从 [全链路方案](docs/2027全链路与行为树方案.md) 第 6～8、11 节开始。
