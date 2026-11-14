# sentry_nav_bt

`sentry_nav_bt` 是基于 ROS 2、BehaviorTree.CPP 和 Nav2 的哨兵导航决策包。节点接收裁判系统状态和 TF 位姿，从 JSON 加载命名路径点，通过行为树组织导航、巡逻、回补、复活、打符、前哨站进攻及姿态控制。

默认启动树为：

```text
config/bt/uc_adaptive_training.xml
```

简易赛使用独立入口 `ros2 launch sentry_nav_bt com_game.launch.py`，不会改变上面的默认赛季树。下面的“简易赛”一节记录了全部改动和测试方法。

## 简易赛：单补给点、单作战点

### 目标与改动范围

本版只让哨兵在 `init`、`supply_point`、`center_point` 三个命名点之间行动：`init` 只用于赛前，比赛中只在补给点和作战点之间切换。不追英雄、不打符、不购买弹丸。这里的“攻击姿态”是向裁判系统请求姿态，**不等于自瞄已经识别并命中两块板**。

| 文件 | 这次做了什么 |
|---|---|
| `config/com_game.xml` | 修复赛前子树参数；比赛中按复活、低血补给、发射机构无供电等待、驻点作战的顺序选分支 |
| `config/bt/modules/simple_supply_return.xml` | 新建仅按血量回补的子树；低于 150 HP 触发，锁存到血量恢复到 `maximum_hp`，只用 `supply_point` |
| `launch/sentry_nav_bt.launch.py` | 增加 `use_sim_time` 参数，默认仍为 `false`，旧入口行为不变 |
| `launch/com_game.launch.py` | 新增简易赛入口，固定加载 `com_game.xml`；默认 `use_sim_time=true`，可传 `validate_bt_only:=true` |

原 `SupplyReturn`、`ShooterPowerRecovery` 和其它赛季树未修改。现有 `config/bt/com_game_supply_return.xml` 是独立草稿，不在 `modules` 目录，简易赛主树不调用它。

`config/waypoints.json` 只把 `center_point` 调整为 `(0.5,-0.5)` 供测试，`init=(0,0)`、`supply_point=(-0.87,-5.04)` 未改；这些是原地图点，不保证适合长方形新场地。在新场地试车前，必须测量三点在 `map` 坐标系中的 `x/y/yaw`，更新这个 JSON；模拟裁判插件也从该包安装后的路径点文件读取补给点。不能凭空把旧坐标当成新赛场坐标。

模拟裁判插件若仍从 JSON 读到 `supply_point_2`，它也会把那一处认作补给区；本版行为树虽然绝不会主动前往那里，但这还不是“严格只有一个补给区”的新场地模型。真要验证单补给区规则，应给模拟器提供只含实测补给点的场地配置，再核对 RFID 与回血。

### 控制顺序与数据流

```text
rm_referee_mock /rm_referee/game_status ───────────────┐
rm_referee_mock /rm_referee/robot_status ──────────────┼─> topic_listener ─> 黑板 ─> com_game.xml
仿真 TF map -> base_link ─> waypoint_now/位姿有效标志 ┘                      │
                                                                           ├─ HP=0：Resurrection
                                                                           ├─ HP<150 或回补锁存：SimpleSupplyReturn
                                                                           ├─ 发射机构无供电：原地等待
                                                                           └─ 其他：PointHold(center_point)

GoalSelector(supply_point/center_point) -> ReliableNavigateToPose -> Nav2 navigate_to_pose
    -> 仿真底盘移动 -> TF 更新 -> 裁判模拟器判定是否进入补给点(半径 0.35 m)
    -> 模拟器回血并发布 RobotStatus -> 黑板 HP 更新 -> 行为树切回作战点
```

赛前阶段 `1` 返回 `init`；阶段 `0/2/3` 不新发目标；阶段 `4` 进入比赛分支。阶段离开 `4` 时，`ReactiveSequence` 会停止当前运行的导航子树，`ReliableNavigateToPose` 再请求取消 Nav2 goal。`KeepRunningUntilFailure` **不是 C++ 的 `break`**：主程序仍每 10 ms tick 一次，后续若阶段再次变为 `4` 会重新评估。

`PointHold` 到点后请求姿态 `1`（普通攻击），去点途中请求姿态 `3`（移动）；补给点驻守请求姿态 `2`（防御）。发射机构无供电时不会“自动解锁”——行为树只能等待裁判状态恢复，并停止作战导航。低血回补优先于供电等待。现阶段没有弹量触发、购弹和第二补给点。

### 测试流程：先校验，再仿真，最后才考虑实车

下面假定源码工作空间是 `/home/yjfy/SentryNav2026_XDU`，PB 仿真工作空间是 `/home/yjfy/integrate/pb_rmsimulation`。**不要同时连接真实底盘。**各终端使用同一个、未被其它测试占用的 `ROS_DOMAIN_ID` 和同一种 RMW。若全在本机，可沿用已验证的 loopback CycloneDDS 配置：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI=file:///home/yjfy/Documents/Codex/2026-09-18/new-chat/work/SentryBT_Groot_WS/cyclonedds-local.xml
```

1. 编译并只校验树。在工作空间根目录执行：

   ```bash
   cd /home/yjfy/SentryNav2026_XDU
   colcon build --packages-up-to sentry_nav_bt rm_referee_mock --symlink-install
   source install/setup.bash
   ros2 launch sentry_nav_bt com_game.launch.py validate_bt_only:=true save_terminal_log:=false
   ```

   必须看到 `行为树 XML 校验通过` 且 `navigate_bt_node` 正常退出。只看 `ros2 launch` 命令返回 0 不够；进程可以崩溃而 launch 本身仍正常结束。此步不需要 Nav2，也不会 tick 行为树。

2. 启动 PB+Nav2（新终端，先执行上面的共同环境设置，再 source 两个工作空间）：

   ```bash
   source /home/yjfy/integrate/pb_rmsimulation/install/setup.bash
   source /home/yjfy/SentryNav2026_XDU/install/setup.bash
   ros2 launch rm_nav_bringup bringup_sim.launch.py world:=RMUC mode:=nav localization:=amcl nav_rviz:=false
   ```

   `RMUC` 仅用于验证已有旧地图与旧坐标的流程，**不是长方形新场地**。等 `/clock`、`navigate_to_pose` action server 和 `map -> base_link` 都可用后再继续：

   ```bash
   ros2 topic echo /clock --once
   ros2 action info /navigate_to_pose
   ros2 run tf2_ros tf2_echo map base_link
   ```

   `tf2_echo` 会持续输出，确认坐标在变化后按 Ctrl-C 退出。若地图或 TF 不成立，先修仿真定位，不要用裁判模拟器掩盖导航问题。

3. 启动裁判模拟器（新终端，相同环境并 `source /home/yjfy/SentryNav2026_XDU/install/setup.bash`）：

   ```bash
   ros2 run rm_referee_mock rqt_clean_start
   ```

   在 Match Control 中先设比赛阶段 `1`、哨兵血量非零、发射机构供电开启。确认它持续发布 `/rm_referee/game_status` 与 `/rm_referee/robot_status`。模拟器只有在比赛阶段 `4` 且计时运行时，进入补给区才会自动回血；其补给检测半径为 0.35 m。

4. 启动简易赛树（新终端，相同环境，source 仿真和本工作空间）：

   ```bash
   source /home/yjfy/integrate/pb_rmsimulation/install/setup.bash
   source /home/yjfy/SentryNav2026_XDU/install/setup.bash
   ros2 launch sentry_nav_bt com_game.launch.py
   ```

   Groot2 连接本机端口 `1667` 看实际运行分支。另开终端可核对输入：

   ```bash
   ros2 topic echo /rm_referee/game_status --once
   ros2 topic echo /rm_referee/robot_status --once
   ros2 topic echo /rm_referee/rfid_status --once
   ```

5. 逐项手动验收，每次记录 **预计分支 → Groot2 实际分支 → Nav2 goal/取消日志 → TF 位置 → RobotStatus**：

   | 操作 | 预期 |
   |---|---|
   | 只有 GameStatus、没有 RobotStatus | 不发作战/回补目标 |
   | 阶段 1 → 2/3 | 先回 `init`，随后不再发新目标 |
   | 阶段 4，HP≥150，供电开启 | 去 `center_point`，到点后请求普通攻击姿态 |
   | 作战途中把 HP 调到 149 | 取消旧目标，去 `supply_point`；不能同时保留两个活动 goal |
   | 补给点到达，RFID 检测成功 | 模拟器回血；HP 达到 `maximum_hp` 后清锁存，返回作战点 |
   | HP 设为 0 | 复活分支优先，不能把 0 HP 当成普通低血导航 |
   | 供电关闭→恢复 | 暂停作战导航，等待；恢复后重新去作战点 |
   | 阶段 4 → 结束阶段 | 不再新发目标，旧导航应收到取消请求 |

   先在 Match Control 手动改 HP 验证分支，再验证“机器人进入补给点 → RFID → 自动回血”的闭环。**只在 GUI 勾选补给区不能证明位置联动成功**；应看到模拟器 `进入supply_point`、RFID 状态变化及 TF 位置接近补给坐标。

### 已校验与仍未证明

本次源码用临时隔离目录完成 `sentry_msgs`、`rm_referee_msgs`、`sentry_nav_bt` 构建，并通过 `validate_bt_only` 的 BehaviorTree.CPP 工厂加载校验；Python 启动文件和 XML 也通过语法检查。隔离环境中 DDS 曾报告 UDP socket 权限警告，虽然工厂校验日志显示通过、进程正常退出，但**不代表 ROS 通信已测通**。尚未启动 PB 仿真、裁判 GUI 或真实底盘，因此上表的导航取消、姿态反馈和自动回血仍需你按实际日志验收。缺少新赛场地图/实测坐标时，不能把 RMUC 上的成功当成小比赛实场可用。

已知边界：`game_status_received` 只表示“至少收到过一次”，不是断线超时检测；若裁判话题中断，黑板可能保留旧值。上实车前还需要数据新鲜度/失联停车保护。姿态请求成功也不等于自瞄开火或板被击中，射击链路需要单独验证。

额外发现：同一临时构建中，**原默认树** `uc_adaptive_training.xml` 的校验报 `point_hold_wait_duration` 曾按 `int`、后按 `double` 创建的端口类型冲突；旧模块里同时存在 `point_hold_wait_duration="0"` 与 `"0.5"`。这不是简易赛树的校验结果，也没有在本次简易赛改动中顺手重写原赛季模块。测试时请明确启动 `com_game.launch.py`，不要误用默认入口。

## 运行结构

```text
/rm_referee/* ─┐
map -> base_link ─┼─> 黑板与局内状态 ─> BehaviorTree.CPP ─> NavigateToPose
waypoints.json ──┘                    ├─> /rm_referee/tx
                                     ├─> /scan_mod_type
                                     ├─> /auto_shoot_type
                                     ├─> /yaw_controller
                                     ├─> /outpost_mode_type
                                     └─> /vw
```

启动后，节点会：

1. 加载路径点和行为树 XML；
2. 用 `init` 路径点向 `/initialpose` 发布初始位姿；
3. 等待 `navigate_to_pose` action server；
4. 监听裁判系统和 TF，并持续更新黑板；
5. 创建并循环 tick 行为树；
6. 启用 Groot2 实时监视。

## 依赖与编译

主要依赖：

- ROS 2 Humble
- BehaviorTree.CPP
- Nav2
- `rclcpp`、`rclcpp_action`
- `geometry_msgs`
- `tf2`、`tf2_ros`、`tf2_geometry_msgs`
- `nlohmann_json`
- `sentry_msgs`
- `rm_referee_msgs`

在工作空间根目录执行：

```bash
colcon build --packages-up-to sentry_nav_bt
source install/setup.bash
```

## 启动

启动默认的适应性训练树：

```bash
ros2 launch sentry_nav_bt sentry_nav_bt.launch.py
```

仅检查 XML、子树和注册节点，不执行导航：

```bash
ros2 launch sentry_nav_bt sentry_nav_bt.launch.py \
  validate_bt_only:=true
```

选择其他主树：

```bash
ros2 launch sentry_nav_bt sentry_nav_bt.launch.py \
  bt_xml_filename:="$(ros2 pkg prefix sentry_nav_bt)/share/sentry_nav_bt/config/bt/uc_patrol.xml"
```

启动供 `sentry_nav_bt_gui` 控制的单点导航树：

```bash
ros2 launch sentry_nav_bt simple_nav.launch.py
```

## 启动参数

### `sentry_nav_bt.launch.py`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `bt_xml_filename` | `config/bt/uc_adaptive_training.xml` | 主行为树 XML |
| `bt_main_tree_id` | `MainTree` | 执行的主树 ID |
| `waypoints_file` | `config/waypoints.json` | 路径点文件 |
| `bt_subtree_dir` | `config/bt/modules` | 子树目录 |
| `validate_bt_only` | `false` | 只校验行为树 |
| `use_sim_time` | `false` | PB 仿真时设为 `true`；`com_game.launch.py` 默认 `true` |
| `save_terminal_log` | `true` | 保存完整 stdout/stderr |
| `terminal_log_dir` | 启动目录下 `data/sentry_nav_bt` | 终端日志根目录 |

`terminal_log_dir` 使用启动 `start.sh` 或执行 `ros2 launch` 时的当前工作目录。默认情况下，每次启动会在该目录下建立带时间和 PID 的运行目录，并写入 `launch.log`。

关闭完整终端日志：

```bash
ros2 launch sentry_nav_bt sentry_nav_bt.launch.py \
  save_terminal_log:=false
```

指定日志目录：

```bash
ros2 launch sentry_nav_bt sentry_nav_bt.launch.py \
  terminal_log_dir:=/tmp/sentry_bt_logs
```

### `simple_nav.launch.py`

该入口固定使用 `simple_nav.xml`，支持 `validate_bt_only`，路径点和子树从本包安装目录加载。

## 行为树

主树位于 `config/bt/`，可复用子树位于 `config/bt/modules/`。

| 主树 | 用途 |
|---|---|
| `uc_adaptive_training.xml` | 默认适应性训练流程；在打符、回补等阶段之间执行 `myhero` 与梯形高地往返巡逻 |
| `com_game.xml` | 简易赛：复活、单点补血、供电等待、单点作战 |
| `uc_myhero.xml` | 英雄高地驻守策略 |
| `uc_highland_hold.xml` | 梯形高地驻守策略 |
| `uc_chase.xml` | 高地追击策略 |
| `uc_patrol.xml` | 五个命名点的随机巡逻 |
| `simple_nav.xml` | GUI 动态配置的一次性单点任务 |
| `train.xml` | 随机巡逻子树的独立训练入口 |
| `5.3tree.xml` | 保留的历史测试树 |

### 适应性训练

默认树沿用 UC 比赛阶段、复活、发射机构恢复、回补、小符、大符和前哨站处理。在巡逻时间窗中调用 `AdaptiveTrainingShuttle`，在以下两个命名点之间往返：

- `myhero`
- `trapezoid_highland`

到点后停留 5 秒再前往另一点。

### 随机巡逻

`RandomWaypointNavigate` 子树内部定义五个点：

```text
point1 point2 point3 point4 point5
```

每次随机选择一个点，完成导航后停留 5 秒。`random_wait_time_threshold` 用于组成 `stage_remain_time > threshold` 条件。在 `train.xml` 中传入 `0` 仍然有作用：比赛剩余时间大于 0 时持续巡逻，归零时退出。

主树只负责调用该子树，不再重复定义五个巡逻点或到点等待流程。

### 单点导航

`simple_nav.xml` 从运行时参数取得目标、姿态和阈值。比赛开始后执行一次任务，完成后保持完成状态；参数被成功更新时会清除完成标记并执行新任务。

配套界面见 `sentry_nav_bt_gui`。

## 路径点

路径点文件为 `config/waypoints.json`：

```json
{
  "waypoints": [
    {
      "name": "init",
      "x": 0.0,
      "y": 0.0,
      "yaw": 0.0
    }
  ]
}
```

每个点必须包含唯一的 `name` 以及 `x`、`y`、`yaw`。加载后会写入：

- `waypoint_0`、`waypoint_1` 等顺序键；
- `waypoint_<name>` 命名键；
- `waypoints` 和 `waypoints_count`。

初始位姿按 `init`、第一个路径点、`start` 的顺序查找。默认树还会使用 `supply_point`、符点、前哨站、堡垒、基地防守、高地和巡逻点等命名点。

## 单点导航运行时参数

| 参数 | 默认值 | 校验 |
|---|---:|---|
| `runtime_goal_name` | `init` | 非空；使用命名点时必须存在 |
| `runtime_use_custom_pose` | `false` | `true` 时使用临时坐标 |
| `runtime_goal_x` | `0.0` | 有限数值 |
| `runtime_goal_y` | `0.0` | 有限数值 |
| `runtime_goal_yaw` | `0.0` | 有限数值 |
| `runtime_move_posture` | `3` | 1～6 |
| `runtime_wait_posture` | `1` | 1～6 |
| `runtime_reach_threshold` | `0.25` | 大于 0，单位 m |
| `runtime_wait_time_threshold` | `5.0` | 不小于 0，单位 s |

参数通过 `/sentry_nav_bt/set_parameters_atomically` 更新。所有值校验成功后才会同时生效。

## 自定义行为树节点

| 类别 | 节点 |
|---|---|
| 导航与路径点 | `ReliableNavigateToPose`、`GoalSelector`、`PatrolGoalSelector`、`RandomSelector`、`CheckGoalReached` |
| 黑板与条件 | `SetBlackboardValue`、`CheckCondition`、`CompareValues`、`PrintNode`、`PrintBlackboardValue` |
| 裁判系统动作 | `MaintainSentryPosture`、`ResolveSentryPosture`、`ConfirmResurrection`、`BuySentryProjectile`、`EngageRune`、`EngageOutpost` |
| 控制话题 | `PublishScanMode`、`PublishVw` |
| 其他 | `Wait`、`UseTrackingPlanner` |

`EngageRune` 在到达符点并开始请求前先发布 yaw 控制；当前从 yaw 发出到能量机关激活请求的等待时间为 3 秒。

## ROS 接口

主要输入：

- `navigate_to_pose`
- TF `map -> base_link`
- `/rm_referee/game_status`
- `/rm_referee/game_robot_hp`
- `/rm_referee/event_data`
- `/rm_referee/robot_status`
- `/rm_referee/power_heat_data`
- `/rm_referee/projectile_allowance`
- `/rm_referee/rfid_status`
- `/rm_referee/sentry_info`
- `/rm_referee/hurt_data`
- `/rm_referee/robot_pos`
- `/rm_referee/ground_robot_position`
- `/rm_referee/map_command`

主要输出和调用：

| 接口 | 用途 |
|---|---|
| `/initialpose` | 发布初始位姿 |
| `/rm_referee/tx` | 姿态、复活、买弹、能量机关等裁判系统指令 |
| `/scan_mod_type` | 视觉扫描模式 |
| `/auto_shoot_type` | 自动射击开关 |
| `/yaw_controller` | 打符云台转向 |
| `/outpost_mode_type` | 前哨站模式 |
| `/vw` | 特定任务的底盘控制标志 |

## Groot2

节点创建 `BT::Groot2Publisher`，可用 Groot2 连接运行中的行为树。项目文件为：

```text
config/bt/sentry_nav_bt.btproj
```
