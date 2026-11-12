# 哨兵决策仿真联调指南（pb_rmsimulation + rm_referee_mock）

用 PB 仿真提供 Gazebo 环境 + Nav2 导航，用 `rm_referee_mock`（rqt 插件）模拟裁判系统话题，跑通 `sentry_nav_bt` 行为树。本文以简易赛主树 `com_game.xml` 为例。

## 0. 数据链路总览

```
+---------------------------+
| pb_rmsimulation (Gazebo)  |
|  - /clock                 |
|  - Nav2 => navigate_to_pose (action)
+------------+--------------+
             | 导航目标
+------------v--------------+       /rm_referee/* 话题        +----------------------+
| sentry_nav_bt             |<---------------------------------| rm_referee_mock(rqt) |
| navigate_bt_node          |  game_status / robot_status /    |  MatchControl 插件    |
|  - 订阅 /rm_referee/*     |  game_robot_hp / event_data /    |  (可选 FakeLocation) |
|  - 下发 NavigateToPose    |  rfid_status / sentry_info ...   +----------------------+
+---------------------------+
```

行为树启动后会先**阻塞等待 `navigate_to_pose` action server**，所以必须先启动仿真导航栈，再启动决策。

## 1. 前置条件

| 依赖 | 说明 |
| --- | --- |
| ROS 2 Humble + Gazebo Classic 11 | pb_rmsimulation 的运行环境 |
| `~/integrate/pb_rmsimulation` | 仿真工作空间，需已 `colcon build --symlink-install` |
| `~/IRobot-SentryNav27` | 本仓库，含通信包与决策包 |
| rm_referee_mock 依赖 | PySide6/Qt、rqt（mock 以 rqt 插件运行） |

本仓库内决策相关的包：

| 包 | 位置 | 作用 |
| --- | --- | --- |
| `rm_referee_msgs` | `src/communication/rm_referee_msgs` | 2027 裁判系统消息定义 |
| `sentry_msgs` | `src/communication/sentry_msgs` | 姿态/扫描模式控制消息 |
| `sentry_nav_bt` | `src/decision/sentry_nav_bt` | 行为树决策节点 |
| `rm_referee_mock` | `src/tools/rm_referee_mock` | 裁判系统 mock（rqt 插件） |

## 2. 编译本仓库

```bash
cd ~/IRobot-SentryNav27
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to sentry_nav_bt rm_referee_mock
source install/setup.bash
```

只校验行为树 XML 是否能加载（不启动 Nav2、不 tick）：

```bash
ros2 run sentry_nav_bt navigate_bt_node --ros-args \
  -p bt_xml_filename:=$PWD/src/decision/sentry_nav_bt/config/com_game.xml \
  -p bt_main_tree_id:=MainTree \
  -p bt_subtree_dir:=$PWD/src/decision/sentry_nav_bt/config/bt/modules \
  -p waypoints_file:=$PWD/src/decision/sentry_nav_bt/config/waypoints.json \
  -p validate_bt_only:=true
```

看到 `行为树 XML 校验通过` 即可进入联调。

## 3. 联调步骤（三个终端）

每个终端都先 `source` 两个工作空间：

```bash
source /opt/ros/humble/setup.bash
source ~/integrate/pb_rmsimulation/install/setup.bash
source ~/IRobot-SentryNav27/install/setup.bash
```

### 终端 1：启动 PB 仿真 + 导航栈

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RM2027 \
  mode:=mapping \
  lio:=fastlio \
  localization:=icp \
  lio_rviz:=False \
  nav_rviz:=True
```

- `world` 可选 `RMUL`（3V3 场地）/ `RMUC` / `RM2027`。
- `mode:=mapping` 可边建图边导航，首次跑建议先用 mapping 建图再切 nav。
- 等待条件：Nav2 就绪。可用 `ros2 action list | grep navigate_to_pose` 确认 action server 出现。

### 终端 2：启动裁判系统 mock（rqt）

```bash
ros2 run rm_referee_mock rqt_clean_start
```

会自动带 `--clear-config --force-discover` 打开 `MatchControl` 插件。启动后必须手动设置：

1. **Topic prefix** 保持 `/rm_referee`（默认值，行为树只认这个前缀）。
2. **Robot ID** 设为红方哨兵 `7`（或蓝方 `107`）。行为树靠 `/rm_referee/robot_status` 拿 `robot_id`，不设则决策里所有裁判交互动作报 `robot_id_unavailable` 跳过。
3. **比赛阶段** 设为 `[4] 比赛中`。`com_game.xml` 在阶段 1 会反复回 `init` 点，阶段 0/2/3 不发目标，只有阶段 4 进入主循环。
4. 需要测试回补弹分支时，把剩余时间、血量、RFID/阵地状态按场景勾选。

### 终端 3：启动行为树

```bash
ros2 launch sentry_nav_bt com_game.launch.py use_sim_time:=true
```

- `use_sim_time:=true` 与 Gazebo 的 `/clock` 对齐，**PB 仿真下必须开**。
- 换其他树：`ros2 launch sentry_nav_bt sentry_nav_bt.launch.py bt_xml_filename:=$PWD/src/decision/sentry_nav_bt/config/bt/uc_adaptive_training.xml use_sim_time:=true`（注意旧 uc_* 树目前有端口类型问题，尚未逐棵修复）。
- 完整终端日志自动保存在启动目录 `data/sentry_nav_bt/<时间戳>/launch.log`。

## 4. 联调中的观察点

| 想确认什么 | 命令/位置 |
| --- | --- |
| 裁判话题是否在发 | `ros2 topic list | grep /rm_referee`、`ros2 topic echo /rm_referee/game_status` |
| 行为树收到比赛状态 | 终端 3 出现 `[简易赛] 裁判系统已连接，等待比赛开始` |
| 哨兵是否开始导航 | rviz 中 Nav2 出现目标；`ros2 topic echo /navigate_to_pose/_action/status` |
| 位域解析是否正确 | `MatchControl` 改阵地/符状态后，对比 `ros2 topic echo /rm_referee/event_data` 与终端日志 |
| 姿态/复活等交互 | 以 `/rm_referee/sentry_info` 回读为准；`/rm_referee/tx` 的 `ok=true` 只代表链路成功 |

## 5. 常见问题

1. **行为树卡在等待 action server**：Nav2 没起好。先跑通终端 1，`ros2 action list` 里能看到 `navigate_to_pose` 再启动终端 3。
2. **裁判交互动作全部跳过（`robot_id_unavailable`）**：mock 里 Robot ID 没设或仍是 0。
3. **树一直停在"等待比赛开始"**：mock 比赛阶段不在 `[4] 比赛中`，或 topic prefix 不是 `/rm_referee`。
4. **机器人导航目标点不对/撞墙**：`config/waypoints.json` 的坐标是按场地坐标系写的，换 `world`（RMUL/RMUC/RM2027）后需要核对或重标路标点。`init/supply_point/center_point/fallback_point` 等名字是树里引用的键，坐标改值不改名。
5. **时间不走**：决策没开 `use_sim_time:=true`，或 Gazebo 暂停了。
6. **想给行为树喂裁判系统自身位置**：行为树订阅 `/rm_referee/robot_pos`，可在 rqt 里开 `FakeLocation` 插件，并把它的话题前缀从默认 `/rm_referee/mock` 改成 `/rm_referee`。
7. **rqt 打开是空白页**：用 `rqt_clean_start` 启动器（自带 `--clear-config`），不要直接 `rqt`。
