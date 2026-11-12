# sentry_nav_bt 源码阅读指南

按"从数据流到决策流"的顺序读，每一步都给出文件、要回答的问题和关键行号。读完本篇你应能独立回答：**一帧裁判数据进来后，如何变成机器人的一次导航目标**。

前置阅读：[../README.md](../README.md)（包功能与简易赛说明）、[../../README.md](../../README.md)（仿真联调步骤）。

## 0. 目录结构

```text
sentry_nav_bt/
├── launch/            三个启动入口（主入口 / 简易赛 / simple_nav）
├── config/
│   ├── com_game.xml   简易赛主树          ├── bt/*.xml      各赛季主树
│   ├── bt/modules/    可复用子树           └── waypoints.json 命名路径点
└── src/               本目录，C++ 源码
    ├── sentry_nav_bt.cpp        ★ 节点入口：参数、BT 工厂、tick 循环
    ├── topic_listener.cpp       ★ 数据层：话题/TF → 黑板
    ├── blackboard/defaults.cpp  黑板键初值清单
    ├── referee_*                裁判系统交互动作（发送 0x0301/0x0120）
    ├── reliable_navigate_to_pose*  可靠导航封装
    └── patrol_nodes / random_selector / publish_vw ...  通用 BT 节点
```

## 1. 入口：sentry_nav_bt.cpp —— 程序怎么跑起来的

读 [sentry_nav_bt.cpp](sentry_nav_bt.cpp)，回答三个问题：

1. **参数怎么进来**：`bt_xml_filename`、`bt_main_tree_id`、`bt_subtree_dir`、`waypoints_file`、`validate_bt_only`、`use_sim_time`（对应 launch 里的同名参数）。
2. **三大组件怎么组装**：创建 BT::Blackboard → `BlackboardManager`（订阅+初始化默认值+加载路点）→ 注册 BT 节点工厂 → 加载 XML → 等待 `navigate_to_pose` action server → 进入 tick 循环。
3. **tick 循环长什么样**：固定频率调 `tree.tickRoot()`；`validate_bt_only=true` 时只加载校验、不 tick（用于 CI/预检）。

顺带看 [runtime_config.cpp](runtime_config.cpp)：运行期参数的读写封装。

## 2. 数据层：topic_listener.cpp —— 外界数据怎么进黑板

即本目录的 [topic_listener.cpp](topic_listener.cpp)（`BlackboardManager`）。核心心智模型：

- 行为树节点**只读黑板**，从不订阅话题；这里是唯一的翻译层。
- 12 个裁判话题（best_effort QoS）→ 拆成扁平黑板键；2027 位域解析都在回调里（含 robot_id 红蓝换算）。
- 三套"状态流 → 事件"的加工：0x0303 去抖（0.35s 聚类）、比赛时间窗 window_id（>390s 为窗1 … ≤60s 为窗10）、受击 5s / 英雄位置 2.5s 超时复位。
- TF `map→base_link` 每 100ms 写 `waypoint_now`，0.5s 新鲜度门控 `waypoint_now_valid`。

关键函数：`isNewMapCommandEvent`、`competitionTimeWindowId`、`updateCurrentPosition`。

## 3. 黑板契约：defaults.cpp —— 有哪些键、初值是什么

读 [defaults.cpp](blackboard/defaults.cpp)。它就是**黑板键的字典**：树 XML、C++ 节点、监听器三方共享的键名都在这里登记初值。改任何键名前先看这里，再全局搜键名（config + src）。

## 4. 决策层：自定义 BT 节点分四类

### 4.1 条件与黑板工具

| 文件 | 节点/作用 |
| --- | --- |
| [check_condition.hpp](../include/sentry_nav_bt/check_condition.hpp) | `CheckCondition`：黑板值 vs 阈值（eq/gte/lte），树里 90% 的分支判断 |
| [compare_values.hpp](../include/sentry_nav_bt/compare_values.hpp) | 比较逻辑实现 |
| [set_blackboard.hpp](../include/sentry_nav_bt/set_blackboard.hpp) | `SetBlackboardValue` |
| [print_node.hpp](../include/sentry_nav_bt/print_node.hpp) / [print_blackboard_node.hpp](../include/sentry_nav_bt/print_blackboard_node.hpp) | 调试打印，树里到处都是 |
| [blackboard_utils.hpp](../include/sentry_nav_bt/blackboard_utils.hpp) | 类型安全的黑板读写助手 |

### 4.2 导航类（怎么走到目标点）

| 文件 | 节点/作用 |
| --- | --- |
| [reliable_navigate_to_pose.hpp](../include/sentry_nav_bt/reliable_navigate_to_pose.hpp) + [实现](reliable_navigate_to_pose.cpp) | `ReliableNavigateToPose`：带重发间隔/超时/重试的 Nav2 导航，树里主用 |
| [reliable_navigate_to_pose_action.cpp](reliable_navigate_to_pose_action.cpp) | 底层 action client 封装 |
| [goal_selector_node.cpp](goal_selector_node.cpp) | `GoalSelector`：按名字从黑板取路点坐标放进 `{goal}` |
| [patrol_nodes.cpp](patrol_nodes.cpp) | 巡逻/多点轮换 |
| [navigation_transition_gate.cpp](navigation_transition_gate.cpp) | 导航切换门控（防止频繁换目标） |
| [navigation/waypoint_loader.cpp](navigation/waypoint_loader.cpp) | 解析 waypoints.json 进黑板 |

### 4.3 裁判交互类（怎么"上报"和"买东西"）

| 文件 | 作用 |
| --- | --- |
| [referee_actions.hpp](../include/sentry_nav_bt/referee_actions.hpp) | 交互动作基类：拿 robot_id、建 `/rm_referee/tx` client、0x0301 组包 |
| [referee_actions_common.cpp](referee_actions_common.cpp) | 姿态请求 `MaintainSentryPosture` 等；含冷却/跳过原因（`robot_id_unavailable`） |
| [referee_engage_actions.cpp](referee_engage_actions.cpp) | 复活、打符、远程兑换弹丸/血量（0x0120 决策位） |
| [control_topic_publishers.cpp](referee/control_topic_publishers.cpp) | 发 `sentry_msgs`：`/scan_mod_type`（扫描模式）、PublishVw 用 |
| [cmd_utils.hpp](../include/sentry_nav_bt/cmd_utils.hpp) | 交互协议打包工具 |

### 4.4 控制与杂项

| 文件 | 作用 |
| --- | --- |
| [publish_vw_action.cpp](publish_vw_action.cpp) | 直接发 `/vw` 速度指令 |
| [center_hold_vw_controller.cpp](topic_listener.cpp) 同文件内创建 | 中心驻守的 /vw 控制器（受 `waypoint_now_valid` 门控） |
| [random_selector_node.cpp](random_selector_node.cpp) | `RandomSelector`：巡逻随机选点 |
| [trapezoid_highland.hpp](../include/sentry_nav_bt/trapezoid_highland.hpp) | 梯形高地几何判定：红蓝坐标先归一化到红方再判 inside |

## 5. 配置层：树 XML 与路点

- 主树结构：[../config/com_game.xml](../config/com_game.xml)——Fallback 优先级从上到下：未收到比赛状态不动作 → 赛前回 init → 主循环（复活 > 低血补给 > 等待供电恢复 > 驻点作战）。
- 子树：[../config/bt/modules/](../config/bt/modules/)，每个文件一个可复用行为（PointHold、Resurrection、SimpleSupplyReturn…），主树用 `<SubTree ID="...">` + 端口传参调用。**注意 BT.CPP 按字面量推断端口类型**：同一端口一处写 `"0"`（int）另一处写 `"0.5"`（double）会导致加载失败，这是旧树最常见的坑。
- 路点：[../config/waypoints.json](../config/waypoints.json)，`name → (x, y)`；换场地必须重测坐标，键名不能改。

## 6. 推荐阅读路线（半天）

```text
sentrv_nav_bt.cpp(入口)  →  topic_listener.cpp(数据)  →  defaults.cpp(键字典)
        ↓
com_game.xml(主树怎么分支)  →  modules/ 挑 3 个子树(PrestartInitReturn/PointHold/SimpleSupplyReturn)
        ↓
check_condition.hpp(分支怎么判断)  →  reliable_navigate_to_pose.cpp(怎么动)
        ↓
referee_actions.hpp + referee_engage_actions.cpp(怎么和裁判系统交互)
```

## 7. 调试手段

| 手段 | 用法 |
| --- | --- |
| 只校验树 | `validate_bt_only:=true`（launch 参数或 ros2 run 直传），加载失败会报具体端口类型冲突 |
| 看决策日志 | launch 默认把完整终端输出存到启动目录 `data/sentry_nav_bt/<时间戳>/launch.log`；树里大量 `PrintNode` 是给人看的路标 |
| 打印黑板 | 树里临时插 `PrintBlackboard` 节点 |
| 单独验证解析 | 起 rm_referee_mock 改状态，对照 `ros2 topic echo /rm_referee/event_data` 与日志 |
