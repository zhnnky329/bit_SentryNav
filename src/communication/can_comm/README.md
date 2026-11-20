# can_comm

NUC ↔ MCU 的 CAN 总线通信包，基于 C++（rclcpp）和 Linux SocketCAN。负责把导航速度指令、云台目标 yaw 和全向感知装甲板检测标志下发 MCU，并接收 MCU 回传的大 yaw 角。

## 组件

| 可执行文件 | 节点名 | 方向 | 功能 |
|---|---|---|---|
| `can_send` | `can_send` | NUC → MCU | 订阅 `/cmd_vel`、`/detector/armor_presence`、`/can_comm/target_yaw`，发送 `0x190` |
| `can_receive` | `can_receive` | MCU → NUC | 接收 `0x149`，发布 `/can_comm/yaw_deg` |

## 数据流

```text
/cmd_vel ─────────────────┐
                          │
/detector/armor_presence ─┼── can_send ── 0x190 ──> MCU
                          │
/can_comm/target_yaw ─────┘

MCU ── 0x149 ──> can_receive ──> /can_comm/yaw_deg
```

## 通信协议

### NUC → MCU：`0x190`

标准帧，8 字节，多字节大端。

| 字节 | 字段 | 编码 |
|---|---|---|
| Byte 0 | `vx` | m/s，限幅 `[-1.0, 1.0]`，乘 127 后按 `int8` 发送 |
| Byte 1 | `vy` | 同 `vx` |
| Byte 2–3 | `target_yaw` | 度，限幅 `[-180, 180]`，乘 100 后按大端 `int16` 发送；订阅 `/can_comm/target_yaw`，未收到时为 `00 00` |
| Byte 4 | `left` | 左方检测标志，`0x00` / `0x01` |
| Byte 5 | `behind` | 后方检测标志，`0x00` / `0x01` |
| Byte 6 | `right` | 右方检测标志，`0x00` / `0x01` |
| Byte 7 | 保留 | 固定 `0x00` |

### MCU → NUC：`0x149`

Byte 6–7 为电控回传的实测大 yaw 角：大端有符号 `int16` 除以 100，单位度。

| 原始字节 | 角度 |
|---|---|
| `23 28` | `+90.00°` |
| `DC D8` | `-90.00°` |

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 取 `linear.x` / `linear.y` 作为 `vx` / `vy` |
| `/detector/armor_presence` | `sentry_msgs/msg/ArmorPresence` | 取 `left` / `behind` / `right` 检测标志 |
| `/can_comm/target_yaw` | `std_msgs/msg/Float32` | 云台目标大 yaw 角，单位度，超时后保持最后值 |

### 发布

| 话题 | 类型 | 说明 |
|---|---|---|
| `/can_comm/yaw_deg` | `std_msgs/msg/Float32` | 实测大 yaw 角，单位度 |

## 参数

`config/can_params.yaml`：

| 节点 | 参数 | 默认值 | 说明 |
|---|---|---|---|
| `can_send` | `interface` | `can0` | SocketCAN 接口 |
| `can_send` | `send_frequency_hz` | `500.0` | 发送频率 |
| `can_send` | `cmd_vel_topic` | `/cmd_vel` | 速度指令话题 |
| `can_send` | `armor_presence_topic` | `/detector/armor_presence` | 检测标志话题 |
| `can_send` | `target_yaw_topic` | `/can_comm/target_yaw` | 云台目标 yaw 话题 |
| `can_send` | `cmd_vel_timeout_s` | `0.1` | `cmd_vel` 超时后 `vx` / `vy` 归零 |
| `can_receive` | `interface` | `can0` | SocketCAN 接口 |
| `can_receive` | `yaw_topic` | `/can_comm/yaw_deg` | yaw 回传发布话题 |
| `can_receive` | `publish_rate_hz` | `100.0` | 发布频率 |

## 构建与运行

```bash
colcon build --packages-select sentry_msgs can_comm
source install/setup.bash
colcon test --packages-select can_comm

ros2 launch can_comm can_comm.launch.py
```

单独调试：

```bash
ros2 run can_comm can_send
ros2 run can_comm can_receive
```

## 安全说明

`can_send` 对 `/cmd_vel` 做超时门控：超过 `cmd_vel_timeout_s` 未收到新指令时，`vx` 和 `vy` 均按 0 发送，避免失联后继续运动。检测标志不做超时处理，保持最后一次收到的值。

## 文件结构

```text
can_comm/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── can_params.yaml
├── include/can_comm/
│   ├── can_socket.hpp
│   └── protocol.hpp
├── launch/
│   └── can_comm.launch.py
├── src/
│   ├── can_send.cpp
│   └── can_receive.cpp
├── test/
│   └── test_protocol.cpp
└── README.md
```
