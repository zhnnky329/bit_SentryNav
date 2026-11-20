# relocalizer — 平面重定位 ROS 2 包

`relocalizer` 把“已知一张二维栅格地图、一张三维点云地图，收到一帧实时激光点云后给出地图坐标系下的位姿”整理成一个独立、可构建、可测试的 ROS 2 包。它不依赖原导航工程中的任何节点或脚本，算法、数据结构和命名均为独立实现。

- 输入：PGM/YAML 二维栅格描述、PCD 三维点云地图、实时 `sensor_msgs/PointCloud2`。
- 输出：`map -> body`（或 `map -> odom`）TF、`PoseStamped`、状态 JSON、候选位姿数组、触发服务。
- 适用：室内、近似平面运动、激光雷达水平安装、已有离线建图的环境。
---

## 1. 目录结构

```
src/lm/backend/relocalization/
├── README.md                     # 本文件
├── include/relocalizer/      # 公共头文件
├── src/                      # 核心库、节点、离线探针
├── config/relocalizer.yaml   # 示例参数
├── launch/                   # 启动文件
├── rviz/                     # 可视化配置
├── scripts/extract_sweep.py   # 从 npz 录制中导出单帧 PCD
├── test/                     # gtest
└── tools/
    ├── run_validation.py         # 五场景离线精度验证
    ├── ros_smoke_test.py         # 端到端 ROS 冒烟测试
    └── start_pipeline.zsh         # Livox + FAST-LIO + 重定位的一键启动脚本
```

核心文件职责：

| 文件 | 职责 |
|---|---|
| `plan_atlas.{hpp,cpp}` | 读取 PGM/YAML，生成 blocked 栅格、距离场和 affinity 场 |
| `surface_atlas.{hpp,cpp}` | 读取 PCD，建立多分辨率 NDT 体素与最近点索引 |
| `sweep_prep.{hpp,cpp}` | 点云裁剪、距离过滤、二维降采样，生成粗搜索/精配准两套点 |
| `finder.{hpp,cpp}` | 多分辨率全局搜索：平移网格 × 36 个 heading 分区 |
| `aligner.{hpp,cpp}` | 3-DoF NDT Gauss–Newton 精配准 |
| `inspector.{hpp,cpp}` | 独立质量检查：表面贴合、开放空间、离障碍距离 |
| `engine.{hpp,cpp}` | 串联搜索、精配准、检查、歧义判定，输出 `Outcome` |
| `relocalizer_node.cpp` | ROS 节点、状态机、TF/话题/服务 |
| `relocalizer_probe.cpp` | 无 ROS 的离线命令行探针 |

---

## 2. 依赖与构建

已验证环境：

- Ubuntu 24.04 + ROS 2 Jazzy
- CMake ≥ 3.16，g++（C++17）
- Eigen3、yaml-cpp、OpenMP（可选，用于并行搜索）
- 不依赖 PCL；PCD 与 PGM 解析器均在本包内实现
- zsh 5.x（仅 `tools/start_pipeline.zsh` 一键启动脚本需要；构建与节点本身不依赖 zsh）

```bash
cd ~/Desktop/IRobot-SentryNav27
source /opt/ros/jazzy/setup.bash
colcon build --packages-select relocalizer --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> 必须在工作区根目录 `IRobot-SentryNav27/` 执行 `colcon build`。在包目录内构建会生成一套嵌套的 `build/install/log`，导致后续命令用到旧二进制。

运行测试：

```bash
colcon test --packages-select relocalizer
colcon test-result --verbose
# 或直接运行
./build/relocalizer/test_relocalizer
```

当前 6 个 gtest 全部通过：平面位姿的环绕与复合、PGM 读取、距离场、二值 PCD 读取、点云降采样。

---

## 3. 地图输入

### 3.1 二维栅格：PGM + YAML

YAML 使用 ROS map_server 的常规字段，例如：

```yaml
image: mid360_pgo_csf.pgm
resolution: 0.05
origin: [-18.634989, -31.891941, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
mode: trinary
```

加载规则：

- 支持 P2（ASCII）与 P5（binary）PGM。
- 按 `negate` 计算占用概率：`negate=0` 时 `prob = (max_value - pixel) / max_value`，否则 `prob = pixel / max_value`。
- `prob >= occupied_thresh` 的格点记为 blocked；其余（包括 free 与 unknown）不参与打分。
- 图像行号会翻转，使内部栅格的第 0 行对应世界坐标最小 y。
- `origin` 的第三个分量是栅格坐标系相对世界坐标系的 heading；`PlanAtlas::locate` 会处理该旋转。
- 对 blocked 栅格做精确欧氏距离变换（Felzenszwalb 1D EDT 先行后列），得到每个格点到最近障碍的距离 `d`。
- affinity 场：`exp(-0.5 * (d / affinity_sigma)^2)`，默认 `affinity_sigma = 0.50 m`。粗搜索时，点落在障碍上得 1，离障碍 0.5 m 得约 0.61，离障碍 1.5 m 得约 0.011。

三维地图的 z 范围通常比二维图更可靠；如果 PGM 中有大量 unknown（205/254 灰值），本包不会把它们当作障碍，也不会把它们当作自由空间证据，只使用明确 blocked 的格点。

### 3.2 三维点云：PCD

- 支持 `DATA binary` 与 `DATA ascii`，字段至少包含 `x y z`；`intensity` 等额外字段会被跳过。
- 只保留 `z ∈ [voxel_floor, voxel_ceiling]`（默认 `[0.25, 3.00] m`）的点，避免地板/天花板压过墙面。
- 为每个体素计算均值与协方差，协方差对角线加 `1e-2` 正则，保证可逆。
- 两个分辨率层级默认 `2.0 m` 与 `1.0 m`；精配准时从细到粗匹配。
- 另建一个 `bag_span=0.5 m` 的最近点索引，供质量检查使用。

### 3.3 坐标系与点云帧

- `world_frame`：输出位姿所在坐标系，默认 `map`。
- `body_frame`：实时点云所在坐标系，默认 `base_link`；示例配置用 `body`，以匹配 FAST-LIO 的 `/cloud_registered_body`。
- 节点假设输入点云已经表达在 `body_frame` 下，且 `body_frame` 的 z 轴向上、x 轴向前。若雷达外参不同，请先转换到机体坐标系再送入本节点。
- `odom_frame` 为空时，节点直接广播 `world_frame -> body_frame`；非空时，节点查询 `odom_frame -> body_frame`，再广播 `world_frame -> odom_frame`，便于与现有 `odom -> body` 发布者共存。若沿用 FAST-LIO 的 `camera_init -> body` TF 链，可设 `odom_frame:=camera_init`，节点会发布 `map -> camera_init`，与原有导航栈的 TF 结构一致。

---

## 4. 运行

### 4.1 使用示例配置

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Desktop/IRobot-SentryNav27
source install/setup.bash

ros2 launch relocalizer relocalizer.launch.py \
  plan_descriptor:=/path/to/maps/nav2/mid360_pgo_csf.yaml \
  surface_map:=/path/to/maps/pgo_session/map_consistent.pcd \
  sweep_topic:=/cloud_registered_body \
  body_frame:=body \
  world_frame:=map \
  automatic:=true
```

`relocalizer.launch.py` 默认同时启动 RViz（`rviz:=true`）；无图形界面的环境请追加 `rviz:=false`。

在参数文件中填写两张地图的路径，或用启动参数逐项覆盖。二维地图 YAML 的 image 需指向配套的 PGM。

### 4.2 话题与服务

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/relocalizer/pose` | `geometry_msgs/PoseStamped` | 发布 | 当前位姿，z 由 `spark_height` 决定 |
| `/relocalizer/status` | `std_msgs/String` | 发布 | JSON：`state`、`confidence`、`x`、`y`、`heading_deg`、`note` |
| `/relocalizer/proposals` | `geometry_msgs/PoseArray` | 发布 | 每次全局搜索后的候选位姿（已排序） |
| `/relocalizer/plan` | `nav_msgs/OccupancyGrid` | 发布（latched） | PGM 栅格，0 为自由、100 为占用，供 RViz 显示 |
| `/relocalizer/surface_map` | `sensor_msgs/PointCloud2` | 发布（latched） | 3-D 点云地图，供 RViz 显示 |
| `/relocalizer/relocalize` | `std_srvs/Trigger` | 服务 | 立即用最近一帧点云做一次全局重定位 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 订阅 | 外部先验；置状态为 `PRIOR` 并进入跟踪 |
| `sweep_topic` | `sensor_msgs/PointCloud2` | 订阅 | 实时点云，默认 `/cloud_registered_body` |
| `/tf` | `tf2_msgs/TFMessage` | 发布 | `world -> body` 或 `world -> odom` |

状态机：

- `LOST`：尚未定位，或跟踪连续失败达到 `missed_limit`。
- `PRIOR`：收到 `/initialpose` 或刚刚由全局搜索确认，正在用 NDT 跟踪。
- `TRACKING`：跟踪质量达标，持续发布 TF。
- 服务调用成功会直接进入 `TRACKING`；失败时保持 `LOST`/`PRIOR`。

`automatic=true` 时，节点在收到可用点云后自动执行全局搜索；`automatic=false` 时只响应 `~/relocalize` 服务（或先验）。全局搜索默认最多每 `1.5 s` 一次，避免在无解场景中占满 CPU。

### 4.3 手动触发与初始先验

```bash
# 触发一次全局重定位
ros2 service call /relocalizer/relocalize std_srvs/srv/Trigger "{}"

# 给一个粗略先验（例如人工在 RViz 中设置 2D Pose Estimate）
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

### 4.4 RViz

`relocalizer.launch.py` 默认会启动 RViz，使用包内配置 `rviz/relocalizer.rviz`。需要关闭时：

```bash
ros2 launch relocalizer relocalizer.launch.py rviz:=false
```

也可以单独启动 RViz：

```bash
rviz2 -d "$(ros2 pkg prefix --share relocalizer)"/rviz/relocalizer.rviz
```

配置已加入：

- `Plan raster`：`/relocalizer/plan`，PGM 栅格；
- `Surface map`：`/relocalizer/surface_map`，点云地图；
- `Live body surface`：`/cloud_registered_body`，实时点云；
- TF、`/relocalizer/pose`、`/relocalizer/proposals`。

Fixed Frame 为 `map`。

### 4.5 与 Livox MID360 + FAST-LIO 一键启动

`tools/start_pipeline.zsh` 封装了从雷达驱动到重定位的完整顺序。脚本使用 zsh（`#!/usr/bin/env zsh`，需要 zsh 5.x），并通过 ROS 2 的 `setup.zsh` 加载环境：

```bash
cd ~/Desktop/IRobot-SentryNav27
./src/lm/backend/relocalization/tools/start_pipeline.zsh \
  --plan /path/to/site.yaml \
  --surface /path/to/site.pcd
```

脚本的执行顺序是：

1. `livox_ros_driver2` 启动 `/livox/lidar` 与 `/livox/imu`；
2. 等待这两个话题出现后启动 FAST-LIO；
3. 等待 `/cloud_registered_body` 出现，并且实际收到一帧 `sensor_msgs/PointCloud2`；
4. 可选启动 `bw_fastlio_nav2/laser_map_ground_grid` 地面/障碍滤除，以及 `body -> base_footprint` 静态 TF；
5. 启动 `relocalizer`（日志实时输出到终端），并设置 `odom_frame:=camera_init`，发布 `map -> camera_init`。

脚本默认启动 RViz（`rviz:=true`）；无图形界面时用 `--no-rviz` 关闭。若在 SSH 终端运行且未做 X11 转发，`DISPLAY` 为空，RViz 无法打开，脚本会打印警告，此时请用 `ssh -X` 或者在机器人桌面上运行。

脚本启动 FAST-LIO 时额外设置 `pcd_save.pcd_save_en:=false`，避免定位过程中把全部点云缓存到内存/磁盘；如果需要建图，请使用生产建图 launch。

第 4 步的滤除节点消费 `/cloud_registered_body`，因此它在 FAST-LIO 出点之后才启动；它服务于 Nav2 代价地图，不是重定位的前置条件。若现场已经由导航 launch 启动了雷达、FAST-LIO 和滤除，不要再运行本脚本（否则会重复启动驱动和 FAST-LIO），直接启动重定位节点即可：

```bash
ros2 launch relocalizer relocalizer.launch.py \
  plan_descriptor:=/path/site.yaml \
  surface_map:=/path/site.pcd \
  sweep_topic:=/cloud_registered_body \
  body_frame:=body world_frame:=map odom_frame:=camera_init
```

其他常用参数：

```bash
./src/lm/backend/relocalization/tools/start_pipeline.zsh --timeout 90                 # 等待首帧点云的最长时间
./src/lm/backend/relocalization/tools/start_pipeline.zsh --manual                     # 不自动搜索，等 ~/relocalize 服务
./src/lm/backend/relocalization/tools/start_pipeline.zsh --no-filter --no-base-tf     # 只启动驱动 + FAST-LIO + 重定位
./src/lm/backend/relocalization/tools/start_pipeline.zsh --no-rviz                     # 无图形界面，不启动 RViz
./src/lm/backend/relocalization/tools/start_pipeline.zsh --log-dir /tmp/live_pipeline # 指定日志目录
```

脚本会检查主工作区的 `install/setup.zsh`、地图文件和所需 ROS 包；任一缺失会直接报错。日志默认写入 `IRobot-SentryNav27/.pipeline/<时间戳>/`，包含驱动、FAST-LIO、滤除和重定位日志。按 `Ctrl-C` 或脚本收到 `SIGTERM` 时，会递归停止脚本启动的全部子进程（包括 `ros2 launch` 派生的节点），不会留下孤儿进程。

TF 链为：

```text
map -> camera_init -> body -> base_footprint
        FAST-LIO      FAST-LIO   静态 TF
```

若 FAST-LIO 的机体坐标系不是 `camera_init`，用 `--odom-frame <frame>` 覆盖；若不需要间接 TF，可传 `--odom-frame ""`，此时节点直接发布 `map -> body`，但不要与 FAST-LIO 的 `camera_init -> body` 同时存在。

---

## 5. 算法

### 5.1 预处理

1. 节点丢弃非有限点、超过 `100 m` 的点、点数不足 200 的帧；离线探针跳过这一帧级过滤，直接按下面的点级规则处理。
2. 只保留 `z ∈ [sweep_core_floor, sweep_core_ceiling]`（默认 0.30–2.30 m）的核心点；地板和天花板在此步被排除。
3. 粗搜索另取 `z ∈ [sweep_search_floor, sweep_search_ceiling]`（0.35–2.20 m）的点，按 `sweep_search_cell=0.15 m` 做二维体素降采样，最多 800 点。
4. 核心点按 `sweep_core_cell=0.20 m` 降采样，最多 1500 点，用于 NDT 与质量检查。
5. 距离过滤：`0.8 m ≤ range ≤ 60 m`；可用帧至少要有 40 个粗搜索点、80 个核心点。

### 5.2 全局搜索

- 搜索范围取三维地图点云在 xy 平面的包围盒。
- heading 以 `search_heading_step_deg=10°` 分为 36 个分区，覆盖 `[-180°, 180°)`。
- 平移网格步长初始 `0.40 m`；若格点数超过 `search_max_positions=22000`，按面积自动放大步长（不小于 0.15 m）。
- 对每个 `(x, y, heading)`，把 800 个粗搜索点旋转、平移后查询 affinity 场，取平均作为 `support`。
- `support ≥ search_coarse_floor=0.45` 的 spark 保留，按 support 排序后做非极大值抑制（1.0 m / 25°），最多留 200 个。
- 对每个 spark 在 ±2 格、±2 heading 分区内做 5×5×5 的细化（0.20 m、2°），`support ≥ 0.55` 的保留，最多 24 个。
- 每个细化结果用 NDT 精配准，再按 0.6 m / 15° 去重，输出最多 12 个 proposal。

搜索由 OpenMP 按 heading 分区并行；`search_threads=0` 表示使用 OpenMP 默认线程数。

### 5.3 NDT 精配准

- 状态量为 `(x, y, heading)`，点云 z 不参与优化。
- 每个核心点在世界坐标下查找最近的 NDT 体素；残差 `e = world - mean`，信息矩阵为协方差逆。
- 目标是最小化 Mahalanobis 残差；使用 Huber 形式降权：`maha² > 9` 时权重取 `3 / sqrt(maha²)`。
- 正规方程加 `1e-2` 对角阻尼，用 LDLT 求解；步长超过 `0.8 m` 或 `0.5 rad` 时停止。
- 收敛条件：平移步长与 heading 步长均小于 `1e-4`。
- 报告 `coherence` 为 Mahalanobis 距离 ≤ 3 的内点比例，`overlap` 为成功关联的点数。

### 5.4 质量检查与确认

`inspect_pose` 用核心点与三维地图最近点索引独立检查：

- `surface_ratio`：到最近地图点 ≤ `quality_surface_radius=0.30 m` 的点占比。
- `median_distance`：上述最近距离的中位数。
- `open_space_ratio`：位姿附近（`quality_nearest_limit=0.80 m`）落在栅格 clearance > `0.60 m` 区域的点占比；用于惩罚“把点云塞进墙里”的解。
- `pose_clearance`：位姿中心到最近 blocked 格点的距离。

确认条件（全部满足）：

```
surface_ratio   >= minimum_surface_ratio (0.62)
surface_points  >= minimum_surface_points (60)
open_space_ratio<= maximum_open_ratio (0.30)
pose_clearance  >= minimum_pose_clearance (0.05)
```

此外，若第二名的 `surface_ratio` 与第一名相差小于 `ambiguity_margin=0.08`，视为歧义，拒绝确认。置信度由 `surface_ratio`、`(1 - open_space_ratio)` 和 `coherence` 组合而成。

### 5.5 跟踪

- 跟踪时只做 NDT 精配准，不做全局搜索。
- 接受条件：`overlap >= minimum_tracking_overlap`、`coherence >= minimum_tracking_coherence`、`surface_ratio >= 0.5 * minimum_surface_ratio`。
- 连续 `missed_limit=4` 帧不达标则进入 `LOST`；`automatic=true` 时下一帧重新做全局搜索。
- 跟踪阶段不做 heading 分区搜索，因此单帧耗时通常在毫秒级；全局搜索在 5 个验证场景中约 170–230 ms（8 线程、Release，随 CPU 负载波动）。

---

## 6. 验证

### 6.1 五场景离线验证

数据位于 `bbs_test/recordings/body/*.npz`。每个场景包含一段 FAST-LIO 机体坐标系点云和已知平面参考位姿；`tools/run_validation.py` 为每个场景取一帧有效点云，写成 PCD，调用 `relocalizer_probe`，与参考位姿比较。

```bash
cd ~/Desktop/IRobot-SentryNav27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/lm/backend/relocalization/tools/run_validation.py --threads 8
```

结果（Release，8 线程；耗时为一次测量，随 CPU 负载波动）：

| scene (x, y, heading) | estimate (x, y, heading) | 距离误差 | 角度误差 | confidence | 搜索耗时 |
|---|---|---|---|---|---|
| `x:0_y:0_yaw:0` | 0.054, −0.024, 0.22° | 0.060 m | 0.22° | 0.904 | 193 ms |
| `x:0_y:0_yaw:90` | −0.125, 0.081, 88.80° | 0.149 m | −1.20° | 0.894 | 200 ms |
| `x:0_y:0_yaw:180` | −0.257, −0.101, 179.46° | 0.277 m | −0.54° | 0.877 | 199 ms |
| `x:0_y:-2.0_yaw:90` | −0.011, −2.154, 88.59° | 0.154 m | −1.41° | 0.918 | 168 ms |
| `x:0.9_y:1.5_yaw:0` | 1.042, 1.441, 0.02° | 0.153 m | 0.02° | 0.883 | 205 ms |

**5/5 场景在 0.5 m / 5° 容差内确认。** 最大距离误差 0.277 m，最大角度误差 1.41°。录制数据中包含部分 FAST-LIO 发散帧（z 可达 1e6）；验证脚本为每个场景选取已知有效的帧索引，离线探针本身也会拒绝无结构点云。

### 6.2 端到端 ROS 冒烟测试

该测试启动真实节点，把同一帧点云以 `PointCloud2` 发布到 `/sweep_test`，检查 TF、pose、status，并调用一次服务。请先执行 6.1 的 `run_validation.py`，它会在 `.validation/` 下生成示例帧 PCD。

```bash
cd ~/Desktop/IRobot-SentryNav27

# 终端 A：启动节点（示例地图）
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_LOG_DIR=$PWD/.validation/roslog
export ROS_HOME=$PWD/.validation/roshome
ros2 launch relocalizer relocalizer.launch.py sweep_topic:=/sweep_test automatic:=true \
  plan_descriptor:="$PWD/bbs_test/bbs_test_record/mid360_pgo_csf.yaml" \
  surface_map:="$PWD/bbs_test/bbs_test_record/map_consistent.pcd"

# 终端 B：发布一帧并检查
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_LOG_DIR=$PWD/.validation/roslog
export ROS_HOME=$PWD/.validation/roshome
python3 src/lm/backend/relocalization/tools/ros_smoke_test.py \
  --sweep .validation/x_0_y_0_yaw_0_120.pcd \
  --body-frame body --world-frame map \
  --expect-x 0 --expect-y 0 --expect-heading 0
```

实测结果：

```json
{
  "ok": true,
  "transform": {
    "x": 0.0545, "y": -0.0242, "heading_deg": 0.219,
    "distance_error": 0.0596, "heading_error": 0.219
  },
  "status": {"state": "TRACKING", "confidence": 0.904},
  "service": {"success": true, "message": "confirmed"}
}
```

> 沙箱环境中 `/home/pgd/.ros` 为只读，因此示例显式设置了 `ROS_LOG_DIR` 与 `ROS_HOME`。在普通用户环境中可省略。若节点与测试不在同一个 shell/网络命名空间中启动，DDS 发现可能失败；建议按上面两个终端的方式运行，或写成同一脚本。

### 6.3 离线探针

`relocalizer_probe` 不依赖 ROS，适合快速检查地图与单帧点云：

```bash
./build/relocalizer/relocalizer_probe \
  --plan /path/to/map.yaml \
  --surface /path/to/map.pcd \
  --sweep /path/to/sweep.pcd \
  --reference 0,0,0 \
  --threads 8 \
  --verbose
```

参数：

- `--plan`、`--surface`、`--sweep`：三个输入。
- `--reference x,y,heading_deg`：可选，打印误差。
- `--threads`：OpenMP 线程数。
- `--coarse-step`、`--coarse-floor`、`--fine-floor`、`--heading-step`、`--affinity-sigma`、`--keep`：覆盖搜索参数。
- `--verbose`：打印每个 proposal 的 support、coherence、surface ratio 等。

退出码：`0` 表示确认，`2` 表示拒绝，`1` 表示输入/加载错误。

---

## 7. 参数说明

参数全部为节点私有参数（`config/relocalizer.yaml` 中位于 `relocalizer/ros__parameters`）。常用参数如下。

### 7.1 输入与坐标系

| 参数 | 默认值 | 说明 |
|---|---|---|
| `plan_descriptor` | 空字符串 | PGM 的 YAML 描述文件 |
| `surface_map` | 空字符串 | 三维 PCD 地图 |
| `world_frame` | `map` | 输出位姿/TF 的父坐标系 |
| `body_frame` | `base_link` | 输入点云所在坐标系 |
| `odom_frame` | `""` | 非空时改为发布 `world -> odom` |
| `sweep_topic` | `/cloud_registered_body` | 实时点云话题 |
| `spark_height` | `0.0` | 发布位姿/TF 的 z 值 |
| `initial_pose` | `[]` | 可选 `[x, y, heading_deg]` 初始先验；服务/先验话题优先 |

### 7.2 搜索与配准

| 参数 | 默认值 | 说明 |
|---|---|---|
| `affinity_sigma` | `0.50` | 距离场高斯宽度（米） |
| `voxel_spans` | `[2.0, 1.0]` | NDT 分辨率层级（米） |
| `voxel_floor` / `voxel_ceiling` | `0.25` / `3.00` | 地图 z 过滤 |
| `bag_span` / `bag_radius` | `0.5` / `0.8` | 最近点索引与开放空间查询半径 |
| `sweep_search_floor/ceiling` | `0.35` / `2.20` | 粗搜索 z 带 |
| `sweep_core_floor/ceiling` | `0.30` / `2.30` | 精配准/检查 z 带 |
| `sweep_search_cell` / `sweep_search_limit` | `0.15` / `800` | 粗搜索降采样 |
| `sweep_core_cell` / `sweep_core_limit` | `0.20` / `1500` | 核心点降采样 |
| `search_heading_step_deg` | `10.0` | heading 分区宽度 |
| `search_coarse_step` | `0.40` | 初始平移步长 |
| `search_coarse_floor` | `0.45` | 粗搜索 support 门限 |
| `search_max_positions` | `22000` | 单次搜索最大格点×heading 数 |
| `search_coarse_keep` | `200` | 粗搜索保留数 |
| `search_fine_step` | `0.20` | 细化平移步长 |
| `search_fine_heading_deg` | `2.0` | 细化 heading 步长 |
| `search_fine_floor` | `0.55` | 细化 support 门限 |
| `search_fine_keep` | `24` | 细化保留数 |
| `search_place_separation` | `1.0` | 平移 NMS 半径 |
| `search_keep_hypotheses` | `12` | 最终 proposal 数 |
| `search_threads` | `0` | OpenMP 线程数，0 为默认 |
| `align_max_iterations` | `40` | NDT 最大迭代 |
| `align_maximum_shift` / `align_maximum_turn` | `0.8` / `0.5` | 单步上限（米/弧度） |
| `align_minimum_overlap` | `25` | 关联点不足则停止 |

### 7.3 质量门限

| 参数 | 默认值 |
|---|---|
| `minimum_surface_ratio` | `0.62` |
| `minimum_surface_points` | `60` |
| `maximum_open_ratio` | `0.30` |
| `minimum_pose_clearance` | `0.05` |
| `ambiguity_margin` | `0.08` |
| `minimum_tracking_coherence` | `0.35` |
| `minimum_tracking_overlap` | `0.35` |
| `missed_limit` | `4` |
| `search_period_sec` | `1.5` |

---

## 8. 调参与排障

- **完全搜不到**：先看 `~/proposals` 和 `~/status.note`。若 proposals 为空，检查 `plan_descriptor` 是否指向正确 PGM/YAML、点云 z 带是否与地图一致、`sweep_topic` 是否为机体坐标系。可用 `relocalizer_probe --verbose` 判断是“support 太低”还是“质量门限太严”。
- **找到了但被拒绝**：看 `surface_ratio`、`open_space_ratio`、`pose_clearance`。室内狭窄场景可适当降低 `minimum_surface_ratio`（如 0.55），或在点云噪声大时增大 `quality_surface_radius`（如 0.35–0.40）。
- **重复定位/歧义**：提高 `ambiguity_margin`，或增加 `search_heading_step_deg` 的采样密度（减小到 5° 会增加约 2 倍搜索时间）。
- **长走廊/对称场景**：本算法只使用几何信息，长走廊中沿走廊方向的平移天然不可观。建议融合轮速计/IMU 先验，或使用 `/initialpose` 给纵向约束。
- **动态障碍**：质量检查会惩罚开放空间中的点；人和车辆较多时，可把 `maximum_open_ratio` 放宽到 0.4–0.5，并依赖跟踪阶段的连续性。
- **CPU 占用高**：减小 `search_max_positions` 或增大 `search_coarse_step`；设置 `search_threads` 限制线程数；把 `search_period_sec` 增大。
- **TF 冲突**：若已有其他节点发布 `map -> odom` 或 `map -> body`，请设置 `odom_frame` 走 `map -> odom` 间接模式，或关闭本节点的 TF 发布（修改节点代码/参数，当前版本未提供独立开关）。

---

## 9. 命名纪律

为满足“不得与旧项目变量同名、不得复制代码”的要求，本包采用一套独立的领域词汇：

| 本包用语 | 含义 |
|---|---|
| plan atlas | 二维 blocked 栅格及其距离/affinity 场 |
| surface atlas | 三维 PCD 的多分辨率 NDT 体素索引 |
| sweep | 一帧实时点云及其二维切片 |
| spark | 粗搜索得到的初始位姿假设 |
| proposal | 经过细化、待确认的位姿假设 |
| support | 栅格 affinity 平均分（粗搜索） |
| coherence | NDT 内点比例（精配准质量） |
| placement | 平面位姿 `(x, y, heading)` |
| confirmed | 通过质量门限、可以发布的结果 |

实现上：

- 算法是“affinity 场多分辨率搜索 + NDT Gauss–Newton + 独立最近点检查”，与原工程的 BBS 分支限界、FFT 命中图、NDT_OMP 配准不是同一套代码路径。
- 没有 include/链接原工程的头文件或库；PGM/PCD 解析器、体素哈希、距离变换、搜索器均为本包实现。
- 仅保留 ROS 标准接口名称（消息字段/类型、`/tf`、`map`/`odom`/`base_link` 等）、YAML 地图格式键（`image`、`negate`、`occupied_thresh` 等）以及 `x`/`y`/`pose`/`state` 这类通用编程词；这些是框架/格式契约，不构成对原工程自定义命名的复用。
- 若后续扩展引入新类型，应继续沿用上述词汇表；新增参数名先查旧工程清单，避免复用。

---

## 10. 已知限制

- 仅估计 4-DoF 中的 `(x, y, heading)`；z、roll、pitch 由外部里程计/固定安装保证。
- 依赖二维栅格中的 blocked 格点；若 PGM 只有 free/unknown、没有墙，粗搜索会退化。
- 三维地图必须与二维地图同源、同坐标系；两者轻微错位会直接降低 `surface_ratio`。
- 全局搜索是穷举式多分辨率搜索，超大园区（>200 m × 200 m）需要增大 `search_coarse_step` 或分区域建图。
- 动态物体多、玻璃/镜面多的场景，几何一致性检查可能误拒；此时应放宽门限并依赖跟踪。
- 未实现多帧投票、回环或地图增量更新；这些属于后续扩展。

---

## 11. 许可

Apache-2.0。`bbs_test/` 中的录制数据与地图仅用于本仓库的验证，版权归原数据提供者。
