# 算法、地图格式与参数参考

本文件保留原实现的算法与参数说明。迁移、构建和启动步骤以包根目录 README 为准。按用户要求，已知的粗搜索重复旋转问题未修改；历史精度结果不能替代目标机器验收。

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


## 7. 参数说明

参数全部为节点私有参数（`config/relocalizer.yaml` 中位于 `relocalizer/ros__parameters`）。常用参数如下。

### 7.1 输入与坐标系

| 参数 | 默认值 | 说明 |
|---|---|---|
| `plan_descriptor` | 空，必须提供 | PGM 的 YAML 描述文件 |
| `surface_map` | 空，必须提供 | 三维 PCD 地图 |
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

