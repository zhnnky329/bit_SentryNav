# relocalizer — ROS 2 Jazzy 独立迁移包

将整个 `relocalizer/` 目录放入目标工作区的 `src/` 即可构建。本包使用 `ament_cmake`，目标环境为 Ubuntu 24.04 + ROS 2 Jazzy，保留原包名、节点名、话题、服务和算法参数。

本目录包含完整源码、配置、launch、RViz 配置、测试及辅助工具。它不依赖原 `relocaization` 工作区的目录结构，也不包含旧构建产物、地图、录制数据或外部导航工程源码。

## 1. 迁移与构建

在未加载 Humble 或旧 overlay 的干净 bash 中：

```bash
mkdir -p ~/robot_ws/src
# 将本目录复制到 ~/robot_ws/src/relocalizer
# 或解压发布压缩包，压缩包顶层目录就是 relocalizer/
tar -xzf /path/to/relocalizer-jazzy.tar.gz -C ~/robot_ws/src

cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --packages-select relocalizer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash

colcon test --packages-select relocalizer
colcon test-result --verbose
```

`rosdep` 首次使用需先在普通终端执行 `sudo rosdep init`、`rosdep update`。本包可以放入现有 Jazzy 工作区，不要求目标工作区使用固定名称或自定义 build/install 目录。复制目录和解压压缩包两种方式任选其一。

## 2. 目录与依赖

```text
relocalizer/
├── package.xml
├── CMakeLists.txt
├── include/relocalizer/  # 公共头文件
├── src/                 # 算法、ROS 节点、离线探针
├── config/              # 参数模板
├── launch/              # ROS 2 launch
├── rviz/                # RViz 显示配置
├── scripts/             # 录制数据提取
├── tools/               # 完整链路启动、离线验证、ROS 冒烟
├── test/                # C++ 与 Python 测试
├── docs/                # 算法、参数与迁移说明
├── LICENSE
└── README.md
```

核心依赖为 rclcpp、ROS 标准消息、tf2、Eigen3 和 yaml-cpp；OpenMP 可选，不依赖 PCL。Python 工具依赖 NumPy，ROS 冒烟依赖 rclpy/TF，一键链路脚本依赖 zsh。

Livox、FAST-LIO、`bw_fastlio_nav2` 是可选完整链路的外部依赖，单独运行重定位节点不要求安装它们。现场地图必须包含同坐标系的二维 YAML/PGM 和三维 PCD，实时输入必须是机体坐标系下的 `PointCloud2`。

## 3. 启动重定位

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch relocalizer relocalizer.launch.py \
  plan_descriptor:=/maps/site.yaml \
  surface_map:=/maps/site.pcd \
  sweep_topic:=/cloud_registered_body \
  body_frame:=body world_frame:=map \
  odom_frame:=camera_init rviz:=false
```

这里的 `camera_init -> body` 由上游里程计提供，本节点发布 `map -> camera_init`。如果没有上游里程计并需要直接发布 `map -> body`，省略 `odom_frame` 即可；若要清空自定义 YAML 中的值，使用 `odom_frame:='""'`。

参数优先级为显式 launch 参数 > 参数 YAML > 节点默认值。地图模板中的路径为空，必须提供现场地图。可复制 `config/relocalizer.yaml` 为现场配置并使用 `config:=/path/site_parameters.yaml`，未显式传入的参数会保留 YAML 中的值。

`rviz` 默认为 `true`；无图形界面时设置 `rviz:=false`。独立打开 RViz：

```bash
rviz2 -d "$(ros2 pkg prefix relocalizer)/share/relocalizer/rviz/relocalizer.rviz"
```

常用接口：

| 接口 | 类型 | 用途 |
|---|---|---|
| `/relocalizer/pose` | `geometry_msgs/PoseStamped` | 地图坐标系位姿 |
| `/relocalizer/status` | `std_msgs/String` | JSON 状态和置信度 |
| `/relocalizer/proposals` | `geometry_msgs/PoseArray` | 自动全局搜索候选 |
| `/relocalizer/plan` | `nav_msgs/OccupancyGrid` | 二维地图显示 |
| `/relocalizer/surface_map` | `sensor_msgs/PointCloud2` | 三维地图显示 |
| `/relocalizer/relocalize` | `std_srvs/Trigger` | 手动触发全局重定位 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 地图坐标系初始先验 |

```bash
ros2 service call /relocalizer/relocalize std_srvs/srv/Trigger '{}'
```

## 4. 完整传感器链路

目标工作区需另外构建现场使用的 Jazzy 版 `livox_ros_driver2`、`fast_lio`，使用滤除时还需 `bw_fastlio_nav2`。先确认上游配置中的雷达 IP、订阅话题、机体 frame、里程计 frame 和外参。

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
ros2 run relocalizer start_pipeline.zsh \
  --nav-setup /path/to/upstream/install/setup.zsh \
  --plan /maps/site.yaml --surface /maps/site.pcd \
  --livox-launch msg_MID360s_launch.py \
  --fastlio-params /path/to/mid360.yaml \
  --odom-frame camera_init --body-frame body --no-rviz
```

脚本使用当前已加载的重定位包，不再假设原工作区存在 `install_jazzy/`。也可在直接运行源码脚本时用 `--relocalizer-setup /path/install/setup.zsh` 显式加载。上游已加载时可以省略 `--nav-setup`。

链路为 `map -> camera_init -> body -> base_footprint`。已有上游节点时直接启动重定位，避免重复启动雷达、FAST-LIO 或重复发布 TF。

其他选项：

- `--no-filter`：不启动 `bw_fastlio_nav2` 地面/障碍滤除。
- `--filter-params /path/filter.yaml`：使用现场滤除参数。
- `--no-base-tf`：底盘 TF 已由其他节点发布时使用。
- `--base-transform X Y Z ROLL PITCH YAW`：设置机体到底盘的平移与旋转，单位为米、弧度。默认保留原底盘值 `-0.14 0 -0.18 0 0 0`，迁移时需核对。
- `--sweep-topic`、`--world-frame`、`--body-frame`、`--odom-frame`：匹配现场话题与 TF。话题重映射不会改变消息的 frame。
- `--manual`、`--timeout 90`、`--log-dir /path/logs`：手动定位、首帧等待和日志目录。

默认日志写入运行命令时当前目录的 `.pipeline/<时间戳>/`。完整链路要求非空且独立的里程计 frame；驱动 launch 文件名和所有上游参数按实际版本指定。

## 5. 工具与验证

工具通过 `ros2 run` 调用，不依赖源码所在的绝对路径。

```bash
# 从外部 NPZ 中提取一帧，FRAME_INDEX 为整数
ros2 run relocalizer extract_sweep.py /data/scene.npz 120 /data/sweep.pcd

# 五场景录制数据验证；默认查找本包安装的离线探针
ros2 run relocalizer run_validation.py \
  --recordings /data/recordings/body \
  --plan /maps/site.yaml --surface /maps/site.pcd --threads 8

# 也可直接调用无 ROS 通信的离线探针
ros2 run relocalizer relocalizer_probe \
  --plan /maps/site.yaml --surface /maps/site.pcd --sweep /data/sweep.pcd \
  --reference 0,0,0 --threads 8
```

离线验证输出默认放在当前目录 `.validation/`。自定义探针或输出位置可使用 `--probe`、`--workdir`。五场景 NPZ 不随本包提供，场景名与已选帧索引沿用原验证脚本。

ROS 冒烟测试需要另外一个终端启动节点，并令节点使用 `sweep_topic:=/sweep_test`：

```bash
ros2 run relocalizer ros_smoke_test.py --sweep /data/sweep.pcd \
  --body-frame body --world-frame map \
  --expect-x 0 --expect-y 0 --expect-heading 0
```

间接 TF 模式追加 `--odom-frame camera_init`。在没有真实里程计的隔离测试中，可追加 `--odom-pose 1.2 -0.7 25` 发布已知里程计变换；不与真实 FAST-LIO 同时使用。`--initial-pose X Y HEADING_DEG` 用于测试 `/initialpose` 跟踪路径。

## 6. 保留事项与验收边界

按用户要求，原算法的 C++ 源码和公共头文件保持不变，已知粗搜索重复旋转问题保留。独立包需在目标机器验证地图加载、实时点云、服务、TF 连续性和定位精度。

本次独立导出已在解压后的新工作区通过 Jazzy Release 构建、6 个 C++ 单测和 11 个 Python 测试，4 个已安装工具入口和离线探针查找也已验证。结果见 [迁移验证记录](docs/migration-validation.md)。当前执行沙箱禁止 DDS 创建 UDP socket，端到端通信、真实硬件和 RViz 图形效果尚未验收。

算法与参数详见 [参考文档](docs/algorithm-and-parameters.md)。许可为 Apache-2.0，见 [LICENSE](LICENSE)。
