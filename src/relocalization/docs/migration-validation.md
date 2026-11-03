# 独立迁移包验证

目标为 Ubuntu 24.04 + ROS 2 Jazzy。

独立包包含自己的工具、测试与文档。测试定位包根目录，离线验证定位已安装的探针；完整链路使用已加载的 ROS 环境或显式 setup 参数。构建和运行不要求原工作区目录存在。

验证日期：2026-11-02。已在压缩包解压后的新工作区完成以下验证，构建、安装和测试路径均在 `/tmp`，未使用原工作区的构建产物：

| 检查 | 结果 |
|---|---|
| Jazzy Release 构建 | 通过 |
| C++ gtest | 6 个通过 |
| Python pytest | 11 个通过 |
| 4 个已安装工具的 `ros2 run ... --help` | 全部通过 |
| 从非源码目录查找已安装离线探针 | 通过 |
| 包发现与已安装可执行程序发现 | 通过 |
| Python、XML 和 zsh 语法 | 通过 |
| 全部算法 C++ 源码、公共头文件 SHA-256 对比 | 与导出前一致 |

Python 测试覆盖自定义 YAML 与显式参数、间接 TF 复合、外部数据缺失提示、完整启动/清理命令、混合 underlay 拒绝，以及已经加载包环境时不再指定重定位 setup 的用法。完整链路测试使用命令替身，没有启动雷达或 FAST-LIO。

安装后可发现 `relocalizer_node`、`relocalizer_probe`、`extract_sweep.py`、`run_validation.py`、`ros_smoke_test.py`、`start_pipeline.zsh`。工具从包安装位置调用，日志与验证输出使用当前工作目录或显式输出路径。

DDS/UDP 在当前沙箱中受限，真实节点间的通信和硬件效果仍需在目标机器人验收。
