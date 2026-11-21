# sentry_navigation

整车入口见[工作区 README](../../../README.md)。本包提供 Super-LIO 到 `/odom` 的桥接、导航 TF 检查和 Nav2 启动编排。

自动重定位时由 `relocalizer` 提供 `map → odom`。`tf_only_odom` 发布静态安装外参 `base_link → livox_frame`，并由 `/lio/robo/odom` 推导 `odom → base_link`。在手动模式下它也根据配置和 `/initialpose` 维护 `map → odom`。

单独调试导航编排时，通过参数传入地图和 Nav2 配置：

```bash
ros2 launch sentry_navigation navigation.launch.py \
  map:=/绝对路径/map.yaml params_file:=/绝对路径/nav2_params.yaml
```
