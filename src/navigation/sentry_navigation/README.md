# sentry_navigation

迁移版保留 tf_only_odom、TF 就绪检查与 Nav2 启动编排。map -> odom 为静态单位变换，base_link -> livox_frame 为安装外参，odom -> base_link 由 /lio/robo/odom 推导。

不再构建 Odin TF 和 Lightning-LM 桥接节点。导航参数与执行树位于 bringup；运行方式、定位限制和标定要求见工作区根 README。

直接启动 Nav2 编排（需先启动雷达、LIO 和 tf_only_odom）：

```bash
ros2 launch sentry_navigation navigation_launch.py map:=/绝对路径/地图.yaml use_rviz:=true
```

全局规划 SmacPlanner2D，局部控制 GoalApproachController + 官方 MPPI。
