# stable 雷达障碍残影修正

用户截图中有多个孤立占用点及其膨胀区域。本次修改当前 stable 基础，
同步到 /home/polarbear/ws_aic。未改机器人、任务顺序或速度参数。

运行参数确认修正前局部和全局 obstacle_layer.scan.inf_is_valid 均为 false。
Gazebo 雷达配置为 samples=360、resolution=0.5。

## 修正

- originbot_nav2.yaml 和备用 originbot_nav2b.yaml 的局部、全局扫描源均开启 inf_is_valid。
- lidar_gazebo.xacro 使用 1441 条真实射线，resolution=1，避免插值回波及稀疏清障漏格。
- 保持 marking、膨胀层和原有距离保护。无回波端点在量程上限，不进入近处障碍标记范围。

正无穷光束开启此选项后参与清障，依据
[Humble ObstacleLayer 源码](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)。
稀疏射线仍可能漏掉此前标记的栅格，因此同时增加真实射线密度。

## 验证（2026-10-05）

在隔离 DDS 域运行本机安装的 Nav2 代价地图，模拟直径 0.75 米移动圆形障碍，
离开后持续发布无回波扫描。另保留一个固定真实障碍。

| 配置 | 离开后走廊致命障碍格 | 保留真实障碍占用 |
| --- | ---: | ---: |
| 361 束，inf_is_valid=false | 133 | 100 |
| 361 束，inf_is_valid=true | 31 | 100 |
| 1441 束，inf_is_valid=true | 0 | 100 |

361 束是精确 1 度采样的对照，不能逐点复原原 Gazebo 的 360×0.5 配置或用户截图。
结果在 log/ghost-obstacles-20261005/clearing-check.json。
隔离测试最初因默认 DDS 传输无法收到扫描而失败；测试现使用独立 Cyclone DDS
环回单播小包配置。此设置仅用于测试，不改变生产 DDS 配置。

运行副本重建 bot_navigation 和 mybot_description 成功。
等待原三件红块任务完成后重启；WSL 随后重启，2026-10-05 再次无界面冷启动通过。
实际全局、局部 inf_is_valid 均查询为 true。
真实 /scan 收到 116 帧，全部 1441 束；仿真频率约 10.00 Hz，墙钟约 8.02 Hz。
该时段实时因子约 0.80，单次测量未对比旧配置的性能。

本轮不发送搬运指令。未逐格检查 Foxglove 截图；遮挡后未重新观测的位置和
地图内部位姿停更仍可能影响清障，不能据此保证所有场景没有残影。
仿真保持启动，等待 /command 人工输入。

旧配置备份：/home/polarbear/ws_aic_backups/before-ghost-fix-20261004。
