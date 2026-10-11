# 孤立障碍残影清理（2026-09-30）

## 第二轮：开启无回波清障后仍有残影

用户再次提供障碍运动后的黑点截图。读取运行参数确认局部、全局 inf_is_valid 均为 true，排除未加载配置。上一轮只验证两个固定方向的单点障碍，未覆盖连续运动与射线栅格化，验证范围不足。

扩展隔离测试：直径 0.75 m 的圆形障碍在距离雷达约 4.75 m 的走廊连续移动，离开后持续发布空闲扫描。360 度覆盖、约 1 度间隔的光束在远处间距超过 5 cm 地图栅格，射线清障存在漏格。实际 Nav2 测得：

| 配置 | 离开后走廊残留致命障碍格 |
| --- | --- |
| 361 束，inf_is_valid=false | 127 |
| 361 束，inf_is_valid=true | 31 |
| 1441 束，inf_is_valid=true | 0 |

三组均保留未移走的真实障碍，占用值仍为 100。新增高密度扫描的残影清零断言。361 束用于隔离实验的精确 1 度采样，原 Gazebo 为 360 束，结果说明稀疏射线可复现残影，不能逐点还原截图。

修正 lidar_gazebo.xacro 的真实射线 samples 为 1441（约 0.25 度），resolution 仍为 1，避免通过插值生成伪回波。7 m 清障范围内相邻光束最大间距约 3.1 cm，低于地图栅格宽度。

同步运行副本，重建 mybot_description 后使用一键脚本 --headless 冷启动，全部就绪门和启动体检通过。20 秒实际扫描测试：低/高雷达均为 1441 束，仿真频率约 10 Hz，墙钟频率约 9.40 Hz，实时因子 0.940。两路消息均持续接收。证据 live-lidar-check.json 与 clearing-check.json 位于同一日志目录，后者现在包含三组扩展实验。新测试模拟几何光束与栅格清障；实际 Gazebo 仅验证数据流及启动，未逐格对照用户截图。

仿真当前保持运行。Nav2 内部 TF 位姿过期仍可能阻止清障处理，本修正不解决该问题，也不能清除雷达被遮挡、未重新观测的位置。尚未重复完整抓取任务。

用户截图显示多个孤立黑点及周围膨胀区域。检查当前静态地图未发现对应的孤立圆点；截图缺少世界坐标及原始扫描，不能逐个确认点的来源。

在两套 Nav2 参数中发现可复现的残影机制：扫描源未设置 inf_is_valid，Humble 默认值为 false。Gazebo 无回波的 +Inf 光束因此不参与清障；曾被移动障碍标记过的位置，在后续仅有无回波光束穿过时可能继续保持占用。即便 observation_persistence 默认为零，已写入障碍层的单元也不会自动按时间过期。

依据：[Humble ObstacleLayer 源码](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)。启用 inf_is_valid 后，正无穷读数转换为 range_max 附近的点参与射线清障。当前雷达量程 30 m，清障范围 7 m、标记范围 6 m，因此这些量程端点不会被标记为近处障碍。NaN 和负无穷没有被当作空闲读数。

## 修改与验证

originbot_nav2.yaml 与 originbot_nav2_odom.yaml 的局部、全局 obstacle_layer.scan 均设置 inf_is_valid: True。保持标记功能、膨胀半径和已有安全保护。

tools/check_obstacle_clearing.py 在独立 DDS 域启动本机安装的 Nav2 代价地图。使用合成扫描先标记 (2,0) 与 (0,2) 两个障碍，再移除第一个、持续发布 +Inf；没有向机器人发送速度命令。

| 配置 | 移开的障碍位置 | 保留的真实障碍 |
| --- | --- | --- |
| 原配置 false | 占用 100，残影复现 | 占用 100 |
| 修正 true | 空闲 0，残影清除 | 占用 100 |

结果：`/mnt/e/workspace/AIC/log/ghost-obstacles-20260930/clearing-check.json`。测试程序先前的生命周期/发现失败记录已被最终成功运行日志替换，最终报告只覆盖成功对照。

配置已同步至 `/home/polarbear/ws_aic` 并重建 bot_navigation。下一次一键启动生效。本轮未启动完整 Gazebo 任务，不能宣称截图中的所有黑点已现场消失。遮挡导致的未观测位置、静态地图内的真实标记及之前发现的 Nav2 内部 TF 位姿过期仍需分别处理；本改动不修复 TF 停更。
