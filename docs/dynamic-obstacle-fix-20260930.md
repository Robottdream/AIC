# 移动障碍物处理修正与验证（2026-09-30）

本次针对“行驶过程中被移动障碍物撞到、随后飞起”的反馈，修改导航响应链和障碍物物理参数。最终通过一次受控横穿导航与一次完整抓放任务；没有飞起、倾覆或 Gazebo 崩溃。当前仍没有障碍物速度估计或轨迹预测，不能据此保证所有相遇方向都不会碰撞。

## 检查结果

- 故障现场备份：`log/collision-20260930/before/`。障碍物日志出现水平速度 23.4356 m/s、竖直速度 -23.534 m/s，随后 gzserver 因 Ogre 包围盒断言退出。该日志证明场景发生物理失稳，但没有碰撞传感器数据，不能单凭日志认定具体哪个接触点先失稳。
- NavFn 按当前全局代价地图寻路；RPP 跟踪已有路径，其碰撞预测投射车辆自身运动，不预测障碍物未来位置。原默认行为树每秒重规划一次，局部地图每秒更新 5 次，控制失败容忍仅 0.3 秒，且恢复首先清空地图、随后可能旋转/倒车。
- 障碍物插件将角阻尼设为 2.0。Gazebo ODELink 直接将该数值传给 ODE；ODE 阻尼是每步的速度缩减比例，合法范围 0～1，超过 1 会使速度每步反向。见 [Gazebo ODELink 源码](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODELink.cc) 和 [ODE 阻尼说明](https://www.ode.org/wiki/index.php/Manual#Damping)。同时 0.3 kg 障碍物允许施加 10 N 推力，立方体惯量也与质量、尺寸不一致。

## 最终改动

速度输出统一为：

```text
controller_server / behavior_server
    → /cmd_vel_nav → velocity_smoother
    → /cmd_vel_unfiltered → collision_monitor
    → /cmd_vel_guarded → scan_velocity_guard → /cmd_vel → 底盘
```

使用本机已安装的 Humble Collision Monitor，不增加安装依赖。局部维护的 `safe_bringup.launch.py`、`safe_navigation.launch.py` 基于已安装 Nav2 的 Apache 2.0 启动文件，保留原许可证；只调整速度路由及行为树路径。恢复动作也必须经过平滑器和防碰撞链路，最终 `/cmd_vel` 的唯一发布者为 `scan_velocity_guard`，已检查实际 ROS 图。

| 配置 | 最终值/行为 |
| --- | --- |
| 控制频率 | 10 → 20 Hz |
| 局部/全局地图更新 | 5/2 → 10/5 Hz |
| 全局路径重规划 | 1 → 2 Hz |
| 控制失败容忍/进展超时 | 3/20 秒，为短暂让行留时间 |
| 近障停车区 | 前 0.38 m、后 0.40 m、左右 0.42 m；至少 3 个雷达点 |
| 碰撞时间减速 | 基于车辆包络及当前运动，保留 2 秒时间余量 |
| 雷达/命令失联保护 | 雷达接收或时间戳过期 0.6 秒、命令过期 0.3 秒则输出零；使用独立墙钟定时器 |
| 恢复动作 | 优先等待和重规划；较后才清图并等待传感器重新标记，取消自动旋转/倒车恢复 |
| 障碍物线/角阻尼 | 0.001 / 0.01 |
| 障碍物最大推力 | 10 → 1.5 N，速度反馈增益 12；端点附近避免被地面摩擦卡住 |
| 障碍物惯量 | 各主轴 0.028125 kg·m²，符合 0.3 kg、边长 0.75 m 立方体 |
| 障碍物接触 | 摩擦 0.2，刚度 100000、阻尼 100、接触修正速度 0.5 m/s，允许深度 0.001 m |

Humble 版本会忽略过期观测并继续输出速度，因此额外的 `scan_velocity_guard` 放在最终输出端；Collision Monitor 进程失联时它也能停车。Collision Monitor 的碰撞时间检查使用当前观测点，详见 [Humble polygon.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/polygon.cpp)，这不等于动态物体轨迹预测。

一键启动增加 Collision Monitor active 与速度保护节点就绪门，停止/恢复清扫名单同时更新。任务顺序、物块坐标、抓放动作和 0.70 m/s 直行上限保持原设置。

## 验证

最终仿真经 `/home/polarbear/ws_aic/ros_competition_start.sh --headless` 启动，启动体检通过；横穿与抓放期间看门狗没有重启 Nav2。证据在 `log/collision-20260930/`，日志默认不纳入 Git。

1. `tools/check_collision_monitor.py`：独立话题测试七项通过，不向真实底盘发命令。畅通 0.6 m/s；前方 0.6 m 障碍按碰撞时间减至 0.135 m/s；进入停车区为零；障碍消失恢复；雷达停更、监控进程退出均停车；近处观测不与旋转包络相交时保持 0.6 rad/s 原地转向。结果 `monitor-check.json`。
2. `tools/crossing_probe.py`：在不与车辆重叠的位置重置 obstacle3，然后请求车辆从原点附近前往 `(-4.5, 0)`，障碍物继续沿原轨道横穿。29.83 秒到达，日志明确记录 `Robot to stop due to NearStop polygon`，随后继续导航。最近中心距离 0.67965 m，实际线速度峰值 0.65001 m/s，最大单轴倾角 0.02382°。这是中心距离与姿态检查，没有直接测量接触力；不能把它当作“零接触”证明。结果 `crossing/`。
3. 横穿后下发 `抓取1个蓝色去B`：选择 blue_cube_5，11.69 秒完成抓取前导航，40.10 秒放置成功，仿真耗时 39.70 秒；导航恢复次数 0，最大单轴倾角 0.02648°，局部/全局地图最大时间戳滞后 0.135/0.199 秒。blue_cube_5 中心相对 B 区偏移 `(0.154734, 0.009139)` m，在区域边界内。结果 `blue-b-motion-aware/`。起点位于横穿任务终点，不能与此前从原点出发的提速基准直接比较。
4. 任务后独立体检 `PREFLIGHT=OK`，机器人定位与 Gazebo 真值一致，路径起点误差 0.021 m。`final-preflight.txt` 与 `blue-b-motion-aware/runtime_audit.json` 保存检查结果。
5. WSL 构建 `bot_navigation`、`mybot_description`、`gazebo_physics_obstacle_plugin` 成功；Python 语法、shell 语法和维护文件 `git diff --check` 通过。运行结束使用一键 stop，清扫无残留。

## 未采用的试验与局限

- 全局接触修正速度降为 0.5 m/s 并修改接触层的试验，使实际车速降至约 0.0325 m/s，横穿 90 秒超时。已恢复世界原全局设置；仅障碍物自身接触面保留较低修正速度。记录 `rejected-global-contact/`。
- 固定大范围减速区会同时缩小角速度，与 RPP 转向加速度反馈叠加，原地转向长时间维持约 0.03 rad/s。该试验抓放虽成功但耗时 132.98 秒，已移除固定减速区。记录 `crossing-fixed-slow/`、`blue-b-final/`。
- 第一轮速度话题重映射出现作用域泄漏，导致链路错误；已弃用通用 `SetRemap(cmd_vel)`，改为逐节点明确连接，并检查最终唯一发布者。
- 调试中仍复现既有代价地图时间戳停更。最终横穿、抓放及独立体检通过后，空闲阶段 13:49:25 再次出现 `FAIL:11`（全局时间戳滞后 18.293 秒），看门狗重启了 Nav2 并重设 AMCL；随后执行 stop，没有等待该次恢复完整复检。这不表示该根因已经解决，记录见 `final-run/11_watchdog.log`。
- 只验证了一个横穿相位及一个完整任务，未验证迎面追撞、侧后方持续逼近、全部 C 区路线或大量随机相位。停车本身无法保证阻止一个继续朝静止车行驶的障碍物；进一步改善需要基于雷达跟踪的动态速度估计、时间相关路径规划和更多相遇回归测试。

复现实验在 WSL 中 source ROS、运行工作空间的 `install/setup.bash`、`tools/ros_env.sh` 后执行上述工具。横穿探针要求新场景下机器人靠近原点，并会在安全分离的位置重新放置 obstacle3；不要在正在抓放的任务中运行它。
