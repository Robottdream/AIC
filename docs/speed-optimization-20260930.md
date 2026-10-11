# 车速与导航闭环优化（2026-09-30）

本轮修改 `/mnt/e/workspace/AIC`，同步并构建于 `/home/polarbear/ws_aic`。所有比赛任务通过项目一键脚本 `./ros_competition_start.sh --headless` 启动的仿真执行，动态障碍物保留，任务顺序、机械臂动作、地图和导航包络保持原值。

## 最终配置

- 两份正式导航参数从 DWB 改为本机已安装的 Regulated Pure Pursuit，期望直行速度 0.70 m/s，按曲率及障碍距离减速，接近目标时降速。原地转向 1.00 rad/s，速度平滑器角速度上限 1.20 rad/s，线/角加速度上限 0.35 m/s²、1.20 rad/s²，保留更强的制动能力。
- Gazebo 差速驱动 `max_wheel_acceleration` 从 5.0 改为 0.0，由 Nav2 速度平滑器统一控制加减速，轮径、轮距和 5 N·m 扭矩保持原值。导航链中仍有加速度限制，并非取消整车限加速。
- `tools/ros_env.sh` 默认启用本机 DDS 通信，并加载 `tools/fastdds_local.xml` 的环回接口与单播初始对端。ROS setup 会预置 `ROS_LOCALHOST_ONLY=0`，因此使用项目覆盖项 `AIC_ROS_LOCALHOST_ONLY`，默认值为 1。远程 ROS 场景可在 source/启动前设为 0；用户提供的 XML 路径保留。
- 本配置不是纯 UDP：Humble 的 Fast DDS 本机模式还会添加共享内存传输。不能声称已经证明共享内存是唯一根因。
- `tools/preflight_check.py` 新增局部/全局代价地图足迹时间戳检查。超过仿真时钟 3 秒、向未来偏移超过 1 秒或缺少足迹时，返回 `FAIL:11`，沿用现有 Nav2 自愈流程。只检查 TF 和规划起点无法识别所有控制器时钟停更。
- 增加 `tools/speed_probe.py`、`tools/summarize_speed.py`，记录墙钟/仿真耗时、命令与实际速度、IMU 倾角、代价地图时钟及恢复次数。修正 `bright_costmap.py` 行尾，避免脚本直接执行时退出 127。

## 当前对照结果

同一冷启动场景下，两轮前三件选择的物块、接近方向、目标停车位一致；动态障碍物相位并未锁定。命令下发至 `place_succeeded` 计时，包含解析、选块、导航和抓放；任务间 3 秒等待不计入合计。

| 任务 | 0.58 m/s 基线墙钟 | 最终配置墙钟 | 基线仿真时间 | 最终仿真时间 |
| --- | ---: | ---: | ---: | ---: |
| 蓝块→B，blue_cube_5 | 52.93 s | 41.74 s | 52.70 s | 41.40 s |
| 红块→A，red_cube_4 | 61.93 s | 62.25 s | 61.80 s | 62.10 s |
| 蓝块→A，blue_cube_2 | 25.80 s | 21.16 s | 25.80 s | 21.30 s |
| 合计 | 140.66 s | 125.15 s | 140.30 s | 124.80 s |

前三件墙钟减少约 **11.0%**；单件并非全部提速。两轮仿真时间与墙钟接近，因此本轮没有把严重仿真降速误计为速度收益。这是本机本轮对照结果，不是多个障碍相位的统计结论，也没有测量裁判评分。

最终默认配置冷启动后的六件结果如下（未额外手动 export 通信变量或传入临时参数）：

| 顺序 | 任务 | 墙钟 | 仿真时间 |
| --- | --- | ---: | ---: |
| 1 | 蓝→B | 41.74 s | 41.40 s |
| 2 | 红→A | 62.25 s | 62.10 s |
| 3 | 蓝→A | 21.16 s | 21.30 s |
| 4 | 红→C | 76.99 s | 76.90 s |
| 5 | 蓝→C | 41.39 s | 41.40 s |
| 6 | 红→B | 62.34 s | 62.10 s |
| 合计任务耗时 | | **305.87 s** | **305.20 s** |

六个物块中心均在目标区内；最大 IMU 单轴倾角 0.0278°，导航恢复次数均为 0，六件执行期间看门狗未重启 Nav2。局部/全局代价地图时间戳最大落后 0.943/1.143 秒。轮后独立体检 `PREFLIGHT=OK`。实际底盘最高线速度约 0.65 m/s，因此 0.70 m/s 是目标/输出上限，不是宣称实际轮速全程达到该值。C 区两件此次未出现恢复循环，但不据单轮保证所有动态障碍相位均无墙角卡顿。

**仍存在长期稳定性风险。** 六件完成并通过独立体检后，继续空闲观察时，看门狗于 12:36:47 检测局部足迹时间戳落后 7.707 秒，重启 Nav2 并于 12:38:04 复检成功；12:42:17 又检测落后 38.103 秒并触发重启。它检测到的是代价地图时间戳/接收链停更，尚未确定具体 DDS 或节点时钟根因。任务用时结论只覆盖六件执行期间，不能把本机通信配置写成根治。完整后续日志保存在 `post-audit-run-logs/`。本轮结束已用一键脚本停止全部模块，无残留。

证据：`log/speed-20260930/final-default/` 中的 `trace.jsonl`、`results.json`、`summary.json`、`runtime_audit.json`、`preflight.txt`、`controller-params.yaml`、`smoother-params.json` 及 `run-logs/`。ROS CLI 的短期图发现曾漏掉参数节点，直接参数服务复核成功；没有用 CLI 一次查不到节点就断言运行节点已退出。完整轮次还包含探针主动插入的任务间等待，第一条命令至最后一次放置总跨度 **320.93 秒**，不能把合计任务耗时当成这个总跨度。

## 失败试验与改进证据

初次基线首件因规划起点停在启动位置，被看门狗重启中断；复测前三件才用于对照。仅把 DWB 平滑器上限升到 0.70 m/s 的试验，蓝 B 为 61.08 秒，红 A 返程倾翻，已撤回。

RPP 最初仍在启动位持续转圈，控制器日志使用固定的 100.599 秒时间戳，而外部仿真已走到约 190 秒。仅使用本机通信限制也曾发生局部时钟停更和主控路径服务未就绪。加入单播发现配置后的一轮 RPP（原地转向 0.60 rad/s，旧驱动限加速）前三件 165.76 秒；关闭驱动插件二次限加速后，另一轮六件全部完成，合计 327.66 秒，六个物块中心均在目标区，零恢复、无看门狗重启。这一轮保存在 `log/speed-20260930/rpp070-direct/`。

新的时钟检查在故障现场报告局部/全局时钟落后 256.188/152.488 秒；另一轮看门狗检测局部落后 35.123 秒并成功自愈。相关记录在 `final070/clock-freeze-preflight.txt` 和 `final070-local/run-logs/11_watchdog.log`。足迹缺失可能也来自通信或代价地图未就绪；检查检测的是不可用状态，不能单凭它认定具体 DDS 缺陷。

插件源码的非零限加速分支会累积内部轮速指令，部分直接设置目标速度的分支未同步该内部值；移除这层斜坡后实测减少了绕行。该现象支持统一加减速控制，但不能把所有倾翻都归因于这个分支。依据：[Gazebo 差速驱动源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)、[Humble RPP 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)、[Humble Fast DDS participant 配置](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/participant.cpp)。

## 复现

```bash
cd ~/ws_aic
./ros_competition_start.sh --headless
# 等一键脚本完整输出启动结果后，再执行以下命令。
source /opt/ros/humble/setup.bash
source install/setup.bash
source tools/ros_env.sh
python3 tools/speed_probe.py --output log/speed-repeat
python3 tools/summarize_speed.py log/speed-repeat
./ros_competition_start.sh doctor
./ros_competition_start.sh stop
```

构建、Python/shell 语法、两份导航参数一致性、XML 解析，以及新鲜/冻结/未来时间戳判断、DDS 默认/远程覆盖/自定义 XML 保留检查已通过。原始运行记录在 `/mnt/e/workspace/AIC/log/speed-20260930/`。
