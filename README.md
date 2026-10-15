# AIC

ROS 2 Humble + Gazebo Classic 搬运仿真工程。当前默认使用 `scenarios/short_routes_20261007/` 中的新办公室地图、13 个橙色静态障碍和两条斜向移动障碍轨迹。

## 当前目录

| 路径 | 用途 |
|---|---|
| `src/` | ROS 包及机器人、传感器、仿真插件源码 |
| `scenarios/short_routes_20261007/` | 默认世界、地图、任务坐标和导航参数 |
| `scenarios/usb_office_20261007/` | 导入的原始办公室场景及地图生成参考，生成器仍依赖，保留 |
| `tools/` | 进程管理、计时、地图生成及诊断工具 |
| `tests/` | 规则、路径选择、状态计时、启动及货物惯量回归 |
| `docs/` | 历史试验与修复记录；当前结果见下方链接 |
| `ros_competition_start.sh` | 主入口；`run.sh` 转发到同一入口 |
| `demo1.json` | Foxglove 布局 |
| `usb_start.sh`、`usb_check.sh` | 比赛 USB 环境入口和环境检查，默认端口 8765 |
| `test_map.sh`、`test_usb_office.sh` | 将指定场景复制到 `/tmp` 的隔离测试入口 |

`build/`、`install/`、`log/` 和 `runtime/` 属于本机运行数据，Git 忽略。模型权重、编译输出及原始运行日志不上传。已移除 `源码（国赛）/` 中未被使用的旧 `~/dev_ws` 启动脚本；历史版本仍可从 Git 查阅。

## WSL 中构建与运行

ROS 2、Gazebo、colcon 和 llama-server 均在 WSL bash 中执行。当前开发副本为 `/home/polarbear/ws_aic`。本机 `runtime/` 需要已有 llama-server 和 GGUF 权重，也可用 `LLAMA_BIN`、`MODEL` 指定绝对路径；具体查找路径见 `tools/project_launcher.py`。

```bash
cd /home/polarbear/ws_aic
bash ros_competition_start.sh build
bash ros_competition_start.sh start --headless
bash ros_competition_start.sh status
bash ros_competition_start.sh logs 09_main
bash ros_competition_start.sh stop
```

构建入口始终使用 `colcon build --symlink-install --base-paths src`。`--headless` 关闭 Gazebo 客户端和 RViz，保留仿真与可视化数据；兼容此前拼写 `--headlees`。

`start` 等待人工通过 `/command` 发自然语言任务。只有显式 `benchmark` 才自动发送三红 A、两蓝 B 的五件任务：

```bash
# 启动并执行一次独立计时；结束后退出本次启动的流程
bash ros_competition_start.sh benchmark --headless
# 或在已经启动的栈上执行；完成后保留该栈，检测到失稳时停栈
bash ros_competition_start.sh benchmark --headless --use-running
```

基准结果保存在 `log/benchmark/<时间戳>/`，含 `result.json`、`motion.json`、`phases.json`。计时从已解析任务发布到完成或中止，分别记录墙钟与仿真时间，不含自然语言解析和冷启动。

默认入口统一选择同一场景的 world、map、tasks、nav2 文件。通过 `AIC_SCENE` 选择场景；显式 `AIC_WORLD`、`AIC_MAP`、`AIC_TASK_CONFIG`、`AIC_NAV_PARAMS` 优先。

Foxglove 导入 `demo1.json`，普通主入口连接 `ws://localhost:9090`；USB 入口默认 `ws://localhost:8765`。不要同时启动两套栈。

## 当前参数与验证

2026-10-15 发布准备采用2026-10-14跑通五件的导航源码和配置。直行目标2.2 m/s、角速度4 rad/s、角加减速度10 rad/s²，单轮转矩上限1000 Nm，世界`contact_max_correcting_vel=100`，驱动轮与地面两个摩擦方向均为1；Gazebo单轮加速度限幅关闭。底盘10 kg、驱动轮各1 kg；货物质量0.1 kg、边长3 cm、主惯量0.000015 kg·m²。

新地图使用实时滚动预测规划，控制20 Hz、目标重规划5 Hz；不插入固定等待点，不封锁移动障碍整条轨道。静态净距与动态轨迹采用软代价，真实轮廓和预测碰撞检查保留；墙边距离减速和RPP代价减速关闭。默认终点检查采用本轮实测的SimpleGoalChecker。

三红A、两蓝B五件均完成，落点与高度合格，没有触发碰撞或失稳中止。按主控任务阶段事件计时为**160.78墙钟秒 / 134.5仿真秒**，不含冷启动及自然语言解析。采集器轮询计时另为162.25/136.4，详情见[100档流程与路径图](docs/limit100-flow-20261014.md)。这是单轮验证，起步、大弯及末端停姿仍有偏差，不代表长期成功率。

发布检查包括14个ROS包构建、49项Python与13项C++回归、生成器输出一致性和默认headless启动。恢复后的导航库SHA-256与本轮五件实测使用的库一致。未验证的WorldGoalChecker草案已另存本机，不包含在发布源码或默认配置中。

基准工具等待主控空闲及订阅匹配后只发一次任务，并检查接收确认；不重复发送任务。结果保留原有轮询计时，并在完整完成时额外输出`task_event_wall_seconds`、`task_event_sim_seconds`。

- [小地图测试与绘图交接](docs/small-map-testing-handoff-20261015.md)
- [100档五件实测、路径与速度图](docs/limit100-flow-20261014.md)
- [ODE约束20/34/60对照与真实关节转速](docs/contact-limit-20261014.md)
- [固定摩擦1的扭矩对照](docs/torque-sweep-20261014.md)
- [墙角坐标、净距与制动检修](docs/corner-frame-and-braking-20261014.md)
- [实时预测路径与历史试验](docs/live-obstacle-routing-20261013.md)
- [货物惯量修正证据](docs/cargo-inertia-and-turning-20261012.md)
- [规则输入格式](docs/formal-problem-20261009.md)
- [USB无界面运行](docs/usb-headless-20261009.md)

历史日期文档描述当时的配置，不作为当前参数入口；以当前场景文件和本README为准。
