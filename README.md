# AIC

2026-10-18参赛快照：包含当前导航恢复、运动稳定性保护、礼品盒题型解析及比赛展示布局；桌面参赛包与本仓库核心源码一致。正式题目仍由确定性代码求解后交本地模型校核，官方映射原文2/3与宠物店题型的缺口尚未修复，详见[规则核查](docs/competition-requirements-audit-20261018.md)。最近4红A、1蓝C五件日志完成耗时175.40现实秒，不含解析和人工确认；没有本轮仿真计时。入口与部署见[参赛启动说明](docs/competition-bundle-20261018.md)。

2026-10-18后续复测：修复“停车定位已恢复、路线仍不可用”时过快耗尽恢复次数的问题，取得新鲜且终点正确的路线后才重启导航。原卡点带载返回成功；两轮五件均区域/高度合格，177.997/173.8、167.891/164.5墙钟/仿真秒，每轮实际恢复一次。另有一次Nav2启动超时单独留档；现有TF/到达误报核验仍保留，见[返程路线恢复](docs/return-route-recovery-20261018.md)。

2026-10-18：修复新目标首次规划失败时沿用旧路径的问题；导航中止或到达核验拒绝后，新增有次数和时间上限的停车定位刷新与重规划。1红C、4蓝A单轮五件区域/高度合格，166.45墙钟/162.7仿真秒，无到达误报；本轮未触发新增恢复分支，详细证据和验证范围见[最后一件导航修复](docs/final-cargo-navigation-20261018.md)。

2026-10-17：按 Gazebo 实际侧翻反馈增加控制器/底盘的联合转弯约束和 5° 倾斜暂停留存。直行上限仍为 6 m/s、角速上限 4 rad/s，运动中根据真实速度限制急转；质量、摩擦、施力限额、世界和全局规划器保持。当前版单轮 1红C、4蓝A 五件合法完成，155.19墙钟/151.5仿真秒，最大倾角0.05692°，未检测到非地面撞击。原事故具体接触原因尚未完整复现，单轮通过不代表长期保证。见 [修改与验证](docs/stability-turn-limit-20261017.md)、[本机可复制启动命令](docs/local-start-20261017.md)。

当前车体施力研究分支按用户要求设线速上限6 m/s、角速上限4 rad/s，线与角加减速度均10；前向限力1000 N、横向限力1000 N、偏航限矩1000 N·m。控制器、规划器、平滑器、任务预测、生成器与驱动限幅已同步，本档单轮五件区域与高度均合格，120.75墙钟/115.3仿真秒，无碰撞/失稳中止，实速峰值6.121 m/s；[记录与图](docs/speed6-body-force-20261016.md)。昨日8/4与20/5、20/10档结果见[记录](docs/acc20-force1000-flow-20261015.md)。

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

复现当前配置的一轮无头五件流程并自动画图（输出目录每次新建）：

```bash
cd /home/polarbear/ws_aic
source /opt/ros/humble/setup.bash
source install/setup.bash
run_id=$(TZ=Asia/Shanghai date +%Y%m%d-%H%M%S)
python3 tools/physics_benchmark/repeat_flow.py --repeats 1 \
  --output "$HOME/.local/share/aic-runs/manual-$run_id" \
  --windows-output "/mnt/c/Users/21920/Desktop/AIC-test-$run_id"
```

该工具要求没有正在运行的栈；每轮自动冷启动、核验实际参数/URDF、执行任务、停栈并输出桌面目录中的 `index.html`。当前参数和单轮验证边界见 [6/4 与全10、全1000记录](docs/speed6-body-force-20261016.md)。
Foxglove 导入 `demo1.json`，普通主入口连接 `ws://localhost:9090`；USB 入口默认 `ws://localhost:8765`。不要同时启动两套栈。

## 当前参数与验证

本研究分支 `codex/body-velocity-control` 默认使用[车体施力驱动](src/aic_body_drive/README.md)：真实速度反馈、有限水平力和偏航转矩、被动车轮，取代轮子电机驱动。`AIC_DRIVE_MODEL=wheel` 可在冷启动时选回旧驱动作对照，两个驱动互斥。导航目标6 m/s、角速4 rad/s，线与角加减速均10；前向/横向/偏航限额均1000，摩擦1、ODE200与1 ms保持。下面日期成绩保留为旧轮子驱动的历史结果；新方案验证见[车体施力记录](docs/body-force-drive-20261015.md)。 3/4/5 m/s各一轮完整五件及全部图表见[三档流程记录](docs/body-force-speed345-flow-20261015.md)；这些为历史隔离试验；当前配置见本文开头。

2026-10-15后续本地试验：世界与生成器的`contact_max_correcting_vel`已改为**200**，正式导航目标仍为2.2 m/s。隔离空场测试2.5/3.0/3.5 m/s，每档直行和圆弧各重复两次。直行均达目标；3.0/3.5圆弧出现轮速已到位、车身偏航加剧与轨迹内卷，不能当作跟踪通过。详情见[三档速度、轮速换算与实测](docs/linear-speed-limits-20261015.md)。以下100档的五件成绩属于此前发布验证，200未跑完整五件。

2026-10-15 发布准备采用2026-10-14跑通五件的导航源码和配置。直行目标2.2 m/s、角速度4 rad/s、角加减速度10 rad/s²，单轮转矩上限1000 Nm，世界`contact_max_correcting_vel=100`，驱动轮与地面两个摩擦方向均为1；Gazebo单轮加速度限幅关闭。底盘10 kg、驱动轮各1 kg；货物质量0.1 kg、边长3 cm、主惯量0.000015 kg·m²。

新地图使用实时滚动预测规划，控制20 Hz、目标重规划5 Hz；不插入固定等待点，不封锁移动障碍整条轨道。静态净距与动态轨迹采用软代价，真实轮廓和预测碰撞检查保留；墙边距离减速和RPP代价减速关闭。默认终点检查采用本轮实测的SimpleGoalChecker。

三红A、两蓝B五件均完成，落点与高度合格，没有触发碰撞或失稳中止。按主控任务阶段事件计时为**160.78墙钟秒 / 134.5仿真秒**，不含冷启动及自然语言解析。采集器轮询计时另为162.25/136.4，详情见[100档流程与路径图](docs/limit100-flow-20261014.md)。这是单轮验证，起步、大弯及末端停姿仍有偏差，不代表长期成功率。

发布检查包括14个ROS包构建、49项Python与13项C++回归、生成器输出一致性和默认headless启动。恢复后的导航库SHA-256与本轮五件实测使用的库一致。未验证的WorldGoalChecker草案已另存本机，不包含在发布源码或默认配置中。

基准工具等待主控空闲及订阅匹配后只发一次任务，并检查接收确认；不重复发送任务。结果保留原有轮询计时，并在完整完成时额外输出`task_event_wall_seconds`、`task_event_sim_seconds`。

- [小地图测试、运动诊断与轨迹绘图手册](docs/small-map-testing-handoff-20261015.md)：测试协议、实测结果、计时口径、坐标与指标、脚本复用和故障排查。
- [100档五件实测、路径与速度图](docs/limit100-flow-20261014.md)
- [ODE约束20/34/60对照与真实关节转速](docs/contact-limit-20261014.md)
- [固定摩擦1的扭矩对照](docs/torque-sweep-20261014.md)
- [墙角坐标、净距与制动检修](docs/corner-frame-and-braking-20261014.md)
- [实时预测路径与历史试验](docs/live-obstacle-routing-20261013.md)
- [货物惯量修正证据](docs/cargo-inertia-and-turning-20261012.md)
- [规则输入格式](docs/formal-problem-20261009.md)
- [USB无界面运行](docs/usb-headless-20261009.md)

历史日期文档描述当时的配置，不作为当前参数入口；以当前场景文件和本README为准。

空场物理对照可用 [自动测试脚本](tools/physics_benchmark/README.md) 一条命令完成启动、采样、停栈和出图；ODE100/200的有限对照与脚本集成验证见 [2026-10-15记录](docs/ode100-vs200-20261015.md)。

高速圆弧的步长、求解器与有限旋转对照已接入同一工具的 --suite numerical；十次冷启动没有得到可采用的修复，失败记录与限制见 [数值求解对照](docs/high-speed-numerical-20261015.md)。

匹配实时率后的1/2/4 ms复测十二项输入有效，圆弧响应仍异常，见[加长步长复核](docs/longer-step-20261015.md)。参考项目的车体PID与直接施力模型核对见[pb2025底盘分析](docs/pb2025-drive-reference-20261015.md)。
