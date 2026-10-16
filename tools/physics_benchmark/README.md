# 空场物理对照脚本

这套脚本测试底盘指令到轮速、车身运动的响应，替代逐次手工改场景、启动、发指令和绘图。它不执行 Nav2 导航目标或五件搬运，不能代替路径跟踪回归。

在 WSL bash 中执行，使用当前仓库已经构建的机器人。无需修改生产配置，也不覆盖生产 world、Nav2 配置或机器人模型。运行目录必须是一个不存在的新目录；已有结果通过 `--analyze-only` 重画。

```bash
cd /home/polarbear/ws_aic
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/physics_benchmark/run.py \
  --output "$HOME/.local/share/aic-runs/ode-comparison-$(date +%Y%m%d-%H%M%S)" \
  --ode-values 100 200 --repeats 2 --speed 3.0 --omega 2.0
```

默认顺序为 100→200→200→100，即每个 ODE 值两次独立冷启动，每次一次直行、一次圆弧，共八项。每一轮重复反转参数顺序，减少顺序影响；这不是严格确定性的物理重放。修改 `--speed` 可测2.5、3.0、3.5等档位；`--omega` 为圆弧角速度。`--expected-torque`、`--expected-mu` 只核验当前模型，不修改转矩或摩擦。

脚本会拒绝接管已运行的仿真。先用根目录入口显式停止已有栈，再启动对照。整个流程 headless，退出、报错或中断时尝试自动停栈；运行结束还会核验进程状态。

依赖：ROS 2 Humble、Gazebo Classic开发头文件、g++、pkg-config、Python yaml/numpy/matplotlib，以及已构建的工程和现有 llama 运行资源。Windows 字体存在时自动用于中文图表。

## 自动执行内容

1. 在输出目录生成独立场景、地图和每组参数，核对场景差异仅为ODE值和日志路径。
2. 编译原生采样 WorldPlugin，使用根目录 `ros_competition_start.sh start --headless` 启动。
3. 从 ODE 原生接口读取实际 `contact_max_correcting_vel`，从运行中的 robot_description 核验驱动轮转矩和两个方向摩擦；保存模型和控制参数。
4. 在每个动作前发零速、重置 `six_arm` 到原点、清零速度，再等待稳定。显式设置实体名，避免空名重置失败。
5. 向 `/cmd_vel_nav` 发指令，经过活动的 `velocity_smoother`。线加速3.5、减速5.0 m/s²，角加减速10 rad/s²，角速度硬上限4 rad/s。正式导航目标不参与这项测试。
6. 目标保持3.2仿真秒，再观察2.2仿真秒停车过程；原生约100 Hz记录轮关节 `GetVelocity`、世界位姿/速度、前向速度和倾角，同时采样ROS里程计和实际底盘指令。
7. 核验指令到达时间和后段平台；倾角超5°、越过空场保护边界或时钟停滞时中断。输入核验通过不代表轨迹跟随正确。
8. 停栈后生成指标表、报告、速度/角速度/左右轮速/倾角/高度图，以及速度着色轨迹对照图。

短测空场为70×16 m，轮速换算使用当前两驱机器人半径0.065 m、轮距0.53 m；适用速度范围(0,3.5] m/s。模型换轮径、轮距或结构后，需同步采样分析公式和保护范围。场景地面两个方向摩擦固定1，转矩与轮摩擦由当前机器人模型提供，默认要求1000 Nm和1。

## 只生成配置 / 只重画

```bash
python3 tools/physics_benchmark/run.py \
  --prepare-only --output /tmp/aic-physics-config-check \
  --ode-values 100 200 --repeats 1 --speed 2.5 --omega 1.5

python3 tools/physics_benchmark/run.py \
  --analyze-only --output /home/polarbear/.local/share/aic-runs/20261015-ode100-vs200
```

`--prepare-only` 不编译、不启动ROS，也不接触已有进程；生成结果仅供审阅，实际执行请指定另一个全新的输出目录。`--analyze-only` 从已有 `protocol.json` 读取测试参数，不使用命令行默认值重新解释数据。

## 结果和解释边界

- `protocol.json`：实际测试顺序、目标和观察时长。
- `input-provenance.json`：脚本、场景和输入配置的SHA256。
- 各 `boot-*`：场景和配置、`native.jsonl`、ROS采样、逐项指令、真实robot_description、实际控制参数、原始栈日志及数据质量记录。
- `individual-metrics.json/csv`：每次工况指标；`metrics.json`：按ODE和工况汇总的均值与范围。
- `verification.json`：模型哈希一致性、试验数量和核验结果。
- `REPORT.md`、`response-comparison.png`、`trajectories.png`：报告与图表。

后段统计窗口为请求停车前0.5至0.1仿真秒；异常圆弧持续变化，不能把这个均值叫稳态值。角速度峰值包含停车阶段，逐项指标另外保留停车前峰值。高度是参考连杆相对动作开始的变化，不是底盘实体中心的绝对高度。

轨迹虚线由采集到的平滑后底盘指令积分得到，存在ROS接收时间误差，不能称为Nav2规划路径。轨迹颜色是实际水平速度，表格中的前向速度是世界速度投影到车身X轴，两者在侧滑时不同。只允许忽略所有测试结束之后的一个不完整关闭尾行，并保留原始文件和排除记录；试验区间内损坏记录直接失败。

本工具不录制视频，也没有测量每个轮子的接触力。车身没有明显倾斜，不能单凭这一点证明轮子始终正常接地或没有滑移。
## 高速数值误差对照套件

```bash
python3 tools/physics_benchmark/run.py \
  --suite numerical --repeats 2 --speed 3.0 --omega 2.0 \
  --output "$HOME/.local/share/aic-runs/numerical-$(date +%Y%m%d-%H%M%S)"
```

默认五种配置，各做两次独立冷启动，顺序为正向后反向，每次依次尝试直行和圆弧：

| 名称 | 求解器 | 物理步长 | 驱动轮有限旋转 |
|---|---|---|---|
| baseline | quick | 1 ms | 关闭，读取原生默认值核验 |
| step-half | quick | 0.5 ms | 关闭 |
| step-quarter | quick | 0.25 ms | 关闭 |
| world | world | 1 ms | 关闭 |
| finite | quick | 1 ms | 开启，每物理步更新世界轮轴向量 |

所有配置的ODE接触纠偏上限均为200，CFM=0、quick迭代60、驱动轮转矩上限1000 Nm、摩擦1保持。`world` 不使用quick迭代次数。现在默认启用匹配实时节奏：更新频率按1/步长设置，使名义仿真/墙钟比为1。--no-pace-realtime仅用于复现早期固定1000 Hz的旧套件，旧数据存在控制频率随仿真时间改变的混杂。

可用 `--variants baseline finite` 只选部分配置。有限旋转通过测试WorldPlugin仅作用于两个驱动轮，不修改正式模型或世界；原生日志核验实际模式为0/1。轮轴偏角使用轮子link局部+Y与车身局部+Y转换后的世界向量比较；改变机器人轴向或结构后需同步更新这些轴线定义。

数值套件中，单组启动/运动失败会保存 `error.json` 或 `runner-error.json`，停栈后继续下一组。只通过输入平台核验不能叫运动成功；如车身不动但轮子达到目标，也属于需要在报告中单独判断的响应失败。失稳中止的圆弧不纳入后段均值，图中用点线和叉号保留其已观测部分；发令前就失稳的配置保留失败记录，不伪造轨迹。

中断后继续同一份协议：

```bash
python3 tools/physics_benchmark/run.py --resume \
  --output /home/polarbear/.local/share/aic-runs/20261015-numerical-suite/run-data
```

`--resume` 从已有协议读取参数，保留已完成和已失败组，只运行尚未尝试的组；不会覆盖原始失败数据。必须保留原采样插件二进制且native_probe.cpp哈希不变。输入指令/物理配置需要修改时请开新输出目录。进程仍运行时拒绝接管。

数值套件还生成 `geometry-comparison.png`、`native-runtime.json`、`failures.json`。速度响应图对齐重复试验，实线为完整动作的重复均值、阴影为范围；中断轨迹不外推到完整窗口。原生记录包含左右轮有限旋转模式、轴线偏角。较老ODE套件分析入口仍要求每组都有完整两项记录。
将图表和报告直接放到Windows目录，可给首次运行、`--analyze-only`或`--resume`增加：

```bash
--windows-output /mnt/c/Users/21920/Desktop/AIC-高速物理对照-20261015
```

导出会逐文件核对SHA256，写入 `export-checksums.json`。原始大体量日志仍留在WSL输出目录。所有组都已完成或失败时，`--resume`只重画和导出，不启动仿真；若还有未尝试组，才要求原native_probe.cpp哈希不变。
## 加长物理步长对照

```bash
python3 tools/physics_benchmark/run.py \
  --suite numerical --pace-realtime --variants baseline step-double step-quadruple \
  --repeats 2 --speed 3.0 --omega 2.0 \
  --output "$HOME/.local/share/aic-runs/longer-step-$(date +%Y%m%d-%H%M%S)" \
  --windows-output /mnt/c/Users/21920/Desktop/AIC-加长步长对照-20261015
```

三档步长为1、2、4 ms，求解器均为quick，有限旋转模式关闭。--pace-realtime将物理更新频率同步设为1000/500/250 Hz，使名义实时率均为1；其余物理配置保持。执行顺序1→2→4→4→2→1 ms。原生实际步长会写入 `native-runtime.json` 并逐组核验；导航/平滑器更新频率不随这个选项改变。

原生采样器以不少于约10 ms的物理时间间隔记录：1/2 ms组约100 Hz，4 ms组约83 Hz；`individual-metrics.json/csv`中的 `native_sample_interval_s` 给出每项实际平均采样间隔。实际仿真/墙钟比记录在observed_sim_wall_ratio。若不加--pace-realtime，固定1000 Hz会使2/4 ms的仿真时间跑快，墙钟定时的平滑器每仿真秒指令数下降；这批数据不能与已匹配节奏的基线作纯步长因果比较。
只读GetParameters查询失败时最多重试3次，每次等待服务和响应各最多10秒，记录parameter-query-retries.json；状态修改请求保持单次执行。没有读到实际参数的组不发运动指令，不计为物理失败。

若某组在终态记录写入前被中断，--resume会先将该组已有观测数据移入interrupted-attempts子目录，再重跑该组；配置文件保持，部分日志不会被新采样覆盖。

## 轮子电机与车体施力对照

```bash
python3 tools/physics_benchmark/run.py \
  --suite drive --variants wheel body_force --repeats 2 --speed 3.0 --omega 2.0 \
  --output "$HOME/.local/share/aic-runs/body-force-$(date +%Y%m%d-%H%M%S)" \
  --windows-output /mnt/c/Users/21920/Desktop/AIC-body-force
```

先构建 aic_body_drive 与 mybot_description。四次冷启动顺序 wheel→body_force→body_force→wheel，各执行直行和圆弧，物理节奏1 ms/1000 Hz、ODE200保持；每次通过 AIC_DRIVE_MODEL 明确选择一种驱动。旧 ODE/数值套件也明确选择 wheel，避免本分支默认驱动改变旧协议含义。

机器人除驱动插件外的 XML 必须一致；同类驱动模型哈希必须一致。原生 left/right_motor_fmax 核验旧电机上限1000、新方案0；不把“车轮达到指令值”作为新方案通过条件。新方案轮子被动滚动，转速只作观测；力/矩在 /body_drive/wrench 采样。正式 μ=1 保持。

body_force 每次另执行原地转向、倒车、断指令制动检查，结果保存在 servo-checks.json。断指令检查先停用平滑器，再停止直接发布，验证驱动自身0.5仿真秒看门狗确实制动。该检查不计入八项直行/圆弧指标，观察时长也不计入圆弧曲线。

图中的灰色参考为本次实际平滑后指令的理想积分，包含ROS接收时刻偏差，并非Nav2规划路径。新旧不同驱动的启动前静止时间可能不同；轨迹从各动作起点对齐。后段速度接近目标和完整流程通过分别判断。

完整搬运实际速度路径可从标准结果重画（必须有同场景layout.json）：

```bash
python3 tools/plot_delivery_motion.py --run-dir /path/to/benchmark-result \
  --output /mnt/c/Users/21920/Desktop/delivery-speed-route.png --drive-label 车体施力驱动
```

此图使用世界静态几何，未显示移动障碍历史，也不混入未校准的全局路径。--drive-label仅是调用者提供的标签，不从旧日志猜测驱动类型。

## 3/4/5 m/s完整五件流程

WSL bash，先确保上一套栈已经停止：

```bash
source /opt/ros/humble/setup.bash
source /home/polarbear/ws_aic/install/setup.bash
python3 tools/physics_benchmark/speed_ladder.py --flows-only \
  --output "$HOME/.local/share/aic-runs/flow-speed345-$(date +%Y%m%d-%H%M%S)" \
  --windows-output /mnt/c/Users/21920/Desktop/AIC-flow-speed345
python3 tools/physics_benchmark/analyze_flow_ladder.py \
  --output /absolute/path/to/the/same/output \
  --windows-output /mnt/c/Users/21920/Desktop/AIC-flow-speed345
```

固定3→4→5 m/s各一轮五件，先核验实际参数再发令。隔离副本同步巡航、规划nominal_speed、平滑器上限、simple_nav2_navigator.crossing_cruise_speed；通过AIC_BODY_MAX_LINEAR_SPEED将驱动指令限幅统一5 m/s。脚本不修改源场景文件；当前正常默认线/角速度为6/4，线与角加减速均10；本历史3/4/5协议会显式覆盖回其原角速度4、加减速3.5/5/10及相应预测参数，不跟随默认档位漂移。质量/μ1/ODE200/1ms和力/矩、加减速、转弯与预测保护保持。

每轮保存世界与配置、实际机器人URDF和运行参数、原生100 Hz位姿/左右轮速、标准benchmark阶段/命令/落点、注入力和路径记录。record_navigation.py按路径消息时间查询TF，将map/base_link等路径变换到物理odom/world；转换失败保留原因，不将未转换点直接叠加。优先绘制控制器收到路径，其次全局计划；路径版本采样最多5 Hz，图中每导航阶段最多展示4个版本。

输出包括三档同色标实际轨迹、各档十段预期/实际图、速度/角速度响应、被动轮速/倾角/相对高度、注入力/矩、阶段耗时和CSV/JSON报告。主计时采用任务阶段事件，不含冷启动或自然语言解析；原始采集轮询时间另存。每档单轮，动态障碍相位和重规划路线可能不同，结果不能证明长期成功率。

预检使用与子进程同一Fast DDS配置；预检ROS上下文不安装自己的信号处理器，保留运行器中止行为。指定输出必须不存在，失败或中断记录保留；脚本未提供覆盖或自动删除旧数据。

短测工具现在支持最高5 m/s（需--body-speed-limit 5），隔离场地横向边界扩展到±12 m，保护线±11 m。既有数值试验保持原始世界快照，旧记录不被新夹具覆盖。

## 当前配置重复完整流程

```bash
python3 tools/physics_benchmark/repeat_flow.py --repeats 2 \
  --output "$HOME/.local/share/aic-runs/current-twoflows-$(date +%Y%m%d-%H%M%S)" \
  --windows-output /mnt/c/Users/21920/Desktop/AIC-current-twoflows
```

此入口保持当前场景配置（当前6 m/s、4 rad/s、线/角加减速10、前向/横向/偏航限额均1000），不调用历史3/4/5协议的旧限幅覆盖。每轮冷启动、读回所有速度/加减速/预测参数和实际URDF，保存原生采样、计划与TF、施力、阶段及落点，结束后停栈。两轮之间不调参，不把失败尝试自动补成成功记录。输出必须是新目录。

新入口在最后自动调用analyze_repeats.py，生成重复对比轨迹、速度/角速度、轮速/倾角/高度、平滑微分得到的速度变化率、施力、阶段耗时、各轮预期/实际路径及Windows index.html。也可单独用该分析脚本对已有输出重画。计时区分完成事件时间与原始轮询/中止尝试耗时；失败不计入完成均值。


## 既有流程转向受力诊断

`python3 tools/physics_benchmark/diagnose_turning.py --trial <trial目录> --output <输出目录>`只读取repeat_flow既有日志，输出横向速度、施力与偏航矩片段图及JSON。包含world施力到车头坐标的转换、近限幅样本比例与质量估算；向心力m*v*omega参考不包含接触反力。不会启动仿真或更改参数。2026-10-15分析见`../../docs/turning-force-diagnosis-20261015.md`，旧20/40日志不能作为已恢复4/10/10配置的验证。
