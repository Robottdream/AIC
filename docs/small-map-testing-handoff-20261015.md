# 小地图测试与绘图交接（2026-10-15）

本文交接已经执行过的测试方法、数据解释和绘图流程。当前已发布基线为 `49928732f535bc76be90c5985dc9b5c7964cdcab`，远程 `main` 已包含该提交。本次交接只整理文档，没有重新运行仿真或调整参数。

## 1. 接手时先确认环境

真正的开发、构建、仿真副本是 **WSL `/home/polarbear/ws_aic`**。Windows 的 `C:/Users/21920/.codex/worktrees/4d7b/AIC` 是另一个副本，不能直接覆盖过去。Windows Git 通过 `//wsl.localhost/Ubuntu-22.04/home/polarbear/ws_aic` 管理实际副本；ROS、Gazebo、colcon、llama 在 WSL bash 执行。

先检查实际副本的 Git status/diff、HEAD、场景和运行进程。当前本地分支名可能仍是 `codex/no-wall-slowdown`，判断版本看提交，不要把分支名当作代码过期的证据。不要为了整理环境执行整目录 restore、checkout 或 clean。

```bash
# 以下均在 WSL bash 中执行
cd /home/polarbear/ws_aic
bash ros_competition_start.sh status
# 有本项目旧栈时，先正常停止
bash ros_competition_start.sh stop
# 源码有变化且需要构建时
bash ros_competition_start.sh build
```

根入口构建使用 `colcon build --symlink-install --base-paths src`。修改源码后确认构建和实际加载的库一致，不能“测试旧 install 库，交付新源码”。测试前保存源码、配置和库的 SHA-256，运行后读取实际参数再核对。

### 当前参数基线

| 项目 | 已发布值 |
|---|---|
| 默认场景 | `scenarios/short_routes_20261007`，新的办公室地图 |
| 线速度 / 线加速 / 线减速 | 2.2 m/s / 3.5 m/s² / 5 m/s² |
| 角速度 / 角加减速度 | 4 rad/s / 10 rad/s² |
| 驱动轮转矩上限 / 单轮加速度限幅 | 1000 Nm / 关闭 |
| 驱动轮与地面摩擦 | 两个方向均为 1；支撑球保持 0.01 |
| 车体 / 驱动轮 | 底盘 10 kg，两个驱动轮各 1 kg，轮半径 0.065 m、轮距 0.53 m |
| 货物 | 0.1 kg、边长 3 cm、主惯量 0.000015 kg·m² |
| ODE | `contact_max_correcting_vel=100`，迭代 60、步长 0.001 s |
| 控制 / 平滑器 | 20 Hz / 30 Hz，平滑器 OPEN_LOOP、`scale_velocities=false` |
| 导航 | TimeAwarePlanner + TimeAwareController，SimpleGoalChecker |
| 减速与保护 | 墙边距离减速、RPP 代价减速关闭；曲率、终点和预测碰撞保护保留 |

不添加固定等待点，不把移动障碍整条轨道永久涂黑。允许查询仿真障碍真实状态，采用滚动预测。`mu1/mu2` 是两个摩擦方向，不能简单写成“静摩擦/动摩擦”。

## 2. 三层测试分别回答什么

| 层级 | 做法 | 能回答的问题 | 不能据此证明 |
|---|---|---|---|
| A：空场物理响应 | 简单平地直接发速度，固定坡道，测轮子与车身 | 转矩、接触约束、加速制动、圆弧响应 | Nav2 跟踪和完整搬运成功率 |
| B：简单路线跟踪 | 同一空场给定直线/弯道，实际走规划器和控制器 | 前视、曲率、转向切换、末端跟踪 | 复杂动态场景稳定性 |
| C：新地图五件 | 当前办公室地图，三红 A、两蓝 B，记录十段导航 | 综合路径、动态避障、抓放、总耗时 | 单轮不能代表长期成功率 |

过去扭矩与 ODE 对照主要属于 A。空场配置里虽然启动了普通 Navfn/RPP，直接速度测试并没有实际执行路径跟踪，不能称为 B 的回归。先用 A 缩小物理原因，再做 B，最后做 C，避免每调一个参数都耗时跑五件。

## 3. 隔离空场与参数

已经用过的空场资源位于：

`/home/polarbear/.local/share/aic-runs/20261014-torque1000-friction100/simple-field/`

其中 `mu1.world` 是固定摩擦 1 的空场，配套 `map.yaml`、`map.pgm`、`tasks.yaml`、`nav2.yaml`。不要因目录名含 friction100，就误以为所有子文件仍是摩擦 100；应解析具体文件并读取运行模型。更早资源来自 `20261012-turn-only/simple-field/`。

为新试验建立新的外部运行目录，把这组资源复制过去再修改，不覆盖历史记录。world、map、tasks、nav2 四个路径一起显式指定：

```bash
cd /home/polarbear/ws_aic
# 替换为本轮已准备好的绝对路径
export AIC_WORLD=/absolute/path/to/trial/field.world
export AIC_MAP=/absolute/path/to/trial/map.yaml
export AIC_TASK_CONFIG=/absolute/path/to/trial/tasks.yaml
export AIC_NAV_PARAMS=/absolute/path/to/trial/nav2.yaml
bash ros_competition_start.sh start --headless
```

`start` 只启动、等待发令；`benchmark` 才自动发五件。空场物理测试用 `start`，不要误触完整搬运。必须 `--headless`，检查没有 `gzclient`；Foxglove 数据仍可用。空场边界和安全余量必须适合测试动作，不能在办公室墙边直接复用空场速度探针。

每档冷启动，核对 `/robot_state_publisher` 的实际 `robot_description`、驱动插件转矩、两轮摩擦、地面摩擦和实际物理参数。单看源文件不足以证明加载成功。历史扭矩脚本临时改过机器人 xacro，并在 finally 恢复；后续优先采用隔离资源。若仍需临时修改，逐文件备份、退出恢复、最后审阅 diff，不用全仓库还原。

回到新地图前清掉上面四个测试环境变量，或在干净 shell 中启动，防止继续加载空场。不要把空场的普通 Navfn/RPP 配置覆盖回新地图。

## 4. 两套有效的短测协议

### 4.1 固定摩擦 1，只扫描转矩

实际测试档位为 **1、20、200、500、1000 Nm**，其余保持。当时 ODE 约束为 **34**，不是当前发布的 100。每档同样五项：

| 动作 | 目标 v / ω | 保持时间（仿真秒） |
|---|---|---:|
| 正向原地转向 | 0 / +4 | 1.3 |
| 反向原地转向 | 0 / −4 | 1.3 |
| 直行 | 2.2 / 0 | 1.6 |
| 低速圆弧 | 0.5 / (0.5÷1.1) | π÷(0.5÷1.1) |
| 高速圆弧 | 2.2 / 2 | π÷2 |

从 `/cmd_vel_nav` 发令，让现有 velocity_smoother 生效，同时记录其输出 `/cmd_vel`。每项前停车约 0.8 秒，复位到空场中心、朝向 0、速度归零，再稳定约 0.7 秒；动作后给停车指令并记录约 1.2 秒。正式统计前剔除复位跳变。

25 项有效记录均核验：首个非零底盘指令延迟不超过 0.25 秒，达到目标不迟于理论爬升时间加 0.25 秒。1000 Nm 首次记录曾有约 0.5/1 秒输入迟到，原始记录保留，但以冷启动复测替换，不能把它当成低扭矩/高扭矩响应差异。

结论仅限当时条件：1 Nm 加速/制动与高速圆弧明显差，20 Nm 以上收益很小。旧限制未解除前，不能据此认为继续加转矩就能解决约 1 rad/s 平台。现在若在 ODE=100 重做，要单列新结果。

### 4.2 只扫描 ODE 接触约束

实际档位 **20、34、60**；转矩固定 1000 Nm、摩擦固定 1。每档直行 2.2、直行 3.5、高速圆弧 2.2+2，各重复两次，共 18 项。三份世界除约束值、诊断输出路径外保持相同，并做归一化 XML 哈希校验。

为隔离物理层，先停用 velocity_smoother 并确认 lifecycle 为 inactive；直接向 `/cmd_vel` 发相同限加减速坡道，线加/减速 3.5/5、角加减速 10。不能与仍在输出的平滑器争抢 `/cmd_vel`。3.5 m/s 只是诊断目标，不改变默认导航上限 2.2。

测试保持时间分别为 1.8、1.7、2.3 秒，统计窗口从动作开始后 1.1 秒到停车前 0.1 秒。若新试验在窗口里仍未稳定，要明确标为动态响应，不能假定稳态。

| ODE 值 | 3.5 指令实速 m/s | 高速圆弧车身 ω rad/s | 外轮原生 ω rad/s |
|---:|---:|---:|---:|
| 20 | 1.300 | 约 0 | 20.000 |
| 34 | 2.210 | 1.027 | 34.005 |
| 60 | 3.500 | 2.101 | 42.007 |

这证明该值在本 Gazebo 11.10.2/ODE 模型里影响实际关节限速；它原本是接触穿透修正速度参数，不能普遍解释成“车辆最大速度”。60 档只请求到 3.5，未测得其最高速度。当前 100 是随后完整流程采用的值，不属于这 18 项三档对照。

短测探针在倾角超过 5°、车身中心 |x| 或 |y| 超过 7 m、z 超过 0.3 m，或仿真时钟长时间不推进时中止；finally 发零速并由外层停栈。5° 是中止阈值，不能写成可接受倾角。新地图不能照搬空场边界阈值。

## 5. 采样和判定：最重要的经验

同时记录输入、执行输出、物理响应，才能定位限制在哪层：

| 数据 | 用途与注意事项 |
|---|---|
| `/cmd_vel_nav`、`/cmd_vel` | 区分导航目标与平滑后真正送到底盘的命令；Twist 无源时间戳，记录接收单调时钟及当时仿真时钟 |
| `/odom` | 保留源仿真时间戳、位姿、v/ω；校验采样间隔，不能把接收墙钟当仿真时间 |
| Gazebo 原生车体状态 | WorldUpdateEnd 约 100 Hz，世界位姿、线速度、角速度和倾角；核验 ROS 记录 |
| 原生关节状态 | `Joint::GetVelocity(0)` 与 Position 同时保存，不能只根据 TF 差分推断真实转速 |
| 任务与导航事件 | `/task_stage`、`/manual_nav_target`，用于分件、区分抓取/放置和确定目标 |
| 计划与预测诊断 | `/plan`、`/received_global_plan`、`/known_timed_plan`、`/known_predictive_control` |
| 障碍物 | `/design_obstacle_left/motion_state`、`/design_obstacle_right/motion_state` |

**TF 陷阱已有实证：** ODE=34 高速圆弧时，外轮 TF/Position 差分约 42 rad/s，原生 GetVelocity 只有约 34。先前“轮子达到目标而车身没跟上，因此是打滑”的解释已经撤回。车身轨迹和限幅对照仍有效，不要继续引用旧轮速图证明打滑。

原生插件找车体 link 时先试 `base_link`，找不到再试 `base_footprint`，固定关节合并可能改名；曾因没有 fallback 导致文件为空。采样开始必须确认参数首行和 motion 行都已产生。读取 `dWorldGetContactMaxCorrectingVel` 验证内核实际值。非有限值写 JSON null，不能当 0。

交叉核验可用：展开 yaw 后对仿真时间拟合角速度，与原生 angular.z 比较；线速度同时看车体前向投影和世界水平模长。差速理想关系为 `v=r(ωR+ωL)/2`、`ω=r(ωR−ωL)/b`，r=0.065、b=0.53；它用于诊断，不是实际无滑移的保证。

减速指标应同时给“收到停车命令后多久停下”和“额外走了多远/转了多少”。软件爬坡、消息迟到、接触限制都可能影响结果。旧 Twist 记录的仿真时钟约 0.1 秒量化，小于几十毫秒的档位差不作结论。没有真实稳定平台就不报告稳态速度。

## 6. 小测通过后怎样跑完整流程

恢复默认场景、配置和需要的控制节点状态后，在无旧 Gazebo 的环境里执行：

```bash
cd /home/polarbear/ws_aic
unset AIC_WORLD AIC_MAP AIC_TASK_CONFIG AIC_NAV_PARAMS AIC_SCENE
bash ros_competition_start.sh benchmark --headless
```

默认三红 A、两蓝 B，顺序保持输入约定。基准工具等待 `/competition/ready` 空闲就绪及 `/chat` 主控订阅匹配，只发布一次已解析任务，并等待接收确认。未确认属于基础设施/发令失败，不能算搬运运动失败，也不能在同一栈里盲目重发导致任务重复。

要画规划对照，计划观察器必须在任务发出前开始记录；仅运行默认 benchmark 得到 motion/phases/result，还不足以补画历史规划。要用原生物理轨迹，还需在本轮 world 启动前装入诊断 WorldPlugin。实际参考编排为外部 `20261014-limit100-flow/run_flow.py`，不要未经检查直接重放旧脚本。

`/task_stage` 仅在状态变化时发布，不能用它当周期存活信号；`/competition/ready` 周期发布。完整成功要核对五次抓取、五次放置、最终货物区域/高度、是否重导航和是否触发失稳/碰撞中止，不能只看最后 WAIT_TASK。

当前 100 档单轮成功：任务事件计时 **160.78 墙钟秒 / 134.5 仿真秒**；原始轮询计时 **162.25 / 136.4**。后者含起始 `/clock` 积压与末尾轮询/查询开销，不能混用。新基准输出 `task_event_wall_seconds`、`task_event_sim_seconds`；同时保留原始字段。计时不含冷启动与自然语言解析。

## 7. 绘图流程与解释边界

### 7.1 先整理数据，再出图

1. 保留原始 JSON/JSONL；用任务事件的源仿真时间划分五件、十段导航，不按空间邻近猜测所属件号。
2. 记录每段目标、开始/结束状态、带载状态。抓放和阶段衔接另用灰线，不混进导航指标。
3. 原生 motion 按 sim 排序、去除或拒绝重复/倒退时间。复位跳变、丢包和跨阶段不能直接连线。
4. 地图从本轮 `result.scene.map` 读取 resolution、origin、图像及对应 layout；不要凭截图估坐标，或误用旧地图。当前绘图实现假定地图 origin yaw=0；其他地图需补旋转变换。
5. 参考脚本用 `imshow` 的 world extent 与图像朝向对应地图；检查几个墙角/目标坐标对齐。坐标轴等比例，单位米，关键转弯用局部放大。

### 7.2 预期路径与实际路径

同时画三层：蓝色实际世界轨迹、橙色首次全局计划、绿色滚动局部计划抽样。后两者含义不同；系统持续重规划，实际轨迹不必始终贴住首次计划。

全局 `/plan` 的 map frame 不能无条件视为世界真值坐标；本系统已做坐标对应修正，换系统后需重新验证。局部 `/received_global_plan` 若为 base_link frame，使用**该消息 stamp 对应的真实底盘位姿**转换到世界：

```text
world_x = body_x + cos(yaw) * local_x − sin(yaw) * local_y
world_y = body_y + sin(yaw) * local_x + cos(yaw) * local_y
```

yaw 先 unwrap 再插值。参考脚本遇到相邻原生位姿间隔超过 0.20 秒或时间超出记录范围就不插值，不能用“当前位姿”变换几百毫秒前的路径。按本段请求时刻、目标端点和阶段筛选计划，剔除上段残留及候选预览；不要把所有 `/plan` 都当作已执行路径。局部计划约每 0.2 墙钟秒抽样，图例说明抽样而非完整计划流。

100 档原生记录的 `wall` 是从 ROS 接收时间插值得到，只供绘图辅助；真实分段使用 sim，不能拿插值 wall 宣称精确墙钟耗时。保留 `native-motion.jsonl` 和 `plotting-provenance.json`，不要用加工数据覆盖原始数据。

运动样本到最近局部折线的几何距离可作辅助诊断，但重规划常从当前车身附近起步，会使距离偏小。100 档报告的中位/P95并非固定参考的严格横向误差。要验证控制器跟踪能力，另做固定路线试验，限制参考路径变化并记录路径版本与沿程进度。

### 7.3 用颜色表达速度

用 Matplotlib `LineCollection` 按相邻点组成线段，颜色取两端实测水平速度均值。五张分图统一 `Normalize(0, 2.2)` 和同一色条；超过 2.2 的瞬态峰值显示最高色，但报告真实峰值。不要每张图自动归一化。

当前原生速度采用 `hypot(vx,vy)`；短测前向速度是另一种量，轴标题需区分。相邻时间间隔不在 (0,0.20) 秒、件号或阶段不同时不连线。不要平滑掉真实弯钩/蛇行，也不要把位姿跳变画成高速直线。

### 7.4 短测曲线和预期积分轨迹

每项相对动作开始时间对齐，叠加输入指令、平滑器输出和实际 v/ω；阴影标出统计平台，竖线标出停车请求。参数对照尽量使用统一坐标轴，重复试验单列或说明合并方法。

预期运动曲线从实际送到底盘的命令按零阶保持分段积分，而非直接积分名义阶跃目标。对常值 v、ω，圆弧精确积分；ω 接近 0 时用直线形式。它代表理想运动学参考，含指令时间戳误差，不等于规划器路线，也不能单靠偏离它判定轮胎打滑。

### 7.5 出图后人工验图

导出 PNG 后实际打开，检查中文字体、图例、色条、方向箭头、裁切与标注遮挡。参考脚本使用 Agg 后端、`/mnt/c/Windows/Fonts/msyh.ttc`；换机器需选可用中文字体。优先给全局图、五件/十段分图、问题局部三种视角，图片直接标明试验条件与数据来源。

当前复核：第二件没有原来的反复蛇行，但切入仍有弧线；第三件右侧大弯外扩；第五件末端弯钩且真实停车朝向约 14.1°，请求为 0°。五件放置合格不代表路径完美或停车朝向准确。不同轮次障碍相位、接近方向和参数不同，不能从两张图直接给严格改善百分比。

## 8. 脚本、证据与复用注意事项

以下短名均相对于 **`/home/polarbear/.local/share/aic-runs/`**，是本机外部证据目录，Git 克隆不会自动带走。移交到其他机器需要另行复制；不要把大型日志、生成库或模型权重提交进仓库。

| 目录 | 接手优先看 | 说明 |
|---|---|---|
| `20261014-torque-sweep-mu1` | `REPORT.md`、`comparison.png`、`input-validation.json`、`torque_sweep.py`、`response_probe.py` | 五档扭矩 25 项有效短测，包含 1000 档替换复测；旧 TF 轮速推断已更正 |
| `20261014-torque1000-friction100/simple-field` | `mu1.world`、`map.yaml`、`map.pgm`、`tasks.yaml`、`nav2.yaml` | 空场资源；同级还保存被排除的旧实验，不混用 |
| `20261014-contact-limit-test` | `REPORT.md`、`comparison.png`、`input-validation.json`、`metrics.json` | ODE 三档 18 项；原生数据在 `limit-*/native.jsonl` |
| 同上 | `contact_probe.cpp`、`prepare.py`、`run_limits.py`、`probe.py` | 原生 API、编译、隔离世界与短测编排参考 |
| `20261014-limit100-flow` | `REPORT.md`、`provenance.json`、`plotting-provenance.json`、`round-1/result.json` | 100 档五件、计时与版本证据 |
| 同上 | `run_flow.py`、`observe_plans.py`、`route_native_probe.cpp` | 完整流程编排、规划观察与原生采样 |
| 同上 | `prepare_plots.py`、`plot_routes.py`、`plot_speed_routes.py`、`plot_planned_vs_actual.py`、`focus_and_summary.py` | 图表数据加工与绘图；准备脚本部分依赖历史 07 目录 |
| `20261015-main-publish` | `build.log`、`python-tests-final.log`、`cpp-result.log`、`default-start-proof.json` | 14 包构建、49 Python、13 C++、默认启动验证 |
| 同上 `/unvalidated-world-goal-draft` | `README.md` 与原文件 | 未验证终点修正草案，未包含在当前发布源码，不能误用 |

**旧脚本是试验记录，不是开箱即用的通用测试工具。** 多数写死输出路径和旧参数；`prepare.py`、扫描脚本可能改源码/启动仿真，部分 `analyze.py`、绘图脚本还会重写仓库文档或旧报告。复用前读全部副作用，复制到新运行目录，改输出和配置路径，固定输入快照，禁止直接覆盖已有结果。

绘图依赖还包括 `20261014-clearance-routing-07` 的历史脚本/快照，以及场景 map/layout。准备交给另一台机器时，一并带上必要资源或把路径参数化。重画已整理好的 100 档图时顺序是 `plot_routes.py` → `plot_speed_routes.py` / `plot_planned_vs_actual.py` → `focus_and_summary.py`；先在副本中验证输出，保留原图。

仓库内可长期阅读的记录：

- [ODE 三档对照](contact-limit-20261014.md)
- [固定摩擦扭矩对照](torque-sweep-20261014.md)
- [100 档完整流程与已审阅图片](limit100-flow-20261014.md)
- [墙角坐标与制动检修](corner-frame-and-braking-20261014.md)
- [货物惯量与第二件蛇行修复](cargo-inertia-and-turning-20261012.md)

## 9. 下一位执行者的最小交付

每轮至少保留：版本与实际参数、输入有效性结论、原始带时间戳数据、分项指标、失败/中止原因、全局和局部路径图、统一色标速度图，以及实际/预期路径对照。注明是否空载、是否经过平滑器/导航、统计窗口和计时口径。

推荐从当前 ODE=100 基线做固定路线跟踪与末端制动短测；已有物理限幅证据不能再拿来解释所有剩余跟踪偏差。先通过图片定位问题段，再同步检查目标/输出/实际角速度、路径切换、坐标系和末端判定。参数或算法有改变后，仍需恢复新地图跑完整五件并核对最终落点。
