# AGENTS.md

> 2026-10-04 三轮完整任务：当前圆形全向车型4快档，经自然语言入口依次执行3红A、2蓝B，每轮完整复位；按Gazebo /clock下令至主控全部完成计时195.0 / 207.5 / 190.9仿真秒，平均197.8秒。15次物块落点均在指定区域内，最终再次复位且PREFLIGHT=OK。第三轮发令前Gazebo通信断言退出，未发令检查记录另存，恢复后使用单次pose/info订阅快照代替反复gz model查询。动态相位未固定，未做旧档同任务三轮对照，不能据此量化相对提速；见 `docs/three-task-trials-20261003.md`。

> 2026-10-03 高速与快制动：新车型默认最高合速度4.0m/s、平移加速度2.0m/s²、减速度5.0m/s²，DWB/平滑器/最终保护统一；H1.8s、前方裁剪8m、局部18×18m、标记/清障8/9m。原0.405m停车圈、2s TTC、失联保护及内置轨迹预测保留。两套源码/运行配置同步，独立副本ws_aic_mecanum_20261002运行就绪；旧ws_aic未覆盖。只提高上限时1/2/3/4档路线128.67/108.01/112.09/142.77墙钟秒；用户指出可加强仿真制动后，4快档81.13墙钟/50.8仿真秒、无恢复，门口进出通过、入口偏离0.102m、最近点0.643m，峰值3.220m/s。普通4档受保护直线曾达到4.000m/s；快档从3.099m/s取消导航后约0.82墙钟/0.50仿真秒、1.020m停下，保留全速度链。双真实270°雷达各1081→811点、10→8Hz仿真频率，名义射线量减40%；等效清障试验残影0，兼容/scan仍1441投影格，不是物理射线。PathFollow缓存几何、平方距离，曲线自适应前瞻和高速中速采样补充；快档隔离弯道0.0946m/短尾及9项预测回归通过。各档仅一次最终路线、动态相位/实时因子未固定，未证明全业务吞吐；理想平面速度驱动，未改摩擦/质量，不能当作实体制动性能证据。仿真重启保留业务进程/抓取标记并恢复全部物块位置，GAZEBO_IP默认环回（允许覆盖）修复WSL DNS地址导致的控制失联。详见 `docs/speed-ladder-20261003.md`。

> 2026-10-03 门口弦线切弯修复：新车型PathFollowCritic替代末端PathDist/GoalDist，沿有序路径0.65m前瞻，约束0.6s候选前缀偏离≤0.12m，完整1.8s软评分；起点偏离允许回到路线，短曲尾回归通过。实际DWB隔离圆弧偏离0.8035→0.1310m、到点成功；现场原三红A目标经两次有意Nav2恢复后接续，red_cube_4/3/2全部放置，最终门口偏离最大0.115m、最近雷达点0.616m。保留内置轨迹预测、1m/s限值及原0.405m停车圈；仍有7次20Hz周期超时，其他门口/动态迎面/完整带货三维包络未穷尽。首件包含调试，不构成整批提速证据。见 `docs/path-following-fix-20261003.md`。

> 2026-10-03 门角贴墙卡住修复：现场雷达最近0.386m落入0.405m停车圈，但原导航包络约0.339m；另捕获到点粗速度候选越过目标、PathDist使停止得分更低。新车型规划半径0.45+padding0.01，实体2s碰撞足印0.325单独发布，原0.405停车圈保留；FineOmni生成器在原11×11网格外补充48个0.025m/s低速候选。受控0.12m/s退让后恢复原保护，真实A门进出成功、无恢复、峰值约1m/s；隔离门洞/DWB/失联保护回归通过。未完成抓放业务，其他门口和完整带货三维包络仍未验证。见 `docs/doorway-fix-20261003.md`。

> 2026-10-03 圆形全向车型最高合速度0.70→1.00m/s（限值+42.9%）、X/Y轴加速度0.35→0.70m/s²；DWB/平滑器/最终保护统一，局部地图扩至6m，预测和近障/失联停车保留。新运行副本导航恢复后参数实查通过，隔离DWB无障碍1.00、横穿0.20、迎面/折返-0.20，真实最终保护斜向合速度及失联停车回归通过。原配置现场4m试跑90s无位移已取消，尚无有效行驶对照或完整任务提速证明，未重发抓取；见 `docs/round-speed-20261003.md`。

> 2026-10-03 按用户要求内置障碍2/3场景路线、0.35m/s速度及0.2m端点折返，真位姿仅同步相位，未来4s序列进入全局KnownTrajectoryLayer软代价、未来1.8s进入DWB时间评分。此方案刻意使用仿真场景先验，不能写成纯雷达自主预测。隔离真实NavFn路径4.000→5.055m、绕行1.450m，清除后恢复4.000m；真实DWB横穿降速0.70→0.14、迎面退让-0.14，折返序列对照通过。现场双预测流/地图插件/保护链已加载；尚无本版真实行车完整任务证明，原三红A任务此前区域导航失败结束。本次只导航恢复，未重发任务。见 `docs/predictive-local-planner-20261003.md`。

> 2026-10-03 新车型预判开关遗漏已恢复：代码未删，现场参数false/disabled源于新入口漏带AIC_DYNAMIC_GUARD。圆形入口/launch现默认启用，显式0可关闭，旧车型默认保持；导航恢复保留原目标并接续。GetParameters=true，现场跟踪计数573→584、预测判断47次；72静态视角及迎面/横穿/远离/横移回归通过。无新真实动态相遇证明，仍为1.8s预测停车保护。见圆形报告后续记录。

> 2026-10-03 旧模型显示已排查：旧ws_aic重新启动Gazebo，与新版桥接占用9090冲突。现清理两套进程，旧常用入口通过本机 `.aic_active_workspace` 默认转交圆形测试副本，原脚本已备份，旧源树未覆盖；`AIC_ROBOT_MODEL=legacy` 显式启动旧车型。真实桥接核验24link/30mesh均可读，显示底盘STL半径0.240m，共92028三角面。未直接读取Foxglove窗口；需要时刷新/重连获取新描述。见圆形报告后续记录。

> 2026-10-03 可选新车型已改为直径48cm圆形底盘、四径向轴普通全向轮，入口 `ros_round_omni_start.sh`（原mecanum入口为兼容别名）。接近目标保持底盘朝向，回转座预转并在到点复核后弯臂。独立副本连续蓝B/红A/蓝A成功，车体最大偏转0.030°，三块XY均在指定区；耗时61.95/80.58/27.16s，尚未证明总体墙钟提速。7373姿态传感器包围盒检查通过；理想驱动与虚拟吸附保留，完整带货三维包络未解决。旧副本未覆盖，新版测试系统保持空闲。详见 `docs/round-omni-20261003.md`。

> 2026-10-02 四麦轮、机械臂回转座、左右双雷达已接入可选测试车型（`ros_mecanum_start.sh`）。独立运行副本 `/home/polarbear/ws_aic_mecanum_20261002`，原 `/home/polarbear/ws_aic` 未覆盖。底盘为 Gazebo 理想平面速度驱动，不能作为真实滚子牵引性能证据。新旧各 12 段固定路线通过，侧向到点墙钟均值 8.27→6.09 s，但前向 3.53→5.26 s，蓝 B 完整任务 50.13→77.38 s；新版红 A 在出口受阻，206.93 s 后中止，第三件未执行。未证明整体提速，旧默认与旧运行系统已恢复。双雷达分别以真实原点清障，合成 /scan 仅用于兼容。回转动作、24 link / 30 网格及地图图像桥接通过，详见 `docs/mecanum-integration-20261002.md`。

> 2026-09-30 Foxglove 模型资源修复：rosbridge safe launch 在原 9090 端口提供白名单网格 HTTP 读取，并在桥接出站 URDF 中解析 package URI，兼容 CBOR-raw；ROS 内部模型不变。真实连接验证 22 link、15 网格全部可读，Windows HTTP 与地图/图像回归通过；尚未直接检查界面渲染。仅适用于本机客户端，见 `docs/foxglove-robot-assets-20260930.md`。

> 2026-09-30 TF 停更根因已捕获：GDB 确认 tf2_ros 0.25.23 异步等待存在锁顺序反转。新增 aic_tf2_fix，仅导航进程预加载兼容修复；原库压力测试 15 s 超时，修复后三次通过。420/240/120 s 运行及空闲监测均无位姿过期、无看门狗重启。静态 map→odom 模式下主控选块改用 /odom。首轮前四件成功，第五件与移动障碍互堵；调整 blue_cube_4→C 北侧引导点后 53.84 s 成功，红块送 B 57.31 s 成功。并非六件连续成功，动态相遇仍为有限验证；ROS 升级需审阅兼容库。详见 `docs/tf-deadlock-fix-20260930.md`。

> 2026-09-30 中途停车接续：看门狗重启前后通知导航执行节点，保留目标，等待两张地图位姿与服务稳定后重新规划；急停/暂停不续跑，120 s 超时。静态里程计定位下不再因未参与导航的 AMCL 漂移误重启。主动重启后红块抓取接续成功，但返程真正位姿停更仍复现，超时后主控重试，最终 202.84 s 放置成功。根因未解决，见 `docs/task-recovery-20260930.md`。

> 2026-09-30 转向提速：RPP 转向目标 1.0→1.5 rad/s，控制器与平滑器角加速度 1.2→2.4 rad/s²；直行与近障保护保持。出生点 90° 转向原配置 4.45 s，新配置三次 2.52–2.57 s；蓝块送 B 47.73 s 成功。仅有限验证，不能宣称所有墙角/动态障碍已解决。详见 `docs/turn-speed-20260930.md`。

> 2026-09-30 桥接显示回归已修正：safe launch 漏带原 XML 消息上限，默认 1 MB 导致 CBOR-raw 大地图/相机分片序列化失败并大量打印。现显式 16 MB、发送队列 16；真实 WebSocket 已收到约 3.94 MB 地图与 2.76 MB 图像，发令复测通过。未测量 Foxglove 界面渲染 FPS；仿真实时因子约 0.946。详见 docs/foxglove-command-20260930.md 后续回归记录。

> 2026-09-30 Foxglove 发令入口修正：demo1.json 的自然语言发布面板改为 /command；/chat 只接收解析后的任务数组。启动脚本使用 tools/rosbridge_safe.launch.py，服务请求工作线程、5 秒超时，并通过 rosapi_safe.py 处理消失节点查询返回 None 的崩溃。真实 WebSocket 测试消息已抵达解析器，无运动测试。旧的已打开布局需修改发布话题或重新导入。详见 docs/foxglove-command-20260930.md。

> 2026-09-30 障碍残影二次修正：运行参数确认 inf_is_valid 已启用，连续移动圆形障碍试验仍在约 1 度光束下留下 31 个障碍格。将真实射线增加至 1441 束、保持 resolution=1 后残影 0，真实障碍保留。实际一键冷启动通过，两路雷达约 10 Hz（仿真时间）、实时因子 0.940；未逐格复核用户截图，也不解决 Nav2 内部 TF 位姿过期。详见 `docs/ghost-obstacles-20260930.md` 第二轮记录。

> 2026-09-30 障碍残影：两套导航参数的局部/全局雷达源启用 inf_is_valid，让 Gazebo +Inf 无回波光束参与射线清障。隔离 Nav2 对照复现旧配置残留占用 100，修正后清空为 0，同时保留真实障碍 100。配置已同步并重建，尚未现场复核截图所有黑点；不解决 TF 位姿停更。详见 `docs/ghost-obstacles-20260930.md`。

> 2026-09-30 原地旋转排障：最终速度保护新增局部/全局 Nav2 位姿过期停车，10 项隔离测试通过；两件蓝块送 B 159.83 秒完成，但途中仍有导航失败、接受超时及看门狗重启。默认 ROS 环境改用已安装 Cyclone DDS 环回小包配置，手动 ROS 命令需 source tools/ros_env.sh。核对 Humble 源码后纠正历史“代价地图时钟冻结”说法：published_footprint 携带 TF 派生位姿时间戳，只能证明内部位姿过期，不能证明节点时钟冻结。该问题仍复现，未根治。详见 `docs/spin-fix-20260930.md`。

> 2026-09-30 新增可选雷达运动预测保护，`AIC_DYNAMIC_GUARD=1` 启用，默认关闭。使用扫描时间戳 TF、轮廓边界估速及 1.8 秒恒速碰撞预测，不依赖 Gazebo 障碍真值。最终版本横穿/反向穿越 31.84/36.71 秒成功，蓝块送 B 52.41 秒成功；接近走廊被初始规划拒绝，真实迎面相遇尚未验证。仍可能误停/漏检，不能替代主动让行；代价地图时钟停更仍复现。详见 `docs/dynamic-prediction-20260930.md`。

> 2026-09-30 移动障碍处理已调整：20 Hz 控制、10/5 Hz 局部/全局地图、2 Hz 重规划；恢复优先等待，恢复速度同样经过平滑器、Collision Monitor 和最终雷达/命令失联保护。保留近障停车与 2 秒碰撞时间减速，移除会导致超慢转向的固定减速区。障碍物非法角阻尼、惯量及持续推力已修正，全局物理参数维持原设置。受控横穿 29.83 秒成功，最近中心距离 0.68 m，无飞起/倾覆；横穿后蓝块送 B 全流程 40.10 秒成功。尚无动态轨迹预测，只完成有限相遇验证；任务后空闲仍复现全局代价地图时间戳停更。证据与局限见 `docs/dynamic-obstacle-fix-20260930.md`。

> 2026-09-30 提速配置已验证：默认改用 RPP 跟踪器，直行目标上限 0.70 m/s、原地转向 1.00 rad/s；Gazebo 驱动不再二次限加速度，由 Nav2 平滑器统一限加减速。本机 DDS 默认使用环回接口与单播发现；通过 `AIC_ROS_LOCALHOST_ONLY=0` 可在启动前覆盖本机限制。启动/看门狗新增代价地图时间戳检查 `FAIL:11`。最终冷启动六件全部完成，任务耗时合计 305.87 秒（含探针等待的总跨度 320.93 秒）；相同前三件由 140.66 降至 125.15 秒。六件执行期间无导航恢复或看门狗重启，六个物块中心在目标区内；轮后空闲仍出现时间戳停更，两次触发看门狗恢复。不能声称 DDS/墙角风险已根治。详情见 `docs/speed-optimization-20260930.md`。

> 2026-09-28 新底盘已接入：48×40×10 cm 底盘、45.4 cm 轮距，导航包络 58×55.2 cm。低位 `/scan` 保留，高位 `/scan_high` 为补充观测。仿真静态定位模式通过 `navigation_tf_relay` 保留原时间戳转发底盘 TF；动态 AMCL 模式不启用此转发。C 区停车点后移 55 cm。运行证据及仍存在的心跳超时风险见 `log/robot-concept-integration/report.md`。

> 面向在本仓库工作的 AI Agent / 协作者：项目定位、架构、接口、构建运行方式与改动约定。
> 内容基于对当前工作区源码的逐文件通读，以及在本机 WSL2 中的实测（实测项标注「已验证」）。
> 最近一次核对：2026-09-28。

---

## 0. 开始之前：5 条硬约束

1. **这个项目只在 WSL 里跑。** ROS 2 Humble、Gazebo Classic、MoveIt 2、Nav2 全部装在本机 WSL2（Ubuntu-22.04）中，Windows 侧只有源码和文档。
   任何 `colcon build`、`ros2 run/launch`、`llama-server` 都必须在 WSL 的 bash 里执行；从 PowerShell 调用请用 `wsl.exe -- bash -lc '...'`。
2. **路径映射。** `E:\workspace\AIC` ⇔ `/mnt/e/workspace/AIC`（同一份文件，DrvFs 挂载）。文档、脚本、给用户看的命令一律写 WSL 路径。
3. **历史文本文件多为 CRLF；本轮修改的维护文件已统一为 LF。** 直接执行 `.sh` 会报 `bad interpreter: /bin/bash^M`；shell 脚本使用前先 `sed -i 's/\r$//' <file>`（实测：原样的 6 个 `.sh` 全是 CRLF；但 `ros_competition_start.sh` 已重写为 LF 并自带 CRLF 自愈，可直接执行）。Python 文件带 CRLF 不影响运行，但改动时建议统一成 LF。
4. **`runtime/models/` 在 Windows 侧那份里是空的**，权重不在 Git 中。可用的 491 MB GGUF 与已编译的 `llama-server` 在 WSL 的 `~/AIC` 与 `~/ws_aic` 里（后者已自包含，见 §2.3 / §2.4）。
5. **Git 基线已恢复**：历史提交 `062b5f8 整理文件` 曾是删除 3789 个文件的空树；2026-09-30 已恢复工作区基线并上传 GitHub，当前基线为 `b2017c5`，源码已跟踪。
   后续先检查 `git status` / `git diff`，选择性暂存维护文件，避免把本地权重、构建产物或运行日志混入提交。不要执行未经审阅的 `git checkout .` / `git restore .` / `git clean -fdx`。

---

## 1. 项目是什么

「全球校园人工智能算法精英大赛 · 大模型技术创新赛」国赛参赛作品：一个跑在 **ROS 2 Humble + Gazebo Classic 11** 仿真里的「月球仓储机器人」系统。

裁判用自然语言下达命令（例：*抓取 3 个蓝色物块去 B 区，2 个红色去 A 区*），系统要闭环完成：

1. 本地小模型把中文命令解析成结构化任务；
2. 按实际可行路径选出物块与接近方向，自主导航到抓取点；
3. 用六轴机械臂 + Gazebo 吸附服务抓取物块；
4. 导航到指定区域（A / B / C）放置；
5. 全程在 Foxglove 面板上展示状态，供裁判观看打分。

赛制相关（来自 [比赛规则.docx](docs/比赛规则.docx)、[大模型技术创新赛_技术文档评分表_20分.docx](docs/大模型技术创新赛_技术文档评分表_20分.docx)、[国赛要求.txt](docs/国赛要求.txt)）：

- 线上直播比赛，**裁判现场发令**，看的是实时运行过程；指令一发出即开始计时，5 分钟内必须已经启动程序。
- 评分点：大模型指令解析、机械臂抓取、放置到指定区域、导航避障、动态障碍物/场景设计。
- 技术文档 20 分，明确「用现成的大模型 / Nav2 / MoveIt2 / OpenCV / Foxglove 不算创新」，要写清**实质性改进**与验证证据（架构图、流程图、运行截图、轨迹）。

> 对 Agent 的直接含义：**不要为了「代码更漂亮」而重排任务执行顺序或改动默认行为**，比赛按出题顺序计分；也不要引入新的重依赖（离线比赛环境）。

---

## 2. 运行环境：本机 WSL2

### 2.1 已实测的环境（无需重装）

| 项 | 实测结果 |
| --- | --- |
| 发行版 | `Ubuntu-22.04`（WSL2，默认发行版），内核 `6.6.87.2-microsoft-standard-WSL2` |
| 用户 / 主机 | `polarbear` / `bq-polarbear` |
| ROS | `/opt/ros/humble`（`ROS_DISTRO=humble`） |
| 关键包 | `rosbridge_server`、`nav2_bringup`、`gazebo_ros`、`moveit_ros_move_group`、`controller_manager`、`joint_trajectory_controller`、`xacro`、`robot_state_publisher`、`cv_bridge`、`tf_transformations` 均在位 |
| Gazebo | `/usr/bin/gazebo`、`/usr/bin/gzserver` 存在 |
| Python 依赖 | `requests`、`cv2`、`cv_bridge`、`tf_transformations`、`numpy` 均可 `import` |
| 编译工具 | `cmake`、`g++ 11.4.0`、`make`（**无 ninja**） |
| 资源 | 32 vCPU、15 GiB 内存 |
| 图形 | WSLg 可用（`/mnt/wslg`），rviz / Gazebo 可以出窗口；Foxglove 建议在 Windows 侧浏览器打开 |
| 已知缺失 | `linkattacher_msgs` 不在 `/opt/ros`，它由本仓库源码构建提供（见 §4） |

### 2.2 路径与命令约定

```powershell
# 从 Windows 侧（PowerShell）执行的推荐形式，避免引号/换行被 PowerShell 吃掉
wsl.exe -- bash -lc 'cd /mnt/e/workspace/AIC && source /opt/ros/humble/setup.bash && colcon build --symlink-install'
```

- 长脚本不要塞进 `wsl.exe -- bash -lc`，写成 `.sh` 文件再 `wsl.exe -- bash /mnt/c/.../x.sh`。
- 在 WSL 里操作 Git 仓库时注意 `core.fileMode`、行尾与属主；**不要用 Windows 侧 Git 与 WSL 侧 Git 交替提交同一份工作区**。

### 2.3 本机已存在的第二份副本 `~/AIC`（重要）

`/home/polarbear/AIC` 是同一项目的**另一份 WSL 原生副本**，与 `/mnt/e/workspace/AIC` 互补但不相同：

| | 本仓库 `/mnt/e/workspace/AIC` | 副本 `~/AIC` |
| --- | --- | --- |
| 布局 | `src/` + `docs/` 在根目录 | `code/src`（colcon 工作空间）+ 根目录放文档 |
| 应用代码 | **更新**：`main_controller_node.py` 1015 行（含按 Nav2 实际路径选块 + 连续任务前瞻） | 较早：774 行 |
| 启动侧补丁 | 较少：launch 仍是旧版（rviz 常开、spawn 无 `-timeout`） | 较多：`rviz:=` 开关、`spawn_entity -timeout 180`、进程组回收式的一键脚本、321 行 `AGENT.md` |
| 构建产物 | 无 | `code/install/` 中 **12 个包全部构建过** |
| 模型权重 | `runtime/models/` **空** | `runtime/models/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf`（491,400,064 B，已验证可加载） |
| llama.cpp | 源码 172 MB，**无 build/** | `runtime/llama.cpp/build/bin/llama-server` **已编译可用** |

因此，跑本仓库时可以直接复用副本里的模型与二进制（**已验证**：`llama-server` 1 秒内 `/health` 返回 ok，`POST /completion` 正常出结果）：

```bash
LLAMA_BIN=$HOME/AIC/runtime/llama.cpp/build/bin/llama-server
MODEL=$HOME/AIC/runtime/models/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf
```

**两份副本的改动不要互相覆盖。** 每次动手前先明确「这次改的是哪一份」，并在提交信息/说明里写清楚；跨副本同步用 `rsync -a --delete`（`src` 只有约 7 MB，很快）。

### 2.4 已迁移并跑通的运行副本 `~/ws_aic`（2026-09-28 实测）

本项目已整体搬到 WSL 原生 ext4：**`/home/polarbear/ws_aic`**（从 `/mnt/e/workspace/AIC` rsync 而来，含 `src/`、`docs/`、`runtime/`、`AGENTS.md`，并补齐了 491 MB 权重与 `llama-server`，可独立运行）。Windows 侧那份原样保留；两份内容一致时优先在 `~/ws_aic` 里跑。

一次完整运行的实测结果（指令：`抓取1个蓝色去B`）：

| 阶段 | 实测结果 |
| --- | --- |
| 全量构建 | `colcon build --symlink-install` 成功；13 个包 4 分 50 秒（含被一并构建的 `runtime/llama.cpp`，见 §6.3） |
| 大模型解析 | 中文指令 → `/chat` = `[{"color": "blue", "num": 1, "to": "B"}]`，约 1.5 s |
| 选块 | `/compute_path_to_pose` 检查 **20 个**抓取位置 → 选定 `blue_cube_5`（接近方向 2，抓取点 `(-3.70, 0.28)`） |
| 导航→抓取 | 到物块 16 s；机械臂 bend → 夹爪闭合 → `/ATTACHLINK` 吸附 → 举高，1.2 s 内发出 `grasp_succeeded` |
| 导航→放置 | 门前引导点 `(-1.80, -2.40)` → B 区 `(-1.75, -6.49)` 共 11 s；`/DETACHLINK` 分离成功，1.2 s |
| 收尾 | 主控日志 `放置完成（1/1）` + `所有优化任务执行完成，等待新任务...` |
| **端到端** | **下发命令 → 放置完成 48 s**（单件任务，场景中动态障碍物同时在跑） |
| 一键脚本 | `./ros_competition_start.sh` 一次通过全部 10 项就绪门，随后再下发同一指令同样走完闭环，`stop` 后独立复核残留进程 **0** |

同时复现了两条排障记录（见 §8）：rviz2 因 Map+LaserScan 的 GLSL 问题崩溃、Nav2 容器首启卡在 `load_node` 需整组重启一次。

---

## 3. 仓库结构

```
AIC/                                   # = /mnt/e/workspace/AIC
├── src/                               # 12 个 colcon 功能包（唯一需要构建的地方）
│   ├── llama_command_parser/           # 自然语言 → 任务 JSON
│   ├── main_controller/                # 任务规划 + 全流程状态机（系统大脑）
│   ├── nav_simple/                     # Nav2 导航执行与结果上报
│   ├── arm_action_integration/         # 六轴臂 + 夹爪 + 吸附服务编排
│   ├── package_detector/               # 红/蓝物块视觉标注
│   ├── gazebo_physics_obstacle_plugin/ # C++ 动态障碍物插件（SimpleMovePlugin）
│   └── yzbot/
│       ├── mybot/                      # MoveIt 配置与 launch
│       ├── mybot_description/          # URDF/xacro 模型 + worlds/room.world
│       ├── bot_navigation/             # 地图、AMCL/Nav2 参数、bringup launch
│       ├── tools_demo/                 # llama.cpp 调试工具（非主链路）
│       └── IFRA_LinkAttacher/          # linkattacher_msgs + ros2_LinkAttacher（吸附插件）
├── docs/                              # 赛题/评分/步骤资料（docx、pdf、txt）
├── runtime/
│   ├── llama.cpp/                     # llama.cpp 源码（无 build/，需自行编译）
│   └── models/                        # GGUF 权重目录（Windows 侧为空；~/ws_aic 里已放好 491 MB 权重）
├── demo1.json                         # Foxglove 布局（3D + /chat + 两路图像面板）
├── ros_competition_start.sh           # 一键启动/停止/状态脚本（WSL 适配版，LF；原版见 *.before-wsl）
├── .gitattributes / .gitignore
└── .git/                              # 已恢复并上传的 Git 基线（见 §0.5）
```

参考文档（二进制，简单工具读不了，需要时用 Office/PDF 工具打开）：[运行步骤.txt](docs/运行步骤.txt)、[总步骤.pdf](docs/总步骤.pdf)、[相关接口说明.pdf](docs/相关接口说明.pdf)、[总评分表.pdf](docs/总评分表.pdf)、[技术文档（国赛）.pdf](docs/技术文档（国赛）.pdf)。

---

## 4. 功能包与节点职责

| 包 | 语言/类型 | 角色 | 启动命令 |
| --- | --- | --- | --- |
| `main_controller` | Python | 任务规划 + 状态机（1015 行） | `ros2 run main_controller main_controller_node` |
| `llama_command_parser` | Python | 自然语言 → 任务 JSON | `ros2 run llama_command_parser command_parser` |
| `nav_simple` | Python | Nav2 动作客户端 | `ros2 run nav_simple simple_navigator` |
| `arm_action_integration` | Python | 机械臂/夹爪/吸附动作编排 | `ros2 run arm_action_integration arm_grab_place_node` |
| `package_detector` | Python | 红/蓝物块标注（仅可视化） | `ros2 run package_detector color_detector_node` |
| `gazebo_physics_obstacle_plugin` | C++ (ament_cmake) | `libsimple_move_plugin.so`：给 obstacle2/obstacle3 施力往复运动 | 由 world 文件加载 |
| `yzbot/mybot` | MoveIt 配置 | MoveIt 配置与 launch（含 `gazebo_world.launch.py`、`my_moveit_rviz.launch.py`） | `ros2 launch mybot ...` |
| `yzbot/mybot_description` | URDF/xacro | 机器人模型 + `worlds/room.world`（比赛场景） | 由 launch 使用 |
| `yzbot/bot_navigation` | Nav2 配置 | 地图、参数、`nav_bringup_gazebo.launch.py`、`bright_costmap.py` | `ros2 launch bot_navigation ...` |
| `yzbot/IFRA_LinkAttacher/linkattacher_msgs` | 消息 | `AttachLink` / `DetachLink` 服务定义 | 随工作空间构建 |
| `yzbot/IFRA_LinkAttacher/ros2_LinkAttacher` | C++ | Gazebo 吸附/分离插件（第三方） | 由 world 文件加载 |
| `yzbot/tools_demo` | Python | llama.cpp 交互式聊天调试（**非主链路**，发布到 `/llama_command`） | `ros2 run tools_demo chat_interactive_ros` |

其余辅助脚本（均为 CRLF，用前转换）：[cube_control.sh](src/yzbot/cube_control.sh)、[pick_on.sh](src/yzbot/pick_on.sh)、[pick_off.sh](src/yzbot/pick_off.sh)、[my_send_goal.sh](src/yzbot/my_send_goal.sh)、[send_goal_pick.sh](src/yzbot/send_goal_pick.sh)。

---

## 5. 架构与数据流

```
  Foxglove 面板（demo1.json，Windows 浏览器）
        │  rosbridge websocket ws://localhost:9090
        ▼
  /command ──► llama_command_parser ──► llama-server :8081（Qwen2.5-Coder-0.5B, /completion）
        │                                          └── 固定 prompt 约束输出为 JSON 数组
        │  /chat（任务 JSON 数组）
        ▼
  main_controller ── /manual_nav_target ──► nav_simple ──► Nav2 ──► /nav_status
        │  ── /current_target_cube ────────► arm_action_integration ──► /arm_status
        │  ── /nav_done_cargo /nav_done_area ─►        │
        │                                              └── /ATTACHLINK /DETACHLINK（Gazebo 吸附）
        └── /color /ask /pick /cur /target_area /number_pick ──► Foxglove 展示面板

  Gazebo ── /camera/image_raw、/camera/camera_info、/scan ──► package_detector ──► /detector/annotated_image
  Gazebo ── SimpleMovePlugin（libsimple_move_plugin.so）驱动 obstacle2 / obstacle3 往复运动
```

### 5.1 话题接口

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/command` | String | 外部 → parser | 自然语言命令入口（也可发 JSON：`{"data":"抓取1个蓝色去B"}`） |
| `/chat` | String | parser → main_controller | 解析后的任务 JSON 数组，如 `[{"color":"blue","num":1,"to":"B"}]` |
| `/manual_nav_target` | String(JSON) | main_controller → nav_simple | `{"type":"custom","x":..,"y":..,"yaw":..}`；也支持 `{"type":"area","target":"A"}`、`{"type":"pause"}` |
| `/nav_status` | String | nav_simple → main_controller | `succeeded` / `failed:*` / `canceled` / `emergency_stop` / `emergency_stop_cleared` / `paused` |
| `/current_target_cube` | String | main_controller → arm | `red_cube_1`…`blue_cube_5`，arm 以此决定吸附目标 |
| `/nav_done_cargo` | String | main_controller → arm | 值恒为 `arrived_at_cargo`，触发抓取（名字是历史遗留） |
| `/nav_done_area` | String | main_controller → arm | 值恒为 `arrived_at_area`，触发放置 |
| `/arm_status` | String | arm → main_controller | arm 节点只发 `grasp_succeeded` / `place_succeeded`；主控也处理 `grasp_failed` / `place_failed`，但失败目前全靠 8 s 超时兜底 |
| `/amcl_pose` | PoseWithCovarianceStamped | AMCL → main_controller | BEST_EFFORT QoS；主控据它计算路径代价 |
| `/emergency_stop` | Bool | 外部 → nav_simple | 急停/解除 |
| `/color`、`/ask` | Int32 | main_controller → Foxglove | 0=蓝、1=红、-1=无任务 |
| `/pick` | Int32 | main_controller → Foxglove | -1=未使能（WAIT/SELECT/NAV_TO_BLOCK）、0=抓取或返程（GRASP/NAV_TO_AREA）、1=放置（PLACE） |
| `/cur` | Int32 | main_controller → Foxglove | 0=空闲、1=选块/前往物块、2=抓取中、3=前往区域、4=放置中 |
| `/target_area` | Int32 | main_controller → Foxglove | 0=A、1=B、2=C、9=空闲 |
| `/number_pick` | Int32 | main_controller → Foxglove | 收到命令后 1，空闲 0 |
| `/camera/image_raw`、`/camera/camera_info`、`/scan` | 传感器 | Gazebo → detector | 检测节点的三路输入 |
| `/detector/annotated_image`、`/detector/status` | Image / String | detector → Foxglove | 标注图 + 1 Hz 状态心跳 |
| `/<model_name>/current_pose` | Pose | SimpleMovePlugin | 运动障碍物位姿，如 `/obstacle2/current_pose` |

### 5.2 动作 / 服务

| 名称 | 类型 | 调用方 |
| --- | --- | --- |
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | nav_simple |
| `/compute_path_to_pose` | nav2_msgs/ComputePathToPose | nav_simple（可达性预检）、main_controller（候选点代价评估） |
| `/arm_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | arm_action_integration |
| `/gripper_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | arm_action_integration |
| `/ATTACHLINK` / `/DETACHLINK` | linkattacher_msgs/AttachLink、DetachLink | arm_action_integration |

### 5.3 各节点关键行为（读代码得到）

- **llama_command_parser**：订阅 `/command`（**不是** `/chat`，避免自订阅回环），POST `http://localhost:8081/completion`，参数 `n_predict=512`、`temperature=0.05`、`stop=["\n"]`、`stream=false`。
  命令必须含 `红/蓝/A/B/C` 之一，否则丢弃；三层 JSON 清洗（直接解析 → 去代码块 → 正则提数组）；任务 ID 过滤，新命令作废旧命令，防止延迟返回覆盖。
- **main_controller**：订阅 `/chat`（BEST_EFFORT）→ 展开 `num>1` 为单件任务 → `optimize_task_order` **保持出题顺序**（只为每个任务选块）→ 状态机执行。
  选块阶段 `SELECT_BLOCK`：对每个同色候选物块的 4 个接近方向，用 `/compute_path_to_pose` 依次算「当前位置→抓取点」和「抓取点→目标区」两段路径长度，取总代价最小者；连续同区域任务时用 `_choose_route_with_lookahead` 预留后续往返；`_approach_penalty` 对已验证可行的接近方向（red_cube_3 的第 3 方向、blue_cube_5 的第 2 方向）给其他方向 +20 惩罚。
  所有回调都写成 `weakref.ref(self)` + 静态方法 + `del self_ref`，用于抑制长时间运行的内存增长，**改动时请沿用**。
- **nav_simple**：发目标前先 `/compute_path_to_pose` 预检，确认有路径才发 `/navigate_to_pose`；`request_id` 作废过期目标（连刚被接受的目标也会立即 cancel）；构造时 `wait_for_server` 各 10 s，**超时直接 `rclpy.shutdown()` 退出** → 必须先起 Nav2。
- **arm_action_integration**：抓取链 = 机械臂 bend → 夹爪 close → `/ATTACHLINK` 吸附 → 机械臂 lift；放置链 = bend → 夹爪 open → `/DETACHLINK` → lift。
  吸附成功后在「举高」步骤开始时**立刻**发 `/arm_status=grasp_succeeded`（边收臂边开车，省时间）；放置同理。
  夹爪只做开合展示，真正的抓持由 Gazebo 吸附服务把物块刚性连到 `six_arm/link6`；`/DETACHLINK` 失败会递归重试。
  构造时对机械臂/夹爪 action `wait_for_server()` **无限等待**，对两个服务循环 `wait_for_service(1.0)` → 必须先把 Gazebo（含吸附插件）与控制器拉起来。
- **package_detector**：HSV 阈值分割红/蓝，轮廓中心经相机内参投影成方向向量，再与 `/scan` 激光点方向做余弦相似度校验，画框发布标注图。**当前只做可视化，不解算物块坐标**——坐标硬编码在主控里。
- **SimpleMovePlugin**：SDF 参数 `speed`、`force`、`start_x/start_y/end_x/end_y`，用 `AddForceAtRelativePosition` 施力驱动障碍物在起终点间往复，并发布 `/<model_name>/current_pose`。

### 5.4 主控状态机（当前版本）

```
WAIT_TASK ──/chat 收到任务──► SELECT_BLOCK ──选出最优候选──► NAV_TO_BLOCK
    ▲                                                            │
    │                                                     /nav_status=succeeded
    │                                                            ▼
    │                                                          GRASP ──/arm_status=grasp_succeeded──► NAV_TO_AREA
    │                                                            ▲                                      │
    │  导航失败：轮换 4 个接近方向 → 全部失败则换同色物块           │                               succeeded
    │  无候选/区域导航失败：_stop_failed_task 清空队列              │                                      ▼
    └────────────────────────────────────────────────────────── 下一个任务 ◄── place_succeeded ──── PLACE
```

- 抓取：`GRASP_TIMEOUT=8 s` 超时，`max_grasp_retry=3`；重试耗尽 → 标记该物块失败 → 重新选同色物块。
- 放置：`PLACE_TIMEOUT=8 s`，失败无限重试。
- 去区域导航失败重试 1 次，仍失败则 `_stop_failed_task`；`emergency_stop` / `paused` 同样停止并清空队列，等待人工重新下令。
- B 区特殊路径：若抓的是 `blue_cube_5`，先去门前引导点 `(-1.8, -2.4, -π/2)` 再进 B 区（避开窄门）。
- 抓取成功的物块在内存表里标记 `is_grasped=True`，避免重复选同一块（重启节点即复位）。

### 5.5 硬编码坐标（改场景必读）

抓取点 = 物块坐标沿 ±x / ±y 偏移 `GRASP_OFFSET = 0.55` m 的 4 个候选之一，yaw 让车头朝向物块。物块坐标与 [room.world](src/yzbot/mybot_description/worlds/room.world) 中的 `<pose>` **逐条一致**（已核对，顺序也一致，z 忽略）：

| 物块 | 世界坐标 (x, y) | 主控索引 |
| --- | --- | --- |
| red_cube_1..5 | (7.632928, 5.523903) / (9.604514, -3.707741) / (-5.735783, 5.507306) / (-8.702837, 1.000008) / (-10.748330, 4.014158) | `RED_BLOCKS[0..4]` |
| blue_cube_1..5 | (8.464876, -7.097603) / (5.040937, -7.441163) / (-1.478995, 6.646223) / (-9.664453, -3.267239) / (-3.703343, 0.829596) | `BLUE_BLOCKS[0..4]` |

区域坐标**不等于**世界坐标，而是「停车位」（世界坐标沿 -x 偏移 0.55，A 另有 +0.08 的 y 修正）：

| 区域 | room.world 中 zone_* 的 pose | main_controller `AREA_COORDS` | nav_simple `area_coords` |
| --- | --- | --- | --- |
| A | (3.143086, -5.807858) | (2.593086, -5.727858) | (2.593086, -5.807858) |
| B | (-1.196544, -6.485499) | (-1.746544, -6.485499) | (-1.746544, -6.485499) |
| C | (-6.873777, -7.785160) | (-7.423777, -7.785160) | (-7.423777, -7.785160) |

注意：主控把区域目标也发成 `{"type":"custom"}`，因此 nav_simple 里那份区域表在当前流程中**实际未被使用**（只影响手工发 `{"type":"area"}` 的场景），但 A 区二者的 y 已经差了 0.08 m —— 改场景时两处都要对齐。
物块在 Gazebo 中的 model name 必须严格是 `red_cube_N` / `blue_cube_N`，arm 节点靠这个字符串调用 `/ATTACHLINK`。

---

## 6. 在 WSL 中构建与运行

### 6.1 依赖

WSL 里已经装好（见 §2.1），无需重复安装。真要从零装一台，参考 [src/yzbot/readme.md](src/yzbot/readme.md)：

```bash
sudo apt install ros-humble-desktop-full gazebo ros-humble-gazebo-* \
  ros-humble-moveit ros-humble-moveit-setup-assistant ros-humble-moveit-* \
  ros-humble-controller-manager ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster \
  ros-humble-nav2-bringup ros-humble-nav2* ros-humble-rosbridge-server \
  ros-humble-cv-bridge python3-opencv
```

### 6.2 模型与 llama.cpp

本仓库 `runtime/models/` 为空，两条路：

```bash
# 路线 A（推荐，已验证）：用已自包含的 WSL 运行副本 ~/ws_aic（本次成功跑通用的是它）
LLAMA_BIN=$HOME/ws_aic/runtime/llama.cpp/build/bin/llama-server
MODEL=$HOME/ws_aic/runtime/models/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf
# 等价地，也可以用更早那份副本：$HOME/AIC/runtime/...

# 路线 B：自己编译 llama.cpp 并把权重放到本仓库
cd /mnt/e/workspace/AIC/runtime/llama.cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release -DLLAMA_BUILD_TESTS=OFF -DLLAMA_BUILD_EXAMPLES=OFF
cmake --build build --target llama-server -j"$(nproc)"
# 权重：cp $HOME/AIC/runtime/models/*.gguf /mnt/e/workspace/AIC/runtime/models/  （或 git lfs pull）
```

自检：`$LLAMA_BIN --version` 有输出；起服务后 `curl -s http://127.0.0.1:8081/health` 返回 ok。

### 6.3 构建工作空间

```bash
cd /mnt/e/workspace/AIC            # 或 WSL 副本 ~/ws_aic
find . -name '*.sh' -not -path './runtime/*' -not -path './.git/*' -exec sed -i 's/\r$//' {} +   # 一次性修行尾
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src    # 关键：只构建 src/
source install/setup.bash
```

- **`--base-paths src` 不能省。** colcon 默认从当前目录递归找包，会把 `runtime/llama.cpp` 当包一起编译（实测：13 个包 4 分 50 秒，其中 llama.cpp 独占 2 分 04 秒）。只要 12 个 ROS 包时用 `--base-paths src`，约 2 分 45 秒（由实测总量推算）；需要 llama.cpp 二进制就按 §6.2 单独编译。
- **已验证**：DrvFs（`/mnt/c`、`/mnt/e` 同一文件系统）与 WSL 原生 ext4 上 `colcon build --symlink-install` 都能正常工作，符号链接与 ament 索引正常生成；单个 Python 包约 10 s。
- 由于 `--symlink-install` 对 ament_python 包走的是 egg-link（开发模式），**改 `.py` 源码无需重新构建**；改了 `setup.py` 入口、`package.xml`、C++（`SimpleMovePlugin.cc`、`ros2_LinkAttacher`）、URDF/xacro、launch 或 Nav2 参数后必须重新 `colcon build`。
- 追求速度时可把源码同步到 ext4 再构建（`src` 仅约 7 MB）：
  ```bash
  mkdir -p ~/ws_aic && rsync -a --delete /mnt/e/workspace/AIC/src/ ~/ws_aic/src/
  cd ~/ws_aic && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash
  ```
  注意：这样改的是副本，改动要记得同步回 `/mnt/e/workspace/AIC`（或用软链接把源码指回来）。

### 6.4 启动（顺序敏感，10 步）

**推荐直接跑一键脚本**（已适配 WSL，见 §6.5）：`./ros_competition_start.sh`。以下是不用脚本时的手工步骤：每步一个终端（推荐 `tmux`）；**先仿真，再 MoveIt/Nav2，最后跑依赖它们的模块**。

```bash
# 0) 每个终端都先做
cd /mnt/e/workspace/AIC && source /opt/ros/humble/setup.bash && source install/setup.bash

# 1 仿真：底盘 + 六轴臂 + 传感器（加载 room.world）
ros2 launch mybot gazebo_world.launch.py
# 2 MoveIt 机械臂规划与控制
ros2 launch mybot my_moveit_rviz.launch.py
# 3 Nav2 导航栈（必须早于第 6 步，否则 simple_navigator 会自己退出）
ros2 launch bot_navigation nav_bringup_gazebo.launch.py
# 4 大模型服务端（单独终端）
$LLAMA_BIN -m $MODEL -c 2048 --threads 8 --port 8081
# 5 语言解析
ros2 run llama_command_parser command_parser
# 6 导航执行
ros2 run nav_simple simple_navigator
# 7 机械臂抓取放置（需 Gazebo + 控制器 + 吸附服务就绪）
ros2 run arm_action_integration arm_grab_place_node
# 8 视觉颜色检测
ros2 run package_detector color_detector_node
# 9 主控
ros2 run main_controller main_controller_node
# 10 rosbridge（Foxglove 用）
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

- 第 1 步要等机器人真的进仿真：`ros2 topic info /scan` 必须有发布者（没有就是 `spawn_entity` 超时，见 §8）。
- 第 2 步的 `my_moveit_rviz.launch.py` 会同时开 `move_group` 和 rviz；其中 `move_group` 被写死 `DISPLAY=:0`（WSLg 通常也是 `:0`，一般无碍）。主链路不经过 MoveIt，但 rviz 面板需要它，且它是唯一给 `move_group` 设了 `use_sim_time: True` 的 launch。
- 第 3 步的 launch 默认启动 rviz2，可用 `rviz:=false` 关闭。Nav2 默认 `use_composition:=False`，planner/controller/AMCL 等独立进程；可显式传 `use_composition:=True` 恢复组件模式。
- **Nav2「就绪」不能只看 `ros2 action list`**：本次实测首启时容器卡在 `load_node`，`/navigate_to_pose`、`/compute_path_to_pose` 仍会出现在动作列表里（图缓存），但 `planner_server` 根本没加载，主控随即报 `Nav2路径规划服务不可用`。可靠判据是 `ros2 lifecycle get /planner_server` 与 `/bt_navigator` 都返回 `active [3]`，且 `ros2 node list` 里能看到它们；否则按 §8 整组重启 Nav2。
- 后台批量拉起（无图形终端也能用）的写法：
  ```bash
  mkdir -p log/run
  setsid nohup ros2 launch mybot gazebo_world.launch.py            > log/run/01_gazebo.log 2>&1 &
  setsid nohup ros2 launch mybot my_moveit_rviz.launch.py          > log/run/02_moveit.log 2>&1 &
  setsid nohup ros2 launch bot_navigation nav_bringup_gazebo.launch.py > log/run/03_nav2.log 2>&1 &
  setsid nohup "$LLAMA_BIN" -m "$MODEL" -c 2048 --threads 8 --port 8081 > log/run/04_llama.log 2>&1 &
  # …其余模块同理，每个都用 setsid 建独立进程组，方便整组回收
  ```
  停止：先 `pkill -f 'ros2 launch'`，再 `pkill -f ros2`、`pkill -f move_group`、`pkill -f rviz2`、`pkill -f gzserver`、`pkill -f gzclient`、`pkill -f llama-server`，最后 `pkill -f spawn_entity`；用 `ros2 node list` / `ps -ef | grep -E 'gzserver|move_group'` 确认没有残留。

### 6.5 一键脚本（已适配 WSL，推荐入口）

[ros_competition_start.sh](ros_competition_start.sh) 已重写为 WSL 可用版；原 40 行版本备份在同目录 `ros_competition_start.sh.before-wsl`（`.gitignore` 的 `*.before-*` 覆盖它）。

```bash
cd ~/ws_aic                        # 或 /mnt/e/workspace/AIC（需先构建）
./ros_competition_start.sh         # 拉起 11 个模块 + 等就绪 + 导航体检 + 后台看门狗 + 状态表
./ros_competition_start.sh --no-rviz      # 同上，不启动 rviz（WSLg 下 rviz2 易崩）
./ros_competition_start.sh --headless     # 不启动 gzclient/rviz；图形显示服务不可用时跑导航抓放验证
./ros_competition_start.sh --no-watchdog  # 同上，不挂后台看门狗
./ros_competition_start.sh doctor      # 只做「导航链路体检」，不合格自动自愈（发令前先跑这个）
./ros_competition_start.sh watchdog    # 前台跑看门狗（start 已自动在后台拉起一个）
./ros_competition_start.sh status      # 进程 / 就绪 / 最近一次体检结论 / 残留 一张表
./ros_competition_start.sh logs 09_main  # tail -f 指定模块日志（不带参数则打印全部尾部）
./ros_competition_start.sh restart     # stop + start
./ros_competition_start.sh stop        # 按进程组 SIGINT → SIGTERM → SIGKILL，再按名称兜底清扫
```

实现要点（改脚本时请保持）：

| 点 | 说明 |
| --- | --- |
| 工作空间 | 按脚本所在目录自动推断，不再写死 `$HOME/dev_ws` |
| 无图形依赖 | 每个模块 `setsid nohup` 进独立进程组，日志落 `log/run/<模块>.log`，PID 记在 `log/run/pids`；不需要 gnome-terminal / tmux |
| 就绪门控 | `/scan` 有发布者 → `/move_group` → `/navigate_to_pose` → `/planner_server`、`/bt_navigator` 为 `active [3]` → `/amcl_pose` 有数据 → llama-server `/health` → 业务节点 → `/ATTACHLINK` → rosbridge `:9090` |
| Nav2 自愈 | 容器卡住（`planner_server` 未加载）时自动整组重启一次；仍失败就继续往下走并提示手工处理 |
| spawn 补救 | 120 s 内 `/scan` 没有发布者时，自动补一次 `spawn_entity.py -timeout 180` |
| CRLF 自愈 | 脚本自身是 CRLF 时自动 `sed` 成 LF 再 `exec` 重跑 |
| `set -u` | 只在自身逻辑里开；source ROS/colcon 的 `setup.bash` 前后必须 `set +u`/`set -u`，否则会因 `AMENT_TRACE_SETUP_FILES` 未定义而**静默退出**（`status` 分支曾把 stderr 丢进 /dev/null，症状是"什么都不打印、exit=1"） |
| 导航体检 | `doctor` 调 [tools/preflight_check.py](tools/preflight_check.py)：调 `/compute_path_to_pose` 看**返回路径的起点**是否等于当前定位；比对定位与 Gazebo 真值；检查 `map→odom` 变换以及近期控制器 TF 过期日志。持续过期才触发自愈，避免短暂抖动中断任务 |
| 自愈方式 | 位姿视图停更 / 没有 `map` 帧 → 整组重启 Nav2；定位跑飞 → 用 [tools/set_initial_pose.py](tools/set_initial_pose.py) 重设 AMCL 初始位姿（连发 5 次并**验证 `/amcl_pose` 已收敛**）。自愈后自动复检，结论写在 `log/run/preflight.txt` |
| 仿真定位模式 | 一键脚本通过 `tools/ros_env.sh` 默认设置 `AIC_ODOM_MAP=1`，使用 Gazebo 里程计与地图对齐的静态 `map→odom`，AMCL 仍运行但不发布此变换。仅适用于当前仿真中里程计与 Gazebo 真值保持一致的条件；真实机器人或里程计漂移时设置 `AIC_ODOM_MAP=0` 回到 AMCL 动态定位。两种模式使用不同 Nav2 参数文件；切换后须重启导航栈 |
| AMCL 重设 | 在 `AIC_ODOM_MAP=0` 的动态定位模式，Nav2 重启后 AMCL 回到参数里的 `initial_pose=(0,0,0)`；机器人不在原点时必须重设。`nav2_recover` 已内置：优先用 Gazebo 真位姿，其次用重启前的 `/amcl_pose` |
| 看门狗 | `11_watchdog` 模块每 30 s 复检一次，发现坏死就自愈，日志 `log/run/11_watchdog.log`；`stop` 会一起回收 |

**实测（2026-09-28，`~/ws_aic`）**：`./ros_competition_start.sh` 一次通过全部 10 项就绪门（Gazebo 3 s、move_group 3 s、Nav2 各项 ≤6 s、llama 3 s、业务节点各 3 s、rosbridge 3 s）；随后下发 `抓取1个蓝色去B` 走完 解析 → 选块 → 导航抓取 → 门前引导点 → B 区放置 的完整闭环（主控 `放置完成（1/1）`）；`stop` 后独立复核残留进程数为 **0**。

**原版为什么跑不起来（实测输出，供对照）**：`$'\r': command not found`（40 行全 CRLF）、`cd: $'/home/polarbear/dev_ws\r\r': No such file or directory`（工作空间写死）、`gnome-terminal: command not found` ×10；而且脚本**静默退出、什么都不启动**（退出码仍是 0），现场很容易误判成"已经在跑"。

### 6.6 Foxglove（裁判看板）

1. 在 Windows 侧浏览器打开 Foxglove，连接 `ws://localhost:9090`（rosbridge 默认端口）；
2. 导入仓库根目录的 [demo1.json](demo1.json)（3D 视角 + `/chat` 消息面板 + 两路图像面板）；
3. 若 Windows 侧连不上，检查 rosbridge 是否在跑、以及 WSL 的 localhost 转发（WSL2 默认可用）。

### 6.7 手动触发一次任务（联调用）

```bash
# 自然语言命令（需要 llama-server 在跑）
ros2 topic pub --once /command std_msgs/String "{data: '抓取1个蓝色去B'}"

# 直接跳过解析，手工投递任务 JSON
ros2 topic pub --once /chat std_msgs/String \
  "{data: '[{\"color\":\"blue\",\"num\":1,\"to\":\"B\"}]'}"

# 直接给一个导航目标
ros2 topic pub --once /manual_nav_target std_msgs/String \
  "{data: '{\"type\":\"custom\",\"x\":2.59,\"y\":-5.73,\"yaw\":0.0}'}"
```

---

### 6.8 连续任务故障修复（2026-09-28）

- 运行和验证仍在 `/home/polarbear/ws_aic`；验证后的维护文件逐个同步回 `/mnt/e/workspace/AIC`。
- Nav2 默认独立进程，启动脚本按进程组和各服务器可执行文件完整回收；重启后仍须恢复 AMCL 初始位姿。组件模式保留为可选参数，不把原 TF 停更归因到某个尚未证实的上游缺陷。
- `preflight_check.py` 保留路径起点、定位与 Gazebo 真值、map 帧检查。绕墙路径长度只用于诊断，不再因长于直线而失败。额外检查 Nav2 日志里当前仿真时刻附近的控制器 TF 过期；code=8 可触发导航自愈。没有相关日志不等于主动证明控制器缓存正常。
- code=9 表示 Gazebo 真位姿越出 100 m 地图边界、离地超过 0.5 m 或倾翻超过 60 度；重启 Nav2 无法修复物理发散，停止自动导航自愈并提示重新启动仿真场景。
- 吸附插件使用真实 fixed joint；吸附、分离和位姿更新由 Gazebo WorldUpdateBegin 队列执行，ROS 服务等待最多 5 秒。不要在 ROS 回调里持有 physics mutex 再调用 Model::SetWorldPose，实测会卡住仿真。分离后删除关节记录，重复分离不能复用旧指针。
- 10 个物块补齐与 0.1 kg / 3 cm 立方体相符的惯量。轮子惯量及 arm 圆柱连杆质心/惯量方向按几何修正；ODE 迭代次数从 20 增至 80。全局 CFM、接触修正速度、接触层厚度保留原值；一轮全局调参试验破坏了底盘转向，已撤回。
- Gazebo server 与 GUI client 分开启动，GUI 失败不应终止 server。`--no-rviz` 在 launch 层关闭两个 rviz；`--headless` 同时关闭 gzclient。默认仍启动比赛观看界面；无图形显示时相机图像可视化未保证可用。
- 任务顺序、JSON/ROS 接口和物块/区域坐标保留。连续任务测量及失败试验记录在 `/mnt/e/workspace/AIC/log/nav-fix-20260928/`。

---

最终复测（2026-09-28）：`--headless` 且看门狗开启，连续六件全部完成，耗时 43.97 / 62.34 / 26.86 / 81.66 / 39.90 / 61.05 s，平均 52.63 s；13 个导航目标的恢复计数均为 0。源码同步后重新构建、冷启动补测红色到 B，57.80 s 完成。没有再复现 TF 过期或物理发散；这是本轮验证结果，不代表所有场景均已穷尽。对齐/重新接近仍可占用数秒，C 区有一件物块靠边。完整报告：`/mnt/e/workspace/AIC/log/nav-fix-20260928/report.md`。

## 7. 改动约定

1. **按问题所在位置修改项目维护文件。** 功能源码主要在 `src/`；为解决构建、启动、运行、诊断或文档问题，允许修改根目录启动脚本、`tools/` 辅助脚本、项目配置、文档及 `AGENTS.md`，不需要仅因文件位于 `src/` 外再次确认。`build/`、`install/` 是生成物，不要手改，也不要当参考实现；`log/` 的运行日志由程序生成，可保存测量记录、报告和临时诊断产物，但不要修改日志伪造验证结果。改动仍须明确运行副本并验证后同步。
2. **`.sh` 一律先修行尾。** 新增脚本时保证是 LF；Windows 侧编辑器/工具容易写成 CRLF。改完一键脚本至少 `bash -n` 语法检查，并实跑一次 `start` + `stop` 确认能起来、无残留。
3. **场景改动要同步三处**：`room.world` 的 `<pose>`、`main_controller` 的 `RED_BLOCKS`/`BLUE_BLOCKS`/`AREA_COORDS`、`nav_simple` 的 `area_coords`。
4. **任务 JSON schema（color/num/to）改动要同步三处**：parser 的 prompt、`clean_and_parse_json`、`main_controller._chat_callback` 的字段校验。prompt 是格式约束的唯一来源，schema 变了 prompt 必须跟着变。
5. **不要「优化」任务顺序。** `optimize_task_order` 有意保持出题顺序（比赛按出题顺序计分）；`optimize_multi_block_order` / `_greedy_multi_block_selection` 目前是历史实现，主流程走的是「每步重新选可达物块」。
6. **保持 `main_controller` 的 weakref + `del self_ref` 回调写法**，长时间比赛下用于抑制内存增长。
7. **物块命名不可改**：`red_cube_N` / `blue_cube_N` 同时被 Gazebo 模型名和 arm 节点的 attach 逻辑依赖。
8. `package_detector/color_detector_node.py` 只做可视化，不参与控制；`color_detector_node_without_rotation.py` 是历史版本（内含 `/cmd_vel` 发布，不要接进主链路）。
9. `yzbot/tools_demo` 是调试工具，`chat_interactive_ros.py` 发布到 `/llama_command`（**不是** `/command`），不要接进主流程。
10. 新增依赖时同步更新对应 `package.xml` 与 `setup.py`，并重新 `colcon build`。
11. **改了机器人出生点或换了地图，要同步 AMCL 初始位姿**：[`originbot_nav2.yaml`](src/yzbot/bot_navigation/param/originbot_nav2.yaml) 里 `amcl` 段的 `set_initial_pose: True` 与 `initial_pose.x/y/yaw` 必须等于 Gazebo 世界里的出生位姿（当前 `spawn_entity` 不带坐标，出生在原点，所以是 0/0/0）。
12. **Git 操作前检查现有改动**：基线已恢复，先查看 `git status` / `git diff`，不要重复全量暂存模型权重或其他本机产物。不要执行未经审阅的 `git checkout .`、`git restore .`、`git clean -fdx`。
13. **改动要落在明确的一份副本上**（`/mnt/e/workspace/AIC`、`~/AIC`、`~/ws_aic`），跨副本同步后要重新 `colcon build` 并复跑验证。

---

## 8. 已知坑与排障

| 现象 | 原因 / 处理 |
| --- | --- |
| `bad interpreter: /bin/bash^M` | 脚本是 CRLF（本仓库所有 `.sh` 都是）。`sed -i 's/\r$//' <file>` |
| `llama-server` 找不到模型 | 本仓库 `runtime/models/` 是空的；用 `$HOME/AIC/runtime/models/*.gguf` 或 `git lfs pull` 后确认真实大小约 491 MB |
| `llama-server` 启动报 `cannot open shared object file` | 二进制是从别处拷来的，RUNPATH 指向旧路径；改用 `$HOME/AIC/runtime/llama.cpp/build/bin/llama-server`（同目录的 `libllama*.so` 会在位）或在 `runtime/llama.cpp` 重新编译 |
| `command_parser` 报解析失败/超时 | 先确认 `curl http://127.0.0.1:8081/health` 正常；`/command` 的文本必须含 `红/蓝/A/B/C`，否则会被主动丢弃 |
| `simple_navigator` 启动约 10 s 后自己退出 | 它 `wait_for_server(timeout_sec=10)` 超时就 `rclpy.shutdown()`。**必须先起 Nav2** |
| `arm_grab_place_node` 卡住不动 | 它在死等 `/ATTACHLINK`、`/DETACHLINK` 服务（`wait_for_service` 循环）与两个 action server，需先起 Gazebo（含吸附插件）与控制器 |
| 导航永远 `failed: no_path` | 抓取点/目标点在代价地图里不可达。主控会自动轮换 4 个接近方向，全失败则换同色物块，再失败就清空队列回到 `WAIT_TASK`（需人工重新下令） |
| AMCL 报 `cannot publish a pose ... set the initial pose` | 参数里 `set_initial_pose: True` 已配（0,0,0）；若仍报错，手工发 `/initialpose`，或先确认 `/scan` 有发布者 |
| 有 AMCL 日志但没有 `map->odom` TF | AMCL 需要一帧激光才会真正发布；先查 `ros2 topic info /scan` 有没有发布者。机器人没进仿真时一切后续都会假失败 |
| Nav2 半死：有 `/amcl`、`/controller_server`，没有 `planner_server`/`bt_navigator`；容器日志出现 `failed to send response to /nav2_container/_container/load_node (timeout)` 后就不再输出 | **已复现**（本次首启一次，整组重启后正常）。修复：`pkill -INT -f nav_bringup_gazebo.launch` → `pkill -f component_container_isolated`（**只匹配 `nav2_container` 会漏掉真正持有节点的进程，残留容器会导致重复 `/amcl` 与双份 `map→odom` TF**）→ 确认 `ps -eo args | grep component_container` 为空 → `setsid nohup ros2 launch bot_navigation nav_bringup_gazebo.launch.py > log/run/nav2.log 2>&1 &` → 等 `ros2 lifecycle get /planner_server` 变 `active [3]` → **重启 `main_controller` 与 `nav_simple`** 清掉指向旧 action server 的状态，再重新下发指令 |
| 机器人没进仿真：`/scan`、`/odom`、`/joint_states` 都没有发布者 | `spawn_entity.py` 等 `/spawn_entity` 服务超时（默认 30 s，`room.world` 加载慢时会超）。本仓库的 `gazebo_world.launch.py` 已设置 `-timeout 180`；仍失败时先排查 Gazebo/spawn 日志，可手工救回：`ros2 run gazebo_ros spawn_entity.py -entity six_arm -topic robot_description -timeout 180` |
| Gazebo 起不来/上一轮残留 | `pkill -f gzserver; pkill -f gzclient`，确认 `ps -ef | grep gz` 干净后再 launch |
| rviz 崩掉/卡死（视口锁死；进程 `exit code -11`） | **已复现**：本仓库 [nav2_default_view.rviz](src/yzbot/bot_navigation/rviz/nav2_default_view.rviz) 里 Map 与 LaserScan **同时 Enabled**，WSLg/Mesa 下先报 `Vertex Program:rviz/glsl120/indexed_8bit_image.vert ... GLSL link result :`，随后进程死亡（ros2/rviz#463）；同一轮 moveit 的 rviz2 也 segfault。**主链路不受影响**（导航/抓放照跑），看图建议改用 Foxglove；一定要用 rviz 就先在 Displays 里关掉 LaserScan |
| 障碍物不动 | `libsimple_move_plugin.so` 没构建/没安装（应由 `gazebo_physics_obstacle_plugin` 构建产出）；确认 `colcon build` 覆盖该包，插件加载日志见 Gazebo 终端 |
| 规划成本明显不对 | AMCL 未就绪时 `current_robot_pose` 停在 (0,0)，主控的路径代价评估会失真 |
| `color_detector_node` 红色 HSV 上限写成 `[185,255,255]` | H 有效范围 0–179，此处无实际影响，但别照抄到新代码 |
| 停止时出现 `RCLError: failed to shutdown: rcl_shutdown already called`、`AttributeError: 'SingleThreadedExecutor' object has no attribute '_sigint_gc'`、`[ros2run]: Process exited with failure 1` | rclpy Humble 在 SIGINT 下的已知噪声，节点其实已正常退出，不代表运行故障 |

---

## 9. 速查

```bash
# 一键启停（推荐）
cd ~/ws_aic && ./ros_competition_start.sh       # 启动 + 等就绪 + 打印状态表
./ros_competition_start.sh status | stop | restart | logs 09_main

# 环境
wsl.exe -l -v                                  # Windows 侧看发行版
source /opt/ros/humble/setup.bash              # 每个新终端
cd /mnt/e/workspace/AIC && source install/setup.bash
cd ~/ws_aic && source install/setup.bash        # WSL 运行副本（推荐）

# 状态观察
ros2 node list ; ros2 topic list
ros2 topic echo /chat          --once
ros2 topic echo /nav_status    --once
ros2 topic echo /arm_status    --once
ros2 topic echo /amcl_pose     --once
ros2 topic info /scan                          # 有没有发布者
ros2 action list | grep -E 'navigate_to_pose|compute_path_to_pose|follow_joint_trajectory'
ros2 service list | grep -E 'ATTACHLINK|DETACHLINK'
curl -s http://127.0.0.1:8081/health

# 构建校验
colcon build --symlink-install --packages-select main_controller nav_simple llama_command_parser arm_action_integration package_detector
colcon test --packages-select <pkg>            # 仓库自带 ament_copyright/flake8/pep257 测试（多为模板默认值）

# 清理
pkill -f 'ros2 launch' ; pkill -f ros2 ; pkill -f move_group ; pkill -f rviz2
pkill -f gzserver ; pkill -f gzclient ; pkill -f llama-server ; pkill -f spawn_entity
```

### 完成一次改动前的验证清单

1. `colcon build --symlink-install` 无报错；`source install/setup.bash` 后新增/改动的节点能被 `ros2 run` 找到。
2. 起绿灯：`/scan` 有发布者 → `/amcl_pose` 有数据 → `/navigate_to_pose` action 可用 → `/ATTACHLINK` 服务可用 → llama-server `/health` ok。
3. 跑一遍最小闭环：`ros2 topic pub --once /command ...` → 观察 `/chat`、`/nav_status`、`/arm_status`、`/current_target_cube`，并确认 Foxglove 面板状态码变化符合 §5.1。本机参考量级（单件任务）：解析 1.5 s → 选块 1 s → 导航抓取 16 s → 抓取 1.2 s → 返程 11 s → 放置 1.2 s，**端到端 48 s**；跑完主控日志应出现 `放置完成（1/1）` 与 `所有优化任务执行完成`。
4. 若改了坐标/场景：确认 §5.5 三处表格仍然一致。
5. 汇报时说明「改的是哪一份副本」，以及是否同步到另一份。

---

## 附：核心文件索引

| 文件 | 说明 |
| --- | --- |
| [main_controller_node.py](src/main_controller/main_controller/main_controller_node.py) | 任务规划 + 状态机（1015 行，最核心） |
| [command_parser_node.py](src/llama_command_parser/llama_command_parser/command_parser_node.py) | 大模型 prompt 与 JSON 清洗 |
| [simple_navigator.py](src/nav_simple/nav_simple/simple_navigator.py) | Nav2 目标下发与可达性预检 |
| [arm_grab_place_node.py](src/arm_action_integration/arm_action_integration/arm_grab_place_node.py) | 抓放流程与吸附服务调用 |
| [color_detector_node.py](src/package_detector/package_detector/color_detector_node.py) | 视觉标注 |
| [SimpleMovePlugin.cc](src/gazebo_physics_obstacle_plugin/src/SimpleMovePlugin.cc) | 动态障碍物插件 |
| [room.world](src/yzbot/mybot_description/worlds/room.world) | 比赛场景与所有坐标来源 |
| [gazebo_world.launch.py](src/yzbot/mybot/launch/gazebo_world.launch.py) | 仿真 + 控制器加载链 |
| [my_moveit_rviz.launch.py](src/yzbot/mybot/launch/my_moveit_rviz.launch.py) | move_group + rviz |
| [nav_bringup_gazebo.launch.py](src/yzbot/bot_navigation/launch/nav_bringup_gazebo.launch.py) | Nav2 bringup + bright_costmap + rviz |
| [originbot_nav2.yaml](src/yzbot/bot_navigation/param/originbot_nav2.yaml) | Nav2/AMCL 参数（含初始位姿） |
| [demo1.json](demo1.json) | Foxglove 布局 |

### 2026-09-28 停车与雷达追加检查

导航请求新增 15 秒稳态时钟超时及迟到回调取消；雷达 resolution 从 0.5 改为 1，避免近远墙面插值产生虚假点，已在独立同位姿仿真确认 8 个偏墙回波降为 0。实际 min_range 从 0.5 改为底盘半对角线加 0.05（0.3894 米）。2 红到 B 重跑 114.43 秒成功。DDS/TF 长期断流根因仍未确定，禁止把短期成功写成根治。证据：log/stuck-20260928/report.md。

### 2026-09-28 后续回归

一键启动在启动 MoveIt/Nav2 前调用 `tools/ensure_controllers.py`，确认 `joint_state_broadcaster`、`arm_controller`、`gripper_controller` 均为 `active`；业务节点启动后再确认机械臂已订阅抓取指令。此前曾出现 Gazebo 已生成机器人、但机械臂控制器未加载，导致车到物块后无限等待的情况。

`bt_navigator.default_server_timeout` 的单位是毫秒，已由 20 调为 200；20 ms 曾让行为树反复报规划 action 应答超时。`tools/ros_env.sh` 默认 Fast DDS，手动发 ROS 命令时在 ROS 和工作空间 setup 后 source 它，使 CLI 与一键脚本使用同一种 RMW。曾试用 Cyclone DDS 与相机可靠传输；相机可靠传输期间发生过 `/scan`、图像、`/clock` 同时断流及控制器 TF 过期，已撤回，实验配置只保留在 `log/stuck-20260928/experiments/`。这是相关性诊断，不把相机 QoS 认定为已经证明的唯一根因。

撤回后使用 `./ros_competition_start.sh --headless` 和 Fast DDS 连续完成蓝 B、红 A、蓝 A、红 C、蓝 C、红 B 六件任务，耗时分别为 68.71、63.39、28.15、136.18、34.02、61.59 秒；整轮看门狗没有触发自愈。红 C 的抓取前导航仍在墙角发生 DWB `No valid trajectories` 与 `Failed to make progress` 恢复循环，耗时约 103 秒，不能声称墙角卡顿已解决。详见 `log/stuck-20260928/report.md`。

后续针对两处实测卡点加入走廊入口途经点：从 A 区去 `red_cube_3` 东侧抓取点时先经 `(-6.5,1.8)`；`blue_cube_4` 去 C 区时先经 `(-7.0,-4.3)`。同时导航客户端在发布 `succeeded` 前核对终点 TF 的新鲜度、位置和朝向，防止 Nav2 内部 TF 停更时的误报；一键脚本的当前仿真默认使用静态 `map→odom` 模式，配置见上表。默认一键脚本 `--headless` 冷启动后再次连续完成同样六件，耗时 46.56、62.86、26.42、79.10、44.97、61.64 秒。红 C 与蓝 C 两段在这轮没有长时间墙角停滞；仍有短暂 `No valid trajectories`，不能据单轮测试保证所有动态障碍时相都无卡顿。原始记录见 `log/stuck-20260928/odom-map-gates-six/` 和 `log/stuck-20260928/report.md`。
