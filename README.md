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

保留直行 2.2 m/s、角速度 4 rad/s、角加速度 10 rad/s²、单轮转矩 20 N·m，Gazebo 单轮加速度限幅关闭。底盘 10 kg、驱动轮各 1 kg；十个货物质量 0.1 kg、边长 3 cm，三个主惯量已修正为 0.000015 kg·m²。

2026-10-13 默认新地图采用实时预测导航：目标重规划5 Hz、控制20 Hz，使用物体实时状态检查未来约2秒；不插入固定等待/穿越目标，不封锁整条移动轨道。两轮五件139.16/134.9、161.06/145.5墙钟/仿真秒完成，落点与高度合格，最大观测倾角约0.0574°，静态和移动障碍安全轮廓重叠样本均为0。存在短暂安全制动；两轮中控制预检坐标有调整，不能作为严格性能对照。Python48项、C++4项测试通过，有限验证不代表长期成功率。

- [实时预测路径、阶段耗时和失败/成功记录](docs/live-obstacle-routing-20261013.md)
- [货物惯量修正证据](docs/cargo-inertia-and-turning-20261012.md)
- [规则输入格式](docs/formal-problem-20261009.md)
- [静态障碍对齐来源](docs/static-obstacles-20261008.md)
- [斜向通道与墙角修复](docs/wall-corner-fix-20261011.md)
- [USB 无界面运行](docs/usb-headless-20261009.md)

历史日期文档描述当时的配置，不作为当前参数入口；以当前场景文件和本 README 为准。
