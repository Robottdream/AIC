# 协作约定

本工作树基于 `codex/stable-062-20261004`（85d3edf）。
目录和运行方法见 `docs/stable-layout-20261004.md`。

2026-10-05：stable 雷达残影修正已同步到 /home/polarbear/ws_aic。
扫描源 inf_is_valid=true，真实射线 1441、resolution=1。
隔离 Nav2 移动障碍试验残影 0、真实障碍保留；实际扫描数据已确认。
雷达修复当时未逐格复核界面或执行搬运回归；后续搬运验证见提速记录。
隔离雷达验证详见 docs/ghost-obstacles-20261005.md。

2026-10-05（首轮）：直行目标1.2 m/s、转向2.0 rad/s，控制20 Hz；平滑器加减速显式配置，
Gazebo关闭二次限加速，ODE约束10→30解除约0.65 m/s实速封顶。
质量、摩擦、5 Nm扭矩保持；交叉预测到达时间同步速度与加速估计。
同路径5米直行仿真8.976→4.964秒；三红A两蓝B五件211.86秒完成且落点合法，
带载最高1.219 m/s、最大倾角0.0458°。单轮有限验证，见 docs/speed-20261005.md。

2026-10-05（第二轮，当前）：直行目标1.6 m/s，线加/减速上限3.5/5.0 m/s²；
前视上限1.0 m、终点减速范围1.2 m，转向维持2.0 rad/s。
底盘6→10 kg，修正轮子圆柱惯量，ODE迭代20→60；摩擦、5 Nm扭矩和几何保持。
释放货物保留新鲜、已核验的分离前位姿，避免删吸附约束时的冲击；最终落点核验保留。
WSL默认Fast DDS使用本机回环UDP及1400字节消息；显式RMW/DDS配置优先。
五件181.84墙钟秒/138.1仿真秒完成且落点合法，最高1.614 m/s、最大倾角0.0587°。
2.0 m/s及原质量惯量1.6档带载失稳，未采用；9项回归通过，见 docs/speed2-20261005.md。

- ROS 2 Humble、Gazebo、colcon 和 llama-server 必须在 WSL bash 中运行。
- ROS 源码在根目录 `src/`，构建必须加 `--base-paths src`。
- 使用根目录 `ros_competition_start.sh`，`run.sh` 是兼容入口。
- `start` 等待人工自然语言发令；只有显式 `benchmark` 才自动发五件任务。
- 不以另一分支的机器人或导航参数覆盖 stable 基础；任务顺序保持裁判输入顺序。
- 模型、构建输出和运行日志不提交；修改前检查 git status/diff。
- 本 worktree 与 `/home/polarbear/ws_aic` 是不同副本，同步前明确目标，不互相覆盖。
- shell 维护文件使用 LF。Windows Git 管理本 worktree，WSL 只执行构建与运行。
- 禁止未经审阅的 git restore .、git checkout . 和 git clean -fdx。
