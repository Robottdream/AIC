# stable 基础与启动结构

开发基线：`origin/codex/stable-062-20261004`，提交 `85d3edf`。
本次修改已按用户要求同步覆盖 /home/polarbear/ws_aic 运行副本。

目录：`src/` ROS 包，`tools/` 启动与计时工具，`docs/` 文档，
`runtime/` 本地模型与 llama.cpp，`log/run/` 运行日志与进程记录。
构建输出为根目录 `build/`、`install/`、`log/`，均不提交。

在 WSL 中进入本工作树：

```bash
cd /mnt/c/Users/21920/.codex/worktrees/f8e1/AIC
bash ros_competition_start.sh build
bash ros_competition_start.sh start --headless
bash ros_competition_start.sh status
bash ros_competition_start.sh logs 09_main
bash ros_competition_start.sh stop
```

`run.sh` 转发到同一入口。`restart` 重新启动；`benchmark` 显式执行
stable 原有红三件 A、蓝两件 B 的计时实验（会自动发任务，完成后退出）。
常规 start 不发任务，等待 `/command` 自然语言输入；Foxglove 导入根目录
`demo1.json`，连接 `ws://localhost:9090`。

模型与二进制优先使用本项目 runtime，再查找 `/home/polarbear/ws_aic/runtime`
和 `/home/polarbear/AIC/runtime`。可用 `LLAMA_BIN` 与 `MODEL` 指定。
仅 tf2_ros 0.25.23 向 Nav2 进程预加载本分支自带 aic_tf2_fix。

结构迁移时保留 stable 导航参数、模型、任务算法和默认 DDS 环境。
后续雷达与提速调整记录在 ghost-obstacles-20261005.md、speed-20261005.md、
speed2-20261005.md；当前通过验证的速度为1.6 m/s，WSL默认DDS通信使用回环UDP。
启动前过滤 WSL 继承的 Windows PATH，避免 CMake 在 Windows SDK 目录中慢速查找。
图形开关只控制 Gazebo 客户端和 RViz，不改变仿真参数。
沿用安全 rosbridge 的服务线程、超时、大消息与网格 HTTP 读取支持。
旧 gnome-terminal 脚本保存在 `docs/ros_competition_start.original.sh` 供参考。
启动失败回收本次启动的进程组；stop 按记录回收，不全局杀 ROS 进程。
未迁入另一分支的导航看门狗、静态 map→odom 和模型替换补丁。

## 验证（2026-10-04）

- WSL 独立原生副本 `/tmp/aic-stable-layout-20261004` 中 13 个 ROS 包构建成功。
- 7 项原有算法回归测试通过；修正测试桩缺少 `_stop_alignment` 方法的适配问题。
- 无界面冷启动通过雷达、三个 active 控制器、Nav2、llama /health、
  `/command` 解析器、机械臂订阅者和 rosbridge 就绪检查。
- `status` 确认 10 个模块运行；`stop` 后状态清空，独立检查无 gzserver 和 llama-server。
- 未下发搬运任务，未测 Foxglove 界面渲染，不将启动验证视为五件搬运回归。

构建时发现继承的 Windows PATH 导致 CMake 长时间访问 DrvFs；过滤后
原生副本增量构建 18.3 秒完成。就绪查询使用 `--no-daemon`，避免旧 CLI daemon
的 ROS 图缓存造成假超时。
## 同步到正式 WSL 目录

已同步到 /home/polarbear/ws_aic，保留 runtime 与原 Git 元数据。
旧源码、工具、文档、启动文件和构建产物备份在
/home/polarbear/ws_aic_backups/before-stable-20261004-233640。
新目录重新构建 13 个包成功（22.2 秒），7 项回归通过。
