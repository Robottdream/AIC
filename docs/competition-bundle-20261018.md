# 当前参赛源码与启动入口

2026-10-18 快照来自当前桌面 AIC 工作树，包含尚未发布的本地修改。
源码、地图、工具及测试与桌面 `AIC_参赛` 核心文件一致，另同步其 `start_competition.sh`。

## GitHub 源码与桌面参赛包的区别

- GitHub 保存源码、地图、参数、Foxglove 布局、测试与维护说明。
- 模型、llama-server 二进制、ROS 编译产物及运行日志均由 `.gitignore` 排除。
- 桌面参赛包另附 `runtime/llama.cpp/build/bin/` 完整动态库和模型；从 GitHub 下载后需要自行提供这些运行文件，或复用官方 U 盘系统已有 `~/llama.cpp`。
- 启动仍依赖 ROS 2 Humble、Gazebo Classic、MoveIt2、Nav2、rosbridge 与构建工具；需要在目标系统重新构建。

## 启动

在源码根目录执行：

```bash
bash start_competition.sh check
bash start_competition.sh build
bash start_competition.sh start --headless
```

入口优先尊重显式 `LLAMA_BIN`、`MODEL`，否则选择用户主目录已有的标准模型环境，再回退到项目 `runtime/`。
同时设置所选服务端的动态库目录，避免旧编译 RUNPATH 导致复制后无法加载库。
若终端残留失效的旧变量，可先执行 `unset LLAMA_BIN MODEL` 再运行。

`start` 只等待人工发令；`--headless` 关闭 Gazebo 窗口并保留仿真数据。
Foxglove 导入根目录 `foxglove_competition_layout.json`，使用 Rosbridge 连接 `ws://127.0.0.1:8765`。
发送当轮原题及映射规则，核对预览后使用本次 `preview_id` 确认启动。
当前未完整支持官方所有题型和文字映射，见下方核查记录，不能把本快照当作已全部满足参赛条件。

## 复位和停止

已运行时不要重复 `start`。任务结束后需恢复场景时执行：

```bash
bash start_competition.sh restart --headless
```

```bash
bash start_competition.sh status
bash start_competition.sh logs 09_main
bash start_competition.sh stop
```

此入口与原 AIC 都使用 `/tmp/aic-map-short_routes_20261007` 暂存目录；不要同时启动两套。
启动会覆盖该目录中的 `log/run/`，保存证据时先复制日志到另一个带时间标记的目录。

## 版本范围

本快照保留用户要求上传时的实际算法，不包含后续计划中的“大模型直接解原题”等修改。
最新4红A、1蓝C五件在主控日志中完成，现实时间175.400231728秒，范围为接收任务至全部完成，不含原题解析及人工确认。
本轮没有准确仿真耗时记录，没有在官方 U 盘启动系统完成本快照的完整验证。

- [题目与参赛要求核查](competition-requirements-audit-20261018.md)
- [当前比赛展示布局](foxglove-competition-layout-20261018.md)
- [返程路线恢复](return-route-recovery-20261018.md)

录制视频及技术文档仍需按组委会要求准备；启动入口不会自动录像。
