# Gazebo 与 RViz 同时显示

启动器新增 `--rviz`，同时打开 Gazebo 客户端与导航 RViz 配置。
当前在用的桌面 `AIC_final` 和桌面 `AIC` 已同步该启动器。

在 AIC_final 根目录执行：

```bash
bash start_competition.sh start --rviz
```

已有程序运行、任务结束后需要复位再执行一轮时：

```bash
bash start_competition.sh restart --rviz
```

不要同时添加 `--headless`；两个参数互斥。省略动作直接运行脚本仍沿用无窗口默认行为。
程序只等待人工发令，仍可同时通过 Foxglove 操作。
RViz 使用 bot_navigation 的 nav2_default_view.rviz，并设置 use_sim_time=true。
RViz 由项目进程管理器保存和停止，MoveIt 自带的第二个 RViz 保持关闭。

导航 RViz 配置订阅 `/visualization/map` 和 `/visualization/global_costmap`，
因此 `--rviz` 会一并启动已有的 `bot_navigation/bright_costmap.py`。
它转发亮色显示副本，导航仍订阅原始地图。首次补开窗口时漏启动此节点，
曾显示“No map received”；现已补入入口，无需改地图或重启当前仿真。
同时补回显示脚本的可执行权限，否则 `ros2 run` 会报“No executable found”。
实际订阅确认原始地图、亮色地图与亮色全局代价地图均为640×440、map坐标系。
RViz 固定坐标系同步设为 `map`，静态地图无需再按零时间戳转换到 `odom`。

本次对已有仿真补开了 gzclient 和 RViz，没有复位场景或发布搬运任务。
确认 gzclient 已连接 Gazebo master，RViz 初始化 OpenGL 并接收地图；
新增选项通过命令行解析与 Python 语法检查，未额外重跑搬运任务。

后续重开窗口时，RViz 在地图已加载后仍出现段错误。GDB 捕获退出栈位于
`rclcpp::CallbackGroup::~CallbackGroup()`，完整根因尚未确定，未修复此崩溃。
因此目前只确认地图转发与启动配置修正，不能将本记录表述为图形客户端稳定性通过。
原始调试日志位于本地 `log/rviz-map-crash-20261019.txt`，不提交运行日志。
