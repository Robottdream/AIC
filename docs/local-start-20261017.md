# 当前本地启动命令

在安装 ROS 2 Humble 的 bash 终端执行。使用桌面 AIC 源码和隔离测试入口；该入口会调用根目录主启动脚本，将运行文件放在 ASCII 路径下。

## 启动 Gazebo 和完整程序

```bash
cd "/home/root123/桌面/AIC"
export PYTHONNOUSERSITE=1
export AIC_DRIVE_MODEL=body_force
export AIC_ROSBRIDGE_PORT=8765
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1

bash test_map.sh short_routes_20261007 stop
bash test_map.sh short_routes_20261007 build
bash test_map.sh short_routes_20261007 start
```

首次使用、修改源码后或临时目录被清理后执行 build。日常重新启动可以省略 build。等待 `Ready. Publish natural language on /command`；start 此时等待任务。

隔离入口自动从桌面 AIC 的 `runtime/` 查找 llama-server 和模型，无需手工设置路径。若使用其他位置的文件，可以显式设置 `LLAMA_BIN` 和 `MODEL`，这些设置优先。

## Foxglove 可视化

仅用 Foxglove 时，将上面最后一行替换为：

```bash
bash test_map.sh short_routes_20261007 start --headless
```

Foxglove 使用 Rosbridge，地址 `ws://127.0.0.1:8765`。`test_map.sh` 默认使用端口 8765，显式设置的 `AIC_ROSBRIDGE_PORT` 优先。

## 同时打开 Gazebo 和 RViz

2026-10-19 新增 `--rviz` 选项，使用导航 RViz 配置。已有程序需复位时执行：

```bash
cd "/home/root123/桌面/AIC"
bash start_competition.sh restart --rviz
```

首次启动使用 `bash start_competition.sh start --rviz`。
`--rviz` 与 `--headless` 互斥；启动后仍等待人工发令。
RViz 与其他组件一起由启动器管理，`stop` 会一并关闭。

## 普通搬运测试任务

另开终端，直接指定颜色、数量和区域：

```bash
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source /tmp/aic-map-short_routes_20261007/install/setup.bash
ros2 topic pub --once /command std_msgs/msg/String \
  "{data: '将1个红色物块搬到C区，再将4个蓝色物块搬到A区'}"
```

正式题目需发布完整题目和映射编号至 `/competition/problem`，核对 `/competition/preview` 后通过 `/competition/start` 确认。详见 `formal-problem-20261009.md`。

## 查看状态和停止

```bash
cd "/home/root123/桌面/AIC"
bash test_map.sh short_routes_20261007 status
bash test_map.sh short_routes_20261007 logs 09_main
bash test_map.sh short_routes_20261007 stop
```

## 倾斜异常时

车体真实横滚或俯仰超过 5°，驱动切断施力并暂停 Gazebo，日志输出 `STABILITY_FAULT`、故障前约五秒的 `STABILITY_TRACE`、邻近模型和故障帧的 `STABILITY_CONTACT`。`STABILITY_PAUSED` 表示现场已暂停；接触列表是故障帧采样，不能保证包含更早已经结束的撞击。

正常运行每秒输出 `BODY_STATE`，包含真实位置、姿态、速度、受限目标和施力。保护触发算任务失败，暂停后不会自动恢复或扶正。

**重新启动前保存日志**，因为启动会覆盖本次运行日志：

```bash
cd "/home/root123/桌面/AIC"
mkdir -p log
cp -a /tmp/aic-map-short_routes_20261007/log/run "log/incident-$(date +%Y%m%d-%H%M%S)"
bash test_map.sh short_routes_20261007 stop
bash test_map.sh short_routes_20261007 start
```

重新开终端时先执行本页开头的环境变量设置。故障详情及本轮测试见 `stability-turn-limit-20261017.md`。
