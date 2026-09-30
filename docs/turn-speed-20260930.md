# 转向提速实测（2026-09-30）

## 修改

两套在用 Nav2 参数（AMCL / 仿真里程计定位）同步调整：

| 参数 | 原值 | 新值 |
| --- | --- | --- |
| FollowPath.rotate_to_heading_angular_vel | 1.0 rad/s | 1.5 rad/s |
| FollowPath.max_angular_accel | 1.2 rad/s² | 2.4 rad/s² |
| velocity_smoother.max_velocity 角速度 | 1.2 rad/s | 1.5 rad/s |
| velocity_smoother.max_accel 角加速度 | 1.2 rad/s² | 2.4 rad/s² |

直行速度、角减速度、目标角度容差及 Collision Monitor / 雷达失联保护保持原设置。基线实际角速度峰值仅 0.485 rad/s，未到上限，因此同时调整角加速度。恢复行为的旋转参数没有改变，默认导航恢复树不执行 Spin。

源码同步到 `/home/polarbear/ws_aic`，在 WSL 选择构建 bot_navigation 成功；通过 ROS 参数服务更新当前控制器和平滑器，复查新值正确。后续看门狗或一键重启使用同样的新配置。

## 验证

当前运行已停止，使用 `/home/polarbear/ws_aic/ros_competition_start.sh --headless` 冷启动，体检通过。`tools/turn_probe.py` 在出生点附近向 Nav2 发送相同位置、相对当前朝向 ±90° 的目标，经过完整速度保护链，记录命令和里程计；未直接驱动轮子或绕过保护。

| 配置 / 方向 | 结果 | 墙钟时间 | 实际角速度峰值 | 位置漂移 |
| --- | --- | --- | --- | --- |
| 原配置 +90° | 成功 | 4.453 s | 0.485 rad/s | 3.47 mm |
| 新配置 -90° | 成功 | 2.562 s | 0.933 rad/s | 1.34 mm |
| 新配置 +90° | 成功 | 2.571 s | 0.958 rad/s | 1.35 mm |
| 新配置 -90° 重复 | 成功 | 2.521 s | 0.934 rad/s | 1.32 mm |

新配置三次平均 2.552 s，比单次基线短约 42.7%。成功按现有目标容差判定，结束角差 0.185–0.237 rad，并非精确旋转到 90°。只有一次旧配置样本，不能当作统计显著性结论。

随后 `tools/speed_probe.py --limit 1` 发出“抓取1个蓝色去B”，抓取 blue_cube_5、经过门前引导点与 B 区窄门、放置成功，墙钟 47.734 s（仿真 44.2 s）。本次三个导航目标全部 succeeded；实际角速度峰值约 1.073 rad/s，IMU 最大倾角约 0.027°。没有相同初始条件的旧配置完整任务对照，不能声称任务总成绩提高特定百分比。

原始证据：`/mnt/e/workspace/AIC/log/turn-speed-20260930/`，各转向子目录包含 summary.json 和 trace.jsonl，task 子目录包含任务轨迹与 results.json。

## 局限

验证覆盖空地双向原地转向和一次抓放路线，没有系统验证所有墙角、载荷和动态障碍相遇。近障时仍可能被碰撞预测减速或停车；TF 派生位姿过期停车问题也没有由本次参数调整解决。
