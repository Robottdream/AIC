# Gazebo 垂直俯视跟随

默认场景 `scenarios/short_routes_20261007/office_test.world` 的 GUI 相机跟随 `six_arm`。
相对模型位置为 `0 0 20` 米，初始俯仰为 π/2，向下垂直看。
`static=true` 固定相对位置，`use_model_frame=true` 随小车平移，
`inherit_yaw=false` 不随小车偏航旋转。需要改变显示范围时调整相对高度。
生成器 `tools/design_short_map.py` 保持相同配置，避免重新生成地图后丢失。

场景 SDF 声明从 1.4 升至 1.6，以支持相机的 static/use_model_frame/xyz/inherit_yaw 字段。
逐项核对除 GUI 和 SDF 版本声明外，模型、障碍轨迹、物理与导航配置均未变化。

与正常启动方法相同：

```bash
bash start_competition.sh start --rviz
```

只要启用 Gazebo 窗口就会应用跟随设置；`--headless` 不显示窗口。
本次保存配置时仿真已经停止，未额外启动搬运任务。当前未对运动中的画面进行目视确认。

参考：[SDFormat GUI camera](https://sdformat.org/spec/1.6/gui/)。
