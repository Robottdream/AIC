# 车体速度闭环与直接施力（2026-10-15）

## 修改范围

研究分支 codex/body-velocity-control，自 main 检查点300659c开始；main未改变。新增 aic_body_drive Gazebo Classic ModelPlugin，默认替代 gazebo_ros_diff_drive。原接口 /cmd_vel、/odom、odom→base_footprint、轮子TF保留；原地差速转向和前向运动接口保持，未改为全向车。

驱动采用前向/横向/偏航三轴 PI，横向目标恒为0。每个物理步读取车体真实速度，施加有限世界水平力与Z向转矩；上限分别150 N、200 N、100 Nm，增益见 src/aic_body_drive/README.md。无姿态/速度强写或垂直扶正。左右轮电机fmax=0，由地面被动带动；因此不能再把轮速与指令差看作驱动误差。保留原有碰撞、重力、μ=1、机器人及货物质量/惯量。

车体力不经由轮胎电机牵引链。该抽象适合研究导航和跟踪，但不代表真实电机/轮胎具备同等能力。参考来源与区别见 pb2025-drive-reference-20261015.md；此次实现不引入Ignition依赖或麦轮横移功能。

## 空场同条件对照

原始目录 `/home/polarbear/.local/share/aic-runs/20261015-body-force-ab-v2`。轮子→车体力→车体力→轮子四次冷启动，每次直行和圆弧各一次，共8项输入有效、0项启动/运动中止。目标前向3.0 m/s，圆弧2.0 rad/s；每次动作3.2仿真秒，停车观察2.2秒。后段窗口是停车请求前0.5到0.1秒，不将短测称为长期稳态。

物理1 ms、1000 Hz、ODE接触纠偏200、quick60、CFM0、有限旋转关闭；质量和摩擦保持。平滑器线加减速3.5/5、角加减速10保持。原生核验轮子驱动电机fmax1000，新驱动0；机器人XML除整个驱动插件块外一致，同类驱动模型哈希相同。导航和任务配置哈希一致。

| 驱动 | 圆弧后段前向 m/s | 圆弧后段偏航 rad/s | 全动作最大偏航 rad/s | 最大车身倾角 ° | 指令积分参考位置RMSE m |
|---|---:|---:|---:|---:|---:|
| 轮子电机 | 0.739560 | 5.322266 | 8.544210 | 0.060063 | 0.875046 |
| 车体施力 | 3.017511 | 1.995911 | 2.005896 | 0.055699 | 0.114404 |

表中前向、偏航、RMSE为两个重复的均值；峰值为两个重复的最大值。RMSE相对本次实际平滑后指令积分，包含指令接收时间误差，**不是Nav2路径跟踪误差**。新方案仍有起步滞后和小的侧向误差，不称为完美跟踪。

两次车体驱动另各执行原地2 rad/s、倒车−1 m/s、断指令停车检查，6项均通过；平滑器停用后刻意停止发布，驱动0.5仿真秒超时制动，1.8秒后速度接近0。servo-checks.json保留实测数值，检查不混入8项直行/圆弧统计。

速度/轮速曲线、轨迹、轮轴/倾角图已逐张审阅。Windows桌面 `AIC-body-force-20261015` 保存报告、指标、三张PNG与SHA256导出清单。当前试验表明车体闭环方案修复了该条件下的高速圆弧异常；没有证明旧轮子驱动异常的唯一底层原因。

## 验证与试验边界

- aic_body_drive、mybot_description构建通过；现有49项Python回归通过。
- wheel/body_force两种xacro展开各只含一个驱动，非法值会报错；运行时原生电机上限和施力遥测核验通过。
- 首次尝试20261015-body-force-ab因xacro内嵌字典表达式错误在生成机器人前失败，没有运动记录，保留原始日志与EXCLUDED.md。已改为dict()并分别验证展开后再执行v2，不纳入结果。
- 首次独立CMake配置误扫Windows PATH而缓慢，取消后在纯WSL PATH重新构建16秒完成；未改变全局Git/系统配置。
- 第三次冷启动一次只读GetParameters响应超时后自动重试成功，发生在运动前；未删除失败查询记录。
- 执行时脚本、驱动源码与xacro快照保存在execution-sources；采集后只补充CLI严格校验/模型来源哈希和无数据报告处理，未改变驱动二进制或采样协议，记录见post-run-tool-changes.json。
- 正式导航保持2.2 m/s，关闭墙边距离减速与RPP代价减速的设置保持，滚动预测与轮廓保护保持。

## 重复运行

WSL bash中先使用根入口构建，然后：

```bash
source /opt/ros/humble/setup.bash
source /home/polarbear/ws_aic/install/setup.bash
python3 tools/physics_benchmark/run.py --suite drive --variants wheel body_force \
  --repeats 2 --speed 3.0 --omega 2.0 \
  --output "$HOME/.local/share/aic-runs/body-force-$(date +%Y%m%d-%H%M%S)" \
  --windows-output /mnt/c/Users/21920/Desktop/AIC-body-force
```

默认启动使用车体施力；`AIC_DRIVE_MODEL=wheel bash ros_competition_start.sh start --headless`可选回旧模型对照，须先停栈。完整参数语义见插件README，脚本完整流程见tools/physics_benchmark/README.md。

## 五件搬运验证

原始目录 `/home/polarbear/.local/share/aic-runs/20261015-body-force-flow`。新驱动独立冷启动后运行正式新地图三红A两蓝B，导航巡航仍为2.2 m/s。五次抓取、五次放置完成，五件区域与高度全部合格，未触发碰撞或失稳中止；静态及移动障碍轮廓采样重叠数均为0。

按主控任务阶段事件为 **129.478949墙钟秒 / 125.8仿真秒**；原始采集轮询计时另为131.089096/127.7，不混用。均不含冷启动与自然语言解析。最高观测水平速度2.255013 m/s、角速度4.002423 rad/s，最大倾角0.056861°。轻微线速度超调仍存在，未借本次驱动替换提高巡航速度。

Windows目录中的 delivery-speed-route.png 为真实位姿按速度着色的全程轨迹，静态障碍使用世界几何。未将未经map→world校准的全局计划当作精确参考；原始规划、目标、预测状态已另存观测JSON，空场预期/实际图仍用于评估驱动响应。该轮为单次流程验证；与ODE100旧成绩相比还存在物理设置、启动相位差异，不据此宣称严格整轮提速百分比。

流程结束已经停栈，processes.json为空且没有gzserver。新增 tools/plot_delivery_motion.py 可从标准benchmark目录重画实际速度路径。
