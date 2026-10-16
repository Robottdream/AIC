# 车体施力驱动（Gazebo Classic）

此包在 codex/body-velocity-control 分支使用，替代 gazebo_ros_diff_drive 的驱动和里程计职责。方案参考 rmoss MecanumDrive2 的车体速度反馈与直接施力思路，是独立实现，未移植 Ignition 插件或其依赖。

## 控制方法

每个物理步读取 base_footprint 合并刚体的真实世界线速度和偏航角速度，将水平速度投影到车头坐标系。三个带条件积分抗饱和的 PI 分别跟踪前向目标、横向零速度、偏航目标。纵横向力按当前 yaw 转到世界水平面，以 AddForce 施加到车体质心；AddTorque 仅施加世界 Z 向转矩。没有 SetVelocity、SetWorldPose、轮子角速度强写、垂直推力或车身扶正。

保持两轮差速接口：只接受 /cmd_vel 的 linear.x/angular.z，不接受横移目标。横向反馈是虚拟侧向牵引，不能据此认为真实轮胎一定能提供同等力。接触、重力、车轮转动惯量、货物和碰撞仍由物理引擎计算；驱动力不再受轮胎接触牵引链同样的限制。轮子没有电机，由地面接触被动滚动，实测轮速不再是执行指令。

| 轴 | P | I | 输出硬限幅 |
|---|---:|---:|---:|
| 前向 | 300 | 150 | ±1000 N |
| 横向 | 1000 | 300 | ±1000 N |
| 偏航 | 200 | 100 | ±1000 Nm |

积分以输出单位存储、以物理 dt 更新；输出饱和时仅允许消除饱和的积分变化。增益与限幅在 xacro 的 body_force_controller 插件块显式配置，非对方车辆参数的直接复制。指令默认另有 ±6 m/s、±4 rad/s 硬限幅；AIC_BODY_MAX_LINEAR_SPEED可在xacro展开时覆盖前者，3/4/5档历史试验统一设为5，当前默认为6。上游平滑器仍负责正常加减速。0.5 仿真秒没有新指令则目标清零并施力制动，不瞬时改速度。仿真重置/时间倒退和非有限输入会清空目标；重置还清积分。

## 选择与构建

在 WSL bash 的仓库根目录使用：

```bash
bash ros_competition_start.sh build
bash ros_competition_start.sh start --headless  # 本分支默认 body_force
AIC_DRIVE_MODEL=wheel bash ros_competition_start.sh start --headless
```

必须先停止已有栈，再选择另一驱动。两个插件在 xacro 中互斥，非法 AIC_DRIVE_MODEL 会使展开失败。力驱动将两个轮子 ODE motor fmax 设为 0，避免叠加。切换不修改质量、摩擦、ODE 或导航配置。

## 反馈与可视化

- /odom 与 odom→base_footprint TF：物理车体真实位姿与局部速度，默认 30 Hz。保持原项目世界坐标里程计约定，不能当作真实编码器积分。
- /joint_states：实际左右轮位置/转速；robot_state_publisher 生成轮子 TF。机械臂原有关节状态发布保留。
- /body_drive/wrench：WrenchStamped，frame_id=world，实际注入的世界水平力和偏航转矩，默认 30 Hz。它不包含接触反力，也不是整车合力。

完整小地图对照命令见 ../../tools/physics_benchmark/README.md。当前默认任务为线速上限6 m/s、角速上限4 rad/s、线与角加/减速均10；全1000限额单轮五件区域与高度均合格，120.75墙钟/115.3仿真秒，无碰撞/失稳中止；见../../docs/speed6-body-force-20261016.md。此前8/4与20/5、20/10档记录见../../docs/acc20-force1000-flow-20261015.md。此前20/40角速试验属于历史记录，见../../docs/angular20-acc40-flow-20261015.md；施力限额保持，实际加速度仍由物理反馈决定。


2026-10-15用户试验：横向限力200→1000 N、偏航限矩100→1000 Nm，前向限力150 N及所有PI增益保持。线速8 m/s、线加/减3.5/5、角速4 rad/s、角加减10/10。本档单轮放置2件，第三件返程查询两次no_path中止，111.61墙钟/109.9仿真秒；最大横向速度0.0753m/s，采集横向/偏航峰值87.75N/69.80Nm，未触旧限额，不能证明提高限额的收益。详见../../docs/force1000-flow-20261015.md。插件CPP内省略参数时的fallback不变，实际启动由xacro显式1000/1000覆盖。

2026-10-16：用户追加要求将前向限力也提高到1000 N，横向1000 N和偏航1000 N·m保持，三轴PI增益不变。线速6/角速4、线与角加减速均10。此前前向150 N的本日尝试由用户中途终止，不能与新限额测试混用。
