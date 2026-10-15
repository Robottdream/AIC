# pb2025配套仿真底盘核对（2026-10-15）

只读核对，没有运行参考项目，也未替换AIC底盘。导航仓库main读取时提交12c4da5318e114044c9d897dd7044b33470f14ff；其配套rmu_gazebo_simulator读取时main为b42d7832500e5a9ad31a09c2edc7bc4fe5b5ce03。导航仓库没有固定该配套仓库的具体提交，因此这是当前源码链路核查，不能保证任意历史演示视频使用相同构建。

配套依赖清单固定rmoss_gazebo为7443ff06c345752d0e23172c479d99b27207e2c7，pb2025_robot_description为a0541dddcbfe376f369a6345b532a10923ad9149，模型资源为b5c759f08844dfda19c79aa870866ace8d4c7b3a。路径跟踪子模块为b542c5794b91fba2bb3fe2cbdc72fcf854fedb1a。

## 已确认的关键差异

1. simulation_robot.sdf.xmacro明确启用MecanumDrive2；其PreUpdate读取车体线/角速度，分别计算x/y/偏航PID，再通过AddWorldWrench直接向chassis施力、施力矩。它没有在该控制循环中把目标换成轮关节驱动转矩来生成牵引；根据逐轮接地状态计算作用力仍是TODO。它不是写死位置或瞬移，仍有刚体动力学和碰撞，但驱动模型是较简化的车体层作用力控制。
2. 源码中x/y/偏航P分别100/500/200，输出限幅分别±100 N、±200 N、±100 N·m。这些是直接车体作用力/力矩控制的限幅，不能与AIC单轮1000 N·m转矩上限直接比大小。
3. 其全向路径控制器输出vx、vy和角速度，默认配置enable_rotation=false；fake_vel_transform根据里程计yaw旋转平移指令，再叠加init_spin_speed=3.14 rad/s。因此平移方向与自旋分开处理。平滑器在这个坐标变换/自旋叠加之前，不能直接把其角速度上限3.0等同于最终自旋上限。
4. 当前配置v_linear_max=2.5 m/s，曲率高时保留0.5倍减速和终点减速；不能仅凭视频视觉判断实速比AIC3 m/s更高。
5. 被引用的四轮模型为简化圆柱碰撞，SDF中mu/mu2均写0.2；这是模型输入，不是本次运行实测摩擦。配套环境为Ignition Fortress，而AIC是Gazebo Classic11/ODE。

这解释配套仿真中较容易维持自旋和平移，并不能证明真实车没有侧滑，也没有定位AIC的唯一物理根因。AIC目前轮速闭环到位，但车体真实速度未跟随；最值得借鉴的是增加真实车体速度反馈。在差速模型上不能直接照搬全向侧向驱动力；目标3m/s+2rad/s圆弧仍属于差速底盘可实现的运动，不应把AIC异常当成底盘形式的正常限制。

## 一手源码链接

- [模型明确启用MecanumDrive2](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description/blob/a0541dddcbfe376f369a6345b532a10923ad9149/resource/xmacro/simulation_robot.sdf.xmacro#L44)
- [车体PID、直接施力和接地模型TODO](https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gazebo/blob/7443ff06c345752d0e23172c479d99b27207e2c7/rmoss_gz_plugins/plugins/mecanum_drive2/MecanumDrive2.cc#L125)
- [自旋时的平移坐标变换](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/12c4da5318e114044c9d897dd7044b33470f14ff/fake_vel_transform/src/fake_vel_transform.cpp#L143)
- [导航速度、自旋和平滑器配置](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/12c4da5318e114044c9d897dd7044b33470f14ff/pb2025_nav_bringup/config/simulation/nav2_params.yaml)
- [轮子摩擦模型输入](https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gz_resources/blob/b5c759f08844dfda19c79aa870866ace8d4c7b3a/resource/models/rmua19_standard_robot/chassis_wheel.def.xmacro#L36)
- [配套依赖固定版本](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator/blob/b42d7832500e5a9ad31a09c2edc7bc4fe5b5ce03/dependencies.repos)