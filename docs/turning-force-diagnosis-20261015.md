# 转向配置恢复与车体受力诊断（2026-10-15）

## 当前配置

仅恢复用户列出的加减速和角速：线加速3.5 m/s²，线制动5 m/s²，角加减速10/10 rad/s²，角速上限4 rad/s。线速度上限保留8 m/s。车体纵向150 N、横向200 N、偏航100 Nm，PI增益、摩擦1、ODE200/1ms、规划几何与墙距减速关闭状态保持。

控制器、规划器、平滑器、任务crossing_acceleration、生成器和默认车体角速限幅已同步。独立目录重新生成场景，nav/tasks与正式配置一致；xacro展开确认8/4限幅及150/200/100力矩参数，且无旧轮子驱动同时运行。本次没有重新跑完整五件，仿真保持停止。旧20/40/15流程不能作为恢复后配置的成功验证。

## 参考项目源码对照

用户参考仓库pb2025_sentry_nav读取提交12c4da5318e114044c9d897dd7044b33470f14ff，配套rmoss插件固定7443ff06c345752d0e23172c479d99b27207e2c7。此次通过Git对象FETCH_HEAD和在线源文件复核，没有执行参考仿真或声称复现其表现。

| 项 | AIC当前车体施力 | 参考MecanumDrive2源码 |
|---|---|---|
| 前向P/I/限力 | 300 / 150 / 150 N | 100 / 0 / 100 N |
| 横向P/I/限力 | 1000 / 300 / 200 N | 500 / 0 / 200 N |
| 偏航P/I/限矩 | 200 / 100 / 100 Nm | 200 / 0 / 100 Nm |
| 驱动 | 每物理步真实车体速度反馈，水平AddForce与偏航AddTorque | 每步真实速度反馈PID，AddWorldWrench |
| 平移接口 | 仅前向vx，横向目标0，转向改变行进方向 | 全向vx/vy，坐标转换允许平移与自旋分开 |
| 导航线速上限 | 8 m/s | 配置v_linear_max=2.5 m/s |
| 高曲率策略 | 保留现有RPP曲率逻辑，墙距减速关闭 | 配置高曲率速度比例0.5 |
| 仿真 | Gazebo Classic / ODE | 配套Ignition Fortress |

参考参数没有更大的横向限力或偏航限矩；AIC已采用相同的车体直接施力原理。不同质量、接触、控制接口、行驶速度和场景不允许直接用同样的200 N证明两车表现相同。参考的全向自旋也不等价于差速接口下以固定前向速度持续绕弯。

## 对上一轮日志的重新分析

数据来源：/home/polarbear/.local/share/aic-runs/20261015-angular20-acc40-flow/trial-1，是20 rad/s、角加速40、角减速15、线速上限8的旧运行，非本次恢复后的新运行。

原生100Hz记录包含世界vx/vy和yaw，用vy_body=-sin(yaw)*vx+cos(yaw)*vy求横向速度。施力遥测为world水平分量，按原生yaw插值转回车头坐标；30Hz遥测和插值可能漏掉短时峰值或饱和，不当作逐物理步精确统计。

机器人URDF各link质量合计14.5063 kg，不含当前吸附货物。在任务开始后约69.572仿真秒，前向3.782 m/s，角速-4.952 rad/s，横向3.565 m/s；注入横向力-199.26 N，偏航矩13.15 Nm。该片段实际角速基本跟随采集指令，横向力却长时间贴近限额，并积累很大的横向运动。图见Windows turning-force-diagnosis.png。

转弯样本定义|前向速度|>1 m/s且|角速|>0.5 rad/s。在753个施力样本中，横向达到99%限幅的比例约11.0%，偏航矩达到99%限幅约2.4%。这是样本占比，不能作为所有路径偏差的根因比例。完整原生横向速度峰值3.577 m/s。

## 解释与后续方向

偏航力矩改变车头朝向，横向合力改变运动方向。理想近似无侧滑圆弧要求a_y=v*omega、F_y=m*v*omega。以8 m/s和新上限4 rad/s同时成立估算，仅机器人质量就需约464.2 N横向合力，大于插件横向200 N；车轮接触还会贡献合力，因此不能据此断言实际绝对做不到，但它揭示速度/曲率与施力能力不匹配的风险。

当前横向PI只根据横向速度误差产生力，没有显式m*v*omega向心力前馈。预测运动模型则用前向速度与yaw积分平面位置，没有横向速度状态；存在明显侧滑时，碰撞预测和实际轨迹可能不一致。该问题不能靠单独提高角速度或偏航力矩保证解决。

证据优先支持检查横向限力与其前馈，而不是先把偏航矩从100 Nm盲目增大。可保持本次恢复的角速/加减速，单独对比横向限力与前馈，量化横向速度、速度指令跟随、饱和比例及路径误差。尚未进行这种对照，不能说增加横向力已经证明修复，也不能称为所有弯道的唯一原因。

## 复用分析命令

```bash
cd /home/polarbear/ws_aic
python3 tools/physics_benchmark/diagnose_turning.py --trial /home/polarbear/.local/share/aic-runs/20261015-angular20-acc40-flow/trial-1 --output /mnt/c/Users/21920/Desktop/AIC-turning-diagnosis-20261015
```

此脚本只读取既有日志，输出诊断图和JSON，不启动仿真、不修改参数。当前配置恢复前文件备份在/home/polarbear/.local/share/aic-runs/20261015-restore-turning-baseline/before，验证输出在同目录validation.json与generated-scene。

一手来源：
- [MecanumDrive2参数与施力循环](https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gazebo/blob/7443ff06c345752d0e23172c479d99b27207e2c7/rmoss_gz_plugins/plugins/mecanum_drive2/MecanumDrive2.cc)
- [参考导航配置](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/12c4da5318e114044c9d897dd7044b33470f14ff/pb2025_nav_bringup/config/simulation/nav2_params.yaml)
- [全向自旋坐标变换](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/12c4da5318e114044c9d897dd7044b33470f14ff/fake_vel_transform/src/fake_vel_transform.cpp)
