"""Write reproducible A/B summaries and the Chinese integration report."""
import json
import math
import statistics
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
EVIDENCE=Path.home()/'aic_mecanum_evidence_20261002'
OUT=ROOT/'docs/mecanum-evidence-20261002'
OUT.mkdir(parents=True,exist_ok=True)

def read(name):
    p=EVIDENCE/name
    return json.loads(p.read_text()) if p.exists() else []

baseline=read('baseline2/results.json')
new=read('new-v3-tasks/results.json')
routes={}
for label,name in [('旧车型','baseline-routes'),('新车型 V3','new-v3-routes')]:
    rows=read(name+'/results.json')
    routes[label]={'completed':sum(r['status']==4 for r in rows),'attempted':len(rows),'kinds':{}}
    for kind in ['lateral','return_lateral','forward','return_forward']:
        # Exclude warmup cycle in both models. Baseline first route's /clock
        # was not initialized when captured, so its sim_seconds is unusable.
        group=[r for r in rows if r['kind']==kind and r['cycle']>=2 and r['status']==4]
        if group:
            routes[label]['kinds'][kind]={'n':len(group),'wall_mean_s':statistics.mean(r['seconds'] for r in group),'sim_mean_s':statistics.mean(r['sim_seconds'] for r in group)}
task_rows=[]
for old in baseline:
    matching=[r for r in new if r['task']==old['task']]
    current=matching[0] if matching else None
    task_rows.append({'command':old['command'],'old':old,'new':current,
                      'wall_time_reduction_percent':100*(1-current['seconds']/old['seconds']) if current and current['status']=='success' else None})
tracepath=EVIDENCE/'new-v3-routes/trace.json'
trace=json.loads(tracepath.read_text()) if tracepath.exists() else []
summary={'tasks':task_rows,'routes_excluding_warmup':routes,
         'new_route_peak_command_norm_mps':max((math.hypot(*r['cmd_vel'][:2]) for r in trace if 'cmd_vel' in r),default=0.),
         'model':'Gazebo ideal planar velocity model, not roller traction physics',
         'baseline_first_attempt':read('baseline/results.json'),
         'new_v2_tasks':read('new-tasks/results.json')}
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
for filename,data in [('baseline-tasks.json',baseline),('new-v3-tasks.json',new),('baseline-routes.json',read('baseline-routes/results.json')),('new-v3-routes.json',read('new-v3-routes/results.json')),('live.json',read('new-live-final.json') or read('new-live.json')),('assets.json',read('new-assets.json')),('bridge.json',read('new-bridge.json'))]:
    (OUT/filename).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')

lines=['# 四麦轮、回转座与左右双雷达接入验证（2026-10-02）','',
'## 实现与边界','',
'Blender 新模型已导出 8 个 link 的 30 份网格，并接入可选 ROS/Gazebo 车型。实际 URDF 为 24 link，Foxglove CBOR-raw 订阅及全部 HTTP 网格读取通过。左右雷达分别作为 Nav2 和 Collision Monitor 的观测源，按各自真实传感器位置清障；合成 `/scan` 仅用于兼容接口，不用于代价地图射线清障。自体命中为 NaN，不伪造自由空间。任一雷达失联时合成心跳停止，最终速度保护停车。','',
'底部增加 `arm_yaw_joint`、独立轨迹控制器及 MoveIt 配置，+0.35 rad 与回零实测通过。抓放主控仍保持底座回转零角，没有改变出题顺序或开发侧向抓取策略。','',
'底盘使用已安装的 Gazebo planar_move 理想速度驱动，四轮速度按 X 型逆运动学显示；碰撞支撑用简化圆柱。此模型直接设置平面速度，没有模拟滚子接触、扭矩、打滑和负载牵引，测量只能说明此仿真配置的表现，不能推断真实麦轮硬件提速。实现参考 [Gazebo 官方插件源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_planar_move.cpp)。','',
'新旧直线合速度上限均为 0.70 m/s，角速度 1.50 rad/s；线加速度 0.35 m/s²、角加速度 2.4 rad/s²。新车型四方向可运动，最终保护按向量模长限制斜向速度。轮毂实际宽 0.569 m，新包络为 0.61×0.61 m；旧包络为 0.58×0.552 m。新包络更宽，窄道表现不能只按驱动方式推断。','',
'新版使用 DWB，旧版使用 RPP；因此对照对象是“车型＋驱动＋导航配置”的组合，不能单独归因于外观、雷达或轮型。','',
'## 固定路线','',
'同一场景、同一地图目标、同一最终到点标准（0.15 m、0.25 rad），从出生区执行侧向 1 m、返回、前向 1 m、返回，三轮。统计排除首轮预热；旧首轮第一个仿真计时因 /clock 未初始化无效。新控制器内部旋转窗口为 0.10 m，外部到点容差保持 0.15 m，以减少边缘停车。','',
'| 路线 | 旧墙钟均值 s | 新墙钟均值 s | 旧仿真均值 s | 新仿真均值 s |','|---|---:|---:|---:|---:|']
labels={'lateral':'侧向到点','return_lateral':'侧向返回','forward':'向前到点','return_forward':'返回原点'}
for kind in labels:
    old=routes['旧车型']['kinds'].get(kind); current=routes['新车型 V3']['kinds'].get(kind)
    def cell(v,key): return f'{v[key]:.2f}' if v else '未完成'
    lines.append(f'| {labels[kind]} | {cell(old,"wall_mean_s")} | {cell(current,"wall_mean_s")} | {cell(old,"sim_mean_s")} | {cell(current,"sim_mean_s")} |')
lines += ['',f'旧路线通过 {routes["旧车型"]["completed"]}/{routes["旧车型"]["attempted"]}；新 V3 通过 {routes["新车型 V3"]["completed"]}/{routes["新车型 V3"]["attempted"]}。新路线实测最终命令峰值合速度 {summary["new_route_peak_command_norm_mps"]:.4f} m/s。','',
'## 完整任务','',
'同一命令顺序：蓝 B、红 A、蓝 A。在路线测试后执行，保留动态障碍物；动态相遇时刻与自动选块方向未锁定，因此单轮任务耗时不能视为统计稳定的车型因果差异。','',
'| 命令 | 旧结果／墙钟 s | 新 V3 结果／墙钟 s | 旧／新仿真 s |','|---|---|---|---|']
for row in task_rows:
    old=row['old']; current=row['new']
    newcell=f'{current["status"]} / {current["seconds"]:.2f}' if current else '未完成'
    sim=f'{old["sim_seconds"]:.2f} / {current["sim_seconds"]:.2f}' if current else f'{old["sim_seconds"]:.2f} / —'
    lines.append(f'| {row["command"]} | {old["status"]} / {old["seconds"]:.2f} | {newcell} | {sim} |')
lines += ['',
'必须同时保留失败和调整过程：旧车型首轮蓝 B 300.00 s 超时（未放置，15 次恢复）；第二轮三件成功。新版初始参数出现无必要转向及计算超时；V2 蓝 B 71.35 s 成功、红 A 300.01 s 超时，固定路线直行在到点容差边缘停住。V3 加入 Twirling 抑制无必要旋转、降低轨迹采样、收紧内部旋转窗口，并在统一合速度限制下开放四方向速度。不能忽略这些失败，只挑选成功样本。','',
'V3 第二件若标为 `stalled_test_stopped`，表示在持续停车、反复恢复后由测试人员中止，不是宣称已等到 300 秒超时；第三件未执行。','',
'## 运行与证据','',
'修改工作区：`/mnt/c/Users/21920/.codex/worktrees/5a40/AIC`。新运行副本：`/home/polarbear/ws_aic_mecanum_20261002`。旧 `/home/polarbear/ws_aic` 的源代码和未提交改动未被覆盖，两者不能同时启动。默认原启动入口仍采用旧车型，新车型通过 `ros_mecanum_start.sh` 显式启用。','',
'```bash','bash /home/polarbear/ws_aic/ros_competition_start.sh stop','cd /home/polarbear/ws_aic_mecanum_20261002','bash ros_mecanum_start.sh start --headless','bash ros_mecanum_start.sh stop','```','',
'完整原始轨迹、启动及构建日志保存在 `/home/polarbear/aic_mecanum_evidence_20261002`；可审阅的小型结果快照位于 `docs/mecanum-evidence-20261002/`。','',
'构建 13 包、URDF/SDF 展开、新旧模型兼容展开、Python 编译通过；轮速符号、端点投影、未知/自体命中、单雷达失联及横移预测测试通过；既有动态追踪 72 个静态视角及迎面/横穿/远离/丢失回归通过。地图约 3.94 MB、图像约 2.76 MB 已由真实 WebSocket 收到，尚未测量 Foxglove 界面 FPS。','',
'当前结论：局部路线效率和完整任务表现必须分别判断。此轮不足以证明新车型稳定提高比赛整体速度，保留原默认配置与新版可选测试入口。测试后新版已停止且无残留，旧运行副本已恢复并通过启动就绪体检，当前主控空闲；结论见 `restored-old-start.log`。','']
if task_rows and task_rows[0]['new'] and task_rows[0]['new']['status']=='success':
    r=task_rows[0]; increase=100*(r['new']['seconds']/r['old']['seconds']-1)
    lines += [f'最终蓝 B：旧 {r["old"]["seconds"]:.2f} s，新 {r["new"]["seconds"]:.2f} s，墙钟耗时增加 {increase:.1f}%；旧仿真 {r["old"]["sim_seconds"]:.2f} s、新仿真 {r["new"]["sim_seconds"]:.2f} s，不能用单纯实时因子差异解释全部变慢。','']
posepath=EVIDENCE/'placed-blue-cube-pose.txt'
if posepath.exists():
    pose=[float(v) for v in posepath.read_text().split()]
    inside=abs(pose[0]+1.196544)+.015<.4 and abs(pose[1]+6.485499)+.015<.4
    (OUT/'placed-blue-cube.json').write_text(json.dumps({'pose':pose,'inside_B_xy':inside},indent=2)+'\n')
    lines += [f'放置后的 `blue_cube_5` Gazebo 实际坐标为 ({pose[0]:.5f}, {pose[1]:.5f}, {pose[2]:.5f}) m，3 cm 物块的 XY 投影位于 B 区 0.8×0.8 m 边界内：{inside}。','']
(ROOT/'docs/mecanum-integration-20261002.md').write_text('\n'.join(lines),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
