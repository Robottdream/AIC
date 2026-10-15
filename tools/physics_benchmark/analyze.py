"""Recompute native-wheel/body metrics and figures without launching the simulator."""
from pathlib import Path
import json, math, hashlib, csv
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
out = args.output.resolve()
protocol = json.loads((out / 'protocol.json').read_text())
order = protocol['order']
speed = protocol['target_v']
omega = protocol['target_w']
hold = protocol['hold_sim_seconds']
brake = protocol['brake_observation_seconds']
font = Path('/mnt/c/Windows/Fonts/msyh.ttc')
if font.exists():
    font_manager.fontManager.addfont(str(font))
    plt.rcParams['font.family'] = ['Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
rows = []
arcs = []
hashes = []
worlds = []
nav_hashes = []
task_hashes = []
for (i, limit) in enumerate(order, 1):
    folder = out / f'boot-{i}-ode-{limit}'
    tree = ET.parse(folder / 'field.world')
    world = tree.find('world')
    parameter = world.find('physics/ode/constraints/contact_max_correcting_vel')
    assert float(parameter.text) == limit
    parameter.text = 'NORMALIZED'
    world.find("plugin[@name='speed_audit_native']/output_file").text = 'NORMALIZED'
    worlds.append(ET.tostring(tree.getroot()))
    nav_hashes.append(hashlib.sha256((folder / 'nav2.yaml').read_bytes()).hexdigest())
    task_hashes.append(hashlib.sha256((folder / 'tasks.yaml').read_bytes()).hexdigest())
    records = json.loads((folder / 'trials.json').read_text())
    assert len(records) == 2
    hashes.append(hashlib.sha256((folder / 'live-robot.urdf').read_bytes()).hexdigest())
    runtime = json.loads((folder / 'runtime-parameters.json').read_text())
    assert runtime['/velocity_smoother']['max_accel']['double_array'] == [3.5, 0.0, 10.0]
    assert runtime['/velocity_smoother']['max_decel']['double_array'] == [-5.0, 0.0, -10.0]
    raw = []
    bad = []
    lines = (folder / 'native.jsonl').read_text().splitlines()
    finished = max((r['stop_request_sim'] + brake for r in records))
    for (k, line) in enumerate(lines):
        try:
            raw.append(json.loads(line))
        except json.JSONDecodeError:
            import re
            m = re.search('"sim":([0-9.]+)', line)
            assert k == len(lines) - 1 and m and (float(m[1]) > finished)
            bad.append({'line': k + 1, 'sim': float(m[1]), 'reason': 'incomplete final shutdown line outside trials'})
    assert raw[0]['actual_contact_max_correcting_vel'] == limit
    motion = [r for r in raw if r['kind'] == 'motion']
    assert all((b['sim'] > a['sim'] for (a, b) in zip(motion, motion[1:])))
    (folder / 'data-quality.json').write_text(json.dumps(dict(discarded_tail_lines=bad, raw_preserved=True, trial_end_sim=finished), indent=2))
    for r in records:
        trace = json.loads((folder / r['file']).read_text())
        start = r['start_sim']
        stop = r['stop_request_sim']
        seq = [s for s in motion if start <= s['sim'] <= stop + brake - 0.05]
        late = [s for s in seq if r['window_start'] <= s['sim'] <= r['window_end']]
        assert len(late) > 20 and all((all((isinstance(v, (int, float)) and math.isfinite(v) for v in s.values() if not isinstance(v, str))) for s in seq))
        assert max(np.diff([s['sim'] for s in seq])) < 0.025
        commands = trace['commands']
        first = next((c['sim'] - start for c in commands if c['sim'] >= start and abs(c['v']) > 0.01))
        reached = next((c['sim'] - start for c in commands if c['sim'] >= start and abs(c['v'] - speed) < 0.05 and (abs(c['w'] - r['target_w']) < 0.05)))
        assert first <= 0.25 and reached <= max(speed / 3.5, abs(omega) / 10) + 0.25
        cmdlate = [c for c in commands if r['window_start'] <= c['sim'] <= r['window_end']]
        assert len(cmdlate) >= 5 and all((abs(c['v'] - speed) < 0.05 and abs(c['w'] - r['target_w']) < 0.05 for c in cmdlate))
        avg = lambda key: float(np.mean([s[key] for s in late]))
        active = [s for s in seq if s['sim'] <= stop]
        row = dict(boot=i, ode=limit, case=r['name'], late_forward_mps=avg('v'), late_angular_radps=avg('w'), late_left_radps=avg('left_joint_w'), late_right_radps=avg('right_joint_w'), late_wheel_ideal_v=0.065 * (avg('left_joint_w') + avg('right_joint_w')) / 2, late_wheel_ideal_w=0.065 * (avg('right_joint_w') - avg('left_joint_w')) / 0.53, peak_active_angular=max((abs(s['w']) for s in active)), peak_full_angular=max((abs(s['w']) for s in seq)), max_tilt_deg=max((s['tilt_deg'] for s in seq)), max_relative_height_mm=max((abs(s['z'] - seq[0]['z']) for s in seq)) * 1000, first_command_s=first, target_command_s=reached, input_valid=True)
        t = np.array([s['sim'] for s in seq])
        yaw = np.unwrap([s['yaw'] for s in seq])
        dyaw = np.gradient(yaw, t)
        row['yaw_rate_vs_pose_derivative_mean_abs_error'] = float(np.mean(np.abs(dyaw[1:-1] - np.array([s['w'] for s in seq])[1:-1])))
        rows.append(row)
        if r['name'] == 'arc-w2':
            arcs.append((row, seq, trace))
assert len(set(hashes)) == 1
assert len(set(worlds)) == 1
assert len(set(nav_hashes)) == len(set(task_hashes)) == 1
(out / 'verification.json').write_text(json.dumps(dict(all_robot_descriptions_identical=True, robot_sha256=hashes[0], valid_trials=len(rows), cold_starts=len(order), only_physics_parameter_varied=True), indent=2))
(out / 'individual-metrics.json').write_text(json.dumps(rows, indent=2))
with (out / 'individual-metrics.csv').open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=rows[0])
    w.writeheader()
    w.writerows(rows)
summary = []
for limit in dict.fromkeys(order):
    for case in ['straight', 'arc-w2']:
        subset = [r for r in rows if r['ode'] == limit and r['case'] == case]
        result = dict(ode=limit, case=case, n=len(subset))
        for key in ['late_forward_mps', 'late_angular_radps', 'late_left_radps', 'late_right_radps', 'max_tilt_deg', 'peak_full_angular']:
            vals = [r[key] for r in subset]
            result[key] = dict(mean=float(np.mean(vals)), min=min(vals), max=max(vals))
        summary.append(result)
(out / 'metrics.json').write_text(json.dumps(summary, indent=2))
(fig, axes) = plt.subplots(3, 2, figsize=(14, 11))
limits = list(dict.fromkeys(order))
colors = {v: plt.get_cmap('tab10')(i % 10) for (i, v) in enumerate(limits)}
seen = {}
plots = [('v', '车身前向速度 / m/s', speed), ('w', '车身角速度 / rad/s', omega), ('left_joint_w', '左轮原生转速 / rad/s', (speed - 0.53 * omega / 2) / 0.065), ('right_joint_w', '右轮原生转速 / rad/s', (speed + 0.53 * omega / 2) / 0.065), ('tilt_deg', '最大横滚/俯仰 / °', None), ('height', '参考连杆高度变化 / mm', None)]
for (row, seq, trace) in arcs:
    t = np.array([s['sim'] - trace['start_sim'] for s in seq])
    seen[row['ode']] = seen.get(row['ode'], 0) + 1
    style = ['-', '--', ':', '-.'][(seen[row['ode']] - 1) % 4]
    label = f"ODE={row['ode']}，冷启动{row['boot']}"
    for (ax, (key, title, target)) in zip(axes.flat, plots):
        vals = [(s['z'] - seq[0]['z']) * 1000 if key == 'height' else s[key] for s in seq]
        ax.plot(t, vals, color=colors[row['ode']], ls=style, label=label, lw=1.5)
for (ax, (key, title, target)) in zip(axes.flat, plots):
    ax.set_ylabel(title)
    ax.set_xlabel('相对动作开始 / 仿真秒')
    ax.grid(alpha=0.2)
    ax.axvline(hold, color='#888888', ls=':', label='请求停车')
    if target is not None:
        ax.hlines(target, max(speed / 3.5, abs(omega) / 10), hold, color='#666666', ls=':', label='目标（轮速为运动学换算）')
axes[0, 0].legend(fontsize=8)
fig.suptitle(f'ODE圆弧对照：目标{speed:g} m/s、{omega:g} rad/s', fontsize=17)
fig.text(0.5, 0.015, f'顺序：{order}；每次直行和圆弧各一次。原生100 Hz。竖线为请求停车。', ha='center', fontsize=10)
fig.tight_layout(rect=(0, 0.035, 1, 0.96))
fig.savefig(out / 'response-comparison.png', dpi=170)
plt.close(fig)

def integrate(seq, trace):
    commands = trace['commands']
    ct = np.array([c['sim'] for c in commands])
    (x, y, yaw) = (seq[0]['x'], seq[0]['y'], seq[0]['yaw'])
    prev = seq[0]['sim']
    points = [[x, y]]
    for s in seq[1:]:
        bounds = [prev] + [v for v in ct if prev < v < s['sim']] + [s['sim']]
        for (a, b) in zip(bounds, bounds[1:]):
            j = np.searchsorted(ct, a, side='right') - 1
            c = commands[j] if j >= 0 else {'v': 0, 'w': 0}
            dt = b - a
            (v, w) = (c['v'], c['w'])
            if abs(w) < 1e-08:
                x += v * math.cos(yaw) * dt
                y += v * math.sin(yaw) * dt
            else:
                x += v / w * (math.sin(yaw + w * dt) - math.sin(yaw))
                y += v / w * (math.cos(yaw) - math.cos(yaw + w * dt))
            yaw += w * dt
        points.append([x, y])
        prev = s['sim']
    return np.array(points)
(fig, axes) = plt.subplots(1, len(limits), figsize=(6.5 * len(limits), 7), squeeze=False)
axes = axes.ravel()
norm = Normalize(0, speed, clip=True)
allp = []
seen = {}
for (row, seq, trace) in arcs:
    ax = axes[limits.index(row['ode'])]
    seen[row['ode']] = seen.get(row['ode'], 0) + 1
    xy = np.array([[s['x'], s['y']] for s in seq])
    ref = integrate(seq, trace)
    allp.extend(xy)
    allp.extend(ref)
    ax.plot(ref[:, 0], ref[:, 1], color='#666666', ls='--', alpha=0.6, lw=1.2, label='预期：平滑后指令积分' if seen[row['ode']] == 1 else None)
    line = LineCollection(np.stack([xy[:-1], xy[1:]], axis=1), cmap='turbo', norm=norm, linewidth=2, alpha=0.8)
    line.set_array(np.array([math.hypot(s['vx'], s['vy']) for s in seq[:-1]]))
    ax.add_collection(line)
    ax.scatter(*xy[0], c='black', s=20)
    ax.scatter(*xy[-1], c='black', marker='x', s=35)
    ax.annotate(f"冷启动{row['boot']}终点", xy[-1], xytext=(7, 17 if seen[row['ode']] % 2 else -19), textcoords='offset points', fontsize=8)
lo = np.min(allp, axis=0) - 0.3
hi = np.max(allp, axis=0) + 0.3
for (ax, limit) in zip(axes, limits):
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal')
    ax.grid(alpha=0.2)
    ax.set_title(f'ODE={limit}（{order.count(limit)}次独立冷启动）')
    ax.set_xlabel('世界X / m')
    ax.set_ylabel('世界Y / m')
    ax.legend(fontsize=8)
fig.subplots_adjust(left=0.065, right=0.88, top=0.85, bottom=0.14, wspace=0.23)
bar = fig.add_axes([0.92, 0.23, 0.015, 0.5])
fig.colorbar(line, cax=bar, label='实际水平速度 / m/s')
fig.suptitle(f'{speed:g} m/s圆弧：实际轨迹（速度着色）与预期轨迹', fontsize=17)
fig.text(0.5, 0.04, '同坐标、同色标，含停车阶段。虚线为底盘指令积分，受ROS接收时间误差影响，不是Nav2规划路径。', ha='center', fontsize=10)
fig.savefig(out / 'trajectories.png', dpi=180)
plt.close(fig)
print(json.dumps(rows, indent=2))

report = ["# 原生轮速与车身运动对照", "", f"目标线速度 {speed:g} m/s，圆弧角速度 {omega:g} rad/s；冷启动顺序 {order}。", "",
          "每次冷启动各执行一次直行和圆弧，指令经过活动的 velocity_smoother。此处没有执行 Nav2 路径跟随或五件搬运。",
          "后段均值窗口为请求停车前 0.5 到 0.1 仿真秒；异常运动仍在变化，不能当作稳态均值。峰值包含停车阶段。", "",
          "| ODE | 工况 | 次数 | 后段前向速度均值 m/s | 后段角速度均值 rad/s | 最大角速度 rad/s | 最大倾角 ° |",
          "|---|---|---|---|---|---|---|"]
for item in summary:
    subset = [r for r in rows if r['ode'] == item['ode'] and r['case'] == item['case']]
    v, w = item['late_forward_mps'], item['late_angular_radps']
    report.append(f"| {item['ode']} | {item['case']} | {item['n']} | {v['mean']:.6f} ({v['min']:.6f}–{v['max']:.6f}) | {w['mean']:.6f} ({w['min']:.6f}–{w['max']:.6f}) | {max(r['peak_full_angular'] for r in subset):.6f} | {max(r['max_tilt_deg'] for r in subset):.6f} |")
report += ["", "原生 GetVelocity 轮速、车身世界位姿/速度以约100 Hz记录；左右轮运动学换算使用半径0.065 m、轮距0.53 m。换算成立依赖无滑移。",
           "", "[时间响应图](response-comparison.png) · [轨迹图](trajectories.png) · [逐项指标](individual-metrics.csv) · [核验记录](verification.json)",
           "", "轨迹虚线为收到的底盘指令运动学积分，受ROS接收时间误差影响，不是Nav2规划路径。颜色为实际水平速度，表格为车体前向速度。",
           "", "原始native.jsonl保留；仅允许剔除所有试验结束之后的单行不完整关闭尾部，记录于各目录data-quality.json。输入有效不等同于运动跟踪通过。", ""]
(out / 'REPORT.md').write_text(chr(10).join(report))
