#!/usr/bin/env python3
"""Analyze numerical-physics suite records without starting Gazebo."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize

LABELS = {'wheel': '轮子电机驱动', 'body_force': '车体力闭环驱动', 'baseline': '基线 quick / 1 ms', 'step-half': 'quick / 0.5 ms',
          'step-quarter': 'quick / 0.25 ms', 'step-double': 'quick / 2 ms',
          'step-quadruple': 'quick / 4 ms', 'world': 'world / 1 ms',
          'finite': 'quick / 1 ms + 有限旋转'}


def load_native(folder, finished):
    lines = (folder / 'native.jsonl').read_text().splitlines()
    data, excluded = [], []
    for index, line in enumerate(lines):
        try:
            data.append(json.loads(line))
        except json.JSONDecodeError:
            import re
            match = re.search(r'"sim":([0-9.]+)', line)
            if index != len(lines) - 1 or not match or float(match[1]) <= finished:
                raise ValueError(f'Corrupt native data overlaps experiment: {folder}:{index + 1}')
            excluded.append(dict(line=index + 1, sim=float(match[1]), reason='Incomplete shutdown tail outside all trials'))
    (folder / 'data-quality.json').write_text(json.dumps(dict(excluded=excluded, raw_preserved=True, trial_end_sim=finished), indent=2))
    return data[0], [r for r in data if r['kind'] == 'motion']


def command_reference(seq, trace):
    commands = trace['commands']
    ct = np.array([c['sim'] for c in commands])
    x, y, yaw = (seq[0][k] for k in ['x', 'y', 'yaw'])
    points, angles = [[x, y]], [yaw]
    previous = seq[0]['sim']
    for sample in seq[1:]:
        cuts = [previous] + [t for t in ct if previous < t < sample['sim']] + [sample['sim']]
        for a, b in zip(cuts, cuts[1:]):
            index = np.searchsorted(ct, a, side='right') - 1
            cmd = commands[index] if index >= 0 else {'v': 0., 'w': 0.}
            v, w, dt = cmd['v'], cmd['w'], b - a
            if abs(w) < 1e-8:
                x += v * math.cos(yaw) * dt
                y += v * math.sin(yaw) * dt
            else:
                x += v / w * (math.sin(yaw + w * dt) - math.sin(yaw))
                y += v / w * (math.cos(yaw) - math.cos(yaw + w * dt))
            yaw += w * dt
        points.append([x, y])
        angles.append(yaw)
        previous = sample['sim']
    return np.array(points), np.array(angles)


def verify_world(folder, config):
    tree = ET.parse(folder / 'field.world')
    world = tree.find('world')
    paths = [('physics/ode/constraints/contact_max_correcting_vel', config['ode']),
             ('physics/ode/solver/type', config['solver']), ('physics/max_step_size', config['step'])]
    if 'update_rate' in config:
        paths.append(('physics/real_time_update_rate', config['update_rate']))
    for path, value in paths:
        assert world.findtext(path) == str(value), (folder, path)
        world.find(path).text = 'NORMALIZED'
    plugin = world.find("plugin[@name='speed_audit_native']")
    assert plugin.findtext('finite_rotation') == str(config['finite']).lower()
    plugin.find('finite_rotation').text = 'NORMALIZED'
    plugin.find('output_file').text = 'NORMALIZED'
    assert float(world.findtext("model[@name='ground_plane']/link/collision/surface/friction/ode/mu")) == 1
    assert float(world.findtext("model[@name='ground_plane']/link/collision/surface/friction/ode/mu2")) == 1
    return ET.tostring(tree.getroot())


def collect(out, protocol):
    rows, arcs, headers, worlds, robot_hashes, nav_hashes, task_hashes = [], [], [], [], [], [], []
    failures, failed_arcs = [], []
    normalized_robots, grouped_robot_hashes = [], {}
    for boot, variant in enumerate(protocol['order'], 1):
        folder = out / f'boot-{boot}-{variant}'
        config = protocol['profiles'][variant]
        records = json.loads((folder / 'trials.json').read_text()) if (folder / 'trials.json').exists() else []
        assert len(records) <= 2 and all(r['input_valid'] for r in records)
        failure = None
        if len(records) != 2 or (folder / 'error.json').exists() or (folder / 'runner-error.json').exists():
            details = json.loads((folder / 'error.json').read_text()) if (folder / 'error.json').exists() else json.loads((folder / 'runner-error.json').read_text())
            failure = dict(boot=boot, variant=variant, completed_trials=len(records), **details)
            failures.append(failure)
        if not (folder / 'native.jsonl').exists() or not (folder / 'native.jsonl').stat().st_size:
            assert failure is not None
            continue
        finished = max([r['stop_request_sim'] + protocol['brake_observation_seconds'] for r in records] + [failure.get('state', {}).get('sim', 0) if failure else 0])
        header, motion = load_native(folder, finished)
        assert header['actual_contact_max_correcting_vel'] == config['ode']
        assert header['solver'] == config['solver'] and abs(header['max_step_size'] - config['step']) < 1e-10
        assert header['left_finite_mode'] == header['right_finite_mode'] == int(config['finite'])
        if 'update_rate' in config:
            assert abs(header['real_time_update_rate'] - config['update_rate']) < 1e-6
        assert header['cfm'] == 0 and header['quick_iterations'] == 60
        headers.append(dict(boot=boot, variant=variant, **header))
        worlds.append(verify_world(folder, config))
        if not (folder / 'live-robot.urdf').exists():
            assert failure is not None and not records
            failure['phase'] = 'preflight_robot_description'
            continue
        robot_hashes.append(hashlib.sha256((folder / 'live-robot.urdf').read_bytes()).hexdigest())
        if protocol.get('suite') == 'drive':
            driver = config['drive']
            expected = 0 if driver == 'body_force' else protocol['expected_torque']
            assert header['left_motor_fmax'] == header['right_motor_fmax'] == expected
            check = json.loads((folder / 'drive-verification.json').read_text())
            assert check == dict(drive=driver, mutually_exclusive=True)
            grouped_robot_hashes.setdefault(driver, set()).add(robot_hashes[-1])
            robot = ET.parse(folder / 'live-robot.urdf').getroot()
            for gazebo in list(robot.findall('gazebo')):
                if any(plugin.attrib.get('name') in ('differential_drive_controller','body_force_controller') for plugin in gazebo.findall('plugin')):
                    robot.remove(gazebo)
            normalized_robots.append(ET.tostring(robot))
        nav_hashes.append(hashlib.sha256((folder / 'nav2.yaml').read_bytes()).hexdigest())
        task_hashes.append(hashlib.sha256((folder / 'tasks.yaml').read_bytes()).hexdigest())
        if not (folder / 'runtime-parameters.json').exists():
            assert failure is not None and not records
            failure['phase'] = 'preflight_parameter_query'
            continue
        runtime = json.loads((folder / 'runtime-parameters.json').read_text())
        smoother = runtime['/velocity_smoother']
        assert smoother['max_velocity']['double_array'] == [protocol['target_v'], 0., 4.]
        assert smoother['max_accel']['double_array'] == [3.5, 0., 10.]
        assert smoother['max_decel']['double_array'] == [-5., 0., -10.]
        assert all(b['sim'] > a['sim'] for a, b in zip(motion, motion[1:]))
        if failure:
            if (folder / 'failed-trial.json').exists():
                failed = json.loads((folder / 'failed-trial.json').read_text())
            else:
                # First interrupted setup predates failed-trial metadata. Its first
                # ROS sample bounds an estimated start; never use it for late means.
                history = json.loads((folder / 'last-motion.json').read_text())
                failed = dict(name='arc-w2' if records else 'straight', start_sim=history[0]['sim'], motion=history, commands=[], estimated_start=True)
            if failed.get('name') == 'arc-w2':
                until = failure.get('state', {}).get('sim', failed['motion'][-1]['sim'])
                partial = [r for r in motion if failed['start_sim'] <= r['sim'] <= until]
                assert partial
                failure['max_observed_tilt_deg'] = max(r['tilt_deg'] for r in partial)
                failure['max_observed_axis_error_deg'] = max(max(r['left_axis_error_deg'], r['right_axis_error_deg']) for r in partial)
                failed_arcs.append((failure, partial, failed, command_reference(partial, failed)[0]))

        for record in records:
            trace = json.loads((folder / record['file']).read_text())
            start, stop = record['start_sim'], record['stop_request_sim']
            seq = [s for s in motion if start <= s['sim'] <= stop + protocol['brake_observation_seconds'] - .05]
            assert len(seq) > 200 and max(np.diff([s['sim'] for s in seq])) < .025
            assert all(isinstance(value, (int, float)) and math.isfinite(value) for s in seq for key, value in s.items() if key != 'kind')
            active = [s for s in seq if s['sim'] <= stop]
            late = [s for s in seq if record['window_start'] <= s['sim'] <= record['window_end']]
            assert len(late) > 20
            commands = trace['commands']
            first = next(c['sim'] - start for c in commands if c['sim'] >= start and abs(c['v']) > .01)
            reached = next(c['sim'] - start for c in commands if c['sim'] >= start and abs(c['v'] - record['target_v']) < .05 and abs(c['w'] - record['target_w']) < .05)
            assert first <= .25 and reached <= max(record['target_v'] / 3.5, abs(record['target_w']) / 10) + .25
            cmdlate = [c for c in commands if record['window_start'] <= c['sim'] <= record['window_end']]
            assert len(cmdlate) >= 5 and all(abs(c['v'] - record['target_v']) < .05 and abs(c['w'] - record['target_w']) < .05 for c in cmdlate)
            avg = lambda key: float(np.mean([s[key] for s in late]))
            row = dict(boot=boot, variant=variant, case=record['name'],
                       observed_sim_wall_ratio=(trace['motion'][-1]['sim']-trace['motion'][0]['sim'])/(trace['motion'][-1]['wall']-trace['motion'][0]['wall']),
                       native_sample_interval_s=float(np.mean(np.diff([s['sim'] for s in seq]))),
                       late_forward_mps=avg('v'), late_angular_radps=avg('w'),
                       late_left_radps=avg('left_joint_w'), late_right_radps=avg('right_joint_w'),
                       late_wheel_ideal_v=.065 * (avg('left_joint_w') + avg('right_joint_w')) / 2,
                       late_wheel_ideal_w=.065 * (avg('right_joint_w') - avg('left_joint_w')) / .53,
                       peak_active_angular=max(abs(s['w']) for s in active),
                       peak_full_angular=max(abs(s['w']) for s in seq), max_tilt_deg=max(s['tilt_deg'] for s in seq),
                       body_response_within_tolerance=abs(avg('v')-record['target_v']) <= .15 and abs(avg('w')-record['target_w']) <= .15,
                       max_axis_error_deg=max(max(s['left_axis_error_deg'], s['right_axis_error_deg']) for s in seq),
                       first_command_s=first, target_command_s=reached)
            assert all(s['left_finite_mode'] == s['right_finite_mode'] == int(config['finite']) for s in seq)
            ref, yawref = command_reference(seq, trace)
            xy = np.array([[s['x'], s['y']] for s in seq])
            mask = np.array([s['sim'] <= stop for s in seq])
            row['active_command_reference_position_rmse_m'] = float(np.sqrt(np.mean(np.sum((xy[mask] - ref[mask]) ** 2, axis=1))))
            times = np.array([s['sim'] for s in seq])
            yaw = np.unwrap([s['yaw'] for s in seq])
            rate = np.gradient(yaw, times)
            row['yaw_rate_pose_derivative_mae'] = float(np.mean(np.abs(rate[1:-1] - np.array([s['w'] for s in seq])[1:-1])))
            rows.append(row)
            if record['name'] == 'arc-w2':
                arcs.append((row, seq, trace, ref))
    if not rows:
        (out / 'failures.json').write_text(json.dumps(failures, indent=2))
        (out / 'verification.json').write_text(json.dumps(dict(completed_input_valid_trials=0, failed_boots=len(failures), cold_starts=len(protocol['order']), motion_verified=False), indent=2))
        return rows, arcs, failures, failed_arcs
    assert len(set(worlds)) == len(set(nav_hashes)) == len(set(task_hashes)) == 1
    if protocol.get('suite') == 'drive':
        assert len(set(normalized_robots)) == 1 and all(len(v) == 1 for v in grouped_robot_hashes.values())
    else:
        assert len(set(robot_hashes)) == 1
    (out / 'native-runtime.json').write_text(json.dumps(headers, indent=2))
    (out / 'verification.json').write_text(json.dumps(dict(completed_input_valid_trials=len(rows), failed_boots=len(failures), cold_starts=len(protocol['order']), worlds_equivalent_outside_intended_parameters=True, robot_descriptions_identical=len(set(robot_hashes)) == 1, robot_equal_except_drive_plugin=protocol.get('suite') == 'drive', robot_sha256_by_drive={k:list(v)[0] for k,v in grouped_robot_hashes.items()}, robot_sha256=robot_hashes[0], navigation_and_tasks_identical=True), indent=2))
    (out / 'failures.json').write_text(json.dumps(failures, indent=2))
    return rows, arcs, failures, failed_arcs


def summaries(rows, variants):
    summary = []
    for variant in variants:
        for case in ['straight', 'arc-w2']:
            subset = [r for r in rows if r['variant'] == variant and r['case'] == case]
            if not subset:
                continue
            result = dict(variant=variant, case=case, n=len(subset))
            for key in ['late_forward_mps', 'late_angular_radps', 'peak_full_angular', 'max_tilt_deg', 'max_axis_error_deg', 'active_command_reference_position_rmse_m']:
                values = [r[key] for r in subset]
                result[key] = dict(mean=float(np.mean(values)), min=min(values), max=max(values))
            summary.append(result)
    return summary


def responses(out, arcs, protocol, variants, colors, failed_arcs, geometry=False):
    if geometry:
        fields = [('left_axis_error_deg', '左轮轴相对车身偏角 / °', None), ('right_axis_error_deg', '右轮轴相对车身偏角 / °', None), ('tilt_deg', '车身横滚/俯仰最大值 / °', None), ('lateral', '车身侧向速度 / m/s', 0)]
    else:
        v, w = protocol['target_v'], protocol['target_w']
        fields = [('v', '车身前向速度 / m/s', v), ('w', '车身角速度 / rad/s', w), ('left_joint_w', '左轮原生转速 / rad/s', (v - .53 * w / 2) / .065), ('right_joint_w', '右轮原生转速 / rad/s', (v + .53 * w / 2) / .065)]
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    grid = np.arange(.04, protocol['hold_sim_seconds'] + protocol['brake_observation_seconds'] - .08, .01)
    for variant in variants:
        trials = [(row, seq, trace) for row, seq, trace, ref in arcs if row['variant'] == variant]
        for ax, (key, ylabel, target) in zip(axes.flat, fields):
            values = []
            for row, seq, trace in trials:
                def value(s):
                    if key == 'axis': return max(s['left_axis_error_deg'], s['right_axis_error_deg'])
                    if key == 'lateral': return -math.sin(s['yaw']) * s['vx'] + math.cos(s['yaw']) * s['vy']
                    return s[key]
                values.append(np.interp(grid, [s['sim'] - trace['start_sim'] for s in seq], [value(s) for s in seq]))
            if not values:
                continue
            values = np.array(values)
            ax.plot(grid, values.mean(axis=0), color=colors[variant], label=LABELS[variant], lw=1.6)
            ax.fill_between(grid, values.min(axis=0), values.max(axis=0), color=colors[variant], alpha=.14)
    failed_seen = set()
    for failure, seq, trace, ref in failed_arcs:
        label = LABELS[failure['variant']]+' 中断' if failure['variant'] not in failed_seen else None
        failed_seen.add(failure['variant'])
        for ax, (key, ylabel, target) in zip(axes.flat, fields):
            if key == 'axis': vals = [max(r['left_axis_error_deg'], r['right_axis_error_deg']) for r in seq]
            elif key == 'lateral': vals = [-math.sin(r['yaw'])*r['vx']+math.cos(r['yaw'])*r['vy'] for r in seq]
            else: vals = [r[key] for r in seq]
            times = [r['sim']-trace['start_sim'] for r in seq]
            ax.plot(times, vals, color=colors[failure['variant']], ls=':', lw=1.2, alpha=.8, label=label)
            ax.scatter(times[-1], vals[-1], marker='x', c=[colors[failure['variant']]], s=25)
    for ax, (key, ylabel, target) in zip(axes.flat, fields):
        ax.axvline(protocol['hold_sim_seconds'], color='#777777', ls=':', lw=1)
        if target is not None: ax.hlines(target, 1., protocol['hold_sim_seconds'], color='#666666', ls=':', lw=1)
        ax.set_xlabel('相对动作开始 / 仿真秒'); ax.set_ylabel(ylabel); ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle('高速圆弧：轮轴约束、车身姿态与侧向运动' if geometry else f"高速圆弧物理对照：目标{protocol['target_v']:g} m/s、{protocol['target_w']:g} rad/s", fontsize=17)
    fig.text(.5, .015, ('实线/阴影：完整动作均值/范围。摩擦1、ODE200、1 ms；旧驱动轮矩1000 Nm，新驱动限力150/200 N、偏航矩100 Nm。竖线为请求停车。' if protocol.get('suite') == 'drive' else '实线/阴影：完整动作均值/范围；点线与叉号：中断记录。原生仿真采样。转矩上限1000 Nm、摩擦1、接触纠偏上限200保持；竖线为请求停车。'), ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .035, 1, .95))
    fig.savefig(out / ('geometry-comparison.png' if geometry else 'response-comparison.png'), dpi=170)
    plt.close(fig)


def trajectories(out, arcs, protocol, variants, failed_arcs):
    panel_count = len(variants) + (1 if len(variants) > 3 else 0)
    nrows = math.ceil(panel_count / 3)
    fig, axes = plt.subplots(nrows, 3, figsize=(15, 5 * nrows + 1), squeeze=False); axes = axes.ravel()
    combined = arcs + failed_arcs
    allpoints = np.vstack([np.array([[s['x'], s['y']] for s in seq]) for row, seq, trace, ref in combined] + [ref for row, seq, trace, ref in arcs])
    lower, upper = allpoints.min(axis=0) - .3, allpoints.max(axis=0) + .3
    norm = Normalize(0, protocol['target_v'], clip=True)
    for ax, variant in zip(axes, variants):
        trials = [a for a in combined if a[0]['variant'] == variant]
        for repeat, (row, seq, trace, ref) in enumerate(trials):
            xy = np.array([[s['x'], s['y']] for s in seq])
            if 'error' not in row:
                ax.plot(ref[:, 0], ref[:, 1], c='#777777', ls='--', alpha=.55, lw=1.2)
            if repeat == 0:
                line = LineCollection(np.stack([xy[:-1], xy[1:]], axis=1), cmap='turbo', norm=norm, linewidth=2.4)
                line.set_array(np.array([math.hypot(s['vx'], s['vy']) for s in seq[:-1]])); ax.add_collection(line)
            else:
                ax.plot(xy[:, 0], xy[:, 1], color='#7757a6', alpha=.7, lw=1.1, ls=':')
            ax.scatter(*xy[0], c='black', s=12); ax.scatter(*xy[-1], c='black', marker='x', s=25)
        if not trials:
            ax.text(.5, .5, '启动/复位中止\n无可绘制圆弧数据', transform=ax.transAxes, ha='center', va='center', fontsize=12, color='#9a3412')
        ax.set_title(LABELS[variant] + ('（有中断）' if any('error' in a[0] for a in trials) else '')); ax.set_xlim(lower[0], upper[0]); ax.set_ylim(lower[1], upper[1]); ax.set_aspect('equal'); ax.grid(alpha=.2); ax.set_xlabel('世界X / m'); ax.set_ylabel('世界Y / m')
    for ax in axes[len(variants):]: ax.axis('off')
    legend_axis = axes[-1] if len(axes) > len(variants) else None
    if legend_axis is not None: legend_axis.text(.05, .7, '彩色：第1次实际轨迹\n紫色点线：第2次实际轨迹\n灰色虚线：平滑后指令积分\n黑点：起点；叉号：记录结束\n\n各图同坐标、同色标\n不是Nav2规划路径', transform=axes[-1].transAxes, fontsize=12, va='top')
    if legend_axis is None: fig.text(.5, .025, '彩色：第1次实际；紫色点线：第2次实际；灰色虚线：指令积分；叉号：记录结束。各图同坐标与色标，不是Nav2规划路径。', ha='center', fontsize=9)
    fig.subplots_adjust(left=.055, right=.91, top=.90, bottom=.09, wspace=.28, hspace=.27)
    bar = fig.add_axes([.94, .25, .012, .5]); fig.colorbar(line, cax=bar, label='实际水平速度 / m/s')
    fig.suptitle(('驱动方案对照：实际与指令预期轨迹' if protocol.get('suite') == 'drive' else '物理求解对照：实际与指令预期轨迹'), fontsize=18)
    fig.savefig(out / 'trajectories.png', dpi=170); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output', type=Path, required=True); args = parser.parse_args(); out = args.output.resolve()
    protocol = json.loads((out / 'protocol.json').read_text()); variants = list(dict.fromkeys(protocol['order']))
    font = Path('/mnt/c/Windows/Fonts/msyh.ttc')
    if font.exists(): font_manager.fontManager.addfont(str(font)); plt.rcParams['font.family'] = ['Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    rows, arcs, failures, failed_arcs = collect(out, protocol)
    if not rows:
        (out / 'REPORT.md').write_text('# No valid motion trials\n\nAll boot attempts failed or stopped before a completed motion. Inspect failures.json and original launch logs. No plots or response-success claims were produced.\n')
        print('NO VALID MOTION TRIALS; failures preserved, no plots produced')
        return
    summary = summaries(rows, variants)
    (out / 'individual-metrics.json').write_text(json.dumps(rows, indent=2)); (out / 'metrics.json').write_text(json.dumps(summary, indent=2))
    with (out / 'individual-metrics.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    colors = {variant: plt.get_cmap('tab10')(i) for i, variant in enumerate(variants)}
    responses(out, arcs, protocol, variants, colors, failed_arcs); responses(out, arcs, protocol, variants, colors, failed_arcs, geometry=True); trajectories(out, arcs, protocol, variants, failed_arcs)
    lines = ['# 高速圆弧驱动对照' if protocol.get('suite') == 'drive' else '# 高速圆弧数值求解对照', '', f"顺序：{protocol['order']}；每次独立冷启动，直行和圆弧各一次。目标{protocol['target_v']} m/s，圆弧{protocol['target_w']} rad/s。", '',
             ('摩擦1、转矩上限1000 Nm、contact_max_correcting_vel=200、CFM=0保持。物理更新频率按1/步长同步调整，名义实时率保持1，以免墙钟定时器改变每仿真秒的控制频率；这是时间节奏调整，不是额外改变物理积分。' if protocol.get('paced_realtime') else '摩擦1、转矩上限1000 Nm、contact_max_correcting_vel=200、CFM=0保持。物理更新频率1000 Hz配置保持；步长会改变实时率与墙钟控制器的仿真时间行为，不宜当作纯步长因果对照。'), '',
             '| 配置 | 次数 | 圆弧后段前向速度 m/s | 圆弧后段角速度 rad/s | 含停车最大角速度 rad/s | 最大轮轴偏角 ° |', '|---|---|---|---|---|---|']
    for item in summary:
        if item['case'] != 'arc-w2': continue
        lines.append(f"| {LABELS[item['variant']]} | {item['n']} | {item['late_forward_mps']['mean']:.6f} | {item['late_angular_radps']['mean']:.6f} | {item['peak_full_angular']['max']:.6f} | {item['max_axis_error_deg']['max']:.8f} |")
    for variant in variants:
        if not any(item['variant'] == variant and item['case'] == 'arc-w2' for item in summary):
            lines.append(f"| {LABELS[variant]} | 0 | 中断/未完成 | — | — | — |")
    if protocol.get('suite') == 'drive':
        lines[4] = '仅驱动插件不同，机器人质量/摩擦1/接触ODE200/1 ms/quick60保持。旧方案轮子电机上限1000 Nm；新方案轮子无电机，车体纵横向限力150/200 N、偏航限矩100 Nm。新方案轮速是被动滚动测量，不作为执行指令。'
    lines += ['', '## 直行与时间节奏核验', '', '| 配置 | 直行次数 | 后段直行均速 m/s | 所有完整动作仿真/墙钟比范围 |', '|---|---|---|---|']
    for variant in variants:
        straight = [r for r in rows if r['variant'] == variant and r['case'] == 'straight']
        ratios = [r['observed_sim_wall_ratio'] for r in rows if r['variant'] == variant]
        if straight and ratios:
            lines.append(f"| {LABELS[variant]} | {len(straight)} | {np.mean([r['late_forward_mps'] for r in straight]):.6f} | {min(ratios):.4f}–{max(ratios):.4f} |")
    lines += ['', '## 失败或中断', '']
    if not failures:
        lines.append('无启动/运动中止。输入有效不等同于车身运动响应通过；仍须检查实际速度和轨迹。')
    for failure in failures:
        lines.append(f"- 冷启动{failure['boot']} {LABELS[failure['variant']]}：{failure.get('error')}；完成{failure['completed_trials']}项动作。观测状态：{failure.get('state', {})}。")
    lines += ['', f'完整输入核验通过的动作数（不等于运动响应通过数）：{len(rows)}；失败/中断的冷启动数：{len(failures)}。失败记录见failures.json，已完成的直行仍单独保留，失败圆弧不纳入后段均值。', '', '后段是请求停车前0.5至0.1仿真秒的平均，不意味着运动已稳定。输入有效不等于运动跟踪通过。虚线是平滑后指令积分，存在ROS接收时刻误差，不是Nav2规划路径。', '',
              '轮轴偏角：轮子link局部+Y轴转换到世界后，相对车身局部+Y轴的偏离。适用于本次已核验的两驱模型。', '',
              '[速度与轮速](response-comparison.png) · [轮轴与姿态](geometry-comparison.png) · [轨迹](trajectories.png) · [原始指标](individual-metrics.csv)', '',
              '各boot目录保留原始日志、指令、运行配置与模型；verification.json和native-runtime.json记录参数及一致性核验。没有执行Nav2路径跟随或五件搬运，也未测量逐轮接触力。', '']
    (out / 'REPORT.md').write_text('\n'.join(lines))
    print(f'ANALYZED {len(rows)} completed input-valid trials, {len(failures)} failed boots; figures, metrics and report saved to {out}')
    print(json.dumps([x for x in summary if x['case'] == 'arc-w2'], indent=2))


if __name__ == '__main__': main()