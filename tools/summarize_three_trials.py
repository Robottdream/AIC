"""Validate reset/placement evidence and write simulation-time trial results."""
import csv
import json
import statistics
from pathlib import Path

root = Path('/home/polarbear/aic_three_trials_20261003')
workspace = Path('/mnt/c/Users/21920/.codex/worktrees/5a40/AIC')
out = workspace / 'docs/three-task-trials-evidence-20261003'
out.mkdir(exist_ok=True)
results = [json.loads((root / f'trial-{i}/result.json').read_text()) for i in (1, 2, 3)]
zones = {0: (3.143086, -5.807858), 1: (-1.196544, -6.485499)}
initial = results[0]['initial_poses']
reset_poses = json.loads((root / 'reset-final-poses.json').read_text())
assert all(abs(reset_poses[name][axis] - initial[name][axis]) < .03 for name in initial for axis in (0, 1)), 'Final reset failed'
assert 'PREFLIGHT=OK' in (root / 'reset-final.log').read_text(errors='replace'), 'Final startup doctor failed'
(out / 'reset-final-poses.json').write_text(json.dumps(reset_poses, indent=2) + '\n')
for r in results:
    assert r['status'] == 'success' and len(r['placements']) == 5, r['trial']
    assert r['clock_resets'] == 0
    assert [(x['cube'].split('_')[0], x['area']) for x in r['placements']] == [('red', 0)] * 3 + [('blue', 1)] * 2
    assert all(abs(r['initial_poses'][name][axis] - initial[name][axis]) < .03 for name in initial for axis in (0, 1)), 'Initial poses differ'
    for x in r['placements']:
        pose = r['final_poses'][x['cube']]
        x['fully_inside_zone'] = all(abs(pose[k] - zones[x['area']][k]) + .015 < .4 for k in (0, 1))
        assert x['fully_inside_zone'], (r['trial'], x, pose)
    r['red_A_sim_seconds'] = r['placements'][2]['sim'] - r['start_sim']
    r['blue_B_sim_seconds'] = r['end_sim'] - r['placements'][2]['sim']
    (out / f"trial-{r['trial']}.json").write_text(json.dumps(r, ensure_ascii=False, indent=2) + '\n')
durations = [r['sim_seconds'] for r in results]
summary = {'time_source': '/clock', 'interval': 'Natural-language command publication to main-controller all-tasks-completed event',
           'trials': results, 'mean_sim_seconds': statistics.mean(durations), 'min_sim_seconds': min(durations),
           'max_sim_seconds': max(durations), 'sample_stddev_sim_seconds': statistics.stdev(durations),
           'raw_evidence': str(root), 'reset': 'Full simulator and business-node restart before every trial and after trial 3',
           'preflight_incident': 'Before trial 3 command publication, Gazebo transport Connection assertion terminated the server. Both checks failed before publishing a command; archived separately. Restarted simulator and replaced repeated gz model queries with one pose/info subscription snapshot.'}
(out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
with (out / 'timings.csv').open('w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['trial', 'start_sim_seconds', 'end_sim_seconds', 'total_sim_seconds', 'red_A_sim_seconds', 'blue_B_sim_seconds', 'status'])
    for r in results: w.writerow([r[k] for k in ('trial', 'start_sim', 'end_sim', 'sim_seconds', 'red_A_sim_seconds', 'blue_B_sim_seconds', 'status')])
lines = ['# 三轮完整任务仿真计时（2026-10-03）', '',
         '指令：抓取3个红色物块去A区，2个蓝色物块去B区。由自然语言解析入口 `/command` 下发，按出题顺序完成。', '',
         '计时使用 Gazebo `/clock`，从发布命令到主控发出“所有优化任务执行完成”。不包括冷启动、复位或完成后的采样等待。读取时钟粒度约0.1仿真秒。', '',
         '使用当前圆形全向轮车型，速度上限4m/s，加速2m/s²、减速5m/s²，左右雷达各811点、8Hz；动态障碍和预判/碰撞保护保持启用。', '',
         '| 轮次 | 仿真开始(s) | 仿真结束(s) | 总耗时(s) | 3红A阶段(s) | 2蓝B阶段(s) | 结果 |',
         '|---|---:|---:|---:|---:|---:|---|']
for r in results:
    lines.append(f"| {r['trial']} | {r['start_sim']:.2f} | {r['end_sim']:.2f} | {r['sim_seconds']:.2f} | {r['red_A_sim_seconds']:.2f} | {r['blue_B_sim_seconds']:.2f} | 5/5成功 |")
lines += ['', f"平均 **{summary['mean_sim_seconds']:.2f} 仿真秒**；范围 {min(durations):.2f}–{max(durations):.2f} 仿真秒，样本标准差 {summary['sample_stddev_sim_seconds']:.2f} 秒。", '',
          '每轮之前完整重启仿真与业务节点，机器人恢复出生点、十个物块恢复原位、抓取标记清空；逐轮真位姿核对一致。第五次放置后读取物块真位姿，15次放置的物块均完整落在对应0.8m区域内。第三轮后再次复位并进行启动就绪检查。', '',
          '三轮启动耗时导致下令时的动态障碍相位可能不同；这是重复完整任务计时，不能单凭这三轮判断相对旧速度配置的提速幅度。', '',
          '第三轮发令前，Gazebo内部transport::Connection断言导致服务器退出，两次准备检查均未发令，记录保存在trial-3-preflight目录。冷启动恢复后，计时探针改用单次pose/info订阅快照代替反复gz model查询。生产模型、业务与导航参数保持一致；三次已发出的完整任务均纳入上表。测试跨10月3日至4日。', '',
          f'完整日志及事件：`{root}`；紧凑结果：`docs/three-task-trials-evidence-20261003/summary.json`、`timings.csv`。']
(workspace / 'docs/three-task-trials-20261003.md').write_text('\n'.join(lines) + '\n')
print(json.dumps({k: summary[k] for k in ('mean_sim_seconds', 'min_sim_seconds', 'max_sim_seconds')}, indent=2))
print('TRIALS', durations)
