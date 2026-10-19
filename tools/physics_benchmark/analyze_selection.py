#!/usr/bin/env python3
"""Compare saved selection phases using monotonic task events, not log clocks."""
import argparse
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


def load_case(label, root):
    result = json.loads((root / 'round-1/result.json').read_text())
    events = result['phase_events']
    phases = [(b['monotonic_ns'] - a['monotonic_ns']) / 1e9
              for a, b in zip(events, events[1:]) if a['step'] == 'SELECT_BLOCK']
    log = (root / 'stack-logs/09_main.log').read_text(errors='replace')
    stats = re.findall(r'选块统计: 精查(\d+)位置，完整查路(\d+)次，耗时([\d.]+)s', log)
    timings = [dict(zip(('end_to_end', 'acceptance', 'result_wait', 'server_reported'),
                       map(float, values))) for values in re.findall(
        r'端到端=([\d.]+)s，受理=([\d.]+)s，取结果=([\d.]+)s，服务端规划=([\d.]+)s', log)]
    candidates = ([int(row[0]) for row in stats] if stats else
                  list(map(int, re.findall(r'按当前代价地图检查(\d+)个抓取位置', log))))
    nav = (root / 'stack-logs/03_nav2.log').read_text(errors='replace')
    return dict(label=label, root=str(root), selection_seconds=phases,
                total_selection_seconds=sum(phases), checked_candidates=candidates,
                full_requests=sum(int(row[1]) for row in stats) if stats else None,
                full_requests_upper_bound=2 * sum(candidates), request_timings=timings,
                task_wall_seconds=result.get('task_event_wall_seconds'),
                task_sim_seconds=result.get('task_event_sim_seconds'),
                complete=result['complete'], legal=result['all_five_on_zone_surface'],
                search_budget_exhaustions=nav.count('Timed route search did not find'),
                fresh_state_rejections=nav.count('Fresh-state validation rejected'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', action='append', required=True, help='label=trial directory')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cases = [load_case(label, Path(path)) for label, path in
             (item.split('=', 1) for item in args.case)]
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'selection-metrics.json').write_text(json.dumps(cases, indent=2))
    font = Path('/mnt/c/Windows/Fonts/msyh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = ['Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
    labels = [case['label'] for case in cases]
    width = .8 / len(cases)
    for i, case in enumerate(cases):
        phases = case['selection_seconds']
        axes[0].bar(np.arange(len(phases)) + i * width, phases, width=width,
                    label=case['label'])
    axes[0].set_xticks(np.arange(max(len(c['selection_seconds']) for c in cases)) +
                      width * (len(cases) - 1) / 2,
                      range(1, max(len(c['selection_seconds']) for c in cases) + 1))
    axes[0].set_xlabel('第几次选块'); axes[0].set_ylabel('墙钟秒（单调时钟）')
    axes[0].set_title('每件选块等待'); axes[0].legend(fontsize=8)
    for i, case in enumerate(cases):
        select, total = case['total_selection_seconds'], case['task_wall_seconds']
        axes[1].bar(i, select, color='#b16dcd', label='选块' if i == 0 else None)
        axes[1].bar(i, total - select, bottom=select, color='#4f8db8',
                    label='其余阶段' if i == 0 else None)
        axes[1].text(i, total + 2, f'{total:.1f}s', ha='center')
        axes[2].bar(i, sum(case['checked_candidates']), color='#4a9f87')
        axes[2].text(i, sum(case['checked_candidates']) + 1,
                     str(sum(case['checked_candidates'])), ha='center')
    axes[1].set_title('完整任务耗时'); axes[1].set_ylabel('墙钟秒'); axes[1].legend(fontsize=8)
    axes[1].set_ylim(0, max(c['task_wall_seconds'] for c in cases) * 1.15)
    axes[2].set_title('进入完整查路的抓取位置数'); axes[2].set_ylabel('位置数')
    axes[2].set_ylim(0, max(sum(c['checked_candidates']) for c in cases) * 1.15)
    for ax in axes:
        ax.grid(axis='y', alpha=.2); ax.set_axisbelow(True)
    for ax in axes[1:]: ax.set_xticks(range(len(cases)), labels, fontsize=9)
    fig.suptitle('选块规划分步优化 · 同运动参数的有限流程记录', fontsize=16)
    fig.tight_layout(rect=(0, .08, 1, .93))
    fig.text(.5, .025, '历史/各次冷启动的障碍相位与系统负载不同；样本有限，不能视为严格性能对照。',
             ha='center', fontsize=10)
    fig.savefig(args.output / 'selection-comparison.png', dpi=160)
    plt.close(fig)
    lines = ['# 选块计时比较', '',
             '按任务事件monotonic_ns计算SELECT_BLOCK，不使用ROS日志时间戳估算墙钟。', '',
             '| 记录 | 精查位置 | 完整请求 | 选块/任务墙钟秒 | 完成且落点合格 |',
             '|---|---:|---:|---:|---|']
    for c in cases:
        requests = c['full_requests'] if c['full_requests'] is not None else f"≤{c['full_requests_upper_bound']}（上界）"
        lines.append(f"|{c['label']}|{sum(c['checked_candidates'])}|{requests}|"
                     f"{c['total_selection_seconds']:.3f}/{c['task_wall_seconds']:.3f}|"
                     f"{c['complete'] and c['legal']}|")
        if c['request_timings']:
            lines.append('')
            lines.append(f"{c['label']}：端到端请求时间之和"
                         f"{sum(t['end_to_end'] for t in c['request_timings']):.3f}s；"
                         f"服务端报告规划时间之和{sum(t['server_reported'] for t in c['request_timings']):.3f}s。"
                         '两者计时范围/时钟可能不同；差额含通信、排队及回调，不等于纯网络耗时或CPU时间。')
    lines.extend(['', '原始目录、逐次选块与请求分项见selection-metrics.json。', '',
                  '![比较图](selection-comparison.png)'])
    (args.output / 'REPORT.md').write_text('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
