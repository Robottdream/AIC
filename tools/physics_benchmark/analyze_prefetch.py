#!/usr/bin/env python3
"""Show saved prefetch work overlapping pickup navigation and guarded reuse."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from analyze_flow_ladder import load_case


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    case = load_case(args.trial, 6.)
    events = json.loads((args.trial / 'observed/return-prefetch.json').read_text())
    phases = case['phases']; start = case['start']
    pickup = [(a['sim_ns']/1e9, b['sim_ns']/1e9)
              for a, b in zip(phases, phases[1:]) if a['step'] == 'NAV_TO_BLOCK']
    jobs = []
    for queued in (e for e in events if e['event'] == 'queued'):
        related = [e for e in events if e['generation'] == queued['generation']]
        ready = next((e for e in related if e['event'] == 'ready'), None)
        interval = next(((a, b) for a, b in pickup if a <= queued['sim'] <= b), None)
        jobs.append(dict(generation=queued['generation'], queued_sim=queued['sim'],
                         ready_sim=ready['sim'] if ready else None,
                         compute_wall_seconds=ready['seconds'] if ready else None,
                         ready_during_pickup=bool(ready and interval and ready['sim'] <= interval[1]),
                         reused=sum(e['event'] == 'reused' for e in related),
                         rejected=sum(e['event'] == 'rejected' for e in related)))
    summary = dict(task_metrics=case['metric'], events=dict(Counter(e['event'] for e in events)),
                   jobs=jobs, validated_reuse_wall_seconds=[e['seconds'] for e in events if e['event'] == 'reused'])
    (args.output / 'prefetch-metrics.json').write_text(json.dumps(summary, indent=2))
    font = Path('/mnt/c/Windows/Fonts/msyh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = ['Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(2, 1, figsize=(14, 7), gridspec_kw=dict(height_ratios=[1.3, 1.]))
    lanes = ['NAV_TO_BLOCK', 'GRASP', 'NAV_TO_AREA', 'PLACE']
    colors = ['#4c8ab5', '#58a68e', '#dc9e4a', '#a77bb5']
    for a, b in zip(phases, phases[1:]):
        if a['step'] not in lanes: continue
        y = lanes.index(a['step']); begin = a['sim_ns']/1e9 - start
        width = (b['sim_ns'] - a['sim_ns'])/1e9
        axes[0].barh(y, width, left=begin, height=.55, color=colors[y])
    markers = dict(queued=('o', '#555555'), ready=('^', '#258051'),
                   reused=('*', '#d62d2d'), rejected=('x', '#b56c11'))
    event_labels = dict(queued='发起', ready='静态算完', reused='复核复用', rejected='复核拒绝')
    for event, (marker, color) in markers.items():
        points = [e['sim'] - start for e in events if e['event'] == event]
        axes[0].scatter(points, [4]*len(points), marker=marker, c=color,
                        s=70 if event == 'reused' else 40, label=event_labels[event], zorder=5)
    axes[0].set_yticks(range(5), ['去抓取', '抓取', '去放置', '放置', '预计算事件'])
    axes[0].set_xlabel('任务开始后 / 仿真秒'); axes[0].set_title('预计算完成时刻与行进阶段')
    axes[0].set_ylim(-.7, 5.2)
    axes[0].legend(ncol=4, fontsize=9, loc='upper right')
    for i, job in enumerate(jobs):
        if job['compute_wall_seconds'] is not None:
            value=job['compute_wall_seconds']*1000
            axes[1].bar(i, value, color='#58a68e')
            axes[1].text(i, value+1, f'{value:.1f}', ha='center', fontsize=9)
    axes[1].set_xticks(range(len(jobs)), [str(j['generation']) for j in jobs])
    axes[1].set_xlabel('预计算请求代际（含方向重试）'); axes[1].set_ylabel('后台计算 / 墙钟毫秒')
    values = [j['compute_wall_seconds']*1000 for j in jobs if j['compute_wall_seconds'] is not None]
    axes[1].set_ylim(0, max(values, default=1.)*1.25)
    for ax in axes: ax.grid(axis='x' if ax is axes[0] else 'y', alpha=.2); ax.set_axisbelow(True)
    fig.suptitle('行进中预计算返程 · 完整流程实测', fontsize=17)
    fig.tight_layout(rect=(0, .06, 1, .93))
    fig.text(.5, .015, '空白含选块等阶段；三角为静态算完，红星为新鲜状态/实际轮廓复核后复用，叉号表示回退现场规划。', ha='center', fontsize=10)
    fig.savefig(args.output / 'prefetch-overlap.png', dpi=160); plt.close(fig)
    index = args.output / 'index.html'
    if index.exists() and 'id="return-prefetch"' not in index.read_text():
        content = ('<section id="return-prefetch"><h2>行进中预计算返程</h2>'
                   '<p><a href="prefetch-metrics.json">事件与复核指标</a></p>'
                   '<img src="prefetch-overlap.png" style="max-width:100%"></section>')
        index.write_text(index.read_text().replace('</body>', content + '</body>'))
    report = args.output / 'REPORT.md'
    link = '[提前规划事件与重叠图](prefetch-overlap.png)'
    if report.exists() and link not in report.read_text():
        report.write_text(report.read_text() + '\n' + link +
                          '；逐个请求及复用/拒绝次数见prefetch-metrics.json。\n')
    manifest = {str(p.relative_to(args.output)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in args.output.rglob('*') if p.is_file() and p.name != 'export-checksums.json'}
    (args.output / 'export-checksums.json').write_text(json.dumps(manifest, indent=2))


if __name__ == '__main__': main()
