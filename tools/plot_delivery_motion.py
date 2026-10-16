#!/usr/bin/env python3
"""Plot benchmark_delivery motion against world static geometry, colored by speed."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.patches import Polygon
from matplotlib import font_manager


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--drive-label',default='',help='Optional explicit drive label; not inferred from old benchmark logs')
    args=parser.parse_args()
    result=json.loads((args.run_dir/'result.json').read_text())
    motion=json.loads((args.run_dir/'motion.json').read_text())
    layout=json.loads((Path(result['scene']['map']).parent/'layout.json').read_text())
    phases=result.get('phase_events',[])
    if phases:
        begin=phases[0]['sim_ns']/1e9
        end=phases[-1]['sim_ns']/1e9 if result['complete'] and phases[-1]['step']=='WAIT_TASK' else motion[-1]['sim']
        motion=[s for s in motion if begin<=s['sim']<=end]
    if len(motion)<2:raise ValueError('No complete motion trace')
    font=Path('/mnt/c/Windows/Fonts/msyh.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family']=['Microsoft YaHei','DejaVu Sans']
    plt.rcParams['axes.unicode_minus']=False
    fig,ax=plt.subplots(figsize=(12,9))
    xy=np.array([[s['x'],s['y']] for s in motion])
    low,high=xy.min(axis=0)-1,xy.max(axis=0)+1
    for shape in layout['wall_boxes']+layout['static_boxes']:
        c,s=math.cos(shape['yaw']),math.sin(shape['yaw'])
        corners=np.array([[a*shape['length']/2,b*shape['width']/2] for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]])
        corners=corners@np.array([[c,s],[-s,c]])+np.array([shape['x'],shape['y']])
        ax.add_patch(Polygon(corners,closed=True,facecolor='#202020',edgecolor='none',zorder=1))
    speed=np.array([s['v'] for s in motion])
    line=LineCollection(np.stack([xy[:-1],xy[1:]],axis=1),cmap='turbo',norm=Normalize(0,max(2.2,float(speed.max()))),linewidth=2.5,zorder=3)
    line.set_array((speed[:-1]+speed[1:])/2);ax.add_collection(line)
    ax.scatter(*xy[0],s=65,c='limegreen',edgecolors='black',label='任务起点',zorder=4)
    ax.scatter(*xy[-1],s=65,c='black',marker='x',label='记录终点',zorder=4)
    ax.set_xlim(low[0],high[0]);ax.set_ylim(low[1],high[1]);ax.set_aspect('equal')
    ax.set_xlabel('世界 X / m');ax.set_ylabel('世界 Y / m');ax.grid(alpha=.15);ax.legend()
    title='五件搬运实际轨迹' if result['complete'] else '搬运实际轨迹（未完成）'
    ax.set_title(title+('：'+args.drive_label if args.drive_label else ''),fontsize=18)
    fig.colorbar(line,ax=ax,label='实际水平速度 / m/s',shrink=.8)
    fig.text(.5,.02,'物理位姿与静态障碍使用世界坐标；未叠加未经坐标校准的全局计划，未显示移动障碍的历史位置。',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.04,1,1));args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=170);plt.close(fig)
    print(args.output)


if __name__=='__main__':main()
