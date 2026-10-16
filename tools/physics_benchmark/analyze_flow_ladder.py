#!/usr/bin/env python3
"""Plot full-flow speed ladder records; keep missing/failed flows explicit."""
import argparse,csv,hashlib,json,math,shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.patches import Polygon
from analyze_numerical import load_native


def geometry(ax,layout):
    for s in layout['wall_boxes']+layout['static_boxes']:
        c,t=math.cos(s['yaw']),math.sin(s['yaw'])
        points=np.array([[a*s['length']/2,b*s['width']/2] for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]])@np.array([[c,t],[-t,c]])+np.array([s['x'],s['y']])
        ax.add_patch(Polygon(points,closed=True,color='#262626',linewidth=0,zorder=0))


def colored(ax,seq,norm):
    xy=np.array([[s['x'],s['y']] for s in seq])
    line=LineCollection(np.stack([xy[:-1],xy[1:]],axis=1),cmap='turbo',norm=norm,linewidth=2.3,zorder=3)
    line.set_array(np.array([math.hypot(s['vx'],s['vy']) for s in seq[:-1]]));ax.add_collection(line)
    return xy,line


def load_case(folder,speed):
    path=folder/'round-1/result.json'
    if not path.exists():return None
    result=json.loads(path.read_text());phases=result['phase_events']
    ros=json.loads((folder/'round-1/motion.json').read_text())
    start=phases[0]['sim_ns']/1e9
    end=phases[-1]['sim_ns']/1e9 if phases[-1]['step']=='WAIT_TASK' else ros[-1]['sim']
    header,seq=load_native(folder,end)
    seq=[s for s in seq if start<=s['sim']<=end]
    assert len(seq)>10 and header['left_motor_fmax']==header['right_motor_fmax']==0
    assert header['actual_contact_max_correcting_vel']==200 and abs(header['max_step_size']-.001)<1e-10
    assert max(np.diff([s['sim'] for s in seq]))<.025
    verified=json.loads((folder/'runtime-verification.json').read_text())
    assert verified['/simple_nav2_navigator.crossing_cruise_speed']==speed
    ros=[s for s in ros if start<=s['sim']<=end]
    t=np.array([s['sim'] for s in seq]);rt=np.array([s['sim'] for s in ros])
    idx=np.maximum(0,np.searchsorted(rt,t,side='right')-1)
    cmdv=np.array([ros[i].get('cmd_v',0.) for i in idx]);cmdw=np.array([ros[i].get('cmd_w',0.) for i in idx])
    v=np.array([s['v'] for s in seq]);w=np.array([s['w'] for s in seq]);xy=np.array([[s['x'],s['y']] for s in seq])
    active=cmdv>.2
    terminal=phases[-1] if phases[-1]['step']=='WAIT_TASK' else None
    metric=dict(speed=speed,complete=result['complete'],terminal_event_wall_seconds=(terminal['monotonic_ns']-phases[0]['monotonic_ns'])/1e9 if terminal else None,terminal_event_sim_seconds=(terminal['sim_ns']-phases[0]['sim_ns'])/1e9 if terminal else None,all_five_legal=result['all_five_on_zone_surface'],task_wall_seconds=result.get('task_event_wall_seconds'),task_sim_seconds=result.get('task_event_sim_seconds'),poll_wall_seconds=result['wall_seconds'],poll_sim_seconds=result['sim_seconds'],peak_native_forward_mps=float(v.max()),peak_native_angular_radps=float(np.abs(w).max()),max_tilt_deg=max(s['tilt_deg'] for s in seq),route_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),mean_moving_speed_mps=float(np.mean(v[v>.1])),fraction_forward_above_90pct_target=float(np.mean(v>=speed*.9)),forward_tracking_mae_mps=float(np.mean(np.abs(v[active]-cmdv[active]))) if active.any() else None,aborted_for_collision=result['aborted_for_collision'],aborted_for_instability=result['aborted_for_instability'])
    phases_rows=[]
    for a,b in zip(phases,phases[1:]):phases_rows.append(dict(speed=speed,step=a['step'],wall_seconds=(b['monotonic_ns']-a['monotonic_ns'])/1e9,sim_seconds=(b['sim_ns']-a['sim_ns'])/1e9))
    observed=folder/'observed'
    plans=json.loads((observed/'plans.json').read_text()) if (observed/'plans.json').exists() else []
    coverage=json.loads((observed/'coverage.json').read_text()) if (observed/'coverage.json').exists() else {}
    forces=json.loads((observed/'wrenches.json').read_text()) if (observed/'wrenches.json').exists() else []
    return dict(speed=speed,folder=folder,result=result,phases=phases,seq=seq,t=t-start,cmdv=cmdv,cmdw=cmdw,metric=metric,phase_rows=phases_rows,plans=plans,coverage=coverage,forces=forces,start=start,end=end)


def response_plots(destination,cases):
    fig,axes=plt.subplots(2,3,figsize=(17,8),sharex='col')
    body,baxes=plt.subplots(3,3,figsize=(17,11),sharex='col')
    force,faxes=plt.subplots(2,3,figsize=(17,8),sharex='col')
    for col,speed in enumerate((3.,4.,5.)):
        case=cases.get(speed)
        if not case:
            for grid in (axes,baxes,faxes):
                for ax in grid[:,col]:ax.text(.5,.5,'无完整记录',transform=ax.transAxes,ha='center')
            continue
        seq,t=case['seq'],case['t']
        for ax,key,cmd,ylabel in [(axes[0,col],'v',case['cmdv'],'车体前向速度 / m/s'),(axes[1,col],'w',case['cmdw'],'车体角速度 / rad/s')]:
            ax.plot(t,cmd,c='#888888',lw=1,ls='--',label='平滑后指令');ax.plot(t,[s[key] for s in seq],lw=1,c='#176bba',label='原生实测');ax.set_ylabel(ylabel);ax.grid(alpha=.2)
        axes[0,col].set_title(f'{speed:g} m/s 档'+('' if case['result']['complete'] else '（未完成）'))
        baxes[0,col].plot(t,[s['left_joint_w'] for s in seq],lw=.9,label='左轮')
        baxes[0,col].plot(t,[s['right_joint_w'] for s in seq],lw=.9,label='右轮');baxes[0,col].set_ylabel('被动车轮转速 / rad/s')
        baxes[1,col].plot(t,[s['tilt_deg'] for s in seq],lw=.9);baxes[1,col].set_ylabel('最大横滚/俯仰 / °')
        z0=np.median([s['z'] for s in seq[:50]])
        baxes[2,col].plot(t,[(s['z']-z0)*1000 for s in seq],lw=.9);baxes[2,col].set_ylabel('参考link高度相对起点 / mm')
        baxes[0,col].set_title(f'{speed:g} m/s 档')
        samples=[s for s in case['forces'] if case['start']<=s['sim']<=case['end']]
        if samples:
            ft=[s['sim']-case['start'] for s in samples]
            faxes[0,col].plot(ft,[math.hypot(s['fx'],s['fy']) for s in samples],lw=1)
            faxes[1,col].plot(ft,[s['tz'] for s in samples],lw=1)
        faxes[0,col].set_title(f'{speed:g} m/s 档');faxes[0,col].set_ylabel('注入水平力合成值 / N');faxes[1,col].set_ylabel('注入偏航矩 / Nm')
        for grid in (axes,baxes,faxes):
            for ax in grid[:,col]:ax.grid(alpha=.2)
            grid[-1,col].set_xlabel('任务开始后 / 仿真秒')
    axes[0,0].legend();baxes[0,0].legend()
    for fig,title,name in [(fig,'完整五件：指令与实际速度','velocity-response.png'),(body,'完整五件：被动轮速与车身姿态','wheel-body-pose.png'),(force,'完整五件：车体注入力与转矩','applied-wrench.png')]:
        fig.suptitle(title,fontsize=18);fig.tight_layout(rect=(0,.035,1,.95));fig.text(.5,.012,'每档单轮冷启动；动力增益/限力、摩擦1、ODE200/1ms与加减速度保持。各列时长不同，不按时间逐点互相比。',ha='center',fontsize=10);fig.savefig(destination/name,dpi=160);plt.close(fig)


def plan_plots(destination,case,layout):
    legs=[]
    for a,b in zip(case['phases'],case['phases'][1:]):
        begin,end=a['sim_ns']/1e9,b['sim_ns']/1e9
        if a['step'] not in ('NAV_TO_BLOCK','NAV_TO_AREA') or end-begin<.5:continue
        seq=[s for s in case['seq'] if begin<=s['sim']<=end]
        if len(seq)<2:continue
        plans=[p for p in case['plans'] if 'world_points' in p and a['monotonic_ns']/1e9<=p['received_wall']<=b['monotonic_ns']/1e9]
        preferred=[p for p in plans if p['topic']=='/received_global_plan'];plans=preferred or [p for p in plans if p['topic']=='/plan']
        legs.append((a['step'],seq,plans))
    if not legs:return
    cols=math.ceil(len(legs)/math.ceil(len(legs)/5));rows=math.ceil(len(legs)/cols);fig,axes=plt.subplots(rows,cols,figsize=(21,4.7*rows),squeeze=False)
    norm=Normalize(0,max(5.2,case['speed']*1.04));line=None
    for ax,(step,seq,plans),number in zip(axes.flat,legs,range(1,len(legs)+1)):
        geometry(ax,layout);xy,line=colored(ax,seq,norm);allxy=[xy]
        if plans:
            indices=sorted(set(np.linspace(0,len(plans)-1,min(4,len(plans)),dtype=int)))
            for i in indices:
                points=np.array(plans[i]['world_points'])
                if len(points):ax.plot(points[:,0],points[:,1],c='#888888',ls='--',lw=1,alpha=.55,zorder=2);allxy.append(points[:,:2])
        points=np.vstack(allxy);low,high=points.min(axis=0)-.45,points.max(axis=0)+.45
        ax.set_xlim(low[0],high[0]);ax.set_ylim(low[1],high[1]);ax.set_aspect('equal');ax.grid(alpha=.15)
        ax.scatter(*xy[0],s=15,c='black',zorder=4);ax.scatter(*xy[-1],s=20,c='black',marker='x',zorder=4)
        ax.set_title(f'{number} · '+('去抓取' if step=='NAV_TO_BLOCK' else '去放置')+f' · {len(plans)}个路径版本',fontsize=11);ax.set_xlabel('世界X / m');ax.set_ylabel('世界Y / m')
    for ax in list(axes.flat)[len(legs):]:ax.axis('off')
    fig.suptitle((f"第{case['trial']}轮 · " if "trial" in case else "")+f"{case['speed']:g} m/s{'完整流程' if case['result']['complete'] else '流程尝试（未完成）'}：预期路径版本与实际轨迹",fontsize=19)
    fig.text(.5,.025,'彩色：原生实际轨迹；灰虚线：本段最多4个路径版本，优先控制器收到的路径；按消息时间TF转到odom/world。滚动重规划，不是单条固定参考。',ha='center',fontsize=10)
    fig.subplots_adjust(top=.89,bottom=.10,left=.045,right=.93,wspace=.35,hspace=.35)
    if line:
        bar=fig.add_axes([.95,.23,.009,.5]);fig.colorbar(line,cax=bar,label='实际水平速度 / m/s')
    fig.savefig(destination/f'planned-actual-{case["speed"]:g}.png',dpi=155);plt.close(fig)


def route_comparison(destination,cases,layout):
    fig,axes=plt.subplots(1,3,figsize=(18,8));norm=Normalize(0,5.2);line=None
    xy=np.vstack([np.array([[s['x'],s['y']] for s in c['seq']]) for c in cases.values()])
    low,high=xy.min(axis=0)-.7,xy.max(axis=0)+.7
    for ax,speed in zip(axes,(3.,4.,5.)):
        geometry(ax,layout);case=cases.get(speed)
        if case:
            points,line=colored(ax,case['seq'],norm)
            ax.scatter(*points[0],c='limegreen',s=30,edgecolors='black',zorder=5)
            ax.scatter(*points[-1],c='black',marker='x',s=35,zorder=5)
            title=f'{speed:g} m/s · '+('五件完成' if case['result']['complete'] else '未完成')
        else:title=f'{speed:g} m/s · 无记录'
        ax.set_title(title);ax.set_xlim(low[0],high[0]);ax.set_ylim(low[1],high[1]);ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_xlabel('世界 X / m');ax.set_ylabel('世界 Y / m')
    fig.suptitle('完整流程实际轨迹：统一坐标与速度色标',fontsize=19)
    fig.subplots_adjust(left=.05,right=.92,bottom=.1,top=.89,wspace=.18)
    if line:
        bar=fig.add_axes([.94,.23,.012,.56]);fig.colorbar(line,cax=bar,label='实际水平速度 / m/s')
    fig.text(.5,.02,'黑色为静态障碍；绿点为任务起点，叉号为记录结束。每档单轮，移动障碍相位与重规划路线可能不同。',ha='center',fontsize=10)
    fig.savefig(destination/'speed-route-comparison.png',dpi=170);plt.close(fig)


def timing_plot(destination,cases):
    steps=['NAV_TO_BLOCK','NAV_TO_AREA','GRASP','PLACE','SELECT_BLOCK'];labels=['去抓取','去放置','抓取','放置','选块'];colors=plt.get_cmap('tab10').colors
    fig,axes=plt.subplots(1,2,figsize=(15,5))
    for ax,field,title in zip(axes,['sim_seconds','wall_seconds'],['各阶段仿真耗时','各阶段墙钟耗时']):
        for row,(speed,case) in enumerate(sorted(cases.items())):
            left=0
            for i,step in enumerate(steps):
                value=sum(r[field] for r in case['phase_rows'] if r['step']==step)
                ax.barh(row,value,left=left,color=colors[i],label=labels[i] if row==0 else None)
                if value>7:ax.text(left+value/2,row,f'{value:.1f}',ha='center',va='center',fontsize=9)
                left+=value
            ax.text(left+1,row,f'{left:.1f}s',va='center')
        ax.set_yticks(range(len(cases)),[f'{s:g} m/s' for s in sorted(cases)]);ax.set_xlabel('秒');ax.set_title(title);ax.grid(axis='x',alpha=.2)
    axes[0].legend(loc='upper center',bbox_to_anchor=(.5,-.18),ncol=5)
    fig.suptitle('五件流程阶段计时（每档一轮；相位/路径可能不同）',fontsize=17);fig.tight_layout(rect=(0,.10,1,.93));fig.savefig(destination/'stage-timing.png',dpi=170);plt.close(fig)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--windows-output',type=Path,required=True);args=parser.parse_args()
    out=args.output.resolve();destination=args.windows_output.resolve();destination.mkdir(parents=True,exist_ok=True)
    font=Path('/mnt/c/Windows/Fonts/msyh.ttc')
    if font.exists():font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']=['Microsoft YaHei','DejaVu Sans']
    plt.rcParams['axes.unicode_minus']=False
    cases={}
    for speed in (3.,4.,5.):
        case=load_case(out/f'flow-{speed:g}',speed)
        if case:cases[speed]=case
    if not cases:raise RuntimeError('No delivery result is available')
    layout=json.loads((Path(next(iter(cases.values()))['result']['scene']['map']).parent/'layout.json').read_text())
    metrics=[c['metric'] for c in cases.values()];rows=[r for c in cases.values() for r in c['phase_rows']]
    for filename,data in [('flow-metrics.json',metrics),('plan-coverage.json',{str(s):c['coverage'] for s,c in cases.items()})]:
        (out/filename).write_text(json.dumps(data,indent=2));shutil.copy2(out/filename,destination/filename)
    for filename,data in [('flow-metrics.csv',metrics),('stage-timing.csv',rows)]:
        with (out/filename).open('w',newline='') as f:writer=csv.DictWriter(f,fieldnames=data[0]);writer.writeheader();writer.writerows(data)
        shutil.copy2(out/filename,destination/filename)
    response_plots(destination,cases);timing_plot(destination,cases);route_comparison(destination,cases,layout)
    for c in cases.values():plan_plots(destination,c,layout)
    lines=['# 车体施力 3/4/5 m/s完整流程测试','', '每档独立冷启动一次，三红A两蓝B；只改变速度相关参数，正式场景源文件未改。驱动指令硬上限统一5 m/s；巡航/规划nominal_speed/平滑器上限及导航器cruise一致。角速度4、线加减速3.5/5、角加减速10、力150/200 N及偏航矩100 Nm、μ1、ODE200/1ms保持。','', '| 档位 | 完成/合法 | 阶段墙钟 s | 阶段仿真 s | 原生峰值前向 m/s | 最大倾角 ° | 路程 m |','|---|---|---:|---:|---:|---:|---:|']
    for m in metrics:lines.append(f"| {m['speed']:g} | {m['complete']}/{m['all_five_legal']} | {m['task_wall_seconds'] if m['task_wall_seconds'] is not None else '未完成'} | {m['task_sim_seconds'] if m['task_sim_seconds'] is not None else '未完成'} | {m['peak_native_forward_mps']:.4f} | {m['max_tilt_deg']:.5f} | {m['route_m']:.2f} |")
    lines+=['','时间使用主控阶段事件；原始轮询时间另存CSV，不混用。不含冷启动和自然语言解析。单轮相位、选路及重试可能不同，不将耗时差全部归于速度。未完成或中止不称为成功。','', '预期/实际图：所有路径保留原始点、消息时间和实际TF转换。优先/received_global_plan，缺少时使用/plan；灰虚线只展示最多4个重规划版本，非固定参考。转换失败记录保留且不画到world；覆盖数见plan-coverage.json。','', '响应图原生100 Hz实测；平滑指令从ROS里程计回调保存的最新命令取零阶保持，带有传输/回调时间误差。车轮被动滚动，轮速不是电机执行目标。高度为参考link相对任务起点变化，不是整车质心高度。施力图是插件注入量，不含接触反力。','', '速度路径按原生/物理里程计实际位置绘制，不显示移动障碍的历史位置。原始世界、导航、任务、实际URDF、运行参数核验、native日志、ROS命令、计划与阶段记录在各flow目录。','']
    (out/'REPORT.md').write_text('\n'.join(lines));shutil.copy2(out/'REPORT.md',destination/'REPORT.md')
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in destination.iterdir() if p.is_file() and p.name!='export-checksums.json'}
    (destination/'export-checksums.json').write_text(json.dumps(hashes,indent=2))
    print(json.dumps(metrics,indent=2));print('PLOTS AND REPORT',destination)


if __name__=='__main__':main()
