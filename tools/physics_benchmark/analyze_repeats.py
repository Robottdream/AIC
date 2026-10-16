#!/usr/bin/env python3
"""Analyze repeated CURRENT-configuration delivery flows, retaining failed attempts."""
import argparse,csv,hashlib,html,json,math,shutil,textwrap
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import Normalize
from scipy.signal import savgol_filter
from analyze_flow_ladder import load_case,geometry,colored,plan_plots


def description(protocol):
    a,d=protocol['acceleration'],protocol['deceleration']
    return f"线速{protocol['linear_speed']:g} m/s、角速{protocol['angular_speed']:g} rad/s；线加/减速{a[0]:g}/{abs(d[0]):g} m/s²，角加/减速{a[2]:g}/{abs(d[2]):g} rad/s²"


def panels(out,cases,protocol):
    columns=len(cases);fig,axes=plt.subplots(2,columns,figsize=(8*columns,8),squeeze=False)
    body,baxes=plt.subplots(3,columns,figsize=(8*columns,11),squeeze=False)
    force,faxes=plt.subplots(3,columns,figsize=(8*columns,11),squeeze=False)
    for col,(index,case) in enumerate(cases.items()):
        t,seq=case['t'],case['seq']
        for row,key,cmd,title in [(0,'v',case['cmdv'],'前向速度 / m/s'),(1,'w',case['cmdw'],'角速度 / rad/s')]:
            ax=axes[row,col];ax.plot(t,cmd,c='#888888',ls='--',lw=1,label='平滑后指令');ax.plot(t,[s[key] for s in seq],lw=1,c='#1874bc',label='原生实测');ax.set_ylabel(title);ax.grid(alpha=.2)
        axes[0,col].set_title(f'第{index}轮'+(' · 完成' if case['result']['complete'] else ' · 未完成'))
        baxes[0,col].plot(t,[s['left_joint_w'] for s in seq],lw=.9,label='左轮');baxes[0,col].plot(t,[s['right_joint_w'] for s in seq],lw=.9,label='右轮');baxes[0,col].set_ylabel('被动轮速 / rad/s')
        baxes[1,col].plot(t,[s['tilt_deg'] for s in seq],lw=.9);baxes[1,col].set_ylabel('最大横滚/俯仰 / °')
        z0=np.median([s['z'] for s in seq[:50]]);baxes[2,col].plot(t,[(s['z']-z0)*1000 for s in seq],lw=.9);baxes[2,col].set_ylabel('参考link相对起点高度 / mm')
        for row,key in enumerate(['v','w']):
            values=np.array([s[key] for s in seq]);filtered=savgol_filter(values,11,2);accel=np.gradient(filtered,t)
            faxes[row,col].plot(t,accel,lw=.9)
            acceleration=protocol['acceleration'][0 if row==0 else 2]
            braking=abs(protocol['deceleration'][0 if row==0 else 2])
            for sign in [-1,1]:
                faxes[row,col].axhline(sign*acceleration,c='#777777',ls='--',lw=1,label='加速幅值' if sign==1 else None)
                if braking!=acceleration:faxes[row,col].axhline(sign*braking,c='#b76e20',ls=':',lw=1,label='制动幅值' if sign==1 else None)
            if braking!=acceleration:faxes[row,col].legend(fontsize=8)
            faxes[row,col].set_ylabel('前向速度变化率 / m/s²' if key=='v' else '角加速度 / rad/s²')
        samples=[s for s in case['forces'] if case['start']<=s['sim']<=case['end']]
        if samples:
            ft=[s['sim']-case['start'] for s in samples];faxes[2,col].plot(ft,[math.hypot(s['fx'],s['fy']) for s in samples],lw=.9,label='注入水平力 / N');twin=faxes[2,col].twinx();twin.plot(ft,[s['tz'] for s in samples],lw=.8,c='#d47a20',alpha=.6,label='偏航矩 / Nm');twin.set_ylabel('偏航矩 / Nm')
        faxes[2,col].set_ylabel('注入水平力 / N')
        for grid in [axes,baxes,faxes]:
            grid[0,col].set_title(f'第{index}轮'+(' · 完成' if case['result']['complete'] else ' · 未完成'))
            for ax in grid[:,col]:ax.grid(alpha=.2)
            grid[-1,col].set_xlabel('任务开始后 / 仿真秒')
    axes[0,0].legend();baxes[0,0].legend()
    for figure,title,name,footnote in [(fig,f"指令与实际速度 · 上限 {protocol['linear_speed']:g} m/s / {protocol['angular_speed']:g} rad/s\n加/减速：线 {protocol['acceleration'][0]:g}/{abs(protocol['deceleration'][0]):g} m/s²，角 {protocol['acceleration'][2]:g}/{abs(protocol['deceleration'][2]):g} rad/s²",'velocity-response.png','指令来自ROS回调记录，带有接收时间误差；每轮时长可不同。'),(body,f'{len(cases)}次流程尝试：轮速与车身姿态','wheel-body-pose.png','被动车轮无电机目标；高度是参考link相对起点变化，非整车质心高度。'),(force,'实际速度变化率与施力','acceleration-wrench.png','变化率由100Hz原生速度经11点、约0.11秒二阶平滑后微分；参考线表示速度幅值增大/减小时的加速/制动幅值；角速度正负方向对称，非实测硬限幅。')]:
        figure.suptitle(title,fontsize=12.5 if columns==1 else 17);figure.tight_layout(rect=(0,.06,1,.90 if columns==1 else .93));figure.text(.5,.012,'\n'.join(textwrap.wrap(footnote,width=48 if columns==1 else 95)),ha='center',va='bottom',fontsize=8.5 if columns==1 else 10);figure.savefig(out/name,dpi=165);plt.close(figure)


def routes(out,cases,layout,protocol):
    fig,axes=plt.subplots(1,len(cases),figsize=(9*len(cases),9),squeeze=False);norm=Normalize(0,max(protocol['linear_speed'],max(math.hypot(s['vx'],s['vy']) for c in cases.values() for s in c['seq'])))
    points=np.vstack([np.array([[s['x'],s['y']] for s in c['seq']]) for c in cases.values()]);low,high=points.min(axis=0)-.8,points.max(axis=0)+.8
    for ax,(index,case) in zip(axes.flat,cases.items()):
        geometry(ax,layout);xy,line=colored(ax,case['seq'],norm);ax.scatter(*xy[0],c='limegreen',edgecolors='black',s=40,zorder=5);ax.scatter(*xy[-1],c='black',marker='x',s=45,zorder=5);ax.set_xlim(low[0],high[0]);ax.set_ylim(low[1],high[1]);ax.set_aspect('equal');ax.set_xlabel('世界 X / m');ax.set_ylabel('世界 Y / m');ax.grid(alpha=.15);ax.set_title(f'第{index}轮'+(' · 五件完成' if case['result']['complete'] else ' · 未完成'))
    fig.suptitle(f'{len(cases)}轮实际轨迹：同坐标、同速度色标',fontsize=19);fig.subplots_adjust(left=.055,right=.92,bottom=.1,top=.90,wspace=.18);bar=fig.add_axes([.94,.22,.012,.55]);fig.colorbar(line,cax=bar,label='实际水平速度 / m/s');fig.text(.5,.025,'黑色为静态障碍；绿点为任务起点，叉号为结束位置。未显示移动障碍历史；失败轨迹不外推。',ha='center',fontsize=10);fig.savefig(out/'route-comparison.png',dpi=170);plt.close(fig)


def timing(out,cases):
    fig,axes=plt.subplots(1,2,figsize=(15,5));steps=['NAV_TO_BLOCK','NAV_TO_AREA','GRASP','PLACE','SELECT_BLOCK'];labels=['去抓取','去放置','抓取','放置','选块/路径评估'];colors=plt.get_cmap('tab10').colors
    for ax,field,title in zip(axes,['sim_seconds','wall_seconds'],['阶段仿真耗时','阶段墙钟耗时']):
        for row,(index,case) in enumerate(cases.items()):
            left=0
            for i,step in enumerate(steps):
                value=sum(r[field] for r in case['phase_rows'] if r['step']==step);ax.barh(row,value,left=left,color=colors[i],label=labels[i] if row==0 else None)
                if value>8:ax.text(left+value/2,row,f'{value:.1f}',ha='center',va='center',fontsize=9)
                left+=value
            ax.text(left+1,row,f'{left:.1f}s',va='center')
        ax.set_yticks(range(len(cases)),[f'第{i}轮'+('' if c['result']['complete'] else '（未完成）') for i,c in cases.items()]);ax.set_title(title);ax.set_xlabel('秒');ax.grid(axis='x',alpha=.2)
    axes[0].legend(loc='upper center',bbox_to_anchor=(.5,-.18),ncol=5);fig.suptitle(f'{len(cases)}次冷启动阶段耗时',fontsize=18);fig.tight_layout(rect=(0,.12,1,.93));fig.savefig(out/'stage-timing.png',dpi=170);plt.close(fig)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--windows-output',type=Path,required=True);args=parser.parse_args();out=args.output.resolve();destination=args.windows_output.resolve();destination.mkdir(parents=True,exist_ok=True)
    font=Path('/mnt/c/Windows/Fonts/msyh.ttc')
    if font.exists():font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']=['Microsoft YaHei','DejaVu Sans']
    plt.rcParams['axes.unicode_minus']=False
    protocol=json.loads((out/'protocol.json').read_text());statuses=json.loads((out/'run-status.json').read_text());cases={};summary=[]
    for index in range(1,protocol['repeats']+1):
        folder=out/f'trial-{index}'
        case=load_case(folder,protocol['linear_speed'])
        if case:
            case['trial']=index;cases[index]=case;r=case['result'];summary.append(dict(trial=index,grasp_count=r['grasp_count'],place_count=r['place_count'],**case['metric']))
            target=destination/f'trial-{index}';target.mkdir(exist_ok=True);plan_plots(target,case,json.loads((Path(r['scene']['map']).parent/'layout.json').read_text()));shutil.copy2(folder/'round-1/result.json',target/'result.json');shutil.copy2(folder/'observed/coverage.json',target/'plan-coverage.json')
        else:summary.append(dict(trial=index,complete=False,error=next((r.get('error') for r in statuses if r['trial']==index),'No completed attempt')))
    (out/'summary.json').write_text(json.dumps(summary,indent=2));shutil.copy2(out/'summary.json',destination/'summary.json')
    if cases:
        layout=json.loads((Path(next(iter(cases.values()))['result']['scene']['map']).parent/'layout.json').read_text());routes(destination,cases,layout,protocol);panels(destination,cases,protocol);timing(destination,cases)
        rows=[dict(trial=index,**r) for index,c in cases.items() for r in c['phase_rows']]
        with (destination/'stage-timing.csv').open('w',newline='') as f:writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    fields=list(dict.fromkeys(k for r in summary for k in r))
    with (destination/'summary.csv').open('w',newline='') as f:writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(summary)
    limits=protocol.get('force_limits')
    if limits is None and cases:
        import xml.etree.ElementTree as ET
        plugin=ET.parse(next(iter(cases.values()))['folder']/'live-robot.urdf').getroot().find(".//plugin[@name='body_force_controller']")
        limits={key:float(plugin.findtext(key)) for key in ['forward_limit','lateral_limit','yaw_limit']}
    force_description=(f"当前车体注入力限额{limits['forward_limit']:g}/{limits['lateral_limit']:g} N、偏航矩{limits['yaw_limit']:g} Nm" if limits else '车体施力限额未取得')
    lines=[f'# 当前参数{protocol["repeats"]}次完整流程测试','', description(protocol)+f'。{force_description}、μ1、ODE200/1ms保持，实际加速度不保证达到指令限值。{protocol["repeats"]}次独立冷启动，流程之间不调参。','', '| 轮次 | 完成/放置件数 | 阶段墙钟 s | 阶段仿真 s | 尝试墙钟 s | 原生峰值前向 m/s | 原生峰值角速 rad/s | 最大倾角 ° |','|---|---|---:|---:|---:|---:|---:|---:|']
    for r in summary:lines.append(f"| {r['trial']} | {r['complete']}/{r.get('place_count','—')} | {r.get('task_wall_seconds') if r.get('task_wall_seconds') is not None else r.get('terminal_event_wall_seconds','—')} | {r.get('task_sim_seconds') if r.get('task_sim_seconds') is not None else r.get('terminal_event_sim_seconds','—')} | {r.get('poll_wall_seconds','—')} | {r.get('peak_native_forward_mps','—')} | {r.get('peak_native_angular_radps','—')} | {r.get('max_tilt_deg','—')} |")
    passed=[r for r in summary if r.get('complete') and r.get('all_five_legal')]
    if len(passed)==len(summary):lines+=['',f"{len(passed)}轮全部完成且合法；完成时间均值：{np.mean([r['task_wall_seconds'] for r in passed]):.2f}墙钟秒 / {np.mean([r['task_sim_seconds'] for r in passed]):.1f}仿真秒。"]
    else:lines+=['','未完成的尝试不纳入完成时间均值，不称为完整流程成功。中止原因见summary.json与各trial原始日志。']
    lines+=['','时间不含冷启动或自然语言解析。阶段计时与采集轮询时间分开保存；失败/中止显示尝试耗时。路径版本在消息时间通过TF变换到物理odom/world；灰虚线展示最多4个滚动计划，非固定全程参考。','', '速度/角速度命令从ROS里程计回调记录取零阶保持，存在接收时刻误差。实际变化率图经约0.11秒平滑微分，不能当作未滤波瞬时加速度。注入力不含接触反力；车轮被动滚动。','', '原始输入配置、实际URDF、参数核验、原生100Hz位姿轮速、任务阶段、命令、计划/TF及施力遥测均在各trial目录，源码/二进制哈希在source-binary-hashes.json。不同轮的障碍相位与重新选路可能不同，不能将时间差当作长期性能结论。','']
    text='\n'.join(lines);(out/'REPORT.md').write_text(text);(destination/'REPORT.md').write_text(text)
    charts=[('route-comparison.png','实际速度着色轨迹'),('stage-timing.png','阶段耗时'),('velocity-response.png','实际与指令速度'),('wheel-body-pose.png','轮速与车身姿态'),('acceleration-wrench.png','速度变化率和施力')]+[(f'trial-{index}/planned-actual-{protocol["linear_speed"]:g}.png',f'第{index}轮预期与实际路径') for index in cases]
    items=''.join(f'<section><h2>{title}</h2><a href="{name}"><img src="{name}" loading="lazy" alt="{title}"></a></section>' for name,title in charts if (destination/name).exists())
    def display_value(row,key):
        value=row.get(key)
        if value is None and key in ('task_wall_seconds','task_sim_seconds'):
            value=row.get('terminal_event_wall_seconds' if key=='task_wall_seconds' else 'terminal_event_sim_seconds')
        return '—' if value is None else value
    table=''.join('<tr>'+''.join(f'<td>{html.escape(str(display_value(r,k)))}</td>' for k in ['trial','complete','place_count','task_wall_seconds','task_sim_seconds','poll_wall_seconds'])+'</tr>' for r in summary)
    (destination/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>AIC当前配置完整流程测试</title><style>body{max-width:1500px;margin:36px auto;padding:0 24px;font:16px/1.6 "Microsoft YaHei",sans-serif;color:#18202c;background:#fafafa}table{border-collapse:collapse}td,th{border:1px solid #ccd2da;padding:8px 20px}img{max-width:100%;background:white}section{margin:40px 0}</style><h1>当前参数完整流程测试</h1><p>冷启动各一轮；失败尝试不计作完成。点击图片查看原图。</p><table><tr><th>轮次</th><th>完成</th><th>放置件数</th><th>阶段墙钟秒</th><th>阶段仿真秒</th><th>尝试墙钟秒</th></tr>'+table+'</table><p><a href="REPORT.md">测试报告</a> · <a href="summary.csv">指标CSV</a> · <a href="stage-timing.csv">阶段CSV</a></p>'+items+'</html>')
    manifest={str(p.relative_to(destination)):hashlib.sha256(p.read_bytes()).hexdigest() for p in destination.rglob('*') if p.is_file() and p.name!='export-checksums.json'};(destination/'export-checksums.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(summary,indent=2));print('REPORT AND CHARTS',destination)


if __name__=='__main__':main()
