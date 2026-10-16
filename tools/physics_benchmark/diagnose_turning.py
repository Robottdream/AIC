#!/usr/bin/env python3
"""Inspect recorded body-force turning; does not run simulation or change parameters."""
import argparse,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from analyze_flow_ladder import load_case

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
 verified=json.loads((args.trial/'runtime-verification.json').read_text());speed=verified['/simple_nav2_navigator.crossing_cruise_speed'];case=load_case(args.trial,speed)
 if not case or not case['forces']:raise RuntimeError('Require valid result/native trace, world-frame wrench records and runtime verification')
 seq=case['seq'];t=np.array([s['sim'] for s in seq]);yaw=np.unwrap([s['yaw'] for s in seq]);v=np.array([s['v'] for s in seq]);w=np.array([s['w'] for s in seq]);vx=np.array([s['vx'] for s in seq]);vy=np.array([s['vy'] for s in seq]);side=-np.sin(yaw)*vx+np.cos(yaw)*vy
 force=[s for s in case['forces'] if case['start']<=s['sim']<=case['end']];assert force and all(s['frame']=='world' for s in force)
 ft=np.array([s['sim'] for s in force]);fyaw=np.interp(ft,t,yaw);fx=np.array([s['fx'] for s in force]);fy=np.array([s['fy'] for s in force]);localx=np.cos(fyaw)*fx+np.sin(fyaw)*fy;localy=-np.sin(fyaw)*fx+np.cos(fyaw)*fy;tz=np.array([s['tz'] for s in force]);fv=np.interp(ft,t,v);fw=np.interp(ft,t,w);fs=np.interp(ft,t,side)
 robot=ET.parse(args.trial/'live-robot.urdf').getroot();plugin=robot.find(".//plugin[@name='body_force_controller']");lat_limit=float(plugin.findtext('lateral_limit'));yaw_limit=float(plugin.findtext('yaw_limit'));mass=sum(float(s.attrib['value']) for s in robot.findall('./link/inertial/mass'))
 moving=(np.abs(fv)>1)&(np.abs(fw)>.5)
 if not moving.any():raise RuntimeError('No force samples meet the turning-speed thresholds')
 peak=int(np.argmax(np.abs(fs)*moving));sat=np.abs(localy)>=lat_limit*.99;ysat=np.abs(tz)>=yaw_limit*.99
 def fraction(mask,condition):return float(np.mean(condition[mask])) if mask.any() else None
 metrics=dict(source_trial=str(args.trial.resolve()),configuration={key:value for key,value in verified.items() if any(name in key for name in ['max_velocity','max_accel','max_decel'])},read_only_existing_record=True,mass_robot_without_cargo_kg=mass,force_samples=len(force),turning_force_samples=int(moving.sum()),turning_definition='abs(v_forward)>1 m/s and abs(w)>0.5 rad/s',lateral_limit_N=lat_limit,yaw_limit_Nm=yaw_limit,peak_injected_forward_N=float(np.max(np.abs(localx))),peak_injected_lateral_N=float(np.max(np.abs(localy))),peak_injected_yaw_Nm=float(np.max(np.abs(tz))),force_samples_above_legacy_lateral_200N=int(np.sum(np.abs(localy)>200.5)),force_samples_above_legacy_yaw_100Nm=int(np.sum(np.abs(tz)>100.5)),turning_lateral_saturation_fraction=fraction(moving,sat),turning_yaw_saturation_fraction=fraction(moving,ysat),peak_lateral_speed_mps=float(np.max(np.abs(side))),turning_lateral_speed_p95_mps=float(np.percentile(np.abs(fs[moving]),95)),peak_turning_lateral_sample=dict(sim=float(ft[peak]),seconds_after_task=float(ft[peak]-case['start']),forward_mps=float(fv[peak]),yaw_radps=float(fw[peak]),lateral_mps=float(fs[peak]),injected_lateral_N=float(localy[peak]),injected_yaw_Nm=float(tz[peak]),ideal_zero_slip_required_lateral_N=float(mass*fv[peak]*fw[peak])),caveats=['Force and odometry sampled at different instants; interpolate native yaw for body-frame force conversion.','Injected force excludes contact reactions; m*v*w is ideal zero-lateral-slip demand, not measured net force.','Mass excludes carried cargo; articulated posture changes inertia.','30 Hz force sampling can miss brief saturation.'])
 (args.output/'turning-diagnosis.json').write_text(json.dumps(metrics,indent=2))
 font=Path('/mnt/c/Windows/Fonts/msyh.ttc')
 if font.exists():font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']=['Microsoft YaHei','DejaVu Sans']
 plt.rcParams['axes.unicode_minus']=False
 fig,axes=plt.subplots(4,1,figsize=(12,12),sharex=True);tt=t-case['start'];ff=ft-case['start'];window=np.abs(ff-ff[peak])<1.5;nativewindow=np.abs(tt-ff[peak])<1.5
 axes[0].plot(tt,v,label='实测前向 m/s');axes[0].plot(tt,w,label='实测角速 rad/s');axes[0].plot(tt,case['cmdw'],ls='--',alpha=.55,label='平滑后角速指令（接收时刻有误差）')
 axes[1].plot(tt,side,label='实测横向速度 m/s',c='#c14845');axes[1].axhline(0,c='gray',lw=.6)
 axes[2].plot(ff,localy,label='注入横向力 N');axes[2].plot(ff,mass*fv*fw,ls='--',label='理想无侧滑所需 m·v·ω（不含接触力）',alpha=.7)
 for sign in [-1,1]:axes[2].axhline(sign*lat_limit,c='red',ls=':',label='横向施力限幅' if sign==1 else None)
 axes[3].plot(ff,tz,label='注入偏航力矩 Nm',c='#cc852a')
 for sign in [-1,1]:axes[3].axhline(sign*yaw_limit,c='red',ls=':',label='偏航力矩限幅' if sign==1 else None)
 for ax in axes:ax.set_xlim(ff[peak]-1.5,ff[peak]+1.5);ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=9)
 axes[-1].set_xlabel('旧流程任务开始后 / 仿真秒');fig.suptitle('既有流程日志：横向偏移最大转弯片段',fontsize=16);fig.tight_layout(rect=(0,.04,1,.95));fig.text(.5,.015,'仅诊断指定旧日志，不代表工作区当前参数已实测。虚线需求为理想估算，未计实际接触反力。',ha='center',fontsize=10);fig.savefig(args.output/'turning-force-diagnosis.png',dpi=155);plt.close(fig)
 print(json.dumps(metrics,indent=2))
if __name__=='__main__':main()