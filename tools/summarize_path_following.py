"""Export compact measured tracking evidence; leave large ROS captures in native WSL."""
import json
import math
import re
import subprocess
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).resolve().parents[1]
native=Path('/home/polarbear/aic_path_following_evidence_20261003')
runtime=Path('/home/polarbear/ws_aic_mecanum_20261002')
out=root/'docs/path-following-evidence-20261003';out.mkdir(parents=True,exist_ok=True)
comparison=json.loads((native/'comparison.json').read_text())
live1=json.loads((native/'live.json').read_text())
live2=json.loads((native/'live-final.json').read_text())
curve=[[i*.05,0.] for i in range(21)]
curve+=[[1.+.8*math.sin(i/40*math.pi/2),.8*(1.-math.cos(i/40*math.pi/2))] for i in range(1,41)]
curve+=[[1.8,.8+i*.05] for i in range(1,45)]
main=(runtime/'log/run/09_main.log').read_text()
selected=re.findall(r'选定可达物块：(red_cube_\d+)',main)
completed=re.findall(r'放置完成（(\d+)/\d+）',main)
summary={'controller':{'lookahead_m':.65,'control_prefix_s':.6,'prefix_corridor_m':.12,
    'off_path_allowance_m':.03,'whole_horizon_mean_weight':.5,'whole_horizon_peak_weight':.25,
    'scale':24.,'simulation_horizon_s':1.8},
    'isolated':{k:{key:value[key] for key in ('status','pose','cmd','max_deviation')} for k,value in comparison.items()},
    'prediction_regression':json.loads((native/'prediction-final/result.json').read_text())['summary'],
    'business':{'requested':'抓取3个红色去A','selected':selected,'placed_count':len(completed),
        'all_tasks_complete':'所有优化任务执行完成' in main,'task_events':live2['events'],
        'note':'Original task preserved. First item includes debugging and two intentional Nav2 recoveries; not a full-task speed comparison.'}}
summary['deployed_parameters']=json.loads((native/'deployed-final.json').read_text())['parameters']
summary['business']['placements']=[]
for name in selected:
    pose=list(map(float,subprocess.check_output(['gz','model','-m',name,'-p'],text=True).split()))
    inside=abs(pose[0]-3.143086)+.015<.4 and abs(pose[1]+5.807858)+.015<.4
    assert inside,('Cube outside A region',name,pose)
    summary['business']['placements'].append({'model':name,'pose_xyz_rpy':pose,'xy_inside_A':inside})
summary['final_control_loop_misses']=(runtime/'log/run/03_nav2.log').read_text().count('Control loop missed')
for label,live in [('initial',live1),('final',live2)]:
    region=[p for p in live['trace'] if 'pose' in p and 2.2<p['pose'][0]<3.8 and -3.2<p['pose'][1]<-1.]
    summary[label+'_door_region']={'samples':len(region),
        'minimum_center_to_lidar_point_m':min((p[k] for p in region for k in ('nearest_left','nearest_right') if p.get(k) is not None),default=None),
        'max_current_path_deviation_m':max((p.get('path_deviation') or 0 for p in region),default=None),
        'max_local_prefix_deviation_m':max((p.get('local_prefix_deviation') or 0 for p in region),default=None)}
(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
for name in ('09_main.log','06_nav.log','07_arm.log'):
    (out/name).write_text((runtime/'log/run'/name).read_text())
fig,axes=plt.subplots(1,2,figsize=(11,5),layout='constrained')
axes[0].plot(*np.array(curve).T,'--',color='#333333',label='Reference path')
for label,color in [('before','#b23a3a'),('after','#19794b')]:
    p=np.array([s['pose'] for s in comparison['curve-'+label]['trace']])
    axes[0].plot(*p.T,color=color,label=label.capitalize())
axes[0].set_title('Actual DWB closed-loop test\nFree map, same curved reference')
axes[0].set_aspect('equal');axes[0].legend();axes[0].grid(alpha=.2)
for label,live,color in [('Initial fix',live1,'#b8892d'),('Final fix',live2,'#245caa')]:
    region=[s['pose'] for s in live['trace'] if 'pose' in s and 2.2<s['pose'][0]<3.8 and -3.2<s['pose'][1]<-1.]
    if region:axes[1].scatter(*np.array(region).T,s=6,color=color,label=label)
axes[1].set_title('Gazebo odometry at A doorway\nRecorded business-task passage')
axes[1].set_aspect('equal');axes[1].legend();axes[1].grid(alpha=.2)
for ax in axes:ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
fig.savefig(out/'tracking.png',dpi=170);fig.savefig(out/'tracking.svg')
print(json.dumps(summary,ensure_ascii=False,indent=2))
