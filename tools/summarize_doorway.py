"""Compact actual doorway evidence and a static plot of the two driven routes."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

native=Path('/home/polarbear/aic_doorway_evidence_20261003')
repo=Path('/mnt/c/Users/21920/.codex/worktrees/5a40/AIC')
out=repo/'docs/doorway-evidence-20261003';out.mkdir(parents=True,exist_ok=True)
before=json.loads((native/'before.json').read_text())
summary={'before_pose':before['pose'],'before_nearest_center_wall_distance':min(s['nearest'] for s in before['scans'].values()),
         'normal_stop_radius':.405,'planning_nominal_radius':.45,'planning_padding':.01,
         'physical_ttc_footprint_radius':.325,'max_speed':1.,'trials':{},
         'full_pick_place_task_tested':False,'all_doorways_tested':False,'carried_load_envelope_not_resolved':True}
for label in ('exit-A','enter-A'):
    raw=json.loads((native/(label+'.json')).read_text());assert raw['passed']
    summary['trials'][label]={k:raw[k] for k in ('status','wall_seconds','sim_seconds','peak_speed','recoveries','minimum_lidar_clearance')}
summary['guard_check']=json.loads((native/'guard.json').read_text())
summary['dwb_check']=json.loads((native/'isolated-dwb/result.json').read_text())['summary']
summary['isolated_plan_check']={k:{a:b for a,b in v.items() if a!='path'} if isinstance(v,dict) else v for k,v in json.loads((native/'isolated-plan/result.json').read_text()).items()}
summary['live']=json.loads((native/'final-live.json').read_text())
(out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
grid=np.load(native/'global-costmap.npy')
info=before['global'];res=info['resolution'];origin=info['origin']
xmin,xmax=1.5,4.3;ymin,ymax=-6.2,-.9
left,right=int((xmin-origin[0])/res),int((xmax-origin[0])/res)
bottom,top=int((ymin-origin[1])/res),int((ymax-origin[1])/res)
image=np.where(grid[bottom:top,left:right]>=100,0.22,1.)
fig,ax=plt.subplots(figsize=(7,8),dpi=160)
ax.imshow(image,cmap='gray',vmin=0,vmax=1,origin='lower',extent=[origin[0]+left*res,origin[0]+right*res,origin[1]+bottom*res,origin[1]+top*res])
for label,color,text in [('exit-A','#119ba2','Drive out of A'),('enter-A','#356de8','Drive into A')]:
    data=json.loads((native/(label+'.json')).read_text())
    xy=np.array([s['pose'] for s in data['samples']])
    ax.plot(xy[:,0],xy[:,1],color=color,lw=2.4,label=text)
ax.plot(*before['pose'],marker='x',color='#cb4748',ms=10,mew=2,label='Original stop near jamb')
circle=plt.Circle(before['pose'],.405,fill=False,color='#cb4748',linestyle='--',lw=1)
ax.add_patch(circle)
ax.scatter(2.593086,-5.727858,marker='*',s=120,color='#404654',label='A parking target')
ax.set(xlim=(xmin,xmax),ylim=(ymin,ymax),xlabel='Map x (m)',ylabel='Map y (m)',title='Actual robot trajectories through doorway A')
ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(loc='lower right',fontsize=8)
closest=min(summary['trials'][label]['minimum_lidar_clearance'] for label in ('exit-A','enter-A'))
fig.text(.5,.02,f'Both directions succeeded; no recoveries. Closest center-to-wall distance: {closest:.3f} m.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.04,1,1));fig.savefig(out/'doorway-routes.png');fig.savefig(out/'doorway-routes.svg');plt.close(fig)
print(json.dumps(summary['trials'],indent=2))
