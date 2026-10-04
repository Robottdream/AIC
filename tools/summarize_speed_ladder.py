"""Compact recorded speed trials; distinguish measured odometry from limits."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
repo=Path(__file__).resolve().parents[1];root=Path('/home/polarbear/aic_speed_ladder_20261003')
out=repo/'docs/speed-ladder-evidence-20261003';out.mkdir(exist_ok=True)
report={'raw_evidence':str(root),'runs':[],'lidar':json.loads((root/'lidar-live.json').read_text()),
        'clearing':json.loads((root/'lidar-clearing/dense.json').read_text())}
if (root/'calibration-long-4.json').exists():
    direct=json.loads((root/'calibration-long-4.json').read_text())
    report['protected_straight_4']={k:v for k,v in direct.items() if k!='legs'}
    report['protected_straight_4']['legs']=[{k:v for k,v in leg.items() if k!='samples'} for leg in direct['legs']]
if (root/'live-final-2.json').exists():report['final_live']=json.loads((root/'live-final-2.json').read_text())
if (root/'live-final-fast-4.json').exists():report['final_live']=json.loads((root/'live-final-fast-4.json').read_text())
if (root/'route-4-fast.json').exists():
    fast=json.loads((root/'route-4-fast.json').read_text());legs=[v for v in fast['legs'] if v['leg']!='preposition']
    report['fast_4']={'passed':fast.get('passed',False),'wall_seconds':sum(v['wall_seconds'] for v in legs),
      'sim_seconds':sum(v['sim_seconds'] for v in legs),'peak_speed':max(v['peak_speed'] for v in legs),
      'min_clearance':min(v['min_clearance'] for v in legs),'max_deviation':max(v['max_deviation'] for v in legs),
      'recoveries':sum(v['recoveries'] for v in legs),'parameters':fast['parameters'],
      'legs':[{k:v for k,v in leg.items() if k!='samples'} for leg in fast['legs']]}
if (root/'fast-stop-4.json').exists():report['fast_stop']={k:v for k,v in json.loads((root/'fast-stop-4.json').read_text()).items() if k!='samples'}
fig,axes=plt.subplots(1,2,figsize=(11,4))
colors={1:'#64748b',2:'#0d9488',3:'#d97706',4:'#dc2626'}
for speed in (1,2,3,4):
    source=root/f'route-{speed}.json'
    if not source.exists():continue
    run=json.loads(source.read_text())
    if 'FollowPath.forward_prune_distance' not in run.get('parameters',{}).get('controller_server',{}):continue
    legs=[v for v in run['legs'] if v['leg']!='preposition']
    row={'limit':speed,'passed':run.get('passed',False),'wall_seconds':sum(v['wall_seconds'] for v in legs),
        'sim_seconds':sum(v['sim_seconds'] for v in legs),'peak_speed':max((v['peak_speed'] for v in legs),default=0),
        'recoveries':sum(v['recoveries'] for v in legs),'min_clearance':min((v['min_clearance'] for v in legs),default=None),
        'max_deviation':max((v['max_deviation'] for v in legs),default=None),'parameters':run['parameters'],
        'legs':[{k:v for k,v in leg.items() if k!='samples'} for leg in run['legs']]}
    report['runs'].append(row)
    offset=0
    for leg in legs:
        samples=leg['samples']
        axes[0].plot([v['sim']+offset for v in samples],[v['speed'] for v in samples],color=colors[speed],label=f'{speed} m/s limit' if offset==0 else None)
        offset+=leg['sim_seconds']
        if leg['leg']=='enter_A':axes[1].plot([v['pose'][0] for v in samples],[v['pose'][1] for v in samples],color=colors[speed],label=f'{speed} m/s limit')
axes[0].set(xlabel='Elapsed simulation time (s)',ylabel='Measured odometry speed (m/s)');axes[0].legend();axes[0].grid(alpha=.2)
if (root/'route-4-fast.json').exists():
    offset=0
    for leg in fast['legs']:
        if leg['leg']=='preposition':continue
        samples=leg['samples'];axes[0].plot([v['sim']+offset for v in samples],[v['speed'] for v in samples],color='#7c3aed',label='4 m/s, faster braking' if offset==0 else None);offset+=leg['sim_seconds']
        if leg['leg']=='enter_A':axes[1].plot([v['pose'][0] for v in samples],[v['pose'][1] for v in samples],color='#7c3aed',label='4 m/s, faster braking')
    axes[0].legend(fontsize=8)
axes[1].set(xlabel='World X (m)',ylabel='World Y (m)',title='A doorway approach',xlim=(2.2,3.5),ylim=(-4.0,-1.5));axes[1].set_aspect('equal',adjustable='box');axes[1].grid(alpha=.2)
fig.tight_layout();fig.savefig(out/'speed-and-doorway.png',dpi=170);plt.close(fig)
(out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps([{k:v for k,v in row.items() if k not in ('parameters','legs')} for row in report['runs']],indent=2),flush=True)
