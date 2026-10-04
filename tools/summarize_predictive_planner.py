import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

raw = Path('/home/polarbear/aic_predictive_planner_evidence_20261003')
out = Path('/mnt/c/Users/21920/.codex/worktrees/5a40/AIC/docs/predictive-planner-evidence-20261003')
out.mkdir(parents=True, exist_ok=True)
global_data = json.loads((raw/'global-isolated/result.json').read_text())
local_data = json.loads((raw/'isolated/result.json').read_text())
live = json.loads((raw/'live.json').read_text())
summary = {'global': global_data['summary'], 'local': local_data['summary'],
    'live': {key: live[key] for key in ('parameters', 'forecast_messages', 'max_tracks',
                                      'marker_messages', 'prediction_markers', 'planner_samples')},
    'live_tracks': [{key: track[key] for key in ('id','name','center','velocity','radius')}
                    for track in live['last_forecast']['tracks']]}
(out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
fig, ax = plt.subplots(figsize=(8, 5), dpi=150)
for i in range(21):
    y = 1.0-i*.07
    ax.add_patch(Circle((0,y), .95, color='#f59e0b', alpha=.025))
ax.plot([0,0], [1,-.4], color='#d97706', lw=3, label='Obstacle: next 4 s')
ax.scatter([0], [1], color='#d97706', s=65, zorder=5)
for name, color, label in [('baseline','#94a3b8','No forecast / cleared: 4.00 m'),
                          ('predicted','#2563eb','With known forecast: 5.05 m')]:
    xy = global_data[name]['path']
    ax.plot([p[0] for p in xy], [p[1] for p in xy], color=color, lw=2.5, label=label)
ax.scatter([-2,2],[0,0], c=['#16a34a','#dc2626'], s=75, zorder=6)
ax.text(-2, .12, 'Start', ha='center'); ax.text(2,.12,'Goal',ha='center')
ax.set(xlim=(-2.6,2.6), ylim=(-2.2,2.2), xlabel='x (m)', ylabel='y (m)',
       title='Actual NavFn path: known-route forecast cost')
ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(loc='upper left', fontsize=8)
fig.text(.5,.015,'Isolated empty-map test (ROS domain 165); robot did not move.', ha='center',fontsize=9)
fig.tight_layout(rect=(0,.03,1,1))
fig.savefig(out/'global-path-comparison.png')
fig.savefig(out/'global-path-comparison.svg')
print(json.dumps(summary['global'], indent=2))
