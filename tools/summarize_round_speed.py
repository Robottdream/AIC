"""Keep compact speed-limit and regression evidence beside the source report."""
import json
from pathlib import Path
import sys

native=Path('/home/polarbear/aic_round_speed_evidence_20261003')
repo=Path('/mnt/c/Users/21920/.codex/worktrees/5a40/AIC')
sys.path.insert(0,str(repo/'tools'))
from check_dynamic_tracker import run_case

count=0
for vx,vy in ((1.,0.),(0.,1.),(.7071,.7071),(-1.,0.)):
    for angular in (0.,.5):
        for center in ((.9,.6),(1.5,.3),(3.,1.),(-1.,-.5)):
            tracker,_=run_case(lambda t:(vx*t,vy*t,angular*t),lambda t:center)
            assert not any(track.stable for track in tracker.tracks),(vx,vy,angular,center)
            count+=1
summary={'old_max_speed_mps':.7,'new_max_speed_mps':1.,'limit_increase_percent':round((1/.7-1)*100,1),
         'old_axis_acceleration_mps2':.35,'new_axis_acceleration_mps2':.7,
         'live':json.loads((native/'live-state.json').read_text()),
         'controller_check':json.loads((native/'isolated/result.json').read_text())['summary'],
         'guard_check':json.loads((native/'guard.json').read_text()),
         'static_ego_compensation_cases_at_new_speed':count,
         'actual_drive_comparison':'4m route attempt timed out with no displacement; canceled; no throughput improvement proven',
         'full_task_tested':False,'original_workspace_modified':False}
live=summary['live']['parameters']
assert live['controller_server']['FollowPath.max_speed_xy']==live['scan_velocity_guard']['max_linear_speed']==1.
assert live['velocity_smoother']['max_accel'][:2]==[.7,.7]
assert summary['live']['prediction']['tracks']==2
out=repo/'docs/round-speed-evidence-20261003'
out.mkdir(parents=True,exist_ok=True)
(out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
print('Live limits/predictions, DWB, final guard and',count,'high-speed ego compensation cases PASS')
