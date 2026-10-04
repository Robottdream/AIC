"""Build compact round-omni task and alignment evidence from native logs."""
import json
import math
from pathlib import Path
root=Path(__file__).resolve().parents[1]
native=Path('/home/polarbear/aic_round_omni_evidence_20261003')
out=root/'docs/round-omni-evidence-20261003'; out.mkdir(parents=True,exist_ok=True)
rows=[json.loads(line) for line in (native/'tasks/trace.jsonl').read_text().splitlines()]
results=json.loads((native/'tasks/results.json').read_text())
latest={}; world={}; checks=[]; samples=[]; targets=[]
for row in rows:
    if row['type']=='sample': latest=row; samples.append(row)
    if row['type']=='event':
        if row['topic']=='/arm_world_target': world=json.loads(row['data'])
        if row['topic']=='/manual_nav_target': targets.append(json.loads(row['data']))
        if row['topic']=='/arm_status' and row['data'] in ('grasp_succeeded','place_succeeded'):
            x,y,heading=latest['odom']; angle=latest['joints']['arm_yaw_joint']
            desired=math.atan2(world['y']-y,world['x']-x)-heading
            error=math.atan2(math.sin(angle-desired),math.cos(angle-desired))
            checks.append({'task':row['task'],'event':row['data'],'wall_s':row['elapsed'],'chassis_yaw_deg':math.degrees(heading),'arm_yaw_deg':math.degrees(angle),'direction_error_deg':math.degrees(error),'base_xy':[x,y],'target':world})
placements=[]
for model,file,zone,center in [('blue_cube_5','blue5-pose.txt','B',(-1.196544,-6.485499)),('red_cube_2','red2-pose.txt','A',(3.143086,-5.807858)),('blue_cube_2','blue2-pose.txt','A',(3.143086,-5.807858))]:
    pose=list(map(float,(native/file).read_text().split()))
    inside=all(abs(pose[i]-center[i])+.015<.4 for i in (0,1))
    placements.append({'model':model,'zone':zone,'pose_xyz_rpy':pose,'cube_xy_fully_inside_0_8m_zone':inside})
    assert inside, f'Placed cube outside zone: {model}'
summary={'tasks':results,'alignment':checks,'placements':placements,'max_abs_chassis_yaw_deg':max(abs(math.degrees(row['odom'][2])) for row in samples if 'odom' in row),'max_abs_commanded_chassis_yaw_rad_s':max(abs(row.get('cmd_xyz',[0,0,0])[2]) for row in samples),'peak_lateral_command_m_s':max(abs(row.get('cmd_xyz',[0,0,0])[1]) for row in samples),'manual_targets':targets,'validation_limits':'One continuous three-task run; ideal planar drive and existing virtual attachment; wall times affected by simulation real-time factor; not hardware performance evidence.'}
assert all(result['status']=='success' for result in results)
assert summary['max_abs_commanded_chassis_yaw_rad_s']==0.
assert summary['max_abs_chassis_yaw_deg']<.2
assert all(abs(row['direction_error_deg'])<3 for row in checks)
for name in ('live.json','arm-geometry.json'):
    (out/name).write_text((native/name).read_text())
(out/'geometry_checks.json').write_text((root/'assets/robot_mecanum/geometry_checks.json').read_text())
(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary,ensure_ascii=False,indent=2))
