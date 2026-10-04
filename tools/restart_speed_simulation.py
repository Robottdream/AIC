"""Restart the Gazebo group only and restore all robot/cube world poses."""
import json
import argparse
import os
import signal
import subprocess
import time
from pathlib import Path
root=Path('/home/polarbear/aic_speed_ladder_20261003');native=Path('/home/polarbear/ws_aic_mecanum_20261002')
p=argparse.ArgumentParser();p.add_argument('--restore-only',action='store_true');options=p.parse_args()
run=native/'log/run';main=(run/'09_main.log').read_text().splitlines()[-1]
assert '所有优化任务执行完成' in main,'Business task became active'
pids=dict(line.split(':') for line in (run/'pids').read_text().splitlines())
for name in (() if options.restore_only else ('11_watchdog','01_gazebo')):
    if name not in pids:continue
    pid=int(pids[name]);assert os.getpgid(pid)==pid,name
    os.killpg(pid,signal.SIGINT)
    end=time.monotonic()+6
    while time.monotonic()<end:
        try:os.kill(pid,0)
        except ProcessLookupError:break
        time.sleep(.1)
    try:os.killpg(pid,signal.SIGKILL)
    except ProcessLookupError:pass
    pids.pop(name)
env=os.environ.copy();env.update(AIC_ROBOT_MODEL='mecanum',AIC_ARM_ALIGNMENT='1',AIC_ODOM_MAP='1',AIC_DYNAMIC_GUARD='1',GAZEBO_IP='127.0.0.1')
process=None
if not options.restore_only:
    log=(run/'01_gazebo.log').open('w')
    process=subprocess.Popen(['ros2','launch','mybot','gazebo_world.launch.py','gui:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env)
    pids['01_gazebo']=str(process.pid);(run/'pids').write_text(''.join(k+':'+v+'\n' for k,v in pids.items()))
poses=json.loads((root/'world-before-lidar.json').read_text())
end=time.monotonic()+80
while time.monotonic()<end:
    assert process is None or process.poll() is None,'Gazebo group exited'
    try:result=subprocess.run(['gz','model','-m','six_arm','-p'],capture_output=True,text=True,timeout=5)
    except subprocess.TimeoutExpired:continue
    if len(result.stdout.split())==6:break
    time.sleep(.5)
else:raise RuntimeError('Robot not spawned')
for name,pose in poses.items():
    args=['gz','model','-m',name]
    for flag,value in zip(('-x','-y','-z','-R','-P','-Y'),pose):args.extend([flag,str(value)])
    subprocess.run(args,check=True,timeout=8)
time.sleep(1)
after={name:[float(v) for v in subprocess.check_output(['gz','model','-m',name,'-p'],text=True,timeout=8).split()] for name in poses}
(root/'world-after-lidar.json').write_text(json.dumps(after,indent=2))
assert all(sum((a-b)**2 for a,b in zip(after[name][:2],poses[name][:2]))<.01 for name in poses),'World pose restore failed'
print('PASS Gazebo group restarted; all cube positions and robot restored; business processes retained',flush=True)
