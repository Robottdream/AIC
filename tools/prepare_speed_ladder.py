"""Generate complete bounded-speed profiles from a saved baseline, never cumulatively."""
import argparse
import math
from pathlib import Path
import shutil
import yaml

p=argparse.ArgumentParser();p.add_argument('--apply',type=float);a=p.parse_args()
repo=Path(__file__).resolve().parents[1];native=Path('/home/polarbear/ws_aic_mecanum_20261002')
evidence=Path('/home/polarbear/aic_speed_ladder_20261003');evidence.mkdir(exist_ok=True)
files=['src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml',
       'src/yzbot/bot_navigation/param/originbot_nav2_mecanum.yaml']
for rel in files:
    backup=evidence/'before-files'/rel;backup.parent.mkdir(parents=True,exist_ok=True)
    if not backup.exists():shutil.copy2(native/rel,backup)
    for speed in (1.,2.,3.,4.):
        data=yaml.safe_load(backup.read_text());c=data['controller_server']['ros__parameters']['FollowPath']
        horizon=max(1.8,speed/2.5+.6)
        c.update(min_vel_x=-speed,max_vel_x=speed,min_vel_y=-speed,max_vel_y=speed,max_speed_xy=speed)
        c['sim_time']=horizon
        # DWB's default 2 m forward pruning artificially makes high-speed
        # rollouts leave the reference even on an unobstructed straight road.
        c['forward_prune_distance']=max(2.,speed*horizon+.8)
        c['PathFollow.lookahead_distance']=.65*speed
        c['PathFollow.max_path_chord_deviation']=.06 if speed>1 else 0.
        # The horizon must fit in every direction, including the footprint.
        size=max(6,int(math.ceil(2*(speed*horizon+.464+.35)/2)*2))
        local=data['local_costmap']['local_costmap']['ros__parameters'];local.update(width=size,height=size)
        marking=max(6.,float(math.ceil(speed*horizon+.8)))
        for scope in (local,data['global_costmap']['global_costmap']['ros__parameters']):
            for side in ('left','right'):
                scope['obstacle_layer'][side].update(obstacle_max_range=marking,raytrace_max_range=marking+1.)
        smoother=data['velocity_smoother']['ros__parameters']
        smoother['max_velocity'][:2]=[speed,speed];smoother['min_velocity'][:2]=[-speed,-speed]
        data['scan_velocity_guard']['ros__parameters'].update(max_linear_speed=speed,prediction_horizon=horizon)
        profile=evidence/'profiles'/str(int(speed))/rel;profile.parent.mkdir(parents=True,exist_ok=True)
        profile.write_text(yaml.safe_dump(data,sort_keys=False,allow_unicode=True))
        if a.apply==speed:shutil.copy2(profile,native/rel)
if a.apply is not None:
    assert a.apply in (1.,2.,3.,4.),'Use one of the tested profile speeds'
    print(f'Applied {a.apply:.1f} m/s profile; reload Nav2 before motion')
else:print('Generated 1/2/3/4 m/s profiles; active parameters unchanged')
