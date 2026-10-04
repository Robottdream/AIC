"""Separate faster-acceleration trial; retain obstacle and stale-data protections."""
import math
from pathlib import Path
import yaml
import argparse
p=argparse.ArgumentParser();p.add_argument('--speed',type=int,default=4);a=p.parse_args();assert a.speed in (2,4)
root=Path('/home/polarbear/aic_speed_ladder_20261003');native=Path('/home/polarbear/ws_aic_mecanum_20261002')
for name in ('originbot_nav2_odom_mecanum.yaml','originbot_nav2_mecanum.yaml'):
    rel=Path('src/yzbot/bot_navigation/param')/name
    data=yaml.safe_load((root/'profiles'/str(a.speed)/rel).read_text())
    c=data['controller_server']['ros__parameters']['FollowPath'];horizon=max(1.8,a.speed/5.+.6)
    c.update(acc_lim_x=2.,acc_lim_y=2.,decel_lim_x=-5.,decel_lim_y=-5.,sim_time=horizon,forward_prune_distance=a.speed*horizon+.8)
    s=data['velocity_smoother']['ros__parameters'];s['max_accel'][:2]=[2.,2.];s['max_decel'][:2]=[-5.,-5.]
    local=data['local_costmap']['local_costmap']['ros__parameters'];size=max(6,2*math.ceil(a.speed*horizon+.464+.35));local.update(width=size,height=size)
    marking=max(6.,float(math.ceil(a.speed*horizon+.8)))
    for scope in (local,data['global_costmap']['global_costmap']['ros__parameters']):
        for side in ('left','right'):scope['obstacle_layer'][side].update(obstacle_max_range=marking,raytrace_max_range=marking+1.)
    data['scan_velocity_guard']['ros__parameters']['prediction_horizon']=horizon
    profile=root/'profiles-fast'/str(a.speed)/rel;profile.parent.mkdir(parents=True,exist_ok=True)
    profile.write_text(yaml.safe_dump(data,sort_keys=False,allow_unicode=True));(native/rel).write_text(profile.read_text())
print(f'Applied trial {a.speed} m/s, acceleration 2 m/s^2, braking 5 m/s^2, horizon {horizon}s',flush=True)
