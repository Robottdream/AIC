#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_doorway_evidence_20261003
files=(src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml
 src/yzbot/bot_navigation/param/originbot_nav2_mecanum.yaml
 src/yzbot/bot_navigation/param/collision_monitor_mecanum.yaml
 src/yzbot/bot_navigation/scripts/scan_velocity_guard.py)
mkdir -p "$evidence/before-files" "$evidence/final-files"
for file in "${files[@]}"; do
  mkdir -p "$evidence/before-files/$(dirname "$file")" "$evidence/final-files/$(dirname "$file")"
  if [ ! -f "$evidence/before-files/$file" ]; then cp "$target/$file" "$evidence/before-files/$file"; fi
  cp "$repo/$file" "$evidence/final-files/$file"
done
python3 - "$evidence" "$target" <<'PY'
import ast,json,math,sys,yaml
from pathlib import Path
evidence,target=map(Path,sys.argv[1:])
data=json.loads((evidence/'before.json').read_text())
points=[p for scan in data['scans'].values() for p in scan['points']]
minimum=min(math.hypot(p[0]-.7*i/40,p[1]-.25*i/40) for p in points for i in range(41))
assert minimum>.375,minimum
print('Measured slow escape sweep clearance',minimum,flush=True)
for name in ('originbot_nav2_odom_mecanum.yaml','originbot_nav2_mecanum.yaml'):
    relative=Path('src/yzbot/bot_navigation/param')/name
    final=yaml.safe_load((evidence/'final-files'/relative).read_text())
    for scope in ('local','global'):
        params=final[scope+'_costmap'][scope+'_costmap']['ros__parameters']
        footprint=ast.literal_eval(params['footprint'])
        assert len(footprint)==24 and all(abs(math.hypot(*p)-.45)<1e-5 for p in footprint)
    config=yaml.safe_load((evidence/'before-files'/relative).read_text())
    follow=config['controller_server']['ros__parameters']['FollowPath']
    for axis in ('x','y'):
        follow['min_vel_'+axis]=-.12;follow['max_vel_'+axis]=.12
    follow['max_speed_xy']=.12
    smoother=config['velocity_smoother']['ros__parameters']
    smoother['max_velocity'][:2]=[.12,.12];smoother['min_velocity'][:2]=[-.12,-.12]
    config['scan_velocity_guard']['ros__parameters']['max_linear_speed']=.12
    (target/relative).write_text(yaml.safe_dump(config,sort_keys=False))
relative=Path('src/yzbot/bot_navigation/param/collision_monitor_mecanum.yaml')
monitor=yaml.safe_load((evidence/'before-files'/relative).read_text())
monitor['collision_monitor']['ros__parameters']['NearStop']['points']=[v for i in range(24) for v in (.36*math.cos(i*math.tau/24),.36*math.sin(i*math.tau/24))]
(target/relative).write_text(yaml.safe_dump(monitor,sort_keys=False))
PY
escaped=0
restore() {
  local phase=before-files
  if [ "$escaped" = 1 ]; then phase=final-files; fi
  for file in "${files[@]}"; do cp "$evidence/$phase/$file" "$target/$file"; done
  bash "$target/ros_round_omni_start.sh" recover --headless > "$evidence/final-recover.log" 2>&1
  tail -10 "$evidence/final-recover.log"
}
trap restore EXIT
bash "$target/ros_round_omni_start.sh" recover --headless > "$evidence/escape-recover.log" 2>&1
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
python3 "$repo/tools/doorway_escape.py" > "$evidence/escape-check.log" 2>&1
cat "$evidence/escape-check.log"
escaped=1
