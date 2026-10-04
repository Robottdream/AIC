#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_round_speed_evidence_20261003
mkdir -p "$evidence/before"
files=(src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml
 src/yzbot/bot_navigation/param/originbot_nav2_mecanum.yaml
 src/yzbot/bot_navigation/scripts/scan_velocity_guard.py
 src/yzbot/bot_navigation/launch/nav_bringup_gazebo.launch.py)
for file in "${files[@]}"; do
  mkdir -p "$evidence/before/$(dirname "$file")"
  if [ ! -f "$evidence/before/$file" ]; then cp "$target/$file" "$evidence/before/$file"; fi
  cp "$repo/$file" "$target/$file"
done
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
python3 - "$target" <<'PY'
import sys, yaml
from pathlib import Path
p=Path(sys.argv[1])/'src/yzbot/bot_navigation/param'
for name in ('originbot_nav2_odom_mecanum.yaml','originbot_nav2_mecanum.yaml'):
    data=yaml.safe_load((p/name).read_text())
    follow=data['controller_server']['ros__parameters']['FollowPath']
    smoother=data['velocity_smoother']['ros__parameters']
    assert follow['max_speed_xy']==data['scan_velocity_guard']['ros__parameters']['max_linear_speed']==1.0
    assert follow['acc_lim_x']==follow['acc_lim_y']==smoother['max_accel'][0]==smoother['max_accel'][1]==.7
    assert smoother['max_velocity'][:2]==[1.,1.] and smoother['min_velocity'][:2]==[-1.,-1.]
    assert follow['decel_lim_x']==-2.5 and follow['sim_time']==1.8
    assert 'PredictiveObstacle' in follow['critics']
    assert 'known_trajectory_layer' in data['global_costmap']['global_costmap']['ros__parameters']['plugins']
    print(name, 'limits/prediction/6m map PASS')
PY
python3 -m py_compile "$target/src/yzbot/bot_navigation/scripts/scan_velocity_guard.py" "$target/src/yzbot/bot_navigation/launch/nav_bringup_gazebo.launch.py"
export ROS_DOMAIN_ID=164
export AIC_DWB_CHECK_OUTPUT="$evidence/isolated"
python3 "$repo/tools/check_predictive_dwb.py" > "$evidence/dwb-check.log" 2>&1
tail -36 "$evidence/dwb-check.log"
