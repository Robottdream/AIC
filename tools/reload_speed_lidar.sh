#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_speed_ladder_20261003
mkdir -p "$evidence/pre-prune" "$evidence/before-files/src/yzbot/mybot_description/urdf"
for f in "$evidence"/route-*.json "$evidence"/straight-*.json "$evidence"/calibration-*.json "$evidence"/recover-*.log; do
  if [ -f "$f" ]; then cp "$f" "$evidence/pre-prune/"; fi
done
for file in lidar_gazebo.xacro originbot_mecanum_gazebo.xacro; do
  rel=src/yzbot/mybot_description/urdf/$file
  if [ ! -f "$evidence/before-files/$rel" ]; then cp "$target/$rel" "$evidence/before-files/$rel"; fi
  cp "$repo/$rel" "$target/$rel"
done
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
python3 "$repo/tools/reload_speed_lidar.py"
