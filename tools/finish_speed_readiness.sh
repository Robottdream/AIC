#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_speed_ladder_20261003
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
cp "$evidence/lidar-live.json" "$evidence/lidar-wall-stall.json"
python3 "$repo/tools/check_lidar_rate.py" > "$evidence/lidar-final.log"
bash "$target/ros_round_omni_start.sh" start --headless > "$evidence/readiness-final.log" 2>&1
tail -12 "$evidence/readiness-final.log"
python3 "$repo/tools/summarize_speed_ladder.py"
