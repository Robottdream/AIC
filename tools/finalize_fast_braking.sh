#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_speed_ladder_20261003
for name in originbot_nav2_odom_mecanum.yaml originbot_nav2_mecanum.yaml; do
  rel=src/yzbot/bot_navigation/param/$name
  cmp "$evidence/profiles-fast/4/$rel" "$target/$rel"
  cp "$target/$rel" "$repo/$rel"
done
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
python3 "$repo/tools/round_speed_state.py" > "$evidence/live-final-fast-4.json"
python3 "$repo/tools/check_lidar_rate.py" > "$evidence/lidar-final-fast.log"
bash "$target/ros_round_omni_start.sh" start --headless > "$evidence/readiness-final-fast.log" 2>&1
python3 "$repo/tools/summarize_speed_ladder.py"
tail -5 "$evidence/readiness-final-fast.log"
