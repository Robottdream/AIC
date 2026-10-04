#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_predictive_planner_evidence_20261003
cp "$repo/src/yzbot/bot_navigation/scripts/scan_velocity_guard.py" "$target/src/yzbot/bot_navigation/scripts/scan_velocity_guard.py"
cp "$repo/src/yzbot/bot_navigation/scripts/known_obstacle_forecaster.py" "$target/src/yzbot/bot_navigation/scripts/known_obstacle_forecaster.py"
bash "$target/ros_round_omni_start.sh" recover --headless > "$evidence/recover.log" 2>&1
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
python3 "$repo/tools/check_predictive_planner_live.py" > "$evidence/live-check.log" 2>&1
cat "$evidence/live-check.log"
