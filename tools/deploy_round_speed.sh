#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_round_speed_evidence_20261003
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
ROS_DOMAIN_ID=166 python3 "$repo/tools/check_round_speed_guard.py" > "$evidence/guard-check.log" 2>&1
cat "$evidence/guard-check.log"
bash "$target/ros_round_omni_start.sh" recover --headless > "$evidence/recover.log" 2>&1
tail -15 "$evidence/recover.log"
python3 "$repo/tools/round_speed_state.py" > "$evidence/live-state.json"
cat "$evidence/live-state.json"
