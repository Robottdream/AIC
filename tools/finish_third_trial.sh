#!/usr/bin/env bash
set -eo pipefail
native=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_three_trials_20261003
source /opt/ros/humble/setup.bash
source "$native/install/setup.bash"
source "$native/tools/ros_env.sh"
cd "$native"
mv "$evidence/trial-3" "$evidence/trial-3-preflight-query-timeout"
mv "$evidence/trial-3-console.log" "$evidence/trial-3-preflight-query-timeout.log"
python3 /mnt/c/Users/21920/.codex/worktrees/5a40/AIC/tools/three_task_trial.py --trial 3 > "$evidence/trial-3-console.log" 2>&1
bash "$native/ros_round_omni_start.sh" restart --headless > "$evidence/reset-final.log" 2>&1
