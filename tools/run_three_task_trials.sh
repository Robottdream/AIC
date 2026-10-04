#!/usr/bin/env bash
set -eo pipefail
native=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_three_trials_20261003
source /opt/ros/humble/setup.bash
source "$native/install/setup.bash"
source "$native/tools/ros_env.sh"
cd "$native"
for trial in 1 2 3; do
  if [ "$trial" != 1 ]; then
    bash "$native/ros_round_omni_start.sh" restart --headless > "$evidence/start-$trial.log" 2>&1
  fi
  python3 /mnt/c/Users/21920/.codex/worktrees/5a40/AIC/tools/three_task_trial.py --trial "$trial" > "$evidence/trial-$trial-console.log" 2>&1
done
# Leave the system reset and ready after the third completed trial too.
bash "$native/ros_round_omni_start.sh" restart --headless > "$evidence/reset-final.log" 2>&1
