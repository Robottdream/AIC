#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_speed_ladder_20261003
# Run after the low-point-count model is loaded and isolated checks pass.
cp "$repo/src/yzbot/bot_navigation/scripts/mecanum_io.py" "$target/src/yzbot/bot_navigation/scripts/mecanum_io.py"
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
python3 "$repo/tools/check_lidar_rate.py"
for speed in 1 2 3 4; do
  echo "Testing final profile $speed"
  if ! bash "$repo/tools/apply_speed_ladder.sh" "$speed"; then
    echo "Profile $speed failed; inspect evidence before choosing default"
    exit 1
  fi
done
