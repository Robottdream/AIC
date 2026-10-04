#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
cp "$target/log/run/01_gazebo.log" /home/polarbear/aic_speed_ladder_20261003/reload-failed-gazebo.log
cp "$repo/src/yzbot/bot_navigation/scripts/mecanum_io.py" "$target/src/yzbot/bot_navigation/scripts/mecanum_io.py"
python3 "$repo/tools/restart_speed_simulation.py" "$@"
python3 "$repo/tools/prepare_speed_ladder.py" --apply 1
bash "$target/ros_round_omni_start.sh" recover --headless > /home/polarbear/aic_speed_ladder_20261003/recover-lidar.log 2>&1
python3 "$repo/tools/check_lidar_rate.py"
for speed in 1 2 3 4; do
  echo "Testing final profile $speed"
  bash "$repo/tools/apply_speed_ladder.sh" "$speed"
done
