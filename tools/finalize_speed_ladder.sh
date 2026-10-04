#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_speed_ladder_20261003
python3 "$repo/tools/prepare_speed_ladder.py" --apply 2
for file in originbot_nav2_odom_mecanum.yaml originbot_nav2_mecanum.yaml; do
  cp "$target/src/yzbot/bot_navigation/param/$file" "$repo/src/yzbot/bot_navigation/param/$file"
done
cp "$repo/src/yzbot/mybot_description/urdf/lidar_gazebo.xacro" "$target/src/yzbot/mybot_description/urdf/lidar_gazebo.xacro"
bash "$target/ros_round_omni_start.sh" recover --headless > "$evidence/recover-final-2.log" 2>&1
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
python3 "$repo/tools/round_speed_state.py" > "$evidence/live-final-2.json"
python3 "$repo/tools/check_lidar_rate.py" > "$evidence/lidar-final.log"
bash "$target/ros_round_omni_start.sh" start --headless > "$evidence/readiness-final.log" 2>&1
tail -12 "$evidence/readiness-final.log"
