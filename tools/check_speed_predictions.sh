#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
source /opt/ros/humble/setup.bash
source /home/polarbear/ws_aic_mecanum_20261002/install/setup.bash
source /home/polarbear/ws_aic_mecanum_20261002/tools/ros_env.sh
export ROS_DOMAIN_ID=164
for speed in "$@"; do
  export AIC_DWB_CHECK_CONFIG=/home/polarbear/aic_speed_ladder_20261003/profiles/$speed/src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml
  export AIC_DWB_CHECK_OUTPUT=/home/polarbear/aic_speed_ladder_20261003/prediction-$speed
  python3 "$repo/tools/check_predictive_dwb.py"
done
