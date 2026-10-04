#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
speed="${1:-4}"
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
source "$target/tools/ros_env.sh"
export AIC_PATH_CHECK_CONFIG=/home/polarbear/aic_speed_ladder_20261003/profiles-fast/$speed/src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml
export AIC_PATH_CHECK_OUT=/home/polarbear/aic_speed_ladder_20261003/isolated-fast-$speed
export AIC_PATH_AFTER_ONLY=1 ROS_DOMAIN_ID=167
python3 "$repo/tools/check_path_following.py"
export AIC_DWB_CHECK_CONFIG="$AIC_PATH_CHECK_CONFIG" AIC_DWB_CHECK_OUTPUT=/home/polarbear/aic_speed_ladder_20261003/prediction-fast-$speed ROS_DOMAIN_ID=164
python3 "$repo/tools/check_predictive_dwb.py"
