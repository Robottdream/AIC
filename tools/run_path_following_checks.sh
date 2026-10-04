#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/polarbear/ws_aic_mecanum_20261002/install/setup.bash
source /home/polarbear/ws_aic_mecanum_20261002/tools/ros_env.sh
export ROS_DOMAIN_ID=167
python3 /mnt/c/Users/21920/.codex/worktrees/5a40/AIC/tools/check_path_following.py
