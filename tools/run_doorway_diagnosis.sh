#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/polarbear/ws_aic_mecanum_20261002/install/setup.bash
source /home/polarbear/ws_aic_mecanum_20261002/tools/ros_env.sh
python3 /mnt/c/Users/21920/.codex/worktrees/5a40/AIC/tools/doorway_diagnosis.py --ros-args -r /tf:=/navigation/tf -r /tf_static:=/navigation/tf_static
