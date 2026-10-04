#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/polarbear/ws_aic_mecanum_20261002/install/setup.bash
source /home/polarbear/ws_aic_mecanum_20261002/tools/ros_env.sh
python3 /mnt/c/Users/21920/.codex/worktrees/5a40/AIC/tools/round_speed_state.py
