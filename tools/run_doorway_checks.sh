#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
evidence=/home/polarbear/aic_doorway_evidence_20261003
source /opt/ros/humble/setup.bash
source /home/polarbear/ws_aic_mecanum_20261002/install/setup.bash
source /home/polarbear/ws_aic_mecanum_20261002/tools/ros_env.sh
ROS_DOMAIN_ID=168 python3 "$repo/tools/check_doorway_plan.py" > "$evidence/plan-check.log" 2>&1
cat "$evidence/plan-check.log"
ROS_DOMAIN_ID=164 AIC_DWB_CHECK_CONFIG="$repo/src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml" AIC_DWB_CHECK_OUTPUT="$evidence/isolated-dwb" \
  python3 "$repo/tools/check_predictive_dwb.py" > "$evidence/dwb-check.log" 2>&1
tail -33 "$evidence/dwb-check.log"
