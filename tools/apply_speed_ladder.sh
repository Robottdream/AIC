#!/usr/bin/env bash
set -eo pipefail
speed="$1"
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
evidence=/home/polarbear/aic_speed_ladder_20261003
python3 "$repo/tools/prepare_speed_ladder.py" --apply "$speed"
bash /home/polarbear/ws_aic_mecanum_20261002/ros_round_omni_start.sh recover --headless > "$evidence/recover-$speed.log" 2>&1
bash "$repo/tools/run_speed_ladder_route.sh" --speed "$speed"
