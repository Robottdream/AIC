#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
speed="${1:-4}"
python3 "$repo/tools/prepare_fast_braking.py" --speed "$speed"
bash "$target/ros_round_omni_start.sh" recover --headless > /home/polarbear/aic_speed_ladder_20261003/recover-fast-$speed.log 2>&1
bash "$repo/tools/run_speed_ladder_route.sh" --speed "$speed" --tag fast
