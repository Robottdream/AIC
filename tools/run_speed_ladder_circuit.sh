#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
# Preconditions: low-point-count scans measured and profile 1 already recovered.
bash "$repo/tools/run_speed_ladder_route.sh" --speed 1
for speed in 2 3 4; do
  echo "Testing final profile $speed"
  bash "$repo/tools/apply_speed_ladder.sh" "$speed"
done
