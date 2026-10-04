#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_speed_ladder_20261003
mkdir -p "$evidence"
python3 "$repo/tools/prepare_speed_ladder.py"
for file in src/path_follow_critic.cpp src/fine_omni_trajectory_generator.cpp scripts/scan_velocity_guard.py; do
  rel=src/yzbot/bot_navigation/$file
  mkdir -p "$evidence/before-files/$(dirname "$rel")"
  if [ ! -f "$evidence/before-files/$rel" ]; then cp "$target/$rel" "$evidence/before-files/$rel"; fi
  cp "$repo/$rel" "$target/$rel"
done
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
cd "$target"
colcon build --base-paths src --packages-select bot_navigation --symlink-install --cmake-args -DBUILD_TESTING=OFF > "$evidence/build.log" 2>&1
tail -n 8 "$evidence/build.log"
