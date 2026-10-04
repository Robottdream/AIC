#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_path_following_evidence_20261003
files=(src/yzbot/bot_navigation/CMakeLists.txt
 src/yzbot/bot_navigation/predictive_critic.xml src/yzbot/bot_navigation/src/path_follow_critic.cpp
 src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml
 src/yzbot/bot_navigation/param/originbot_nav2_mecanum.yaml)
for file in "${files[@]}"; do
  mkdir -p "$evidence/before-files/$(dirname "$file")"
  if [ -f "$target/$file" ] && [ ! -f "$evidence/before-files/$file" ]; then
    cp "$target/$file" "$evidence/before-files/$file"
  fi
  cp "$repo/$file" "$target/$file"
done
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
cd "$target"
colcon build --base-paths src --packages-select bot_navigation --symlink-install --cmake-args -DBUILD_TESTING=OFF > "$evidence/build.log" 2>&1
tail -n 12 "$evidence/build.log"
