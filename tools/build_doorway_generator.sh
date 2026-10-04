#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_doorway_evidence_20261003
files=(src/yzbot/bot_navigation/CMakeLists.txt src/yzbot/bot_navigation/package.xml
 src/yzbot/bot_navigation/predictive_critic.xml src/yzbot/bot_navigation/src/fine_omni_trajectory_generator.cpp
 src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml
 src/yzbot/bot_navigation/param/originbot_nav2_mecanum.yaml)
for file in "${files[@]}"; do
  mkdir -p "$evidence/before-files/$(dirname "$file")"
  if [ -f "$target/$file" ] && [ ! -f "$evidence/before-files/$file" ]; then cp "$target/$file" "$evidence/before-files/$file"; fi
  cp "$repo/$file" "$target/$file"
done
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
cd "$target"
colcon build --base-paths src --packages-select bot_navigation --symlink-install --cmake-args -DBUILD_TESTING=OFF > "$evidence/generator-build.log" 2>&1
tail -n 12 "$evidence/generator-build.log"
bash "$target/ros_round_omni_start.sh" recover --headless > "$evidence/generator-recover.log" 2>&1
source "$target/tools/ros_env.sh"
cp "$evidence/final-approach.json" "$evidence/approach-21-samples-stall.json"
python3 "$repo/tools/doorway_navigation_probe.py" --label final-approach
