#!/usr/bin/env bash
set -eo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_predictive_planner_evidence_20261003
mkdir -p "$evidence/before" "$target/src/yzbot/bot_navigation/src"
files=(src/yzbot/bot_navigation/CMakeLists.txt src/yzbot/bot_navigation/package.xml
 src/yzbot/bot_navigation/predictive_critic.xml
 src/yzbot/bot_navigation/known_trajectory_layer.xml
 src/yzbot/bot_navigation/src/known_trajectory_layer.cpp
 src/yzbot/bot_navigation/src/prediction_math.hpp
 src/yzbot/bot_navigation/src/predictive_obstacle_critic.cpp
 src/yzbot/bot_navigation/scripts/scan_velocity_guard.py
 src/yzbot/bot_navigation/scripts/known_obstacle_forecaster.py
 src/yzbot/bot_navigation/launch/nav_bringup_gazebo.launch.py
 src/yzbot/bot_navigation/param/originbot_nav2_odom_mecanum.yaml
 src/yzbot/bot_navigation/param/originbot_nav2_mecanum.yaml demo1.json)
for file in "${files[@]}"; do
  if [ -f "$target/$file" ] && [ ! -f "$evidence/before/$file" ]; then
    mkdir -p "$evidence/before/$(dirname "$file")"
    cp "$target/$file" "$evidence/before/$file"
  fi
  cp "$repo/$file" "$target/$file"
done
python3 - "$target/ros_competition_start.sh" "$evidence/before/ros_competition_start.sh" <<'PY'
from pathlib import Path
import sys
p, backup = map(Path, sys.argv[1:])
s = p.read_text()
needle = 'NAV2_BIN_PATTERNS="/bot_navigation/scan_velocity_guard.py '
if '/bot_navigation/known_obstacle_forecaster.py' not in s:
    assert needle in s
    if not backup.exists(): backup.write_bytes(p.read_bytes())
    p.write_text(s.replace(needle, needle+'/bot_navigation/known_obstacle_forecaster.py '))
PY
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
source "$target/install/setup.bash"
cd "$target"
colcon build --base-paths src --packages-select bot_navigation --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF > "$evidence/build.log" 2>&1
tail -12 "$evidence/build.log"
python3 -m py_compile "$target/src/yzbot/bot_navigation/scripts/scan_velocity_guard.py"
