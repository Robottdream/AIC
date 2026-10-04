#!/usr/bin/env bash
set -euo pipefail
target="$HOME/ws_aic_mecanum_20261002"
mkdir -p "$target"
rsync -a "$HOME/ws_aic/src/" "$target/src/"
rsync -a "$HOME/ws_aic/tools/" "$target/tools/"
cp "$HOME/ws_aic/ros_competition_start.sh" "$target/"
if [ ! -e "$target/runtime" ]; then ln -s "$HOME/ws_aic/runtime" "$target/runtime"; fi
python3 - <<'PY'
from pathlib import Path
base=Path('/mnt/c/Users/21920/.codex/worktrees/5a40/AIC')
other=Path.home()/'ws_aic'
for name in ['ros_competition_start.sh','src/yzbot/mybot_description/urdf/large_base.xacro','src/yzbot/mybot_description/urdf/arm.xacro','src/yzbot/bot_navigation/param/originbot_nav2_odom.yaml']:
    print(name, 'same' if (base/name).read_text()==(other/name).read_text() else 'DIFFERENT')
PY
