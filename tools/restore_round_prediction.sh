#!/usr/bin/env bash
set -euo pipefail
repo=/mnt/c/Users/21920/.codex/worktrees/5a40/AIC
target=/home/polarbear/ws_aic_mecanum_20261002
evidence=/home/polarbear/aic_round_prediction_evidence_20261003
mkdir -p "$evidence"
cp "$repo/ros_round_omni_start.sh" "$target/ros_round_omni_start.sh"
cp "$repo/src/yzbot/bot_navigation/launch/nav_bringup_gazebo.launch.py" "$target/src/yzbot/bot_navigation/launch/nav_bringup_gazebo.launch.py"
cp "$repo/src/yzbot/bot_navigation/scripts/scan_velocity_guard.py" "$target/src/yzbot/bot_navigation/scripts/scan_velocity_guard.py"
# Install only the explicit navigation recovery command in the native script.
python3 - "$target/ros_competition_start.sh" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]); s=p.read_text()
if 'recover)  load_env; nav2_recover' not in s:
    a='start|stop|status|restart|logs|doctor|watchdog)'
    assert a in s
    s=s.replace(a,'start|stop|status|restart|recover|logs|doctor|watchdog)')
    a='  restart)  stop_all; echo; start_all ;;'
    assert a in s
    s=s.replace(a,a+'\n  recover)  load_env; nav2_recover ;;')
    p.write_text(s)
PY
cd "$target"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
source tools/ros_env.sh
python3 "$repo/tools/check_dynamic_tracker.py" > "$evidence/math-check.log" 2>&1
python3 "$repo/tools/check_mecanum_math.py" >> "$evidence/math-check.log" 2>&1
bash ros_round_omni_start.sh recover --headless > "$evidence/recover.log" 2>&1
