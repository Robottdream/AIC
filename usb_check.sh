#!/usr/bin/env bash
set -eo pipefail
AIC_USB_CHECK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONNOUSERSITE=1
[[ -r /opt/ros/humble/setup.bash ]] || { echo 'ROS 2 Humble missing.' >&2; exit 1; }
source /opt/ros/humble/setup.bash
python3 - "$AIC_USB_CHECK_ROOT" <<'PY'
import importlib
import os
from pathlib import Path
import shutil
import sys
from ament_index_python.packages import get_package_prefix
root = Path(sys.argv[1])
failed = []
for name in ('colcon', 'cmake', 'c++', 'rsync', 'gazebo'):
    present = shutil.which(name)
    print(f'{name}: {present or "MISSING"}')
    if not present:
        failed.append(name)
for name in ('nav2_bringup', 'gazebo_ros', 'gazebo_dev', 'gazebo_ros2_control',
             'controller_manager', 'moveit_ros_move_group', 'moveit_configs_utils',
             'rosbridge_server', 'rosapi', 'cv_bridge', 'tf2_ros'):
    try:
        print(f'{name}: {get_package_prefix(name)}')
    except LookupError:
        failed.append(name)
        print(f'{name}: MISSING')
for name in ('requests', 'numpy', 'cv2', 'tornado', 'cbor2', 'rclpy'):
    try:
        importlib.import_module(name)
        print(f'Python {name}: OK')
    except Exception as exc:
        print(f'Python {name}: {exc}')
        failed.append(name)
binary = Path(os.environ.get('LLAMA_BIN', str(Path.home()/'llama.cpp/build/bin/llama-server')))
model = Path(os.environ.get('MODEL', str(Path.home()/'llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf')))
for p in (binary, model):
    present = p.is_file() and os.access(p, os.X_OK if p == binary else os.R_OK)
    print(f'{p}: {"OK" if present else "MISSING"}')
    if not present:
        failed.append(str(p))
free = shutil.disk_usage(root).free / 1024**3
print(f'Bundle filesystem available: {free:.2f} GiB')
if free < 5:
    print('Space warning: expand the USB system before building and recording; reserve at least 5 GiB.')
print('Recorder ffmpeg:', shutil.which('ffmpeg') or 'not found; install before recording')
print('Foxglove:', '/opt/Foxglove/foxglove-studio' if Path('/opt/Foxglove/foxglove-studio').is_file()
      else 'not found at default path; installers/ contains the desktop installer')
if failed:
    print('Missing runtime/build requirements:', ', '.join(failed))
    sys.exit(1)
print('Basic environment checks passed; this is not a completed delivery test.')
PY
