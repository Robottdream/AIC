#!/usr/bin/env bash
set -eo pipefail
AIC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
mkdir -p /tmp/aic_rebuild
if [[ -f "$AIC_ROOT/runtime/ros_install/setup.bash" ]]; then
  if [[ -e /tmp/aic_rebuild/install && ! -L /tmp/aic_rebuild/install ]]; then
    echo "请先移走已有的 /tmp/aic_rebuild/install 目录" >&2
    exit 1
  fi
  ln -sfn "$AIC_ROOT/runtime/ros_install" /tmp/aic_rebuild/install
elif [[ ! -f /tmp/aic_rebuild/install/setup.bash ]]; then
  # Build under an ASCII path: Gazebo plugins fail to load from this project's Chinese path.
  if [[ -L /tmp/aic_rebuild/install ]]; then
    rm /tmp/aic_rebuild/install
  fi
  colcon --log-base /tmp/aic_rebuild/log build \
    --base-paths "$AIC_ROOT/源码（国赛）/src" \
    --build-base /tmp/aic_rebuild/build \
    --install-base /tmp/aic_rebuild/install \
    --symlink-install
fi
source /tmp/aic_rebuild/install/setup.bash
# tf2_ros 0.25.23 can deadlock in asynchronous transform waits. Build the
# project-scoped repair only for that exact ROS ABI and preload it into Nav2.
TF2_VERSION="$(python3 - <<'PYVERSION'
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import xml.etree.ElementTree as ET
print(ET.parse(Path(get_package_share_directory('tf2_ros')) / 'package.xml').findtext('version'))
PYVERSION
)"
if [[ "$TF2_VERSION" == "0.25.23" ]]; then
  colcon --log-base /tmp/aic_tf2_compat/log build \
    --base-paths "$AIC_ROOT/源码（国赛）/src/aic_tf2_fix" \
    --build-base /tmp/aic_tf2_compat/build \
    --install-base /tmp/aic_tf2_compat/install \
    --event-handlers console_cohesion+ >/tmp/aic_tf2_compat_build.log 2>&1
  export AIC_TF2_FIX_LIB=/tmp/aic_tf2_compat/install/aic_tf2_fix/lib/libaic_tf2_fix.so
fi
set -u
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
exec python3 "$AIC_ROOT/scripts/benchmark_delivery.py" \
  --log-dir "$AIC_ROOT/运行日志/一键运行_$RUN_STAMP" \
  --timeout "${AIC_TIMEOUT:-380}"
