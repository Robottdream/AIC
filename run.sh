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
set -u
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
exec python3 "$AIC_ROOT/scripts/benchmark_delivery.py" \
  --log-dir "$AIC_ROOT/运行日志/一键运行_$RUN_STAMP" \
  --timeout "${AIC_TIMEOUT:-380}"
