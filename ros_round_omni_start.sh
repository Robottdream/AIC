#!/usr/bin/env bash
# Round chassis, four omni wheels, arm pedestal alignment; isolated workspace.
set -euo pipefail
# Retain the established launch/config identifier for compatibility.
export AIC_ROBOT_MODEL=mecanum
export AIC_ARM_ALIGNMENT=1
export AIC_DYNAMIC_GUARD="${AIC_DYNAMIC_GUARD:-1}"
export AIC_ODOM_MAP=1
exec bash "$(cd "$(dirname "$0")" && pwd)/ros_competition_start.sh" "$@"
