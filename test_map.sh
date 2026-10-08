#!/usr/bin/env bash
set -eo pipefail
AIC_SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AIC_SCENE_NAME="${1:-short_routes_20261007}"
case "$AIC_SCENE_NAME" in
  short_routes_20261007|usb_office_20261007) ;;
  *) echo 'Unknown local scene' >&2; exit 2 ;;
esac
if [[ $# -gt 0 ]]; then shift; fi
if [[ $# -eq 0 ]]; then set -- benchmark; fi
case "$1" in
  benchmark|build|stop|status|logs) ;;
  *) echo 'Usage: bash test_map.sh SCENE [benchmark|build|stop|status|logs]' >&2; exit 2 ;;
esac
AIC_TEST_WORKSPACE="/tmp/aic-map-$AIC_SCENE_NAME"
AIC_SCENE_ROOT="$AIC_SOURCE_ROOT/scenarios/$AIC_SCENE_NAME"
mkdir -p "$AIC_TEST_WORKSPACE/src" "$AIC_TEST_WORKSPACE/tools" "$AIC_TEST_WORKSPACE/scene"
rsync -a "$AIC_SOURCE_ROOT/src/" "$AIC_TEST_WORKSPACE/src/"
rsync -a "$AIC_SOURCE_ROOT/tools/" "$AIC_TEST_WORKSPACE/tools/"
rsync -a "$AIC_SCENE_ROOT/" "$AIC_TEST_WORKSPACE/scene/"
cp "$AIC_SOURCE_ROOT/ros_competition_start.sh" "$AIC_TEST_WORKSPACE/"
# Stage scene files under an ASCII path: Nav2's generated YAML escapes
# non-ASCII directory names and otherwise cannot open a Chinese desktop path.
export AIC_WORLD="$AIC_TEST_WORKSPACE/scene/office_test.world"
export AIC_MAP="$AIC_TEST_WORKSPACE/scene/mapn3.yaml"
export AIC_TASK_CONFIG="$AIC_TEST_WORKSPACE/scene/tasks.yaml"
if [[ -f "$AIC_TEST_WORKSPACE/scene/nav2.yaml" ]]; then
  export AIC_NAV_PARAMS="$AIC_TEST_WORKSPACE/scene/nav2.yaml"
else
  unset AIC_NAV_PARAMS
fi
if [[ "$1" == benchmark ]]; then
  bash "$AIC_TEST_WORKSPACE/ros_competition_start.sh" build
fi
exec bash "$AIC_TEST_WORKSPACE/ros_competition_start.sh" "$@"
