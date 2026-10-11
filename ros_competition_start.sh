#!/usr/bin/env bash
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"
case "${1:-start}" in
  -h|--help) echo 'Usage: ./ros_competition_start.sh start|stop|status|restart|logs|build|benchmark [--headless]'; exit 0 ;;
  stop|status|logs) exec python3 "$WS/tools/project_launcher.py" "$@" ;;
esac
export PYTHONNOUSERSITE=1
# WSL restarts can leave Fast DDS shared-memory ports locked. Respect explicit
# middleware/profile choices, otherwise use UDP for the default Fast DDS stack.
if [[ -n "${WSL_DISTRO_NAME:-}" && "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" == rmw_fastrtps_cpp && -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/tools/fastdds_wsl_udp.xml"
fi
# WSL inherits Windows PATH; CMake otherwise searches Windows SDK trees via DrvFs.
export PATH="$(python3 -c 'import os; print(":".join(p for p in os.environ["PATH"].split(":") if not p.startswith(("/mnt/c/", "/mnt/d/", "/mnt/e/"))))')"
source /opt/ros/humble/setup.bash
if [[ "${1:-}" == build || ! -f "$WS/install/setup.bash" ]]; then
  colcon build --symlink-install --base-paths "$WS/src"
fi
source "$WS/install/setup.bash"
TF2_VERSION="$(python3 -c 'from ament_index_python.packages import get_package_share_directory as g; import xml.etree.ElementTree as E; print(E.parse(g("tf2_ros")+"/package.xml").findtext("version"))')"
if [[ "$TF2_VERSION" == 0.25.23 ]]; then
  export AIC_TF2_FIX_LIB="$WS/install/aic_tf2_fix/lib/libaic_tf2_fix.so"
  test -f "$AIC_TF2_FIX_LIB" || { echo 'TF2 compatibility library missing; run build.' >&2; exit 1; }
fi
[[ "${1:-}" != build ]] || exit 0
if [[ "${1:-}" == benchmark ]]; then
  exec python3 "$WS/tools/benchmark_delivery.py" --log-dir "$WS/log/benchmark/$(date +%Y%m%d_%H%M%S)" --timeout "${AIC_TIMEOUT:-380}"
fi
exec python3 "$WS/tools/project_launcher.py" "$@"
