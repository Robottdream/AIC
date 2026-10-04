#!/usr/bin/env bash
# Compatibility alias for the revised round-omni model.
set -euo pipefail
exec bash "$(cd "$(dirname "$0")" && pwd)/ros_round_omni_start.sh" "$@"
