#!/usr/bin/env bash
# Use the current scene and the model already provided by the USB system.
set -eo pipefail
AIC_BUNDLE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
  echo 'Usage: bash usb_start.sh [check|build|start|restart|stop|status|logs] [--headless]'
  echo 'Examples: bash usb_start.sh start --headless; bash usb_start.sh --headless'
  exit 0
fi
AIC_USB_ACTION=start
if [[ $# -gt 0 && "$1" != -* ]]; then
  AIC_USB_ACTION="$1"
  shift
fi
case "$AIC_USB_ACTION" in
  check) exec bash "$AIC_BUNDLE_ROOT/usb_check.sh" ;;
  start|restart|build|stop|status|logs) ;;
  *) echo 'Usage: bash usb_start.sh [check|build|start|restart|stop|status|logs] [--headless]' >&2; exit 2 ;;
esac
export PYTHONNOUSERSITE=1
export AIC_ROSBRIDGE_PORT="${AIC_ROSBRIDGE_PORT:-8765}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export LLAMA_BIN="${LLAMA_BIN:-$HOME/llama.cpp/build/bin/llama-server}"
export MODEL="${MODEL:-$HOME/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf}"
if [[ "$AIC_USB_ACTION" == start || "$AIC_USB_ACTION" == restart ]]; then
  [[ -x "$LLAMA_BIN" ]] || { echo "Missing llama-server: $LLAMA_BIN" >&2; exit 1; }
  [[ -r "$MODEL" ]] || { echo "Missing model: $MODEL" >&2; exit 1; }
fi
# Always pass an action: test_map.sh without one starts its benchmark.
exec bash "$AIC_BUNDLE_ROOT/test_map.sh" short_routes_20261007 "$AIC_USB_ACTION" "$@"
