#!/usr/bin/env bash
set -eo pipefail
AIC_RECORD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONNOUSERSITE=1
export AIC_DRIVE_MODEL="${AIC_DRIVE_MODEL:-body_force}"
if [[ -z "${LLAMA_BIN:-}" ]]; then
  if [[ -x "$HOME/llama.cpp/build/bin/llama-server" ]]; then
    export LLAMA_BIN="$HOME/llama.cpp/build/bin/llama-server"
  else
    export LLAMA_BIN="$AIC_RECORD_ROOT/runtime/llama.cpp/build/bin/llama-server"
  fi
fi
if [[ -z "${MODEL:-}" ]]; then
  if [[ -r "$HOME/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf" ]]; then
    export MODEL="$HOME/llama.cpp/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf"
  else
    export MODEL="$AIC_RECORD_ROOT/runtime/models/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf"
  fi
fi
# The copied llama build has an absolute RUNPATH from the source machine.
# Select the chosen server's own libraries before those obsolete paths.
export LD_LIBRARY_PATH="$(dirname "$LLAMA_BIN")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ $# -eq 0 ]]; then set -- start --headless; fi
exec bash "$AIC_RECORD_ROOT/usb_start.sh" "$@"
