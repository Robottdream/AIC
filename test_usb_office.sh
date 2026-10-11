#!/usr/bin/env bash
set -eo pipefail
AIC_SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$AIC_SOURCE_ROOT/test_map.sh" usb_office_20261007 "$@"
