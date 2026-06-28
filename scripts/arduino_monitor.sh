#!/usr/bin/env bash
set -euo pipefail

# Examples:
#   scripts/arduino_monitor.sh /dev/cu.usbmodemXXXX
#   PORT=/dev/cu.usbmodemXXXX BAUD=115200 scripts/arduino_monitor.sh

PORT="${1:-${PORT:-}}"
BAUD="${BAUD:-115200}"

if [[ -z "$PORT" ]]; then
  echo "PORT is required. Pass as first argument or set PORT=/dev/cu.usbmodemXXXX." >&2
  exit 1
fi

arduino-cli monitor -p "$PORT" -c "baudrate=$BAUD"
