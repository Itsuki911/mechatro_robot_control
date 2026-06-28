#!/usr/bin/env bash
# Upload the main sketch to an Arduino board through a user-specified serial port.
set -euo pipefail

# Examples:
#   scripts/arduino_upload.sh /dev/cu.usbmodemXXXX
#   PORT=/dev/cu.usbmodemXXXX scripts/arduino_upload.sh

FQBN="${FQBN:-arduino:avr:uno}"
SKETCH_DIR="${SKETCH_DIR:-arduino/mechatro_robot_control}"
PORT="${1:-${PORT:-}}"

if [[ -z "$PORT" ]]; then
  echo "PORT is required. Pass as first argument or set PORT=/dev/cu.usbmodemXXXX." >&2
  exit 1
fi

arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"
