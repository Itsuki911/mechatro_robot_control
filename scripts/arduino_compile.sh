#!/usr/bin/env bash
# Compile the main Arduino UNO sketch with Arduino CLI.
set -euo pipefail

# Example:
#   scripts/arduino_compile.sh

FQBN="${FQBN:-arduino:avr:uno}"
SKETCH_DIR="${SKETCH_DIR:-arduino/mechatro_robot_control}"

arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"
