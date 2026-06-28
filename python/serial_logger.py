#!/usr/bin/env python3
"""Save Arduino CSV serial output into logs/ with a timestamped filename."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys
import time

import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Arduino CSV serial logger")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/cu.usbmodemXXXX")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    parser.add_argument("--seconds", type=float, default=0, help="Stop after N seconds. 0 means until Ctrl-C.")
    parser.add_argument("--out-dir", default="logs", help="Output directory")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"robot_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    header: list[str] | None = None
    started = time.monotonic()
    with serial.Serial(args.port, args.baud, timeout=1) as ser, path.open("w", newline="") as f:
        writer = None
        print(f"logging to {path}", file=sys.stderr)
        while True:
            if args.seconds and time.monotonic() - started >= args.seconds:
                break
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw or raw.startswith("#"):
                continue
            parts = next(csv.reader([raw]))
            if parts and parts[0] == "time_ms":
                header = parts
                writer = csv.writer(f)
                writer.writerow(header)
                f.flush()
                continue
            if writer is None:
                continue
            writer.writerow(parts)
            f.flush()
            print(raw)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
