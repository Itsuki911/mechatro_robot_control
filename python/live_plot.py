#!/usr/bin/env python3
"""Minimal live plot for Arduino CSV output.

This is intentionally small: it watches the live serial CSV stream and shows
S1-S4, servo angle, and drive speed for quick bench testing.
"""

from __future__ import annotations

import argparse
import csv
from collections import deque

import matplotlib.pyplot as plt
import serial


def main() -> int:
    """Read Arduino CSV rows from serial and refresh a rolling matplotlib plot."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=int, default=200)
    args = parser.parse_args()

    times = deque(maxlen=args.window)
    sensors = {key: deque(maxlen=args.window) for key in ("s1", "s2", "s3", "s4")}
    servo = deque(maxlen=args.window)
    speed = deque(maxlen=args.window)
    header = None

    plt.ion()
    fig, ax = plt.subplots()
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            parts = next(csv.reader([line]))
            if parts[0] == "time_ms":
                header = parts
                continue
            if header is None:
                continue
            row = dict(zip(header, parts))
            times.append(float(row["time_ms"]) / 1000.0)
            for key in sensors:
                sensors[key].append(float(row[key]))
            servo.append(float(row["servo_deg"]))
            speed.append(float(row["drive_speed"]) * 1000.0)
            ax.clear()
            for key, values in sensors.items():
                ax.plot(times, values, label=key)
            ax.plot(times, servo, label="servo")
            ax.plot(times, speed, label="drive_speed x1000")
            ax.legend()
            ax.set_xlabel("time [s]")
            plt.pause(0.01)


if __name__ == "__main__":
    raise SystemExit(main())
