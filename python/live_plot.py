#!/usr/bin/env python3
"""Minimal live plot for Arduino CSV output."""

from __future__ import annotations

import argparse
import csv
from collections import deque

import matplotlib.pyplot as plt
import serial


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=int, default=200)
    args = parser.parse_args()

    times = deque(maxlen=args.window)
    s2 = deque(maxlen=args.window)
    s3 = deque(maxlen=args.window)
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
            s2.append(float(row["s2"]))
            s3.append(float(row["s3"]))
            servo.append(float(row["servo_deg"]))
            speed.append(float(row["drive_speed"]) * 1000.0)
            ax.clear()
            ax.plot(times, s2, label="s2")
            ax.plot(times, s3, label="s3")
            ax.plot(times, servo, label="servo")
            ax.plot(times, speed, label="drive_speed x1000")
            ax.legend()
            ax.set_xlabel("time [s]")
            plt.pause(0.01)


if __name__ == "__main__":
    raise SystemExit(main())
