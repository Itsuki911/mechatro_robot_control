#!/usr/bin/env python3
"""Generate clean and noisy robot sensor CSV files for offline analysis."""

from __future__ import annotations

import csv
from pathlib import Path
import random


HEADER = [
    "time_ms",
    "state",
    "s1",
    "s2",
    "s3",
    "s4",
    "line_pos",
    "line_error",
    "straight_ms",
    "curve_ms",
    "distance_cm",
    "roll_deg",
    "servo_deg",
    "drive_speed",
    "estimated_distance_cm",
    "error_flags",
    "ultrasonic_ok",
    "mpu_ok",
    "color_ok",
    "all_zero",
    "event_code",
]


def segment(t: int) -> str:
    if t < 2500:
        return "CALIBRATION"
    if t < 6500:
        return "STRAIGHT_TRACE"
    if t < 9000:
        return "CURVE_TRACE_R"
    if t < 11500:
        return "CURVE_TRACE_L"
    if t < 12700:
        return "LINE_RECOVERY"
    if t < 14500:
        return "LINE_TRACE"
    if t < 16000:
        return "OBSTACLE_DETECTED"
    if t < 19000:
        return "LINE_TRACE"
    return "GOAL"


def row_at(t: int, noisy: bool) -> list[object]:
    seg = segment(t)
    s = [260, 260, 260, 260]
    state = seg
    line_pos = 0
    line_error = 0
    straight_ms = 0
    curve_ms = 0
    distance = 55.0
    roll = 1.0
    servo = 90
    speed = 0.0
    error_flags = 0
    all_zero = 0
    event = 0

    if seg == "CALIBRATION":
        state = "CALIBRATION"
    elif seg == "STRAIGHT_TRACE":
        s = [270, 780, 790, 280]
        straight_ms = t - 2500
        speed = 0.38
    elif seg == "CURVE_TRACE_R":
        state = "CURVE_TRACE"
        s = [260, 300, 760, 820]
        line_pos = 20
        line_error = 2
        curve_ms = t - 6500
        servo = 68
        speed = 0.22
    elif seg == "CURVE_TRACE_L":
        state = "CURVE_TRACE"
        s = [830, 770, 310, 260]
        line_pos = -20
        line_error = -2
        curve_ms = t - 9000
        servo = 112
        speed = 0.22
    elif seg == "LINE_RECOVERY":
        s = [220, 230, 225, 235]
        state = "LINE_RECOVERY" if t < 12200 else "BACKTRACK"
        line_error = -2
        servo = 90 if t < 12200 else 110
        speed = 0.16 if t < 12200 else -0.16
        event = 1 if t == 11500 else 3 if t == 12200 else 0
    elif seg == "LINE_TRACE":
        s = [260, 760, 780, 270]
        state = "LINE_TRACE"
        speed = 0.30
        event = 4 if t == 12700 else 0
    elif seg == "OBSTACLE_DETECTED":
        s = [260, 750, 760, 270]
        state = "OBSTACLE_DETECTED" if t < 15300 else "OBSTACLE_AVOIDANCE"
        distance = 12.0
        servo = 112
        speed = 0.0 if t < 15300 else 0.16
    elif seg == "GOAL":
        s = [830, 835, 828, 832]
        state = "GOAL" if t >= 19800 else "LINE_TRACE"
        speed = 0.0 if state == "GOAL" else 0.25
        event = 5 if t == 19000 else 6 if t == 19800 else 0

    estimated = max(0.0, (t - 2500) / 1000.0 * abs(speed) * 28.0)
    if noisy:
        s = [max(0, min(1023, value + random.randint(-35, 35))) for value in s]
        distance += random.uniform(-2.5, 2.5)
        roll += random.uniform(-2.0, 2.0)
        servo += random.randint(-3, 3)
        speed += random.uniform(-0.025, 0.025)
        if 11600 <= t <= 12100:
            all_zero = 1 if t % 200 == 0 else 0
            if all_zero:
                s = [0, 0, 0, 0]
                error_flags |= 1

    return [
        t,
        state,
        *s,
        line_pos,
        line_error,
        straight_ms,
        curve_ms,
        round(distance, 1),
        round(roll, 1),
        servo,
        round(speed, 3),
        round(estimated, 2),
        error_flags,
        1,
        1,
        0 if error_flags & 1 else 1,
        all_zero,
        event,
    ]


def write_csv(path: Path, noisy: bool) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)
        estimated = 0.0
        previous_t = 0
        for t in range(0, 20500, 100):
            row = row_at(t, noisy)
            dt = max(0, t - previous_t) / 1000.0
            previous_t = t
            estimated += abs(float(row[13])) * 28.0 * dt
            row[14] = round(estimated, 2)
            writer.writerow(row)


def main() -> int:
    out_dir = Path("data")
    out_dir.mkdir(exist_ok=True)
    random.seed(42)
    write_csv(out_dir / "dummy_sensor_clean.csv", noisy=False)
    write_csv(out_dir / "dummy_sensor_noisy.csv", noisy=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
