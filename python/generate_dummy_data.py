#!/usr/bin/env python3
"""Generate clean and noisy robot sensor CSV files for offline analysis.

The generated rows follow the same schema as Arduino CSV logs, so the analysis
script can be tested without connecting the robot.
"""

from __future__ import annotations

import csv
from pathlib import Path
import random


MM_PER_SEC_AT_FULL_PWM = 280.0
COURSE_DT_MS = 100

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

COURSE_HEADER = HEADER + [
    "estimated_distance_mm",
    "course_distance_mm",
    "motor_pwm",
    "s1_line",
    "s2_line",
    "s3_line",
    "s4_line",
    "ultrasonic_detected",
    "course_segment",
]


def segment(t: int) -> str:
    """Return the scenario segment name for a given timestamp in milliseconds."""
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
    """Build one CSV row for the current segment, optionally adding realistic noise."""
    seg = segment(t)
    # Sensor layout is diamond-shaped:
    # S1 front, S2 left, S3 right, S4 rear. High values mean white line.
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
        s = [780, 270, 280, 790]
        straight_ms = t - 2500
        speed = 0.38
    elif seg == "CURVE_TRACE_R":
        state = "CURVE_TRACE"
        s = [300, 270, 820, 760]
        line_pos = 20
        line_error = 2
        curve_ms = t - 6500
        servo = 68
        speed = 0.22
    elif seg == "CURVE_TRACE_L":
        state = "CURVE_TRACE"
        s = [760, 830, 300, 270]
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
        s = [770, 300, 310, 760]
        state = "LINE_TRACE"
        speed = 0.30
        event = 4 if t == 12700 else 0
    elif seg == "OBSTACLE_DETECTED":
        s = [760, 300, 310, 750]
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
    """Write one full dummy log and recompute estimated distance as a cumulative value."""
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


def line_pattern(name: str, progress: float) -> tuple[list[int], int, int, int, int]:
    """Return S1-S4 binary values plus line position, error, and servo angle."""
    if name == "straight":
        return [1, 0, 0, 1], 0, 0, 90, 0
    if name == "right_curve":
        return [1 if progress < 0.35 else 0, 0, 1, 1 if progress > 0.65 else 0], 30, 2, 68, 0
    if name == "left_curve":
        return [1 if progress < 0.35 else 0, 1, 0, 1 if progress > 0.65 else 0], -30, -2, 112, 0
    if name == "zigzag":
        if int(progress * 6) % 2 == 0:
            return [1, 0, 1, 0], 30, 2, 70, 0
        return [0, 1, 0, 1], -30, -2, 110, 0
    if name == "all_line":
        return [1, 1, 1, 1], 0, 0, 90, 0
    if name == "floor":
        return [0, 0, 0, 0], 0, 0, 90, 0
    return [0, 0, 0, 0], 0, 0, 90, 0


def raw_from_binary(bits: list[int], noisy: bool) -> list[int]:
    """Convert 0/1 line sensor values to Arduino-like analog readings."""
    raw = [790 if bit else 260 for bit in bits]
    if noisy:
        raw = [max(0, min(1023, value + random.randint(-35, 35))) for value in raw]
    return raw


def course_phases() -> list[dict[str, object]]:
    """Approximate course_page5.png as distance-based sections in millimeters."""
    return [
        {"name": "start_to_first_obstacle", "state": "STRAIGHT_TRACE", "pattern": "straight", "length": 340.0, "speed": 0.38},
        {"name": "first_obstacle_stop", "state": "OBSTACLE_DETECTED", "pattern": "straight", "duration": 450.0, "speed": 0.0, "ultrasonic": 1},
        {"name": "first_obstacle_avoid", "state": "OBSTACLE_AVOIDANCE", "pattern": "right_curve", "length": 160.0, "speed": 0.18, "ultrasonic": 1},
        {"name": "bottom_straight_to_right_curve", "state": "STRAIGHT_TRACE", "pattern": "straight", "length": 900.0, "speed": 0.40},
        {"name": "right_large_curve", "state": "CURVE_TRACE", "pattern": "right_curve", "length": 1500.0, "speed": 0.24},
        {"name": "upper_right_straight", "state": "STRAIGHT_TRACE", "pattern": "straight", "length": 900.0, "speed": 0.38},
        {"name": "upper_cross_line_not_goal", "state": "LINE_TRACE", "pattern": "all_line", "length": 120.0, "speed": 0.30},
        {"name": "upper_zigzag_30deg", "state": "CURVE_TRACE", "pattern": "zigzag", "length": 1000.0, "speed": 0.24},
        {"name": "second_obstacle_stop", "state": "OBSTACLE_DETECTED", "pattern": "straight", "duration": 450.0, "speed": 0.0, "ultrasonic": 1},
        {"name": "second_obstacle_avoid", "state": "OBSTACLE_AVOIDANCE", "pattern": "left_curve", "length": 160.0, "speed": 0.18, "ultrasonic": 1},
        {"name": "central_loop_entry", "state": "CURVE_TRACE", "pattern": "left_curve", "length": 550.0, "speed": 0.24},
        {"name": "central_r300_loop", "state": "CURVE_TRACE", "pattern": "left_curve", "length": 1885.0, "speed": 0.24},
        {"name": "central_loop_exit", "state": "STRAIGHT_TRACE", "pattern": "straight", "length": 500.0, "speed": 0.32},
        {"name": "upper_left_obstacle_stop", "state": "OBSTACLE_DETECTED", "pattern": "straight", "duration": 450.0, "speed": 0.0, "ultrasonic": 1},
        {"name": "upper_left_obstacle_avoid", "state": "OBSTACLE_AVOIDANCE", "pattern": "right_curve", "length": 160.0, "speed": 0.18, "ultrasonic": 1},
        {"name": "upper_left_straight", "state": "STRAIGHT_TRACE", "pattern": "straight", "length": 900.0, "speed": 0.38},
        {"name": "left_side_curve_down", "state": "CURVE_TRACE", "pattern": "left_curve", "length": 1100.0, "speed": 0.24},
        {"name": "line_gap_150mm_forward", "state": "LINE_RECOVERY", "pattern": "floor", "length": 35.0, "speed": 0.18},
        {"name": "line_gap_150mm_backtrack", "state": "BACKTRACK", "pattern": "floor", "duration": 350.0, "speed": -0.18},
        {"name": "line_gap_150mm_search", "state": "LINE_RECOVERY", "pattern": "floor", "length": 115.0, "speed": 0.18},
        {"name": "bottom_left_to_goal", "state": "STRAIGHT_TRACE", "pattern": "straight", "length": 700.0, "speed": 0.30},
        {"name": "goal_candidate", "state": "LINE_TRACE", "pattern": "all_line", "duration": 850.0, "speed": 0.10},
        {"name": "goal_confirmed", "state": "GOAL", "pattern": "all_line", "duration": 700.0, "speed": 0.0},
    ]


def phase_duration_ms(phase: dict[str, object]) -> float:
    """Calculate the phase duration from either explicit duration or length/speed."""
    if "duration" in phase:
        return float(phase["duration"])
    speed = abs(float(phase["speed"]))
    if speed == 0.0:
        return 0.0
    return float(phase["length"]) / (speed * MM_PER_SEC_AT_FULL_PWM) * 1000.0


def course_rows(noisy: bool) -> list[list[object]]:
    """Build rows that follow course_page5.png dimensions more closely."""
    rows: list[list[object]] = []
    estimated_mm = 0.0
    course_distance_mm = 0.0
    straight_ms = 0
    curve_ms = 0
    time_ms = 0

    calibration_rows = int(2500 / COURSE_DT_MS)
    for _ in range(calibration_rows):
        bits = [0, 0, 0, 0]
        raw = raw_from_binary(bits, noisy)
        rows.append([
            time_ms, "CALIBRATION", *raw, 0, 0, 0, 0, 80.0, 0.0, 90, 0.0,
            0.0, 0, 1, 1, 1, 0, 0, 0.0, 0.0, 0, *bits, 0, "calibration",
        ])
        time_ms += COURSE_DT_MS

    for phase in course_phases():
        duration = phase_duration_ms(phase)
        steps = max(1, int(round(duration / COURSE_DT_MS)))
        speed = float(phase["speed"])
        ultrasonic = int(phase.get("ultrasonic", 0))
        for step in range(steps):
            progress = step / max(1, steps - 1)
            bits, line_pos, line_error, servo, all_zero = line_pattern(str(phase["pattern"]), progress)
            raw = raw_from_binary(bits, noisy)

            if noisy:
                servo += random.randint(-3, 3)
                roll = random.uniform(-1.8, 1.8)
                speed_out = 0.0 if speed == 0.0 else speed + random.uniform(-0.018, 0.018)
            else:
                roll = 0.0
                speed_out = speed

            if phase["state"] in ("STRAIGHT_TRACE", "LINE_TRACE") and (bits[0] or bits[3]):
                straight_ms += COURSE_DT_MS
            else:
                straight_ms = 0
            if phase["state"] == "CURVE_TRACE" or bits[1] or bits[2]:
                curve_ms += COURSE_DT_MS
            else:
                curve_ms = 0

            dt_sec = COURSE_DT_MS / 1000.0
            travel_mm = abs(speed_out) * MM_PER_SEC_AT_FULL_PWM * dt_sec
            estimated_mm += travel_mm
            if speed_out >= 0.0:
                course_distance_mm += travel_mm
            else:
                course_distance_mm = max(0.0, course_distance_mm - travel_mm)

            if ultrasonic:
                distance_cm = 18.0 - min(8.0, progress * 8.0)
            else:
                distance_cm = 85.0
            if noisy:
                distance_cm += random.uniform(-1.5, 1.5)

            event_code = 0
            if phase["name"].startswith("line_gap_150mm") and step == 0:
                event_code = 1
            elif phase["name"] == "goal_candidate" and step == 0:
                event_code = 5
            elif phase["name"] == "goal_confirmed" and step == 0:
                event_code = 6

            rows.append([
                time_ms,
                phase["state"],
                *raw,
                line_pos,
                line_error,
                straight_ms,
                curve_ms,
                round(distance_cm, 1),
                round(roll, 1),
                max(58, min(122, servo)),
                round(speed_out, 3),
                round(estimated_mm / 10.0, 2),
                0,
                1,
                1,
                1,
                all_zero,
                event_code,
                round(estimated_mm, 1),
                round(course_distance_mm, 1),
                int(round(abs(speed_out) * 255)),
                *bits,
                ultrasonic,
                phase["name"],
            ])
            time_ms += COURSE_DT_MS
    return rows


def write_course_csv(path: Path, noisy: bool) -> None:
    """Write a course_page5-oriented CSV without removing older dummy files."""
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(COURSE_HEADER)
        writer.writerows(course_rows(noisy))


def main() -> int:
    """Generate both clean and noisy dummy CSV files under data/."""
    out_dir = Path("data")
    out_dir.mkdir(exist_ok=True)
    random.seed(42)
    write_csv(out_dir / "dummy_sensor_clean.csv", noisy=False)
    write_csv(out_dir / "dummy_sensor_noisy.csv", noisy=True)
    write_course_csv(out_dir / "course_page5_sensor_clean.csv", noisy=False)
    write_course_csv(out_dir / "course_page5_sensor_noisy.csv", noisy=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
