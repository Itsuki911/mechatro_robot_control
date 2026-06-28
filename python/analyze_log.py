#!/usr/bin/env python3
"""Analyze robot CSV logs from real serial captures or data/dummy_sensor_*.csv."""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path


REQUIRED_COLUMNS = [
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
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze mechatro robot CSV logs")
    parser.add_argument("csv_path", help="CSV path, e.g. data/dummy_sensor_clean.csv")
    parser.add_argument("--out-dir", default="logs/analysis", help="Directory for PNG graphs")
    parser.add_argument("--no-plot", action="store_true", help="Only print summaries")
    return parser.parse_args()


def load_log(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        rows = [row for row in csv.DictReader(line for line in f if not line.startswith("#"))]
    if not rows:
        raise ValueError("no CSV rows found")
    missing = [col for col in REQUIRED_COLUMNS if col not in rows[0]]
    if missing:
        raise ValueError(f"missing columns: {', '.join(missing)}")
    return rows


def as_float(row: dict[str, str], key: str) -> float:
    return float(row[key])


def print_summary(rows: list[dict[str, str]]) -> None:
    time_values = [as_float(row, "time_ms") for row in rows]
    states = Counter(row["state"] for row in rows)
    errors = Counter(row["error_flags"] for row in rows)
    print("rows:", len(rows))
    print("duration_sec:", round((max(time_values) - min(time_values)) / 1000.0, 2))
    print("state_counts:")
    for state, count in states.most_common():
        print(f"{state}: {count}")
    print("max_straight_ms:", int(max(as_float(row, "straight_ms") for row in rows)))
    print("max_curve_ms:", int(max(as_float(row, "curve_ms") for row in rows)))
    print("final_estimated_distance_cm:", round(as_float(rows[-1], "estimated_distance_cm"), 2))
    print("error_flag_counts:")
    for flag, count in sorted(errors.items(), key=lambda item: int(item[0])):
        print(f"{flag}: {count}")


def plot_log(rows: list[dict[str, str]], out_dir: Path, stem: str) -> None:
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    t = [as_float(row, "time_ms") / 1000.0 for row in rows]

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for key in ["s1", "s2", "s3", "s4"]:
        axes[0].plot(t, [as_float(row, key) for row in rows], label=key)
    axes[0].set_ylabel("color raw")
    axes[0].legend(ncol=4)

    axes[1].plot(t, [as_float(row, "servo_deg") for row in rows], label="servo_deg")
    axes[1].plot(t, [as_float(row, "drive_speed") for row in rows], label="drive_speed")
    axes[1].set_ylabel("actuator")
    axes[1].legend()

    axes[2].plot(t, [as_float(row, "distance_cm") for row in rows], label="distance_cm")
    axes[2].plot(t, [as_float(row, "estimated_distance_cm") for row in rows], label="estimated_distance_cm")
    axes[2].set_ylabel("cm")
    axes[2].legend()

    axes[3].plot(t, [as_float(row, "straight_ms") for row in rows], label="straight_ms")
    axes[3].plot(t, [as_float(row, "curve_ms") for row in rows], label="curve_ms")
    axes[3].plot(t, [as_float(row, "roll_deg") for row in rows], label="roll_deg")
    axes[3].set_xlabel("time [s]")
    axes[3].legend()

    fig.tight_layout()
    fig.savefig(out_dir / f"{stem}_timeseries.png", dpi=150)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    path = Path(args.csv_path)
    rows = load_log(path)
    print_summary(rows)
    if not args.no_plot:
        plot_log(rows, Path(args.out_dir), path.stem)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
