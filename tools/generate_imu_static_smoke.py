#!/usr/bin/env python3
"""Generate a short *synthetic* static IMU CSV to smoke-test analyze_allan.py.

NOT for publishing ARW/BI in README — rates are hand-picked noise, not a DUT.
Real fit requires hours of quiet hardware → docs/imu_static_log.csv (see RUNBOOK).
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
DEFAULT_OUT = REPO / "docs" / "allan" / "smoke" / "imu_static_smoke_60s.csv"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    n = int(args.seconds * args.hz)
    dt_ms = 1000.0 / args.hz

    # Mild white + slow bias wander (illustrative only)
    gyro = rng.normal(0.0, 0.002, size=(n, 3))
    accel = rng.normal(0.0, 0.05, size=(n, 3))
    accel[:, 2] += 9.80665
    t = np.arange(n, dtype=np.float64) * dt_ms
    wander = 0.0002 * np.sin(2.0 * math.pi * t / 20000.0)
    gyro[:, 2] += wander

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            ["time_ms", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]
        )
        for i in range(n):
            w.writerow(
                [
                    f"{t[i]:.3f}",
                    f"{accel[i, 0]:.8f}",
                    f"{accel[i, 1]:.8f}",
                    f"{accel[i, 2]:.8f}",
                    f"{gyro[i, 0]:.8f}",
                    f"{gyro[i, 1]:.8f}",
                    f"{gyro[i, 2]:.8f}",
                ]
            )

    print(f"Wrote {args.out} ({n} samples @ {args.hz} Hz, {args.seconds}s)")
    print("Smoke only — do not paste ARW/BI into Evidence scorecard.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
