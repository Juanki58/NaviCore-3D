#!/usr/bin/env python3
"""Verifica identidad fisica Android: Acc ~ Gravity + LinearAccel en data/real_run."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = REPO_ROOT / "data" / "real_run"
GRAVITY_MPS2 = 9.80665
OUT_JSON = REPO_ROOT / "docs" / "benchmarks" / "android_signal_identity_audit.json"

G_UNITS = GRAVITY_MPS2


@dataclass
class Series:
    name: str
    times: np.ndarray
    xyz: np.ndarray  # columns x,y,z sensor frame as stored in CSV


def load_series(path: Path, name: str) -> Series:
    times: list[float] = []
    rows: list[list[float]] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t = raw.get("seconds_elapsed") or raw.get("time")
            if t is None:
                continue
            if raw.get("seconds_elapsed"):
                ts = float(raw["seconds_elapsed"])
            else:
                ts = float(raw["time"]) * 1e-9
            x = float(raw["x"])
            y = float(raw["y"])
            z = float(raw["z"])
            times.append(ts)
            rows.append([x, y, z])
    if not rows:
        raise ValueError(f"empty: {path}")
    return Series(name=name, times=np.array(times, dtype=float), xyz=np.array(rows, dtype=float))


def interp_at(query_t: np.ndarray, series: Series) -> np.ndarray:
    out = np.zeros((query_t.size, 3), dtype=float)
    for axis in range(3):
        out[:, axis] = np.interp(query_t, series.times, series.xyz[:, axis])
    return out


def summarize_residual(name: str, residual: np.ndarray) -> dict:
    norm = np.linalg.norm(residual, axis=1)
    return {
        "label": name,
        "n": int(norm.size),
        "norm_mean_mps2": float(np.mean(norm)),
        "norm_median_mps2": float(np.median(norm)),
        "norm_p95_mps2": float(np.percentile(norm, 95)),
        "norm_max_mps2": float(np.max(norm)),
        "component_std_mps2": [float(np.std(residual[:, i])) for i in range(3)],
    }


def window_stats(residual_norm: np.ndarray, times: np.ndarray, t0: float, t1: float) -> dict:
    mask = (times >= t0) & (times <= t1)
    if not np.any(mask):
        return {"n": 0}
    seg = residual_norm[mask]
    return {
        "n": int(seg.size),
        "mean_mps2": float(np.mean(seg)),
        "median_mps2": float(np.median(seg)),
        "p95_mps2": float(np.percentile(seg, 95)),
    }


def main() -> int:
    uncal = load_series(DATA_DIR / "AccelerometerUncalibrated.csv", "uncalibrated")
    total = load_series(DATA_DIR / "TotalAcceleration.csv", "total")
    gravity = load_series(DATA_DIR / "Gravity.csv", "gravity")
    linear_g = load_series(DATA_DIR / "Accelerometer.csv", "linear_g")

    # Use uncalibrated timeline as reference (replay input)
    t = uncal.times
    g = interp_at(t, gravity)
    lin_g = interp_at(t, linear_g)
    tot = interp_at(t, total)

    # Hypothesis A: all m/s² in sensor frame, Acc = Gravity + Linear(m/s²)
    lin_mps2_a = lin_g
    recon_a = g + lin_mps2_a
    res_a = uncal.xyz - recon_a

    # Hypothesis B: Accelerometer.csv in g -> m/s²
    lin_mps2_b = lin_g * G_UNITS
    recon_b = g + lin_mps2_b
    res_b = uncal.xyz - recon_b

    # Uncal vs Total (Sensor Logger naming)
    res_uncal_total = uncal.xyz - tot

    # Gravity norm check
    g_norm = np.linalg.norm(g, axis=1)

    report = {
        "dataset": str(DATA_DIR),
        "metadata": {
            "app": "Sensor Logger 1.61.0",
            "platform": "android 36",
            "device": "CPH2791",
            "sensors_recorded": "Accelerometer|Gravity|...|TotalAcceleration|AccelerometerUncalibrated",
        },
        "column_order": "x,y,z as in CSV (Android device frame)",
        "units_assumption_tests": {
            "uncalibrated_equals_total_acceleration": summarize_residual(
                "uncal - total", res_uncal_total
            ),
            "identity_gravity_plus_linear_mps2": summarize_residual(
                "uncal - (gravity + linear_as_mps2)", res_a
            ),
            "identity_gravity_plus_linear_in_g": summarize_residual(
                "uncal - (gravity + linear_g * 9.80665)", res_b
            ),
        },
        "gravity_vector_norm": {
            "mean_mps2": float(np.mean(g_norm)),
            "std_mps2": float(np.std(g_norm)),
            "median_mps2": float(np.median(g_norm)),
        },
        "regimes": {},
    }

    best_key = min(
        (
            "identity_gravity_plus_linear_mps2",
            "identity_gravity_plus_linear_in_g",
        ),
        key=lambda k: report["units_assumption_tests"][k]["norm_median_mps2"],
    )
    best_res = res_b if best_key.endswith("_in_g") else res_a
    best_norm = np.linalg.norm(best_res, axis=1)

    for label, t0, t1 in [
        ("static_0_30s", 0.0, 30.0),
        ("motion_2_10s", 2.0, 10.0),
        ("cruise_11_25s", 11.4, 25.4),
        ("full", 0.0, float(t[-1])),
    ]:
        report["regimes"][label] = window_stats(best_norm, t, t0, t1)

    report["best_identity_model"] = best_key

    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with OUT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")

    print("=" * 72)
    print(" Android signal identity audit (Patron Oro)")
    print("=" * 72)
    print(f"  uncal == total?  median |diff| = {report['units_assumption_tests']['uncalibrated_equals_total_acceleration']['norm_median_mps2']:.6f} m/s2")
    print(f"  |gravity| mean   = {report['gravity_vector_norm']['mean_mps2']:.4f} m/s2")
    for key in (
        "identity_gravity_plus_linear_mps2",
        "identity_gravity_plus_linear_in_g",
    ):
        s = report["units_assumption_tests"][key]
        print(f"  {key}: median |res| = {s['norm_median_mps2']:.4f} m/s2  p95={s['norm_p95_mps2']:.4f}")
    print(f"  Best model: {best_key}")
    for label, stats in report["regimes"].items():
        if stats.get("n", 0):
            print(f"  [{label}] residual median={stats['median_mps2']:.4f} p95={stats['p95_mps2']:.4f} m/s2")
    print(f"  JSON: {OUT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
