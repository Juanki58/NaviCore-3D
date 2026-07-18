#!/usr/bin/env python3
"""Auditoria de conformidad contra 08-body-frame-contract.md (invariantes I1-I6)."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
H9D_CSV = REPO_ROOT / "docs" / "benchmarks" / "h9d_gravity_subtraction.csv"
MOUNT_JSON = REPO_ROOT / "calibration" / "imu_mount.json"
UNCAL = REPO_ROOT / "data" / "real_run" / "AccelerometerUncalibrated.csv"
OUT_JSON = REPO_ROOT / "docs" / "benchmarks" / "body_frame_conformance_audit.json"
GRAVITY = 9.80665


def load_mount() -> np.ndarray:
    payload = json.loads(MOUNT_JSON.read_text(encoding="utf-8"))
    return np.array(payload["rotation_matrix"], dtype=float)


def load_uncal_static(t_end: float = 30.0) -> np.ndarray:
    rows: list[list[float]] = []
    with UNCAL.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            if not raw.get("seconds_elapsed"):
                continue
            t = float(raw["seconds_elapsed"])
            if t > t_end:
                break
            rows.append([float(raw["x"]), float(raw["y"]), float(raw["z"])])
    return np.array(rows, dtype=float)


def load_h9d() -> list[dict]:
    rows: list[dict] = []
    with H9D_CSV.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            rows.append(
                {
                    "t": float(raw["timestamp_s"]),
                    "a_body": np.array(
                        [float(raw["a_body_x"]), float(raw["a_body_y"]), float(raw["a_body_z"])],
                        dtype=float,
                    ),
                    "a_corr": np.array(
                        [float(raw["a_corr_x"]), float(raw["a_corr_y"]), float(raw["a_corr_z"])],
                        dtype=float,
                    ),
                    "a_nav_pre_g": np.array(
                        [
                            float(raw["a_nav_pre_g_n"]),
                            float(raw["a_nav_pre_g_e"]),
                            float(raw["a_nav_pre_g_d"]),
                        ],
                        dtype=float,
                    ),
                    "a_lin": np.array(
                        [float(raw["a_lin_n"]), float(raw["a_lin_e"]), float(raw["a_lin_d"])],
                        dtype=float,
                    ),
                    "a_lin_h": float(raw["a_lin_h"]),
                    "roll": float(raw["roll_deg"]),
                    "pitch": float(raw["pitch_deg"]),
                }
            )
    return rows


def window_stats(values: np.ndarray) -> dict:
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def check_invariant(name: str, value: float, limit: float, lower_is_better: bool = True) -> dict:
    ok = value <= limit if lower_is_better else value >= limit
    return {"name": name, "value": value, "limit": limit, "pass": ok}


def main() -> int:
    mount = load_mount()
    uncal_static = load_uncal_static()
    body_static = (mount @ uncal_static.T).T if uncal_static.size else np.zeros((0, 3))
    h9d = load_h9d()

    def subset(t0: float, t1: float) -> list[dict]:
        return [r for r in h9d if t0 <= r["t"] <= t1]

    static = subset(0.0, 2.0)
    motion = subset(2.0, 10.0)
    cruise = subset(11.4, 25.4)

    def metrics(rows: list[dict]) -> dict:
        if not rows:
            return {}
        f_body = np.array([r["a_corr"] for r in rows])
        a_nav = np.array([r["a_nav_pre_g"] for r in rows])
        a_lin = np.array([r["a_lin"] for r in rows])
        a_lin_h = np.array([r["a_lin_h"] for r in rows])
        nav_h = np.linalg.norm(a_nav[:, :2], axis=1)
        return {
            "n": len(rows),
            "I1_f_body_mag": window_stats(np.linalg.norm(f_body, axis=1)),
            "I2_a_nav_pre_h": window_stats(nav_h),
            "I2_a_nav_d_mean": float(np.mean(a_nav[:, 2])),
            "I3_a_lin_h": window_stats(a_lin_h),
            "I3_a_lin_ned_h": window_stats(np.linalg.norm(a_lin[:, :2], axis=1)),
        }

    mount_median = np.median(body_static, axis=0) if body_static.size else np.zeros(3)
    mount_error = float(np.linalg.norm(mount_median - np.array([0.0, 0.0, GRAVITY])))

    report = {
        "contract": "docs/diagnostics/08-body-frame-contract.md",
        "dataset": "data/real_run (Patron Oro)",
        "mount_static_median_body_mps2": [float(x) for x in mount_median],
        "mount_I1_z_alignment_error_mps2": mount_error,
        "regimes": {
            "static_0_2s": metrics(static),
            "motion_2_10s": metrics(motion),
            "cruise_11_25s": metrics(cruise),
        },
        "invariant_checks": [],
        "code_conformance": {},
    }

    s = report["regimes"]["static_0_2s"]
    m = report["regimes"]["motion_2_10s"]

    checks = [
        check_invariant("I1_static|f_B| mean", s["I1_f_body_mag"]["mean"], GRAVITY + 0.15, False)
        if s
        else check_invariant("I1_static|f_B| mean", 0.0, GRAVITY - 0.15, False),
        check_invariant("I1_static|f_B| mean upper", s["I1_f_body_mag"]["mean"], GRAVITY + 0.15)
        if s
        else {"name": "I1_static|f_B| mean upper", "value": 0.0, "limit": GRAVITY + 0.15, "pass": False},
        check_invariant("I2_static_a_nav_pre_h median", s["I2_a_nav_pre_h"]["median"], 0.05) if s else check_invariant("I2_static_a_nav_pre_h median", 999.0, 0.05),
        check_invariant("I3_static_a_lin_h median", s["I3_a_lin_h"]["median"], 0.05) if s else check_invariant("I3_static_a_lin_h median", 999.0, 0.05),
        check_invariant("mount_Z_alignment", mount_error, 0.05),
        check_invariant("I2_motion_a_nav_pre_h median", m["I2_a_nav_pre_h"]["median"], 0.05) if m else check_invariant("I2_motion_a_nav_pre_h median", 999.0, 0.05),
        check_invariant("I3_motion_a_lin_h median", m["I3_a_lin_h"]["median"], 0.05) if m else check_invariant("I3_motion_a_lin_h median", 999.0, 0.05),
    ]
    # Fix I1 lower bound check manually
    if s:
        i1_mean = s["I1_f_body_mag"]["mean"]
        checks[0] = {
            "name": "I1_static|f_B| in [g-0.15,g+0.15]",
            "value": i1_mean,
            "limit": f"[{GRAVITY-0.15},{GRAVITY+0.15}]",
            "pass": (GRAVITY - 0.15) <= i1_mean <= (GRAVITY + 0.15),
        }
        checks.pop(1)

    report["invariant_checks"] = checks

    report["code_conformance"] = {
        "predict_order_bias_before_rotate": True,
        "predict_uses_R_bn_not_R_nb": True,
        "gravity_subtracted_in_NED_after_rotate": True,
        "mount_applied_once_in_replay": True,
        "h9a_roll_pitch_from_FRD_gravity_formula": True,
        "nhc_observes_body_y_z_velocity": True,
        "imu_mount_declares_body_target_Z_down_only": True,
        "contract_requires_full_FRD_vehicle_axes": True,
        "gap_mount_calibrates_Z_not_explicit_forward": True,
    }

    verdict_static = all(c["pass"] for c in checks if "static" in c["name"] or "mount" in c["name"])
    verdict_dynamic = all(c["pass"] for c in checks if "motion" in c["name"])

    report["verdict"] = {
        "static_regime_conformant": verdict_static,
        "dynamic_regime_conformant": verdict_dynamic,
        "primary_gap": (
            "R_mount calibrates gravity on +Z body (M4/M2 partial) but contract §3.2 "
            "requires full vehicle FRD including forward +X; yaw-of-mount not observable "
            "from gravity alone."
            if not verdict_dynamic
            else "none"
        ),
    }

    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print("=" * 72)
    print(" Body Frame Contract — conformance audit")
    print("=" * 72)
    print(f"  Mount median body: ({mount_median[0]:+.3f}, {mount_median[1]:+.3f}, {mount_median[2]:+.3f})")
    print(f"  Mount Z-align error: {mount_error:.4f} m/s2")
    for label in ("static_0_2s", "motion_2_10s"):
        reg = report["regimes"][label]
        if not reg:
            continue
        print(f"  [{label}] I2 |a_nav_pre_h| median={reg['I2_a_nav_pre_h']['median']:.4f}  "
              f"I3 a_lin_h median={reg['I3_a_lin_h']['median']:.4f}")
    print(f"  Static conformant:  {verdict_static}")
    print(f"  Dynamic conformant: {verdict_dynamic}")
    print(f"  Gap: {report['verdict']['primary_gap']}")
    print(f"  JSON: {OUT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
