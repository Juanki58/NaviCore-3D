#!/usr/bin/env python3
"""GAP-7: IMU raw dynamics in REF consistency-reject windows vs quiet."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[2]
REPLAY = REPO / "docs" / "benchmarks" / "real_run_19082026_baseline" / "real_run_replay.csv"
AUDIT = REPO / "docs" / "benchmarks" / "ekf_v2_ab_3routes" / "REF_19082026" / "v2" / "gnss_nis_audit.csv"
OUT = REPO / "docs" / "benchmarks" / "ekf_v2_ab_3routes" / "gap7_consistency_imu_diag.json"

WINDOWS = {
    "peak_220_240": (220.0, 240.0),
    "peak_390_420": (390.0, 420.0),
    "peak_580_640": (580.0, 640.0),
    "quiet_300_350": (300.0, 350.0),
    "quiet_450_530": (450.0, 530.0),
}


def frac_above(s: pd.Series, thr: float) -> float:
    return float((s > thr).mean()) if len(s) else float("nan")


def main() -> int:
    rep = pd.read_csv(REPLAY)
    imu = rep[rep["type"] == "IMU"].copy()
    gps = rep[rep["type"] == "GPS"].copy()
    for c in ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z", "timestamp_s"]:
        imu[c] = pd.to_numeric(imu[c], errors="coerce")
    gps["timestamp_s"] = pd.to_numeric(gps["timestamp_s"], errors="coerce")
    gps["speed"] = pd.to_numeric(gps["speed"], errors="coerce")

    imu["a_norm"] = np.sqrt(imu.accel_x**2 + imu.accel_y**2 + imu.accel_z**2)
    imu["a_excess"] = (imu.a_norm - 9.81).abs()
    imu["g_norm"] = np.sqrt(imu.gyro_x**2 + imu.gyro_y**2 + imu.gyro_z**2)

    audit = pd.read_csv(AUDIT)
    rows = []
    for name, (t0, t1) in WINDOWS.items():
        w = imu[(imu.timestamp_s >= t0) & (imu.timestamp_s <= t1)]
        g = gps[(gps.timestamp_s >= t0) & (gps.timestamp_s <= t1)]
        a = audit[(audit.timestamp_s >= t0) & (audit.timestamp_s <= t1)]
        rej = a[a.accepted == 0]
        spd = g.speed.dropna()
        dspd = spd.diff().abs().dropna()
        row = {
            "window": name,
            "n_imu": int(len(w)),
            "n_gps": int(len(g)),
            "imu_a_excess_p50": float(w.a_excess.median()),
            "imu_a_excess_p95": float(w.a_excess.quantile(0.95)),
            "imu_a_excess_max": float(w.a_excess.max()),
            "imu_frac_a_excess_gt_2": frac_above(w.a_excess, 2.0),
            "imu_frac_a_excess_gt_4": frac_above(w.a_excess, 4.0),
            "imu_g_norm_p50_radps": float(w.g_norm.median()),
            "imu_g_norm_p95_radps": float(w.g_norm.quantile(0.95)),
            "imu_g_norm_max_radps": float(w.g_norm.max()),
            "imu_g_norm_max_dps": float(w.g_norm.max() * 180.0 / np.pi),
            "imu_frac_g_gt_0_3_radps": frac_above(w.g_norm, 0.3),
            "gps_speed_mean_mps": float(spd.mean()) if len(spd) else None,
            "gps_speed_max_mps": float(spd.max()) if len(spd) else None,
            "gps_dspeed_p95_mps": float(dspd.quantile(0.95)) if len(dspd) else None,
            "gps_dspeed_max_mps": float(dspd.max()) if len(dspd) else None,
            "n_reject": int(len(rej)),
            "innov_h_rej_min_m": float(rej.innov_h_m.min()) if len(rej) else None,
            "innov_h_rej_mean_m": float(rej.innov_h_m.mean()) if len(rej) else None,
            "innov_h_rej_max_m": float(rej.innov_h_m.max()) if len(rej) else None,
            "innov_h_all_max_m": float(a.innov_h_m.max()) if len(a) else None,
            "frac_accepted": float((a.accepted == 1).mean()) if len(a) else None,
        }
        rows.append(row)
        print(f"==== {name} ====")
        print(
            f"  a_excess p50/p95/max={row['imu_a_excess_p50']:.3f}/"
            f"{row['imu_a_excess_p95']:.3f}/{row['imu_a_excess_max']:.3f} "
            f"frac>2={row['imu_frac_a_excess_gt_2']:.3f}"
        )
        print(
            f"  g_norm p50/p95/max={row['imu_g_norm_p50_radps']:.3f}/"
            f"{row['imu_g_norm_p95_radps']:.3f}/{row['imu_g_norm_max_radps']:.3f} rad/s "
            f"(max {row['imu_g_norm_max_dps']:.1f} deg/s)"
        )
        print(
            f"  gps speed mean/max={row['gps_speed_mean_mps']:.2f}/{row['gps_speed_max_mps']:.2f} "
            f"dspeed_max={row['gps_dspeed_max_mps']}"
        )
        print(
            f"  rejects={row['n_reject']} innov_h rej "
            f"{row['innov_h_rej_min_m']}/{row['innov_h_rej_mean_m']}/{row['innov_h_rej_max_m']}"
        )

    rej_all = audit[audit.accepted == 0]
    payload = {
        "thresholds": {
            "NAVICORE_INS_EKF_CONSISTENCY_MAX_GAP_S": 2.0,
            "NAVICORE_INS_EKF_CONSISTENCY_MAX_POS_JUMP_M": 120.0,
            "NAVICORE_INS_EKF_CONSISTENCY_POS_MARGIN_M": 30.0,
            "NAVICORE_INS_EKF_CONSISTENCY_SIGMA_K": 6.0,
            "NAVICORE_INS_EKF_CONSISTENCY_MAX_VEL_JUMP_MPS": 35.0,
            "gate_formula": (
                "gate = min(MAX_POS_JUMP, max(v_h*gap + POS_MARGIN + SIGMA_K*sigma_h, 40))"
            ),
        },
        "threshold_vs_urban_dynamics": {
            "vel_jump_35_mps_over_1s_gap_implies_equiv_accel_mps2": 35.0,
            "urban_hard_brake_mps2": 7.0,
            "vel_gate_vs_hard_brake": "35 m/s jump >> ~7 m/s from 1s hard brake — vel gate is LOOSE",
            "pos_hard_cap_m": 120.0,
            "rejects_with_innov_h_gt_120": int((rej_all.innov_h_m > 120).sum()),
            "rejects_total": int(len(rej_all)),
            "innov_h_rej_median_m": float(rej_all.innov_h_m.median()),
            "innov_h_rej_min_m": float(rej_all.innov_h_m.min()),
        },
        "windows": rows,
        "conclusion_hint": (
            "Peak IMU a_excess/g_norm similar to quiet; GNSS innov_h at rejects "
            "typically >120 m hard cap while vehicle dynamics look normal → multipath/"
            "spurious GNSS (or already-diverged filter), not urban maneuver mistuned by vel gate."
        ),
    }
    OUT.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print("Wrote", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
