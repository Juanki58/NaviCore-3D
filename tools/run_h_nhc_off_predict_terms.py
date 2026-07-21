#!/usr/bin/env python3
"""Decompose predict Δv_E into physical terms — Accept #17 → Reject #18, NHC off.

Only numbers in m/s. No 'parece que'.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
REPLAY = REPO / "build" / "NaviCore3D_Replay.exe"
INPUT = REPO / "docs" / "benchmarks" / "real_run_19082026_baseline" / "real_run_replay.csv"
MOUNT = REPO / "calibration" / "imu_mount.json"
OUT = REPO / "docs" / "benchmarks" / "h_nhc_off_predict_terms"
T0, T1 = 19.301353455, 20.301353455
G = 9.80665  # NAVICORE_INS_EKF_GRAVITY_MPS2; NED g = (0,0,+G)


def euler321_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Match ins_ekf euler321_to_quat (ZYX)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.array([qw, qx, qy, qz], dtype=float)


def quat_to_dcm_bn(q: np.ndarray) -> np.ndarray:
    """Match ins_ekf quat_to_dcm_bn."""
    qw, qx, qy, qz = q
    qw2, qx2, qy2, qz2 = qw * qw, qx * qx, qy * qy, qz * qz
    dcm = np.zeros((3, 3), dtype=float)
    dcm[0, 0] = qw2 + qx2 - qy2 - qz2
    dcm[0, 1] = 2.0 * ((qx * qy) - (qw * qz))
    dcm[0, 2] = 2.0 * ((qx * qz) + (qw * qy))
    dcm[1, 0] = 2.0 * ((qx * qy) + (qw * qz))
    dcm[1, 1] = qw2 - qx2 + qy2 - qz2
    dcm[1, 2] = 2.0 * ((qy * qz) - (qw * qx))
    dcm[2, 0] = 2.0 * ((qx * qz) - (qw * qy))
    dcm[2, 1] = 2.0 * ((qy * qz) + (qw * qx))
    dcm[2, 2] = qw2 - qx2 - qy2 + qz2
    return dcm


def run_h8() -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    h8 = OUT / "h8_propagation.csv"
    cmd = [
        str(REPLAY),
        "--input",
        str(INPUT),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(MOUNT),
        "--yaw-init",
        "zero",
        "--h9a-gravity-tilt-init",
        "--constraint-policy",
        "disabled",
        "--nhc-policy",
        "disabled",
        "--gnss-obs-mode",
        "pos_vel",
        "--p-pv-policy",
        "none",
        "--replay-end-s",
        "21.0",
        # Required: without this CSV open, replay silently drops GNSS vel obs (n_meas=3).
        "--gap3-gnss-nis-audit-csv",
        str(OUT / "gnss_nis_audit.csv"),
        "--gap3-constraint-pipeline-audit-csv",
        str(OUT / "constraint_pipeline_audit.csv"),
        "--h8-propagation-audit-csv",
        str(h8),
        "--output",
        str(OUT / "replay_output.csv"),
    ]
    print("RUN", " ".join(cmd), flush=True)
    log = subprocess.run(cmd, cwd=str(REPO), check=True, capture_output=True, text=True)
    (OUT / "replay.log").write_text(
        (log.stdout or "") + (log.stderr or ""), encoding="utf-8", errors="replace"
    )
    return h8


def main() -> int:
    ap_skip = "--skip-replay" in sys.argv
    h8_path = OUT / "h8_propagation.csv"
    if not ap_skip:
        h8_path = run_h8()
    if not h8_path.is_file():
        print("missing", h8_path, file=sys.stderr)
        return 1

    df = pd.read_csv(h8_path)
    w = df[(df["timestamp_s"] > T0) & (df["timestamp_s"] <= T1)].copy()
    if w.empty:
        print("empty window", file=sys.stderr)
        return 1

    # Direct from integrator: dv_E = a_lin_e * dt  (a_lin = a_nav - g; g_E=0)
    dt = w["dt_s"].to_numpy(dtype=float)
    a_nav_e = w["a_nav_y"].to_numpy(dtype=float)  # H8: a_nav_x/y/z = N/E/D
    a_lin_e = w["a_lin_y"].to_numpy(dtype=float)
    a_nav_n = w["a_nav_x"].to_numpy(dtype=float)
    a_lin_n = w["a_lin_x"].to_numpy(dtype=float)
    a_nav_d = w["a_nav_z"].to_numpy(dtype=float)
    a_lin_d = w["a_lin_z"].to_numpy(dtype=float)

    dv_lin_e = float(np.sum(a_lin_e * dt))
    dv_nav_e = float(np.sum(a_nav_e * dt))
    dv_gravity_e = float(np.sum((a_lin_e - a_nav_e) * dt))  # should be 0
    # Explicit gravity term as coded: -g_E * dt
    dv_gravity_e_explicit = 0.0  # g_E = 0

    # Down channel gravity check (should be ≈ -G * Σdt)
    dv_gravity_d = float(np.sum((a_lin_d - a_nav_d) * dt))
    sum_dt = float(np.sum(dt))

    # Split R*imu vs -R*bias using DCM from euler (validated vs a_nav)
    a_corr = w[["a_corr_x", "a_corr_y", "a_corr_z"]].to_numpy(dtype=float)
    bias = w[["bias_ax", "bias_ay", "bias_az"]].to_numpy(dtype=float)
    a_body = w[["a_body_x", "a_body_y", "a_body_z"]].to_numpy(dtype=float)
    roll = np.deg2rad(w["roll_deg"].to_numpy(dtype=float))
    pitch = np.deg2rad(w["pitch_deg"].to_numpy(dtype=float))
    yaw = np.deg2rad(w["yaw_deg"].to_numpy(dtype=float))

    r_imu_e = np.zeros(len(w))
    r_bias_e = np.zeros(len(w))
    r_corr_e = np.zeros(len(w))
    dcm_err = []
    for i in range(len(w)):
        dcm = quat_to_dcm_bn(euler321_to_quat(roll[i], pitch[i], yaw[i]))
        a_nav_recon = dcm @ a_corr[i]
        dcm_err.append(float(np.linalg.norm(a_nav_recon - np.array([a_nav_n[i], a_nav_e[i], a_nav_d[i]]))))
        r_corr_e[i] = a_nav_recon[1]
        r_imu_e[i] = (dcm @ a_body[i])[1]
        r_bias_e[i] = (dcm @ bias[i])[1]

    dv_R_imu_e = float(np.sum(r_imu_e * dt))
    dv_minus_R_bias_e = float(np.sum(-r_bias_e * dt))
    dv_R_corr_e = float(np.sum(r_corr_e * dt))

    # Code-absent terms
    dv_coriolis = 0.0
    dv_earth = 0.0

    # Closure: v_E from H8 vel_pre/post at endpoints
    v_e_start = float(w.iloc[0]["vel_pre_e"]) if "vel_pre_e" in w.columns else None
    # first row vel_pre should be ~ after accept; last vel_post ~ before reject
    v_e_end = float(w.iloc[-1]["vel_post_e"]) if "vel_post_e" in w.columns else None
    # Prefer first vel_pre of window and last vel_post
    # Actually window starts AFTER t17; first tick's vel_pre is start
    delta_from_vel = None
    if v_e_start is not None and v_e_end is not None:
        delta_from_vel = v_e_end - v_e_start

    terms = [
        {"term": "R·imu (specific force raw→NED)", "dv_E_mps": dv_R_imu_e, "in_code": True},
        {"term": "−R·bias_a", "dv_E_mps": dv_minus_R_bias_e, "in_code": True},
        {"term": "gravity (−g_E)", "dv_E_mps": dv_gravity_e_explicit, "in_code": True},
        {"term": "Coriolis", "dv_E_mps": dv_coriolis, "in_code": False},
        {"term": "Earth rotation", "dv_E_mps": dv_earth, "in_code": False},
    ]
    sum_terms = sum(t["dv_E_mps"] for t in terms)

    # Dominant among in-code nonzero-capable
    abs_map = {
        "R·imu": abs(dv_R_imu_e),
        "−R·bias_a": abs(dv_minus_R_bias_e),
        "gravity": abs(dv_gravity_e_explicit),
    }
    dom = max(abs_map, key=abs_map.get)
    dom_share = abs_map[dom] / (sum(abs_map.values()) or 1.0)

    verdict = {
        "question": "Which predict term produces Δv_E ≈ −8.3 m/s (#17→#18, NHC off)?",
        "interval_s": [T0, T1],
        "n_ticks": int(len(w)),
        "sum_dt_s": sum_dt,
        "observed": {
            "Σ a_lin_E·dt (integrator East)": dv_lin_e,
            "Σ a_nav_E·dt": dv_nav_e,
            "Δv_E from vel_pre→vel_post": delta_from_vel,
            "target_approx": -8.3,
        },
        "terms_m_s": terms,
        "sum_terms_m_s": sum_terms,
        "closure_sum_minus_a_lin": sum_terms - dv_lin_e,
        "checks": {
            "gravity_E_from_a_lin_minus_a_nav": dv_gravity_e,
            "gravity_D_from_a_lin_minus_a_nav": dv_gravity_d,
            "expected_gravity_D": -G * sum_dt,
            "a_corr_vs_body_minus_bias_rms": float(
                np.sqrt(np.mean(np.sum((a_corr - (a_body - bias)) ** 2, axis=1)))
            ),
            "dcm_recon_vs_a_nav_rms": float(np.sqrt(np.mean(np.square(dcm_err)))),
            "Σ R·a_corr_E·dt": dv_R_corr_e,
        },
        "dominance": {
            "dominant": dom,
            "share_of_abs_in_code": dom_share,
            "PASS_single_term_ge_90pct": bool(dom_share >= 0.90),
        },
        "eliminated": [
            "Coriolis — not in predict()",
            "Earth rotation — not in predict()",
            "Gravity East — g_NED=(0,0,G) ⇒ contribution 0 by construction",
        ],
    }

    lines = [
        "# Predict term budget — Δv_E (#17→#18, NHC off)",
        "",
        f"| Término | Δv_E (m/s) | En código |",
        f"|---------|------------|-----------|",
    ]
    for t in terms:
        lines.append(
            f"| {t['term']} | {t['dv_E_mps']:+.4f} | {'sí' if t['in_code'] else 'no'} |"
        )
    lines += [
        f"| **Σ términos** | **{sum_terms:+.4f}** | |",
        f"| **Σ a_lin_E·dt (integrador)** | **{dv_lin_e:+.4f}** | |",
        "",
        f"Dominante: **{dom}** ({dom_share*100:.1f}% del |Σ| in-code)",
        f"PASS ≥90% un término: {dom_share >= 0.90}",
        "",
        "Eliminados: Coriolis, Earth rate, gravity_E (=0).",
    ]
    (OUT / "TABLE.md").write_text("\n".join(lines), encoding="utf-8")
    (OUT / "verdict.json").write_text(json.dumps(verdict, indent=2), encoding="utf-8")
    print(json.dumps(verdict, indent=2))
    print("->", OUT / "TABLE.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
