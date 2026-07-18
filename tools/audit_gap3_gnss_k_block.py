#!/usr/bin/env python3
"""GAP-3 — Auditoría del bloque K_vel,pos y modelo de observación GNSS.

Responde:
  1. ¿Qué observa el update GNSS? (solo posición vs posición+velocidad)
  2. ¿Cuánto acopla K_vel,pos una innovación de posición a velocidad?
  3. ¿P_vel,pos crece lo suficiente en propagate?
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
K_BLOCK_JSON = BENCH_DIR / "gap3_gnss_k_block_audit.json"
REPORT_JSON = BENCH_DIR / "gap3_gnss_k_block_report.json"

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


def run_replay(replay_exe: Path, replay_csv: Path, calibration: Path, skip_run: bool) -> None:
    if skip_run:
        return
    ensure_calibration(calibration)
    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--h9a-gravity-tilt-init",
        "--output",
        str(BENCH_DIR / "gap3_gnss_k_block_replay_output.csv"),
        "--gap3-gnss-k-block-audit-json",
        str(K_BLOCK_JSON),
    ]
    print("RUN:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)


def mat_norm(m: list[list[float]]) -> float:
    return float(np.linalg.norm(np.array(m, dtype=float)))


def analyze(data: dict) -> dict:
    p_vp = np.array(data["P_vel_pos_cross_m2"], dtype=float)
    k_vp = np.array(data["K_vel_pos"], dtype=float)
    k_pp = np.array(data["K_pos_pos"], dtype=float)
    innov = np.array(data["innovation_ned_m"], dtype=float)
    dx = data["delta_x"]
    dx_vel = np.array(dx["vel_ned_mps"], dtype=float)
    dx_pos = np.array(dx["pos_ned_m"], dtype=float)

    dx_vel_from_k = k_vp @ innov
    dx_pos_from_k = k_pp @ innov

    x_prior = data["x_prior"]["vel_ned_mps"]
    x_post = data["x_post"]["vel_ned_mps"]
    vel_prior_h = math.hypot(x_prior[0], x_prior[1])
    vel_post_h = math.hypot(x_post[0], x_post[1])
    gps_speed = data["measurement_model"]["gps_speed_mps"]

    return {
        "fix_timestamp_s": data["timestamp_s"],
        "gps_index": data["gps_index"],
        "measurement_vector": "z = [pN, pE, pD] only",
        "velocity_in_measurement_model": data["measurement_model"]["z_velocity_in_ekf"],
        "gps_speed_available_unused": data["measurement_model"]["gps_speed_available_in_log"],
        "H_structure": data["measurement_model"]["H_on_error_state"],
        "innov_h_m": math.hypot(innov[0], innov[1]),
        "nis": data["nis"],
        "P_vel_pos_frobenius": float(np.linalg.norm(p_vp)),
        "P_vel_pos_max_abs": float(np.max(np.abs(p_vp))),
        "K_vel_pos_frobenius": float(np.linalg.norm(k_vp)),
        "K_vel_pos_max_abs": float(np.max(np.abs(k_vp))),
        "K_pos_pos_frobenius": float(np.linalg.norm(k_pp)),
        "dx_pos_norm_m": dx["pos_norm_m"],
        "dx_vel_norm_mps": dx["vel_norm_mps"],
        "dx_vel_over_dx_pos": dx["vel_norm_mps"] / dx["pos_norm_m"] if dx["pos_norm_m"] > 1e-9 else None,
        "dx_vel_from_Kvp_innov_norm": float(np.linalg.norm(dx_vel_from_k)),
        "dx_pos_from_Kpp_innov_norm": float(np.linalg.norm(dx_pos_from_k)),
        "vel_prior_h_mps": vel_prior_h,
        "vel_post_h_mps": vel_post_h,
        "gps_speed_mps": gps_speed,
        "delta_vel_h_mps": vel_post_h - vel_prior_h,
        "mechanism": (
            "DESIGN_LIMITATION"
            if data["measurement_model"]["z_velocity_in_ekf"] is False
            and float(np.max(np.abs(k_vp))) < 1e-3
            else "CROSS_COV_WEAK"
        ),
        "interpretation": {
            "why_dx_vel_tiny": (
                "H no observa velocidad; dx_vel = K_vel_pos @ innov con K_vel_pos = P_vel_pos @ S_inv. "
                "Si P_vel_pos es pequeño (ZUPT mantiene v≈0, poca correlación pos-vel en P), "
                "la corrección de velocidad es negligible aunque innov_pos sea grande."
            ),
            "architectural_conclusion": (
                "GPS speed/course existe en el log pero no entra al EKF. "
                "El filtro no puede corregir la variable que gobierna la siguiente deriva."
            ),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3 GNSS K_vel,pos audit")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-run", action="store_true")
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    run_replay(args.replay_exe, replay_csv, args.calibration, args.skip_run)

    if not K_BLOCK_JSON.is_file():
        print(f"Falta {K_BLOCK_JSON}", file=sys.stderr)
        return 1

    data = json.loads(K_BLOCK_JSON.read_text(encoding="utf-8"))
    report = analyze(data)
    report["source_json"] = str(K_BLOCK_JSON)
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"Wrote {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
