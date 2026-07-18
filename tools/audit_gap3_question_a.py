#!/usr/bin/env python3
"""GAP-3.7 — Pregunta A: ¿por qué v_nominal ≈ 0?

Separa de Pregunta B (K_vel,pos, ya cerrada en GAP-3.5/3.6).

Auditorías:
  1. Σ‖Δv‖ acumulado por fuente (predict / GNSS / NHC / ZUPT)
  2. Disparo ZUPT vs GPS speed (candidato vs aplicado)
  3. bias_ax vs a_body_x durante arranque
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
VEL_CSV = BENCH_DIR / "gap3_vel_source_audit.csv"
CONSTRAINT_CSV = BENCH_DIR / "gap3_imu_constraint_audit.csv"
REPORT_JSON = BENCH_DIR / "gap3_question_a_report.json"
VEL_ACCUM_PNG = BENCH_DIR / "gap3_vel_accumulation_by_source.png"
ZUPT_GPS_PNG = BENCH_DIR / "gap3_zupt_vs_gps_speed.png"
BIAS_PNG = BENCH_DIR / "gap3_bias_ax_vs_abody_x.png"

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
        str(BENCH_DIR / "gap3_question_a_replay_output.csv"),
        "--gap3-vel-source-audit-csv",
        str(VEL_CSV),
        "--gap3-imu-constraint-audit-csv",
        str(CONSTRAINT_CSV),
    ]
    print("RUN:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)


def analyze(vel: pd.DataFrame, constraint: pd.DataFrame) -> dict:
    accum = (
        vel.groupby("source")["dv_norm"]
        .agg(["count", "sum", "mean", "max"])
        .reset_index()
        .rename(columns={"sum": "sum_dv_norm"})
    )
    accum_dict = {
        row["source"]: {
            "count": int(row["count"]),
            "sum_dv_norm_mps": float(row["sum_dv_norm"]),
            "mean_dv_norm_mps": float(row["mean"]),
            "max_dv_norm_mps": float(row["max"]),
        }
        for _, row in accum.iterrows()
    }

    # ZUPT spurious: GPS speed 2-6 m/s while zupt applied
    moving = constraint[constraint["gps_speed_mps"] >= 2.0]
    spurious = moving[(moving["zupt_armed"] == 1) & (moving["zupt_applied"] == 1)]
    spurious_2_6 = spurious[spurious["gps_speed_mps"] <= 6.0]
    armed_not_applied = moving[
        (moving["zupt_armed"] == 1) & (moving["zupt_applied"] == 0)
    ]

    # Before static_phase_end (default 30s)
    static_end = float(constraint["static_phase_end_s"].iloc[0]) if len(constraint) else 30.0
    early_spurious = spurious_2_6[spurious_2_6["timestamp_s"] < static_end]

    # NHC H coupling: non-zero H on vN from lateral row
    nhc = vel[vel["source"] == "nhc"].copy()
    nhc_h_vn_mag = np.sqrt(
        nhc["h_nhc_r0_vn"].fillna(0) ** 2 + nhc["h_nhc_r1_vn"].fillna(0) ** 2
    )
    nhc_couples_vn = float((nhc_h_vn_mag > 0.05).mean()) if not nhc.empty else 0.0
    nhc_dv_forward = float(nhc["dv_n"].abs().sum()) if not nhc.empty else 0.0
    nhc_dv_lateral = float(nhc["dv_e"].abs().sum()) if not nhc.empty else 0.0
    predict_dv_forward = float(vel.loc[vel["source"] == "predict", "dv_n"].abs().sum())
    zupt_dv_forward = float(vel.loc[vel["source"] == "zupt", "dv_n"].abs().sum())

    # bias vs accel early window
    early = constraint[(constraint["timestamp_s"] >= 1.0) & (constraint["timestamp_s"] <= 20.0)]
    bias_corr = float(early["bias_ax"].corr(early["a_body_x"])) if len(early) > 10 else None

    predict_sum = accum_dict.get("predict", {}).get("sum_dv_norm_mps", 0.0)
    zupt_sum = accum_dict.get("zupt", {}).get("sum_dv_norm_mps", 0.0)
    nhc_sum = accum_dict.get("nhc", {}).get("sum_dv_norm_mps", 0.0)
    gnss_sum = accum_dict.get("gnss", {}).get("sum_dv_norm_mps", 0.0)

    verdict_a = "UNKNOWN"
    if len(spurious_2_6) > 0:
        verdict_a = "ZUPT_ACTIVE_WHILE_GPS_MOVING"
    elif abs(zupt_sum) > abs(predict_sum):
        verdict_a = "ZUPT_DOMINATES_VEL_WRITES"
    elif abs(nhc_sum) > abs(predict_sum):
        verdict_a = "NHC_DOMINATES_VEL_WRITES"

    return {
        "experiment": "GAP-3.7 Question A audit",
        "question_a": "Why v_nominal ≈ 0?",
        "question_b_status": "CLOSED (GAP-3.5/3.6: P_vv/P_pv small + H pos-only → K≈0)",
        "vel_accumulation_by_source": accum_dict,
        "net_vel_change_proxy": {
            "predict_sum_dv_norm": predict_sum,
            "nhc_sum_dv_norm": nhc_sum,
            "zupt_sum_dv_norm": zupt_sum,
            "gnss_sum_dv_norm": gnss_sum,
        },
        "zupt_firing": {
            "static_phase_end_s": static_end,
            "moving_speed_threshold_mps": float(constraint["moving_speed_threshold_mps"].iloc[0])
            if len(constraint) else 0.1,
            "imu_ticks_zupt_armed_while_gps_2_6_mps": int(
                len(moving[moving["zupt_armed"] == 1])
            ),
            "imu_ticks_zupt_applied_while_gps_2_6_mps": int(len(spurious_2_6)),
            "imu_ticks_zupt_applied_while_gps_2_6_before_static_end": int(len(early_spurious)),
            "imu_ticks_zupt_armed_not_applied_gps_2_6": int(len(armed_not_applied)),
            "first_spurious_timestamp_s": float(spurious_2_6["timestamp_s"].min())
            if not spurious_2_6.empty
            else None,
            "max_gps_speed_during_zupt_apply": float(
                constraint.loc[constraint["zupt_applied"] == 1, "gps_speed_mps"].max()
            )
            if (constraint["zupt_applied"] == 1).any()
            else 0.0,
            "mechanism": (
                "Replay usa ZUPT cuando t <= static_phase_end (30s) OR gps_speed <= 0.1 m/s. "
                "NO hay criterio accel/gyro en replay — solo reloj + ultimo GPS speed."
            ),
        },
        "nhc_H_coupling": {
            "fraction_nhc_updates_with_|H_vN|>0.05": nhc_couples_vn,
            "sum_abs_dv_n_mps": nhc_dv_forward,
            "sum_abs_dv_e_mps": nhc_dv_lateral,
            "note": "H observa v_body_y/z proyectado a NED; acopla vN/vE/vD (no solo vy/vz).",
        },
        "dv_forward_axis": {
            "predict_sum_abs_dv_n": predict_dv_forward,
            "nhc_sum_abs_dv_n": nhc_dv_forward,
            "zupt_sum_abs_dv_n": zupt_dv_forward,
        },
        "bias_ax": {
            "corr_bias_ax_a_body_x_t_1_20s": bias_corr,
            "bias_ax_at_10s": float(
                constraint.loc[(constraint["timestamp_s"] - 10.0).abs().idxmin(), "bias_ax"]
            )
            if len(constraint) else None,
        },
        "verdict_question_a": verdict_a,
        "interpretation": {
            "causal_tree": (
                "A: v_nominal≈0 ← ZUPT cada IMU en fase estática (t<30s) anula integración predict. "
                "B: GNSS no corrige v ← P_vv/P_pv + H pos-only (cerrado)."
            ),
        },
        "sources": {"vel_csv": str(VEL_CSV), "constraint_csv": str(CONSTRAINT_CSV)},
    }


def plot_vel_accum(vel: pd.DataFrame, out_png: Path) -> None:
    accum = vel.groupby("source")["dv_norm"].sum().sort_values()
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.barh(accum.index, accum.values)
    ax.set_xlabel("Σ ‖Δv‖ [m/s]")
    ax.set_title("GAP-3.7 — Contribución acumulada a vel_ por fuente")
    ax.grid(True, alpha=0.3, axis="x")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def plot_zupt_gps(constraint: pd.DataFrame, out_png: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(
        constraint["timestamp_s"],
        constraint["gps_speed_mps"],
        c=constraint["zupt_applied"],
        s=4,
        cmap="coolwarm",
        alpha=0.6,
    )
    ax.axvline(30.0, color="k", ls="--", lw=1, label="static_phase_end=30s")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("GPS speed [m/s]")
    ax.set_title("GAP-3.7 — GPS speed vs ZUPT applied (color=applied)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def plot_bias(constraint: pd.DataFrame, out_png: Path) -> None:
    early = constraint[(constraint["timestamp_s"] >= 1.0) & (constraint["timestamp_s"] <= 30.0)]
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(early["timestamp_s"], early["bias_ax"], label="bias_ax")
    ax.plot(early["timestamp_s"], early["a_body_x"], label="a_body_x", alpha=0.7)
    ax.set_xlabel("t [s]")
    ax.set_title("GAP-3.7 — bias_ax vs a_body_x (arranque)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3.7 Question A audit")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-run", action="store_true")
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    run_replay(args.replay_exe, replay_csv, args.calibration, args.skip_run)

    if not VEL_CSV.is_file() or not CONSTRAINT_CSV.is_file():
        print("Faltan CSVs de auditoría", file=sys.stderr)
        return 1

    vel = pd.read_csv(VEL_CSV, index_col=False)
    constraint = pd.read_csv(CONSTRAINT_CSV, index_col=False)
    for col in vel.columns:
        if col != "source":
            vel[col] = pd.to_numeric(vel[col], errors="coerce")
    for col in constraint.columns:
        constraint[col] = pd.to_numeric(constraint[col], errors="coerce")
    report = analyze(vel, constraint)
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    plot_vel_accum(vel, VEL_ACCUM_PNG)
    plot_zupt_gps(constraint, ZUPT_GPS_PNG)
    plot_bias(constraint, BIAS_PNG)
    print(json.dumps(report, indent=2))
    print(f"Wrote {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
