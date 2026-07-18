#!/usr/bin/env python3
"""GAP-3.8 — Matriz de políticas de restricciones (A–E).

Ejecuta el mismo dataset con políticas ZUPT/NHC distintas y compara métricas
de causalidad (v_nominal, ΣΔv, GNSS accepts, NIS, innovación) sin GNSS-velocidad.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks" / "constraint_matrix"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
REPORT_JSON = BENCH_DIR / "gap3_constraint_matrix_report.json"

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402

EXPERIMENTS = {
    "A": {
        "label": "baseline",
        "constraint_policy": "forced_time",
        "nhc_policy": "enabled",
        "zupt": "actual",
        "nhc": "ON",
    },
    "B": {
        "label": "isolate_zupt",
        "constraint_policy": "disabled",
        "nhc_policy": "enabled",
        "zupt": "OFF",
        "nhc": "ON",
    },
    "C": {
        "label": "gps_stationary",
        "constraint_policy": "gps_stop",
        "nhc_policy": "enabled",
        "zupt": "GPS stationary",
        "nhc": "ON",
    },
    "D": {
        "label": "imu_stationary",
        "constraint_policy": "imu_stationary",
        "nhc_policy": "enabled",
        "zupt": "IMU stationary",
        "nhc": "ON",
    },
    "E": {
        "label": "free_reference",
        "constraint_policy": "disabled",
        "nhc_policy": "disabled",
        "zupt": "OFF",
        "nhc": "OFF",
    },
}


def run_experiment(
    replay_exe: Path,
    replay_csv: Path,
    calibration: Path,
    exp_id: str,
    cfg: dict,
) -> dict:
    out_dir = BENCH_DIR / exp_id
    out_dir.mkdir(parents=True, exist_ok=True)

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
        "--constraint-policy",
        cfg["constraint_policy"],
        "--nhc-policy",
        cfg["nhc_policy"],
        "--output",
        str(out_dir / "replay_output.csv"),
        "--gap3-vel-source-audit-csv",
        str(out_dir / "vel_source_audit.csv"),
        "--gap3-imu-constraint-audit-csv",
        str(out_dir / "imu_constraint_audit.csv"),
        "--gap3-constraint-pipeline-audit-csv",
        str(out_dir / "constraint_pipeline_audit.csv"),
        "--gap3-gnss-nis-audit-csv",
        str(out_dir / "gnss_nis_audit.csv"),
        "--gap3-cov-propagation-audit-csv",
        str(out_dir / "cov_propagation_audit.csv"),
    ]
    print(f"\n=== Exp {exp_id} ({cfg['label']}) ===")
    print("RUN:", " ".join(cmd))
    proc = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True, check=True)
    stdout = proc.stdout

    gnss_accept = 0
    gnss_reject = 0
    final_drift = None
    for line in stdout.splitlines():
        if "GNSS aceptadas:" in line:
            parts = line.split("|")
            for part in parts:
                part = part.strip()
                if part.startswith("GNSS aceptadas:"):
                    gnss_accept = int(part.split(":")[1].strip().split()[0])
                elif part.startswith("rechazadas:"):
                    gnss_reject = int(part.split(":")[1].strip().split()[0])
        if "Deriva final H:" in line:
            final_drift = float(line.split(":")[1].strip().split()[0])

    vel = pd.read_csv(out_dir / "vel_source_audit.csv", index_col=False)
    pipeline = pd.read_csv(out_dir / "constraint_pipeline_audit.csv", index_col=False)
    constraint = pd.read_csv(out_dir / "imu_constraint_audit.csv", index_col=False)
    gnss = pd.read_csv(out_dir / "gnss_nis_audit.csv", index_col=False)
    cov = pd.read_csv(out_dir / "cov_propagation_audit.csv", index_col=False)

    for df in (vel, pipeline, constraint, gnss, cov):
        for col in df.columns:
            if col not in ("source", "event", "constraint_policy"):
                df[col] = pd.to_numeric(df[col], errors="coerce")

    vel_accum = vel.groupby("source")["dv_norm"].sum().to_dict()
    early = pipeline[(pipeline["timestamp_s"] >= 9.0) & (pipeline["timestamp_s"] <= 11.0)]
    cruise = pipeline[(pipeline["timestamp_s"] >= 30.0) & (pipeline["timestamp_s"] <= 35.0)]

    gnss_accepted = gnss[gnss["accepted"] == 1] if "accepted" in gnss.columns else gnss.iloc[0:0]
    gnss_pre = cov[cov["event"].astype(str).str.contains("gnss_pre", na=False)]
    innov_h_col = "innov_h_m" if "innov_h_m" in gnss.columns else "innov_h"
    nis_col = "nis_horizontal_2d" if "nis_horizontal_2d" in gnss.columns else "nis"
    p_pv_col = "P_vel_pos_frob" if "P_vel_pos_frob" in cov.columns else "P_pv_frob"
    p_vv_col = "P_vel_vel_frob" if "P_vel_vel_frob" in cov.columns else "P_vv_frob"

    metrics = {
        "experiment_id": exp_id,
        "config": cfg,
        "gnss_accept_count": gnss_accept,
        "gnss_reject_count": gnss_reject,
        "final_drift_m": final_drift,
        "vel_h_mean_t_9_11s": float(early["vel_h_mps"].mean()) if len(early) else None,
        "vel_h_mean_t_30_35s": float(cruise["vel_h_mps"].mean()) if len(cruise) else None,
        "vel_h_median_t_30_35s": float(cruise["vel_h_mps"].median()) if len(cruise) else None,
        "sum_dv_norm_by_source": {k: float(v) for k, v in vel_accum.items()},
        "bias_ax_at_10s": float(
            constraint.loc[(constraint["timestamp_s"] - 10.0).abs().idxmin(), "bias_ax"]
        )
        if len(constraint)
        else None,
        "gnss_nis_mean": float(gnss[nis_col].mean()) if nis_col in gnss.columns and len(gnss) else None,
        "gnss_nis_median_accepted": float(gnss_accepted[nis_col].median())
        if len(gnss_accepted) and nis_col in gnss_accepted.columns
        else None,
        "gnss_innov_h_mean": float(gnss[innov_h_col].mean())
        if innov_h_col in gnss.columns and len(gnss)
        else None,
        "gnss_innov_h_mean_accepted": float(gnss_accepted[innov_h_col].mean())
        if len(gnss_accepted) and innov_h_col in gnss_accepted.columns
        else None,
        "P_pv_frob_gnss_pre_mean": float(gnss_pre[p_pv_col].mean())
        if len(gnss_pre) and p_pv_col in gnss_pre.columns
        else None,
        "P_vv_frob_gnss_pre_mean": float(gnss_pre[p_vv_col].mean())
        if len(gnss_pre) and p_vv_col in gnss_pre.columns
        else None,
        "pipeline_tick_mean_abs_dv_pred": float(
            np.hypot(pipeline["dv_pred_n"], pipeline["dv_pred_e"]).abs().mean()
        )
        if len(pipeline)
        else None,
        "pipeline_tick_mean_abs_dv_nhc": float(
            np.hypot(pipeline["dv_nhc_n"], pipeline["dv_nhc_e"]).abs().mean()
        )
        if len(pipeline)
        else None,
        "pipeline_tick_mean_abs_dv_zupt": float(
            np.hypot(pipeline["dv_zupt_n"], pipeline["dv_zupt_e"]).abs().mean()
        )
        if len(pipeline)
        else None,
        "artifacts_dir": str(out_dir),
    }
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    return metrics


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3.8 constraint policy matrix A–E")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--experiments", nargs="*", default=list(EXPERIMENTS.keys()))
    parser.add_argument("--skip-run", action="store_true")
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    ensure_calibration(args.calibration)
    BENCH_DIR.mkdir(parents=True, exist_ok=True)

    results = []
    if not args.skip_run:
        for exp_id in args.experiments:
            if exp_id not in EXPERIMENTS:
                print(f"Unknown experiment: {exp_id}", file=sys.stderr)
                return 1
            results.append(
                run_experiment(
                    args.replay_exe,
                    replay_csv,
                    args.calibration,
                    exp_id,
                    EXPERIMENTS[exp_id],
                )
            )
    else:
        for exp_id in args.experiments:
            metrics_path = BENCH_DIR / exp_id / "metrics.json"
            if metrics_path.is_file():
                results.append(json.loads(metrics_path.read_text(encoding="utf-8")))

    baseline = next((r for r in results if r["experiment_id"] == "A"), None)
    report = {
        "experiment": "GAP-3.8 constraint policy matrix",
        "note": "No GNSS velocity — causal isolation of ZUPT/NHC replay policies",
        "experiments": results,
        "causal_comparison_vs_A": {},
    }
    if baseline:
        for r in results:
            if r["experiment_id"] == "A":
                continue
            report["causal_comparison_vs_A"][r["experiment_id"]] = {
                "delta_vel_h_t_30_35s": (
                    (r.get("vel_h_mean_t_30_35s") or 0.0)
                    - (baseline.get("vel_h_mean_t_30_35s") or 0.0)
                ),
                "delta_gnss_accepts": r.get("gnss_accept_count", 0)
                - baseline.get("gnss_accept_count", 0),
                "delta_gnss_innov_h_mean": (
                    (r.get("gnss_innov_h_mean") or 0.0)
                    - (baseline.get("gnss_innov_h_mean") or 0.0)
                ),
            }

    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"Wrote {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
