#!/usr/bin/env python3
"""GAP-3.9 — Auditoría de bloque NHC (innov, S, NIS, K, dx, ΔP, v_body).

Compara ZUPT OFF + NHC ON (exp B) vs ZUPT OFF + NHC OFF (exp E).
Enfocado en mecanismo causal: NHC → ΔP_vv/P_pv → GNSS K_vel≈0.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_nhc_block"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
REPORT_JSON = BENCH_DIR / "gap3_nhc_block_report.json"
TIMELINE_PNG = BENCH_DIR / "gap3_nhc_block_timeline.png"
NIS_DECOMP_PNG = BENCH_DIR / "gap3_nhc_block_nis_decomposition.png"
V_BODY_PNG = BENCH_DIR / "gap3_nhc_block_vbody_coupling.png"
HYPOTHESIS_PNG = BENCH_DIR / "gap3_nhc_block_dp_vs_dx.png"

AUDIT_END_S = 60.0
DX_VEL_SMALL = 1.0e-3
DELTA_P_VV_SIGNIFICANT = 1.0e-4

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402

RUNS = {
    "B_nhc_on": {
        "constraint_policy": "disabled",
        "nhc_policy": "enabled",
        "label": "ZUPT OFF, NHC ON",
    },
    "E_nhc_off": {
        "constraint_policy": "disabled",
        "nhc_policy": "disabled",
        "label": "ZUPT OFF, NHC OFF",
    },
}


def run_case(
    replay_exe: Path,
    replay_csv: Path,
    calibration: Path,
    case_id: str,
    cfg: dict,
) -> Path:
    out_dir = BENCH_DIR / case_id
    out_dir.mkdir(parents=True, exist_ok=True)
    nhc_csv = out_dir / "nhc_block_audit.csv"
    gnss_csv = out_dir / "gnss_nis_audit.csv"

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
        "--gap3-nhc-block-audit-csv",
        str(nhc_csv),
        "--gap3-gnss-nis-audit-csv",
        str(gnss_csv),
    ]
    print(f"\n=== {case_id}: {cfg['label']} ===")
    print("RUN:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)
    return nhc_csv


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, index_col=False)
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def analyze_nhc_mechanism(nhc: pd.DataFrame) -> dict:
    if nhc.empty:
        return {}

    early = nhc[nhc["timestamp_s"] <= AUDIT_END_S].copy()
    if early.empty:
        early = nhc.head(5000).copy()

    dx_vel = early["dx_vel_norm_mps"].fillna(0.0)
    dx_pos = early["dx_pos_norm_m"].fillna(0.0)
    d_p_vv = early["delta_P_vv_frob"].fillna(0.0)
    d_p_pv = early["delta_P_pv_frob"].fillna(0.0)
    abs_d_p_vv = d_p_vv.abs()

    small_dx = dx_vel < DX_VEL_SMALL
    sig_dp = abs_d_p_vv > DELTA_P_VV_SIGNIFICANT
    cov_only = small_dx & sig_dp

    dv_body_x = early["dv_body_x_mps"].fillna(0.0)
    v_x_before = early["v_body_x_before_mps"].fillna(0.0)
    abs_dvx = dv_body_x.abs()
    moving = v_x_before.abs() > 1.0

    return {
        "nhc_updates_total": int(len(nhc)),
        "nhc_updates_t_le_audit_end": int(len(early)),
        "nis_total_mean": float(early["nis_total"].mean()),
        "nis_total_median": float(early["nis_total"].median()),
        "nis_contrib_y_mean": float(early["nis_contrib_y"].mean()),
        "nis_contrib_z_mean": float(early["nis_contrib_z"].mean()),
        "dx_vel_norm_mean": float(dx_vel.mean()),
        "dx_vel_norm_median": float(dx_vel.median()),
        "dx_pos_norm_mean": float(dx_pos.mean()),
        "delta_P_vv_frob_mean": float(d_p_vv.mean()),
        "delta_P_pv_frob_mean": float(d_p_pv.mean()),
        "delta_P_aa_frob_mean": float(early["delta_P_aa_frob"].mean()),
        "P_vv_frob_pre_mean": float(early["P_pre_vv_frob"].mean()),
        "P_vv_frob_post_mean": float(early["P_post_vv_frob"].mean()),
        "P_pv_frob_pre_mean": float(early["P_pre_pv_frob"].mean()),
        "P_pv_frob_post_mean": float(early["P_post_pv_frob"].mean()),
        "k_vel_max_mean": float(early["k_vel_max"].mean()),
        "k_pos_max_mean": float(early["k_pos_max"].mean()),
        "cov_only_updates_frac": float(cov_only.mean()),
        "cov_only_updates_count": int(cov_only.sum()),
        "longitudinal_coupling_mean_abs_dvx": float(abs_dvx[moving].mean()) if moving.any() else None,
        "longitudinal_coupling_p95_abs_dvx": float(abs_dvx[moving].quantile(0.95)) if moving.any() else None,
        "longitudinal_coupling_frac_gt_0p1": float((abs_dvx[moving] > 0.1).mean()) if moving.any() else None,
        "sum_abs_delta_P_vv_frob": float(abs_d_p_vv.sum()),
        "sum_dx_vel_norm": float(dx_vel.sum()),
    }


def analyze_gnss(gnss: pd.DataFrame) -> dict:
    gnss_accepted = gnss[gnss["accepted"] == 1] if "accepted" in gnss.columns else gnss.iloc[0:0]
    innov_h_col = "innov_h_m" if "innov_h_m" in gnss.columns else "innov_h"
    nis_col = "nis_horizontal_2d" if "nis_horizontal_2d" in gnss.columns else "nis"

    return {
        "gnss_accept_count": int(gnss_accepted.shape[0]),
        "gnss_reject_count": int(gnss.shape[0] - gnss_accepted.shape[0]),
        "gnss_innov_h_mean_accepted": float(gnss_accepted[innov_h_col].mean())
        if len(gnss_accepted)
        else None,
        "gnss_k_vel_max_mean_accepted": float(gnss_accepted["k_vel_max"].mean())
        if len(gnss_accepted) and "k_vel_max" in gnss_accepted.columns
        else None,
        "gnss_k_vel_max_mean_rejected": float(gnss[gnss["accepted"] == 0]["k_vel_max"].mean())
        if "k_vel_max" in gnss.columns and (gnss["accepted"] == 0).any()
        else None,
        "gnss_nis_median_accepted": float(gnss_accepted[nis_col].median())
        if len(gnss_accepted) and nis_col in gnss_accepted.columns
        else None,
    }


def correlate_nhc_p_with_gnss_k(nhc: pd.DataFrame, gnss: pd.DataFrame) -> dict:
    if nhc.empty or gnss.empty:
        return {}

    rows = []
    for _, g in gnss.iterrows():
        t = g["timestamp_s"]
        prior = nhc[nhc["timestamp_s"] <= t]
        if prior.empty:
            continue
        last = prior.iloc[-1]
        rows.append(
            {
                "timestamp_s": t,
                "accepted": g.get("accepted", 0),
                "k_vel_max": g.get("k_vel_max", np.nan),
                "P_vv_frob_at_last_nhc": last.get("P_post_vv_frob", np.nan),
                "P_pv_frob_at_last_nhc": last.get("P_post_pv_frob", np.nan),
                "innov_h_m": g.get("innov_h_m", np.nan),
            }
        )
    if not rows:
        return {}

    df = pd.DataFrame(rows)
    accepted = df[df["accepted"] == 1]
    rejected = df[df["accepted"] == 0]
    return {
        "gnss_fixes_correlated": int(len(df)),
        "k_vel_max_mean_at_gnss_accepted": float(accepted["k_vel_max"].mean())
        if len(accepted)
        else None,
        "k_vel_max_mean_at_gnss_rejected": float(rejected["k_vel_max"].mean())
        if len(rejected)
        else None,
        "P_vv_frob_mean_before_gnss_accepted": float(accepted["P_vv_frob_at_last_nhc"].mean())
        if len(accepted)
        else None,
        "P_vv_frob_mean_before_gnss_rejected": float(rejected["P_vv_frob_at_last_nhc"].mean())
        if len(rejected)
        else None,
    }


def analyze_case(nhc_csv: Path, gnss_csv: Path, case_id: str, cfg: dict) -> dict:
    nhc = load_csv(nhc_csv)
    gnss = load_csv(gnss_csv)
    return {
        "case_id": case_id,
        "config": cfg,
        **analyze_nhc_mechanism(nhc),
        **analyze_gnss(gnss),
        "gnss_nhc_correlation": correlate_nhc_p_with_gnss_k(nhc, gnss),
        "artifacts": {"nhc_csv": str(nhc_csv), "gnss_csv": str(gnss_csv)},
    }


def downsample(df: pd.DataFrame, max_points: int = 800) -> pd.DataFrame:
    if len(df) <= max_points:
        return df
    idx = np.linspace(0, len(df) - 1, max_points, dtype=int)
    return df.iloc[idx]


def plot_timeline(nhc: pd.DataFrame, gnss: pd.DataFrame, out_png: Path) -> None:
    early = nhc[nhc["timestamp_s"] <= AUDIT_END_S]
    if early.empty:
        early = nhc.head(5000)
    early = downsample(early)
    t = early["timestamp_s"].values

    fig, axes = plt.subplots(5, 1, figsize=(13, 11), sharex=True)

    ax = axes[0]
    ax.plot(t, early["P_pre_vv_frob"], color="#1f77b4", lw=0.8, alpha=0.7, label="P_vv pre")
    ax.plot(t, early["P_post_vv_frob"], color="#ff7f0e", lw=0.8, alpha=0.9, label="P_vv post")
    ax.plot(t, early["P_pre_pv_frob"], color="#9467bd", lw=0.7, alpha=0.6, ls="--", label="P_pv pre")
    ax.set_ylabel("||P block||")
    ax.set_title("GAP-3.9 — P_vv / P_pv superpuesto (0–60 s)")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.25)

    ax = axes[1]
    ax.plot(t, early["innov_norm_mps"], color="#2ca02c", lw=0.7, label="||innov||")
    ax.plot(t, early["nis_total"], color="#d62728", lw=0.7, alpha=0.8, label="NIS total")
    ax.set_ylabel("innov / NIS")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.25)

    ax = axes[2]
    ax.plot(t, early["k_vel_max"], color="#1f77b4", lw=0.7, label="k_vel_max")
    ax.plot(t, early["k_pos_max"], color="#ff7f0e", lw=0.7, label="k_pos_max")
    ax.set_ylabel("K max")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.25)

    ax = axes[3]
    ax.plot(t, early["dx_vel_norm_mps"], color="#9467bd", lw=0.7, label="|dx_vel|")
    ax.plot(t, early["delta_P_vv_frob"].abs(), color="#d62728", lw=0.7, alpha=0.85, label="|ΔP_vv|")
    ax.set_ylabel("dx / ΔP")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.25)

    ax = axes[4]
    g = gnss[gnss["timestamp_s"] <= AUDIT_END_S]
    colors = ["#2ca02c" if a else "#d62728" for a in g.get("accepted", [])]
    ax.scatter(g["timestamp_s"], g.get("k_vel_max", 0), c=colors, s=22, zorder=3)
    ax.set_ylabel("GNSS k_vel")
    ax.set_xlabel("t [s]")
    ax.set_title("k_vel_max en fixes GNSS (verde=aceptado)")
    ax.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def plot_nis_decomposition(nhc: pd.DataFrame, out_png: Path) -> None:
    early = downsample(nhc[nhc["timestamp_s"] <= AUDIT_END_S])
    if early.empty:
        early = downsample(nhc.head(5000))
    t = early["timestamp_s"].values

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.stackplot(
        t,
        early["nis_contrib_y"].fillna(0),
        early["nis_contrib_z"].fillna(0),
        labels=["NIS contrib vy", "NIS contrib vz"],
        alpha=0.85,
        colors=["#1f77b4", "#ff7f0e"],
    )
    ax.plot(t, early["nis_total"], "k-", lw=0.9, label="NIS total")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("NIS")
    ax.set_title("GAP-3.9 — Descomposición NIS por componente body (vy / vz)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def plot_vbody_coupling(nhc: pd.DataFrame, out_png: Path) -> None:
    early = nhc[(nhc["timestamp_s"] <= AUDIT_END_S) & (nhc["v_body_x_before_mps"].abs() > 1.0)]
    if early.empty:
        early = nhc[nhc["v_body_x_before_mps"].abs() > 1.0].head(3000)
    if early.empty:
        return

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ax0 = axes[0]
    ax0.scatter(
        early["v_body_x_before_mps"],
        early["v_body_x_after_mps"],
        c=early["dv_body_x_mps"].abs(),
        cmap="viridis",
        s=8,
        alpha=0.6,
    )
    lim = max(early["v_body_x_before_mps"].abs().max(), early["v_body_x_after_mps"].abs().max()) * 1.05
    ax0.plot([-lim, lim], [-lim, lim], "k--", lw=0.8)
    ax0.set_xlabel("vx_body before [m/s]")
    ax0.set_ylabel("vx_body after [m/s]")
    ax0.set_title("Acoplamiento longitudinal NHC (color=|Δvx|)")
    ax0.grid(True, alpha=0.25)

    ax1 = axes[1]
    ax1.hist(early["dv_body_x_mps"].abs(), bins=60, color="#1f77b4", alpha=0.85)
    ax1.axvline(0.1, color="r", ls="--", lw=0.9, label="|Δvx|=0.1 m/s")
    ax1.set_xlabel("|Δvx_body| [m/s]")
    ax1.set_ylabel("count")
    ax1.set_title("Distribución acoplamiento vx (vx>1 m/s)")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def plot_dp_vs_dx(nhc: pd.DataFrame, out_png: Path) -> None:
    early = downsample(nhc[nhc["timestamp_s"] <= AUDIT_END_S])
    if early.empty:
        early = downsample(nhc.head(5000))

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.scatter(
        early["dx_vel_norm_mps"],
        early["delta_P_vv_frob"].abs(),
        c=early["delta_P_pv_frob"].abs(),
        cmap="plasma",
        s=10,
        alpha=0.55,
    )
    ax.axvline(DX_VEL_SMALL, color="k", ls=":", lw=0.8, label=f"|dx_vel|<{DX_VEL_SMALL}")
    ax.axhline(DELTA_P_VV_SIGNIFICANT, color="r", ls="--", lw=0.8, label=f"|ΔP_vv|>{DELTA_P_VV_SIGNIFICANT}")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("|dx_vel| [m/s]")
    ax.set_ylabel("|ΔP_vv| frob")
    ax.set_title("GAP-3.9 — Hipótesis: Δx pequeño, ΔP grande (color=|ΔP_pv|)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.25, which="both")
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def build_verdict(b: dict | None, e: dict | None) -> str:
    if not b or not e:
        return "INCONCLUSIVE"
    if b.get("gnss_accept_count", 0) < e.get("gnss_accept_count", 0):
        if b.get("cov_only_updates_frac", 0) > 0.1:
            return "NHC_COV_COLLAPSE_WITHOUT_STATE_CHANGE"
        return "NHC_DOMINATES_GNSS_ACCEPTANCE"
    return "INCONCLUSIVE"


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3.9 NHC block audit B vs E")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-run", action="store_true")
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    ensure_calibration(args.calibration)
    BENCH_DIR.mkdir(parents=True, exist_ok=True)

    results = []
    if not args.skip_run:
        for case_id, cfg in RUNS.items():
            nhc_csv = run_case(
                args.replay_exe,
                replay_csv,
                args.calibration,
                case_id,
                cfg,
            )
            gnss_csv = BENCH_DIR / case_id / "gnss_nis_audit.csv"
            results.append(analyze_case(nhc_csv, gnss_csv, case_id, cfg))
    else:
        for case_id, cfg in RUNS.items():
            nhc_csv = BENCH_DIR / case_id / "nhc_block_audit.csv"
            gnss_csv = BENCH_DIR / case_id / "gnss_nis_audit.csv"
            if nhc_csv.is_file() and gnss_csv.is_file():
                results.append(analyze_case(nhc_csv, gnss_csv, case_id, cfg))

    b = next((r for r in results if r["case_id"] == "B_nhc_on"), None)
    e = next((r for r in results if r["case_id"] == "E_nhc_off"), None)

    comparison = {}
    hypothesis = {}
    if b and e:
        comparison = {
            "delta_gnss_accepts": e["gnss_accept_count"] - b["gnss_accept_count"],
            "delta_gnss_k_vel_accepted": (e.get("gnss_k_vel_max_mean_accepted") or 0)
            - (b.get("gnss_k_vel_max_mean_accepted") or 0),
            "nhc_on_updates": b.get("nhc_updates_total", 0),
        }
        hypothesis = {
            "statement": (
                "NHC reduce P_vv/P_pv cada tick; estado cambia poco; "
                "GNSS k_vel→0 y no corrige velocidad."
            ),
            "cov_only_updates_frac_B": b.get("cov_only_updates_frac"),
            "cov_only_updates_count_B": b.get("cov_only_updates_count"),
            "longitudinal_coupling_frac_gt_0p1_B": b.get("longitudinal_coupling_frac_gt_0p1"),
            "k_vel_max_at_gnss_accepted_B": b.get("gnss_nhc_correlation", {}).get(
                "k_vel_max_mean_at_gnss_accepted"
            ),
            "k_vel_max_at_gnss_accepted_E": e.get("gnss_nhc_correlation", {}).get(
                "k_vel_max_mean_at_gnss_accepted"
            ),
        }

    if b and (BENCH_DIR / "B_nhc_on" / "nhc_block_audit.csv").is_file():
        nhc_b = load_csv(BENCH_DIR / "B_nhc_on" / "nhc_block_audit.csv")
        gnss_b = load_csv(BENCH_DIR / "B_nhc_on" / "gnss_nis_audit.csv")
        plot_timeline(nhc_b, gnss_b, TIMELINE_PNG)
        plot_nis_decomposition(nhc_b, NIS_DECOMP_PNG)
        plot_vbody_coupling(nhc_b, V_BODY_PNG)
        plot_dp_vs_dx(nhc_b, HYPOTHESIS_PNG)

    report = {
        "experiment": "GAP-3.9 NHC block audit",
        "design": "ZUPT OFF (disabled); compare NHC ON vs OFF",
        "audit_window_s": AUDIT_END_S,
        "cases": results,
        "comparison_B_vs_E": comparison,
        "hypothesis_NHC_cov_collapse": hypothesis,
        "verdict": build_verdict(b, e),
        "artifacts": {
            "report_json": str(REPORT_JSON),
            "timeline_png": str(TIMELINE_PNG),
            "nis_decomposition_png": str(NIS_DECOMP_PNG),
            "vbody_coupling_png": str(V_BODY_PNG),
            "dp_vs_dx_png": str(HYPOTHESIS_PNG),
        },
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"Wrote {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
