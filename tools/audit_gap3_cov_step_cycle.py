#!/usr/bin/env python3
"""GAP-3.6 — Ciclo predict/NHC/ZUPT/GNSS: quién destruye P_pv y P_vv.

Responde:
  1. Serie temporal pre/post por tipo de update
  2. Evolución diag(P_vv) cada IMU (vía predict_post)
  3. ¿NHC reduce solo vy/vz o también vx (body forward)?
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
STEP_CSV = BENCH_DIR / "gap3_cov_step_audit.csv"
REPORT_JSON = BENCH_DIR / "gap3_cov_step_audit_report.json"
WINDOW_PNG = BENCH_DIR / "gap3_cov_step_window_analysis.png"
P_VV_ROUTE_PNG = BENCH_DIR / "gap3_p_vv_imu_trace.png"
REDUCTION_PNG = BENCH_DIR / "gap3_cov_reduction_by_update.png"

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
        str(BENCH_DIR / "gap3_cov_step_replay_output.csv"),
        "--gap3-cov-step-audit-csv",
        str(STEP_CSV),
    ]
    print("RUN:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)


def _delta_row(pre: pd.Series, post: pd.Series, upd: str, phase_post: str) -> dict:
    return {
        "timestamp_s": post["timestamp_s"],
        "update_type": upd,
        "phase_post": phase_post,
        "d_P_vv_frob": post["P_vv_frob"] - pre["P_vv_frob"],
        "d_P_pv_frob": post["P_pv_frob"] - pre["P_pv_frob"],
        "d_P_pp_frob": post["P_pp_frob"] - pre["P_pp_frob"],
        "d_P_vv_n": post["P_vv_n_m2"] - pre["P_vv_n_m2"],
        "d_P_vv_e": post["P_vv_e_m2"] - pre["P_vv_e_m2"],
        "d_P_vv_d": post["P_vv_d_m2"] - pre["P_vv_d_m2"],
        "d_P_vv_fwd": post["P_vv_body_fwd_m2"] - pre["P_vv_body_fwd_m2"],
        "d_P_vv_lat": post["P_vv_body_lat_m2"] - pre["P_vv_body_lat_m2"],
        "d_P_vv_vert": post["P_vv_body_vert_m2"] - pre["P_vv_body_vert_m2"],
        "P_vv_frob_pre": pre["P_vv_frob"],
        "P_pv_frob_pre": pre["P_pv_frob"],
        "P_vv_frob_post": post["P_vv_frob"],
        "P_pv_frob_post": post["P_pv_frob"],
    }


def compute_step_deltas(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for upd in df["update_type"].unique():
        sub = df[df["update_type"] == upd]
        if upd == "gnss":
            pre = sub[sub["phase"] == "pre"]
            post = sub[sub["phase"].str.startswith("post")]
            for t in pre["timestamp_s"].unique():
                pre_row = pre[pre["timestamp_s"] == t]
                post_row = post[post["timestamp_s"] == t]
                if pre_row.empty or post_row.empty:
                    continue
                post_row = post_row.iloc[0]
                rows.append(_delta_row(pre_row.iloc[0], post_row, upd, str(post_row["phase"])))
            continue
        pre = sub[sub["phase"] == "pre"].reset_index(drop=True)
        post = sub[sub["phase"] == "post"].reset_index(drop=True)
        n = min(len(pre), len(post))
        for i in range(n):
            rows.append(_delta_row(pre.iloc[i], post.iloc[i], upd, "post"))
    return pd.DataFrame(rows)


def analyze(df: pd.DataFrame, window: tuple[float, float]) -> dict:
    for col in df.columns:
        if col not in ("update_type", "phase"):
            df[col] = pd.to_numeric(df[col], errors="coerce")

    deltas = compute_step_deltas(df)
    predict = df[(df["update_type"] == "predict") & (df["phase"] == "post")].copy()

    def agg_reduction(frame: pd.DataFrame, label: str) -> dict:
        if frame.empty:
            return {"label": label, "count": 0}
        return {
            "label": label,
            "count": int(len(frame)),
            "mean_d_P_vv_frob": float(frame["d_P_vv_frob"].mean()),
            "mean_d_P_pv_frob": float(frame["d_P_pv_frob"].mean()),
            "median_d_P_vv_frob": float(frame["d_P_vv_frob"].median()),
            "median_d_P_pv_frob": float(frame["d_P_pv_frob"].median()),
            "mean_d_P_vv_fwd": float(frame["d_P_vv_fwd"].mean()),
            "mean_d_P_vv_lat": float(frame["d_P_vv_lat"].mean()),
            "mean_d_P_vv_vert": float(frame["d_P_vv_vert"].mean()),
        }

    by_update = {
        upd: agg_reduction(deltas[deltas["update_type"] == upd], upd)
        for upd in ["predict", "nhc", "zupt", "gnss"]
    }

    w0, w1 = window
    win = df[(df["timestamp_s"] >= w0) & (df["timestamp_s"] <= w1)].copy()
    win_d = deltas[(deltas["timestamp_s"] >= w0) & (deltas["timestamp_s"] <= w1)].copy()

    last_accept_t = win[
        (win["update_type"] == "gnss") & (win["phase"] == "post_accept")
    ]["timestamp_s"].max()
    pre_last_accept = win[
        (win["update_type"] == "gnss")
        & (win["phase"] == "pre")
        & (win["timestamp_s"] == last_accept_t)
    ]

    nhc_fwd_vs_lat = {}
    nhc_d = deltas[deltas["update_type"] == "nhc"]
    if not nhc_d.empty:
        nhc_fwd_vs_lat = {
            "mean_reduction_fwd_m2": float(nhc_d["d_P_vv_fwd"].mean()),
            "mean_reduction_lat_m2": float(nhc_d["d_P_vv_lat"].mean()),
            "mean_reduction_vert_m2": float(nhc_d["d_P_vv_vert"].mean()),
            "ratio_fwd_over_lat": float(
                abs(nhc_d["d_P_vv_fwd"].mean()) / max(abs(nhc_d["d_P_vv_lat"].mean()), 1e-12)
            ),
            "nhc_also_crushes_forward": bool(
                abs(nhc_d["d_P_vv_fwd"].mean()) > 0.05 * abs(nhc_d["d_P_vv_lat"].mean())
            ),
        }

    pvv_trace = predict[["timestamp_s", "P_vv_n_m2", "P_vv_e_m2", "P_vv_d_m2", "P_vv_frob", "P_pv_frob"]]

    verdict = "UNKNOWN"
    if by_update.get("nhc", {}).get("count", 0) > 0:
        if by_update["nhc"]["median_d_P_pv_frob"] < -1e-4:
            verdict = "NHC_SUPPRESSES_P_PV_EACH_IMU"
        if by_update.get("gnss", {}).get("median_d_P_pv_frob", 0) < -1e-5:
            verdict = "GNSS_ACCEPT_ANNIHILATES_P_PV"

    return {
        "experiment": "GAP-3.6 cov step cycle audit",
        "window_s": [w0, w1],
        "reduction_by_update": by_update,
        "nhc_forward_vs_lateral": nhc_fwd_vs_lat,
        "last_gnss_accept_in_window": (
            pre_last_accept.iloc[0].to_dict() if not pre_last_accept.empty else None
        ),
        "window_step_count": int(len(win)),
        "pvv_at_last_accept_pre": (
            float(pre_last_accept.iloc[0]["P_vv_frob"]) if not pre_last_accept.empty else None
        ),
        "ppv_at_last_accept_pre": (
            float(pre_last_accept.iloc[0]["P_pv_frob"]) if not pre_last_accept.empty else None
        ),
        "verdict": verdict,
        "interpretation": {
            "cycle_hypothesis": (
                "predict aumenta P_pv; NHC/ZUPT la reducen; GNSS accept (Joseph pos-only) "
                "aniquila P_pv residual. P_vv colapsa por ZUPT (3D vel) en fase estática "
                "y por acoplamiento NHC en cuerpo."
            ),
            "nhc_longitudinal": (
                "Si d_P_vv_body_fwd es del mismo orden que d_P_vv_body_lat, NHC está "
                "reduciendo incertidumbre longitudinal vía acoplamiento Joseph, no solo vy/vz."
            ),
        },
        "source_csv": str(STEP_CSV),
        "pvv_trace_rows": int(len(pvv_trace)),
    }


def plot_window(df: pd.DataFrame, deltas: pd.DataFrame, window: tuple[float, float], out_png: Path) -> None:
    w0, w1 = window
    win = df[(df["timestamp_s"] >= w0) & (df["timestamp_s"] <= w1)]
    win_d = deltas[(deltas["timestamp_s"] >= w0) & (deltas["timestamp_s"] <= w1)]

    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)

    for upd, color in [("predict", "C0"), ("nhc", "C1"), ("zupt", "C2"), ("gnss", "C3")]:
        sub = win[(win["update_type"] == upd) & (win["phase"].str.contains("post"))]
        if sub.empty:
            continue
        axes[0].scatter(sub["timestamp_s"], sub["P_pv_frob"], s=8, c=color, label=upd, alpha=0.7)
    axes[0].set_yscale("log")
    axes[0].set_ylabel("||P_pv||")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f"GAP-3.6 ventana t∈[{w0},{w1}] s — P_pv / P_vv")

    for upd, color in [("predict", "C0"), ("nhc", "C1"), ("zupt", "C2"), ("gnss", "C3")]:
        sub = win[(win["update_type"] == upd) & (win["phase"].str.contains("post"))]
        if not sub.empty:
            axes[1].scatter(sub["timestamp_s"], sub["P_vv_frob"], s=8, c=color, alpha=0.7)
    axes[1].set_yscale("log")
    axes[1].set_ylabel("||P_vv||")
    axes[1].grid(True, alpha=0.3)

    if not win_d.empty:
        for upd, color in [("predict", "C0"), ("nhc", "C1"), ("zupt", "C2"), ("gnss", "C3")]:
            sub = win_d[win_d["update_type"] == upd]
            if not sub.empty:
                axes[2].plot(sub["timestamp_s"], sub["d_P_pv_frob"], ".", ms=3, c=color, label=upd)
        axes[2].set_ylabel("Δ||P_pv|| post-pre")
        axes[2].legend(loc="upper right", fontsize=8)
        axes[2].grid(True, alpha=0.3)

        nhc = win_d[win_d["update_type"] == "nhc"]
        if not nhc.empty:
            axes[3].plot(nhc["timestamp_s"], nhc["d_P_vv_fwd"], ".", ms=3, label="ΔP_vv fwd")
            axes[3].plot(nhc["timestamp_s"], nhc["d_P_vv_lat"], ".", ms=3, label="ΔP_vv lat")
            axes[3].plot(nhc["timestamp_s"], nhc["d_P_vv_vert"], ".", ms=3, label="ΔP_vv vert")
            axes[3].set_ylabel("ΔP_vv body [m²/s²]")
            axes[3].set_xlabel("t [s]")
            axes[3].legend(loc="upper right", fontsize=8)
            axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def plot_pvv_route(predict: pd.DataFrame, out_png: Path) -> None:
    if predict.empty:
        return
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(predict["timestamp_s"], np.sqrt(predict["P_vv_n_m2"]), "b-", lw=0.6, label="σ_vN")
    ax.plot(predict["timestamp_s"], np.sqrt(predict["P_vv_e_m2"]), "g-", lw=0.6, label="σ_vE")
    ax.plot(predict["timestamp_s"], np.sqrt(predict["P_vv_d_m2"]), "r-", lw=0.6, label="σ_vD")
    ax.set_yscale("log")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("σ velocity [m/s]")
    ax.set_title("GAP-3.6 — diag(P_vv) tras cada predict (≈ cada IMU)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def plot_reduction_bars(by_update: dict, out_png: Path) -> None:
    labels = []
    pvv = []
    ppv = []
    for key in ["predict", "nhc", "zupt", "gnss"]:
        block = by_update.get(key, {})
        if block.get("count", 0) == 0:
            continue
        labels.append(key)
        pvv.append(block.get("median_d_P_vv_frob", 0.0))
        ppv.append(block.get("median_d_P_pv_frob", 0.0))
    if not labels:
        return
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.bar(x - 0.2, pvv, 0.4, label="median Δ||P_vv||")
    ax.bar(x + 0.2, ppv, 0.4, label="median Δ||P_pv||")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_title("GAP-3.6 — reducción media post-pre por update")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3.6 cov step cycle audit")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--window-start", type=float, default=9.0)
    parser.add_argument("--window-end", type=float, default=12.0)
    parser.add_argument("--skip-run", action="store_true")
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    run_replay(args.replay_exe, replay_csv, args.calibration, args.skip_run)

    if not STEP_CSV.is_file():
        print(f"Falta {STEP_CSV}", file=sys.stderr)
        return 1

    df = pd.read_csv(STEP_CSV)
    window = (args.window_start, args.window_end)
    report = analyze(df, window)
    deltas = compute_step_deltas(df)
    predict = df[(df["update_type"] == "predict") & (df["phase"] == "post")]

    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    plot_window(df, deltas, window, WINDOW_PNG)
    plot_pvv_route(predict, P_VV_ROUTE_PNG)
    plot_reduction_bars(report["reduction_by_update"], REDUCTION_PNG)

    print(json.dumps(report, indent=2))
    print(f"Wrote {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
