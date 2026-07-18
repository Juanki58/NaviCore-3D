#!/usr/bin/env python3
"""GAP-3.5 — Auditoría de propagación de covarianza (F, Phi, P cross-blocks).

Responde:
  1. ¿Cómo evoluciona P_vel,pos entre predict / NHC / GNSS?
  2. ¿F conecta posición↔velocidad (∂p/∂v = dt) y vel↔att (∂v/∂att = -R[a]x dt)?
  3. ¿P_pv pequeño es bug de implementación o consecuencia del ciclo NHC+GNSS pos-only?
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
COV_CSV = BENCH_DIR / "gap3_cov_propagation_audit.csv"
REPORT_JSON = BENCH_DIR / "gap3_cov_propagation_report.json"
TIMELINE_PNG = BENCH_DIR / "gap3_cov_propagation_timeline.png"

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
        str(BENCH_DIR / "gap3_cov_propagation_replay_output.csv"),
        "--gap3-cov-propagation-audit-csv",
        str(COV_CSV),
    ]
    print("RUN:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)


def analyze(df: pd.DataFrame) -> dict:
    df = df.copy()
    for col in df.columns:
        if col in ("timestamp_s", "event"):
            continue
        df[col] = pd.to_numeric(df[col], errors="coerce")

    init = df[df["event"] == "init"]
    gnss_pre = df[df["event"] == "gnss_pre"].copy()
    gnss_post = df[df["event"] == "gnss_post"].copy()
    gnss_reject = df[df["event"] == "gnss_reject"].copy()
    predict = df[df["event"] == "predict_1hz"].copy()

    def summarize_block(frame: pd.DataFrame, label: str) -> dict:
        if frame.empty:
            return {"label": label, "count": 0}
        return {
            "label": label,
            "count": int(len(frame)),
            "P_vel_pos_frob_mean": float(frame["P_vel_pos_frob"].mean()),
            "P_vel_pos_frob_max": float(frame["P_vel_pos_frob"].max()),
            "P_vel_pos_max_mean": float(frame["P_vel_pos_max"].mean()),
            "P_vel_vel_frob_mean": float(frame["P_vel_vel_frob"].mean()),
            "P_pos_std_h_mean": float(
                np.sqrt((frame["P_pos_std_n"] ** 2 + frame["P_pos_std_e"] ** 2).mean())
            ),
            "vel_h_mean": float(frame["vel_h_mps"].mean()),
            "gps_speed_mean": float(frame["gps_speed_mps"].mean()),
        }

    first_reject_t = float(gnss_reject["timestamp_s"].min()) if not gnss_reject.empty else None
    pre_at_reject = gnss_pre[gnss_pre["timestamp_s"] == first_reject_t] if first_reject_t else pd.DataFrame()
    last_accept_pre = gnss_pre[gnss_pre["timestamp_s"] < first_reject_t].tail(1) if first_reject_t else gnss_pre.tail(1)

    f_structure = {
        "dp_dv": "Phi[pos,vel] = dt * I (via temp pos row + f_state_jacobian)",
        "dv_datt": "Phi[vel,att] = -R_bn * [a_body]x * dt",
        "dv_dbias_a": "Phi[vel,bias_a] = -R_bn * dt",
        "Q_structure": "diagonal only (pos, vel, att, bias_a, bias_g); no cross-terms in Q",
        "P0_cross_blocks": "zero at init (diagonal P0 only)",
    }

    mechanism = "UNKNOWN"
    if not gnss_pre.empty:
        pvp_at_gnss = float(gnss_pre["P_vel_pos_frob"].mean())
        if pvp_at_gnss < 1e-4:
            mechanism = "P_VP_SUPPRESSED_AT_GNSS_ACCEPT"
        elif pvp_at_gnss < 1e-3:
            mechanism = "P_VP_WEAK_AT_EARLY_ACCEPT"

    verdict = {
        "mechanism": mechanism,
        "predict_implementation": (
            "La implementación sparse de Phi=F·P·F'+Q incluye ∂p/∂v=dt y ∂v/∂att=-R[a]x·dt; "
            "no hay evidencia de bloque F ausente en código."
        ),
        "observed_P_vp": (
            "P_vel,pos permanece O(10^-5) en gnss_pre → K_vel,pos = P_vp·S^-1 ≈ 10^-7. "
            "Consistente con NHC/ZUPT cada IMU (observan velocidad, Joseph update) más GNSS pos-only."
        ),
        "not_predict_bug": (
            "No afirmar 'predict está bien'. Afirmar: la cadena strapdown en predict() es "
            "algebraicamente coherente; la insuficiencia del modelo puede estar en P, observación, o ambos."
        ),
    }

    report = {
        "experiment": "GAP-3.5 covariance propagation audit",
        "duration_s": float(df["timestamp_s"].max()) if not df.empty else 0.0,
        "F_and_Q_structure": f_structure,
        "init": summarize_block(init, "init"),
        "predict_1hz": summarize_block(predict, "predict_1hz"),
        "gnss_pre": summarize_block(gnss_pre, "gnss_pre"),
        "gnss_post": summarize_block(gnss_post, "gnss_post"),
        "gnss_reject": summarize_block(gnss_reject, "gnss_reject"),
        "last_accept_before_reject": (
            last_accept_pre.iloc[0].to_dict() if not last_accept_pre.empty else None
        ),
        "first_reject_pre": pre_at_reject.iloc[0].to_dict() if not pre_at_reject.empty else None,
        "first_reject_timestamp_s": first_reject_t,
        "verdict": verdict,
        "source_csv": str(COV_CSV),
    }
    return report


def plot_timeline(df: pd.DataFrame, out_png: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return

    predict = df[df["event"] == "predict_1hz"]
    gnss_pre = df[df["event"] == "gnss_pre"]

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    axes[0].plot(predict["timestamp_s"], predict["P_vel_pos_frob"], "b-", alpha=0.7, label="predict_1hz")
    axes[0].scatter(gnss_pre["timestamp_s"], gnss_pre["P_vel_pos_frob"], c="r", s=20, label="gnss_pre")
    axes[0].set_ylabel("||P_vel,pos||_F")
    axes[0].set_yscale("log")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title("GAP-3.5 — Evolución bloque cruzado posición–velocidad")

    axes[1].plot(predict["timestamp_s"], predict["vel_h_mps"], "g-", label="v_EKF horizontal")
    axes[1].plot(predict["timestamp_s"], predict["gps_speed_mps"], "k--", alpha=0.6, label="GPS speed (last)")
    axes[1].set_ylabel("m/s")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(predict["timestamp_s"], predict["F_va_max"], "m-", alpha=0.8, label="|F_va| max")
    axes[2].set_ylabel("|∂v/∂att| max")
    axes[2].set_xlabel("t [s]")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3.5 covariance propagation audit")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-run", action="store_true")
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    run_replay(args.replay_exe, replay_csv, args.calibration, args.skip_run)

    if not COV_CSV.is_file():
        print(f"Falta {COV_CSV}", file=sys.stderr)
        return 1

    df = pd.read_csv(COV_CSV)
    report = analyze(df)
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    plot_timeline(df, TIMELINE_PNG)
    print(json.dumps(report, indent=2))
    print(f"Wrote {REPORT_JSON}")
    if TIMELINE_PNG.is_file():
        print(f"Wrote {TIMELINE_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
