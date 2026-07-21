#!/usr/bin/env python3
"""GAP-4 — Ejecutar brazo G0/G2/G1 con instrumentación y veredicto §3.1."""

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
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
DEFAULT_REPLAY_CSV = REPO_ROOT / "docs/benchmarks" / "real_run_replay.csv"

G0_EXPECT = {
    "accepts": 7,
    "Lambda_n_fix8": 2.34,
    "contrib_n_pct_fix8": 59.0,
    "innov_n_fix8": 27.4,
    "innov_h_accept_mean": 27.2,
    "P_vv_pre_fix3": 2.50,
    "k_vel_fix3": 0.0078,
    "k_vel_mean_accepts": 0.0359,
    "k_vel_mean_rejects": 0.0356,
}

P1_LAMBDA_N_MAX = 1.87
P2_ACCEPTS_MIN = 8

ARM_CONFIG = {
    "G0": {"gnss_obs_mode": "pos", "out_subdir": "G0"},
    "G2": {"gnss_obs_mode": "vel_only", "out_subdir": "G2"},
    "G1": {"gnss_obs_mode": "pos_vel", "out_subdir": "G1"},
}

PPV_POLICIES = ("none", "gap_le_1s", "zero", "cos_pos", "cos_tot", "innov_h")

T_FIX3 = 6.053678513

sys.path.insert(0, str(REPO_ROOT))
from gap4_abort_guardrail import evaluate_abort  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


def load_csv(path: Path) -> pd.DataFrame:
    if not path.is_file():
        return pd.DataFrame()
    df = pd.read_csv(path, index_col=False)
    skip = {"update_type", "phase", "reject_reason"}
    for col in df.columns:
        if col in skip:
            continue
        c = pd.to_numeric(df[col], errors="coerce")
        if c.notna().any():
            df[col] = c
    return df


def run_replay(
    replay_exe: Path,
    replay_csv: Path,
    calibration: Path,
    out_dir: Path,
    gnss_obs_mode: str,
    ppv_policy: str = "none",
    replay_end_s: float | None = None,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(replay_exe),
        "--input", str(replay_csv),
        "--mount-mode", "calibration",
        "--mount-calibration", str(calibration),
        "--yaw-init", "zero",
        "--h9a-gravity-tilt-init",
        "--constraint-policy", "disabled",
        "--nhc-policy", "enabled",
        "--gnss-obs-mode", gnss_obs_mode,
        "--p-pv-policy", ppv_policy,
        "--output", str(out_dir / "replay_output.csv"),
        "--gap3-gnss-nis-audit-csv", str(out_dir / "gnss_nis_audit.csv"),
        "--gap3-nhc-block-audit-csv", str(out_dir / "nhc_block_audit.csv"),
        "--gap3-cov-step-audit-csv", str(out_dir / "cov_step_audit.csv"),
        "--gap3-constraint-pipeline-audit-csv", str(out_dir / "constraint_pipeline_audit.csv"),
        "--gap3-gnss-k-block-audit-json", str(out_dir / "gnss_k_block.jsonl"),
    ]
    if replay_end_s is not None and replay_end_s > 0:
        cmd.extend(["--replay-end-s", str(replay_end_s)])
    print("RUN:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)


def lambda_n(row: pd.Series) -> float:
    s = float(row["s_nn"])
    if s <= 0:
        return math.nan
    return abs(float(row["innov_n_m"])) / math.sqrt(s)


def gnss_dP_over_P(cov: pd.DataFrame) -> list[dict]:
    rows = []
    for _, pre in cov[(cov["update_type"] == "gnss") & (cov["phase"] == "pre")].iterrows():
        ts = float(pre["timestamp_s"])
        post = cov[
            (cov["update_type"] == "gnss")
            & (cov["phase"].isin(["post_accept", "post_reject"]))
            & (np.isclose(cov["timestamp_s"], ts, atol=1e-3))
        ]
        if post.empty:
            continue
        p_pre = float(pre["P_vv_frob"])
        p_post = float(post.iloc[0]["P_vv_frob"])
        dP = p_post - p_pre
        ratio = abs(dP) / p_pre if p_pre > 1e-9 else math.nan
        rows.append(
            {
                "timestamp_s": ts,
                "P_vv_pre": p_pre,
                "P_vv_post": p_post,
                "delta_P_vv": dP,
                "dP_over_P_pre": ratio,
                "phase": str(post.iloc[0]["phase"]),
            }
        )
    return rows


def verdict_h1(*, p1: bool, p2: bool, p5: bool, abort: bool) -> str:
    if abort:
        return "ABORT"
    if p1 and p2 and p5:
        return "H1 CONFIRMADA"
    if p1 and p5 and not p2:
        return "H1 PARCIAL — vía M2"
    if p2 and p5 and not p1:
        return "H1 PARCIAL — vía gate"
    return "H1 REFUTADA"


def analyze(arm: str, out_dir: Path, ppv_policy: str = "none") -> dict:
    gnss = load_csv(out_dir / "gnss_nis_audit.csv")
    cov = load_csv(out_dir / "cov_step_audit.csv")

    acc = gnss[gnss["accepted"] == 1]
    rej = gnss[gnss["accepted"] == 0]
    accepts = int(len(acc))

    k_vel_acc = float(acc["k_vel_max"].mean()) if len(acc) else math.nan
    k_vel_rej = float(rej["k_vel_max"].mean()) if len(rej) else math.nan

    fix8 = rej[rej["gps_index"] == 8]
    fix8_row = fix8.iloc[0] if len(fix8) else None
    lam8 = lambda_n(fix8_row) if fix8_row is not None else math.nan
    contrib_n_pct = (
        100.0 * float(fix8_row["nis_contrib_n"]) / float(fix8_row["nis_full"])
        if fix8_row is not None and fix8_row["nis_full"] > 0
        else math.nan
    )

    f3_pre = cov[
        (cov["update_type"] == "gnss")
        & (cov["phase"] == "pre")
        & (np.isclose(cov["timestamp_s"], T_FIX3, atol=1e-3))
    ]
    pvv_pre3 = float(f3_pre.iloc[0]["P_vv_frob"]) if len(f3_pre) else math.nan

    f3_gnss = acc[acc["gps_index"] == 3]
    k_vel3 = float(f3_gnss.iloc[0]["k_vel_max"]) if len(f3_gnss) else math.nan

    dP_rows = gnss_dP_over_P(cov)
    max_dP_ratio_accept = max(
        (r["dP_over_P_pre"] for r in dP_rows if r["phase"] == "post_accept"),
        default=math.nan,
    )
    max_k_vel_accept = float(acc["k_vel_max"].max()) if len(acc) else math.nan

    obs_mode = ARM_CONFIG[arm]["gnss_obs_mode"]
    if arm == "G2" and ppv_policy != "none":
        obs_mode = "pos_vel"
    abort_eval = evaluate_abort(gnss, dP_rows, obs_mode)  # type: ignore[arg-type]
    abort = abort_eval["abort"]
    abort_flags = abort_eval["abort_flags"]

    p1 = not math.isnan(lam8) and lam8 <= P1_LAMBDA_N_MAX
    p2 = accepts >= P2_ACCEPTS_MIN
    p5 = not abort

    report = {
        "arm": arm,
        "gnss_obs_mode": obs_mode,
        "ppv_policy": ppv_policy,
        "accepts": accepts,
        "k_vel_mean_accepts": k_vel_acc,
        "k_vel_mean_rejects": k_vel_rej,
        "Lambda_n_fix8": lam8,
        "contrib_n_pct_fix8": contrib_n_pct,
        "innov_n_fix8": float(fix8_row["innov_n_m"]) if fix8_row is not None else math.nan,
        "innov_h_accept_mean": float(acc["innov_h_m"].mean()) if len(acc) else math.nan,
        "P_vv_pre_fix3": pvv_pre3,
        "k_vel_fix3": k_vel3,
        "max_k_vel_accept": max_k_vel_accept,
        "max_dP_over_P_accept": max_dP_ratio_accept,
        "gnss_dP_audit": dP_rows,
        "abort_guardrail": abort_eval,
        "abort_flags": abort_flags,
        "criteria": {
            "P1_Lambda_n_le_1.87": p1,
            "P2_accepts_ge_8": p2,
            "P5_no_abort": p5,
        },
        "verdict_h1": verdict_h1(p1=p1, p2=p2, p5=p5, abort=abort),
        "g0_reference": G0_EXPECT if arm != "G0" else None,
    }

    if arm == "G0":
        report["checks"] = {
            "accepts_eq_7": accepts == G0_EXPECT["accepts"],
            "Lambda_n_fix8_within_5pct": abs(lam8 - G0_EXPECT["Lambda_n_fix8"])
            / G0_EXPECT["Lambda_n_fix8"]
            <= 0.05
            if not math.isnan(lam8)
            else False,
            "k_vel_flat_acc_rej": abs(k_vel_acc - k_vel_rej) < 0.005
            if not math.isnan(k_vel_acc)
            else False,
            "no_abort": p5,
        }
        report["g0_pass"] = all(report["checks"].values())
        report["expected"] = G0_EXPECT

    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=sorted(ARM_CONFIG), default="G0")
    parser.add_argument("--ppv-policy", choices=PPV_POLICIES, default="none")
    parser.add_argument("--out-subdir", type=str, default=None)
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--replay-csv", type=Path, default=DEFAULT_REPLAY_CSV)
    parser.add_argument("--replay-end-s", type=float, default=None)
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()

    cfg = ARM_CONFIG[args.arm]
    subdir = args.out_subdir or cfg["out_subdir"]
    gnss_mode = cfg["gnss_obs_mode"]
    if args.arm == "G2" and args.out_subdir:
        gnss_mode = "pos_vel"
    out_dir = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity" / subdir
    report_path = out_dir / f"gap4_{args.arm.lower()}_report.json"
    if args.ppv_policy != "none":
        report_path = out_dir / f"gap4_{args.arm.lower()}_{args.ppv_policy}_report.json"

    ensure_calibration(args.calibration)

    if not args.skip_replay:
        if not args.replay_exe.is_file():
            print(f"ERROR: missing replay exe: {args.replay_exe}", file=sys.stderr)
            return 1
        if not args.replay_csv.is_file():
            print(f"ERROR: missing replay csv: {args.replay_csv}", file=sys.stderr)
            return 1
        run_replay(
            args.replay_exe,
            args.replay_csv,
            args.calibration,
            out_dir,
            gnss_mode,
            args.ppv_policy,
            args.replay_end_s,
        )

    report = analyze(args.arm, out_dir, args.ppv_policy)
    report["out_subdir"] = subdir
    out_dir.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2))
    print(f"\nWrote {report_path}")
    if args.arm == "G0":
        print(f"G0 PASS: {report.get('g0_pass')}")
        return 0 if report.get("g0_pass") else 1
    print(f"Veredicto §3.1: {report['verdict_h1']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
