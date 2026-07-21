#!/usr/bin/env python3
"""Control A/B: G1 NHC enabled vs Pico-equivalent NHC disabled.

Same log, same EKF shell (G1 minus NHC policy), single variable: --nhc-policy.
No seeds, gates, P/R/H changes.

Compares:
- velocity drain window (4.301, 5.301] from constraint pipeline
- GNSS accept/reject + first permanent reject
- residual_h (km-scale drift proxy)
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

REPO = Path(__file__).resolve().parents[1]
REPLAY = REPO / "build" / "NaviCore3D_Replay.exe"
INPUT = REPO / "docs" / "benchmarks" / "real_run_19082026_baseline" / "real_run_replay.csv"
MOUNT = REPO / "calibration" / "imu_mount.json"
OUT = REPO / "docs" / "benchmarks" / "h_nhc_policy_ab"
T0, T1 = 4.301, 5.301


def run_arm(name: str, nhc_policy: str) -> Path:
    arm = OUT / name
    arm.mkdir(parents=True, exist_ok=True)
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
        nhc_policy,
        "--gnss-obs-mode",
        "pos_vel",
        "--p-pv-policy",
        "none",
        "--output",
        str(arm / "replay_output.csv"),
        "--gap3-gnss-nis-audit-csv",
        str(arm / "gnss_nis_audit.csv"),
        "--gap3-nhc-block-audit-csv",
        str(arm / "nhc_block_audit.csv"),
        "--gap3-cov-step-audit-csv",
        str(arm / "cov_step_audit.csv"),
        "--gap3-constraint-pipeline-audit-csv",
        str(arm / "constraint_pipeline_audit.csv"),
    ]
    print("RUN", name, "nhc=", nhc_policy, flush=True)
    log = subprocess.run(cmd, cwd=str(REPO), check=True, capture_output=True, text=True)
    (arm / "replay.log").write_text(
        (log.stdout or "") + (log.stderr or ""), encoding="utf-8", errors="replace"
    )
    return arm


def summarize_vel_drain(pipe: Path) -> dict:
    if not pipe.is_file():
        return {"available": False}
    df = pd.read_csv(pipe)
    need = {"timestamp_s", "dv_nhc_n", "dv_nhc_e", "dv_pred_n", "dv_pred_e", "vel_h_mps", "nhc_applied"}
    if not need.issubset(df.columns):
        return {"available": False, "missing_cols": sorted(need - set(df.columns))}
    w = df[(df["timestamp_s"] > T0) & (df["timestamp_s"] <= T1)].copy()
    if w.empty:
        return {"available": True, "n_ticks": 0}
    sp = w[["dv_pred_n", "dv_pred_e"]].sum()
    sn = w[["dv_nhc_n", "dv_nhc_e"]].sum()
    first = w.iloc[0]
    last = w.iloc[-1]
    return {
        "available": True,
        "window_s": [T0, T1],
        "n_ticks": int(len(w)),
        "sum_dv_pred_h": float(np.hypot(sp["dv_pred_n"], sp["dv_pred_e"])),
        "sum_dv_nhc_h": float(np.hypot(sn["dv_nhc_n"], sn["dv_nhc_e"])),
        "v_h_first": float(first["vel_h_mps"]),
        "v_h_last": float(last["vel_h_mps"]),
        "nhc_applied_frac": float(w["nhc_applied"].mean()),
    }


def summarize_gnss(path: Path) -> dict:
    if not path.is_file():
        return {"available": False}
    df = pd.read_csv(path)
    if "accepted" not in df.columns:
        return {"available": False, "missing": "accepted"}
    n = len(df)
    n_acc = int((df["accepted"] == 1).sum())
    n_rej = int((df["accepted"] == 0).sum())
    t_first_perm = None
    # First reject after which no further accepts
    accepts = df["accepted"].to_numpy()
    times = df["timestamp_s"].to_numpy() if "timestamp_s" in df.columns else None
    if times is not None and n:
        last_acc_idx = None
        for i, a in enumerate(accepts):
            if a == 1:
                last_acc_idx = i
        if last_acc_idx is not None and last_acc_idx + 1 < n:
            # first reject after last accept? better: onset of permanent reject streak
            for i in range(n):
                if accepts[i] == 0 and (accepts[i:] == 0).all():
                    t_first_perm = float(times[i])
                    break
        elif last_acc_idx is None and n:
            t_first_perm = float(times[0])
    return {
        "available": True,
        "n_events": n,
        "n_accept": n_acc,
        "n_reject": n_rej,
        "accept_rate": float(n_acc / n) if n else None,
        "t_first_permanent_reject_s": t_first_perm,
    }


def summarize_residual(path: Path) -> dict:
    if not path.is_file():
        return {"available": False}
    df = pd.read_csv(path)
    if "residual_h_m" not in df.columns:
        # fallback: compute from pos vs gps if present
        cols = set(df.columns)
        if {"pos_n_m", "pos_e_m", "gps_n_m", "gps_e_m"}.issubset(cols):
            mask = df["gps_n_m"].notna() & df["gps_e_m"].notna()
            r = np.hypot(
                df.loc[mask, "pos_n_m"] - df.loc[mask, "gps_n_m"],
                df.loc[mask, "pos_e_m"] - df.loc[mask, "gps_e_m"],
            )
            t = df.loc[mask, "timestamp_s"] if "timestamp_s" in cols else None
        else:
            return {"available": False, "missing": "residual_h_m"}
    else:
        r = df["residual_h_m"].dropna()
        t = df.loc[r.index, "timestamp_s"] if "timestamp_s" in df.columns else None
        r = r.to_numpy()
    if len(r) == 0:
        return {"available": True, "n": 0}
    out = {
        "available": True,
        "n": int(len(r)),
        "residual_h_final_m": float(r[-1]),
        "residual_h_max_m": float(np.max(r)),
        "residual_h_p50_m": float(np.median(r)),
        "residual_h_p95_m": float(np.percentile(r, 95)),
        "residual_h_final_km": float(r[-1] / 1000.0),
        "residual_h_max_km": float(np.max(r) / 1000.0),
    }
    if t is not None and len(t):
        t = np.asarray(t)
        for mark in (10.0, 60.0, 300.0, 600.0):
            w = r[t <= mark]
            if len(w):
                out[f"residual_h_at_{int(mark)}s_m"] = float(w[-1])
    return out


def summarize_arm(arm: Path, nhc_policy: str) -> dict:
    return {
        "nhc_policy": nhc_policy,
        "vel_drain": summarize_vel_drain(arm / "constraint_pipeline_audit.csv"),
        "gnss": summarize_gnss(arm / "gnss_nis_audit.csv"),
        "residual": summarize_residual(arm / "replay_output.csv"),
    }


def classify(a: dict, b: dict) -> dict:
    """Preregistered discrimination — language stays conservative."""
    va = a["vel_drain"]
    vb = b["vel_drain"]
    ga = a["gnss"]
    gb = b["gnss"]
    ra = a["residual"]
    rb = b["residual"]

    drain_gone = False
    if va.get("available") and vb.get("available"):
        # B has near-zero NHC drain and does not collapse v_h like A
        drain_gone = (
            vb.get("sum_dv_nhc_h", 0) is not None
            and float(vb.get("sum_dv_nhc_h", 0)) < 2.0
            and float(vb.get("sum_dv_nhc_h", 0)) < 0.25 * max(float(va.get("sum_dv_nhc_h", 0)), 1e-9)
        )

    gnss_recovered = False
    if ga.get("available") and gb.get("available"):
        # Material recovery: accept rate clearly higher on B
        ar_a = ga.get("accept_rate") or 0.0
        ar_b = gb.get("accept_rate") or 0.0
        gnss_recovered = ar_b >= 0.5 and ar_b > ar_a + 0.3

    residual_gone = False
    if ra.get("available") and rb.get("available"):
        # km-scale gone if B final << 1 km and << 25% of A
        fa = ra.get("residual_h_final_m")
        fb = rb.get("residual_h_final_m")
        if fa is not None and fb is not None:
            residual_gone = fb < 1000.0 and fb < 0.25 * max(fa, 1.0)

    # Mechanism attributable to G1 NHC policy only if all three collapse on B
    mechanism_g1_only = drain_gone and gnss_recovered and residual_gone
    # Mechanism independent of NHC if B still shows early reject + km residual
    still_fails_without_nhc = False
    if gb.get("available") and rb.get("available"):
        early_rej = (
            gb.get("t_first_permanent_reject_s") is not None
            and float(gb["t_first_permanent_reject_s"]) < 30.0
            and (gb.get("accept_rate") or 1.0) < 0.1
        )
        km = (rb.get("residual_h_final_m") or 0) > 1000.0
        still_fails_without_nhc = early_rej and km

    return {
        "drain_absent_on_B": bool(drain_gone),
        "gnss_accept_recovered_on_B": bool(gnss_recovered),
        "residual_km_absent_on_B": bool(residual_gone),
        "mechanism_collapses_without_NHC": bool(mechanism_g1_only),
        "B_still_shows_early_reject_and_km_residual": bool(still_fails_without_nhc),
        "interpretation_keys": {
            "if_mechanism_collapses": (
                "Phenomenon under study is tied to G1 NHC-enabled harness policy; "
                "not demonstrated under Pico-equivalent NHC-off."
            ),
            "if_B_still_fails": (
                "NHC-on is not required for early GNSS reject + km residual; "
                "investigation returns to EKF under product-equivalent policy."
            ),
            "partial": (
                "Mixed outcome — report per-metric; do not claim full harness induction "
                "nor full EKF independence from NHC."
            ),
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--skip-replay", action="store_true")
    args = ap.parse_args()

    if not REPLAY.is_file() and not args.skip_replay:
        print("Missing", REPLAY, file=sys.stderr)
        return 1
    if not INPUT.is_file():
        print("Missing", INPUT, file=sys.stderr)
        return 1

    if not args.skip_build and not args.skip_replay:
        subprocess.run(
            ["cmake", "--build", "build", "--target", "NaviCore3D_Replay"],
            cwd=str(REPO),
            check=True,
        )

    OUT.mkdir(parents=True, exist_ok=True)

    if not args.skip_replay:
        run_arm("A_nhc_enabled", "enabled")
        run_arm("B_nhc_disabled", "disabled")

    a = summarize_arm(OUT / "A_nhc_enabled", "enabled")
    b = summarize_arm(OUT / "B_nhc_disabled", "disabled")
    verdict = {
        "protocol": {
            "input": str(INPUT.relative_to(REPO)),
            "single_variable": "--nhc-policy enabled vs disabled",
            "shared": [
                "constraint-policy disabled",
                "gnss-obs-mode pos_vel",
                "p-pv-policy none",
                "yaw-init zero + h9a-gravity-tilt-init",
                "mount calibration",
                "no seed-velocity / seed-yaw / gates",
            ],
            "note": (
                "A = G1 shell NHC on; B = Pico-equivalent NHC off. "
                "Does not claim B equals full field firmware — only NHC arming."
            ),
        },
        "A_nhc_enabled": a,
        "B_nhc_disabled": b,
        "discrimination": classify(a, b),
    }

    out = OUT / "verdict.json"
    out.write_text(json.dumps(verdict, indent=2), encoding="utf-8")
    print(json.dumps(verdict, indent=2))
    d = verdict["discrimination"]
    if d["mechanism_collapses_without_NHC"]:
        label = "RESULT_mechanism_collapses_without_NHC"
    elif d["B_still_shows_early_reject_and_km_residual"]:
        label = "RESULT_B_still_fails"
    else:
        label = "RESULT_partial_or_mixed"
    print(label, "->", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
