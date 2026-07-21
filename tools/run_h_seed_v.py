#!/usr/bin/env python3
"""Run H-seed ctrl / H1 / H2 and report residual, t_sep, course-yaw, GNSS accepts."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
REPLAY = REPO / "build" / "NaviCore3D_Replay.exe"
INPUT = REPO / "docs" / "benchmarks" / "real_run_19082026_baseline" / "real_run_replay.csv"
MOUNT = REPO / "calibration" / "imu_mount.json"
OUT = REPO / "docs" / "benchmarks" / "h_seed_v"
T_SEED = 4.301  # first eligible GNSS for H1/H2 on this pack


def run_arm(name: str, seed_velocity: str, yaw_from_course: bool, end_s: float) -> Path:
    arm = OUT / name
    arm.mkdir(parents=True, exist_ok=True)
    tick = arm / "tick_stage_audit.csv"
    pipeline = arm / "constraint_pipeline_audit.csv"
    output = arm / "replay_output.csv"
    cmd = [
        str(REPLAY),
        "--input", str(INPUT),
        "--mount-mode", "calibration",
        "--mount-calibration", str(MOUNT),
        "--yaw-init", "zero",
        "--h9a-gravity-tilt-init",
        "--constraint-policy", "disabled",
        "--nhc-policy", "enabled",
        "--gnss-obs-mode", "pos_vel",
        "--p-pv-policy", "none",
        "--seed-velocity", seed_velocity,
        "--replay-end-s", str(end_s),
        "--tick-stage-audit-csv", str(tick),
        "--gap3-constraint-pipeline-audit-csv", str(pipeline),
        "--output", str(output),
    ]
    if yaw_from_course:
        cmd.append("--seed-yaw-from-course")
    print("RUN", name, " ".join(cmd[-8:]))
    subprocess.run(cmd, cwd=str(REPO), check=True)
    return tick


def t_sep_residual(df: pd.DataFrame, thr: float = 30.0, hold_s: float = 10.0) -> float | None:
    """First t where residual_h stays > thr for hold_s (edge B preferred)."""
    b = df[df["edge"] == "B_post_nhc_zupt"].sort_values("timestamp_s")
    if b.empty or "residual_h_m" not in b.columns:
        return None
    t = b["timestamp_s"].to_numpy()
    r = b["residual_h_m"].to_numpy()
    for i in range(len(t)):
        if r[i] <= thr:
            continue
        later = r[(t >= t[i]) & (t <= t[i] + hold_s)]
        if len(later) > 5 and np.min(later) > thr:
            return float(t[i])
    return None


def eval_arm(tick_csv: Path, t_max: float = 6.0) -> dict:
    df = pd.read_csv(tick_csv)
    w = df[df["timestamp_s"] <= t_max].copy()
    b = w[w["edge"] == "B_post_nhc_zupt"]
    out: dict = {"n_B": int(len(b))}

    if "inv_speed_vs_gps_ok" in b.columns:
        valid = b[b["inv_speed_vs_gps_ok"] >= 0]
        out["P1_fail_frac"] = (
            float((valid["inv_speed_vs_gps_ok"] == 0).mean()) if len(valid) else None
        )
        out["P1_n_valid"] = int(len(valid))

    pipe = tick_csv.parent / "constraint_pipeline_audit.csv"
    p2_t = None
    if pipe.exists():
        p = pd.read_csv(pipe)
        vh = np.hypot(p["vel_after_nhc_n"], p["vel_after_nhc_e"])
        gps = p["gps_speed_mps"]
        hit = p[(gps >= 3.0) & (vh >= 0.25 * gps)]
        if len(hit):
            p2_t = float(hit.iloc[0]["timestamp_s"])
    out["P2_t_s"] = p2_t

    a = w[w["edge"] == "A_post_predict"].sort_values("timestamp_s")
    prev_ok = None
    p3_t = None
    for _, r in a.iterrows():
        v = int(r["inv_course_yaw_ok"])
        if v < 0:
            continue
        ok = v == 1
        if prev_ok is True and not ok:
            p3_t = float(r["timestamp_s"])
            break
        prev_ok = ok
    out["P3_first_fail_t_s"] = p3_t

    # P3a: OK->FAIL in [t_seed, t_seed+2]
    p3a_fail = False
    prev_ok = None
    for _, r in a.iterrows():
        t = float(r["timestamp_s"])
        if t < T_SEED or t > T_SEED + 2.0:
            if t > T_SEED + 2.0:
                break
            continue
        v = int(r["inv_course_yaw_ok"])
        if v < 0:
            continue
        ok = v == 1
        if prev_ok is True and not ok:
            p3a_fail = True
            out["P3a_fail_t_s"] = t
            break
        prev_ok = ok
    out["P3a_immediate_break"] = p3a_fail

    out["speed_max_B"] = float(b["speed_mps"].max()) if len(b) else None
    if "residual_h_m" in b.columns and len(b):
        out["residual_h_at_6s"] = float(b["residual_h_m"].iloc[-1])
        out["residual_h_max_0_6"] = float(b["residual_h_m"].max())
    out["t_sep_s"] = t_sep_residual(df)

    # GNSS accept/reject from C edges
    c = df[df["edge"] == "C_post_gnss"]
    if len(c) and "gnss_accepted" in c.columns:
        out["gnss_accept"] = int((c["gnss_accepted"] == 1).sum())
        out["gnss_reject"] = int((c["gnss_accepted"] == 0).sum())
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--end-s", type=float, default=60.0, help="replay length (default 60)")
    ap.add_argument("--only-h2", action="store_true", help="reuse ctrl/H1 if present")
    args = ap.parse_args()

    if not args.skip_build:
        subprocess.run(
            ["cmake", "--build", "build", "--target", "NaviCore3D_Replay"],
            cwd=str(REPO),
            check=True,
        )
    if not REPLAY.exists():
        print("missing", REPLAY)
        return 1

    if not args.only_h2:
        run_arm("ctrl", "zero", False, args.end_s)
        run_arm("H1", "gnss", False, args.end_s)
    run_arm("H2", "gnss", True, args.end_s)

    ctrl = eval_arm(OUT / "ctrl" / "tick_stage_audit.csv")
    h1 = eval_arm(OUT / "H1" / "tick_stage_audit.csv")
    h2 = eval_arm(OUT / "H2" / "tick_stage_audit.csv")

    p1 = h2.get("P1_fail_frac") is not None and h2["P1_fail_frac"] <= 0.30
    p3a = not h2.get("P3a_immediate_break", True)
    p3b = h2.get("P3_first_fail_t_s") is None or (
        ctrl.get("P3_first_fail_t_s") is not None
        and h2["P3_first_fail_t_s"] >= ctrl["P3_first_fail_t_s"] + 2.0
    )

    verdict = {
        "ctrl": ctrl,
        "H1": h1,
        "H2": h2,
        "gates_H2": {
            "P1_speed_vs_gps": p1,
            "P3a_no_break_in_2s_after_seed": p3a,
            "P3b_course_yaw_delay_or_absent": p3b,
        },
        "PASS_H2": bool(p1 and p3a and p3b),
        "compare": {
            "residual_h_at_6s": {
                "ctrl": ctrl.get("residual_h_at_6s"),
                "H1": h1.get("residual_h_at_6s"),
                "H2": h2.get("residual_h_at_6s"),
            },
            "t_sep_s": {
                "ctrl": ctrl.get("t_sep_s"),
                "H1": h1.get("t_sep_s"),
                "H2": h2.get("t_sep_s"),
            },
            "P3_first_fail_t_s": {
                "ctrl": ctrl.get("P3_first_fail_t_s"),
                "H1": h1.get("P3_first_fail_t_s"),
                "H2": h2.get("P3_first_fail_t_s"),
            },
            "gnss_accept_reject": {
                "ctrl": [ctrl.get("gnss_accept"), ctrl.get("gnss_reject")],
                "H1": [h1.get("gnss_accept"), h1.get("gnss_reject")],
                "H2": [h2.get("gnss_accept"), h2.get("gnss_reject")],
            },
        },
    }
    out_json = OUT / "verdict_h2.json"
    out_json.write_text(json.dumps(verdict, indent=2), encoding="utf-8")
    print(json.dumps(verdict, indent=2))
    print("PASS_H2" if verdict["PASS_H2"] else "FAIL_H2", "->", out_json)
    return 0 if verdict["PASS_H2"] else 2


if __name__ == "__main__":
    sys.exit(main())
