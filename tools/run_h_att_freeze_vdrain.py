#!/usr/bin/env python3
"""Discriminate: does NHC velocity drain require attitude rotation?

H2 = att free (baseline). Freeze = q latched after seed + NHC H_att=0.
Compare ΣΔv_NHC over (4.301, 5.301].
"""

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
OUT = REPO / "docs" / "benchmarks" / "h_att_freeze_vdrain"
T0, T1 = 4.301, 5.301


def run_arm(name: str, freeze: bool) -> Path:
    arm = OUT / name
    arm.mkdir(parents=True, exist_ok=True)
    pipe = arm / "pipeline.csv"
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
        "--seed-velocity", "gnss",
        "--seed-yaw-from-course",
        "--replay-end-s", "6.5",
        "--gap3-constraint-pipeline-audit-csv", str(pipe),
        "--output", str(arm / "out.csv"),
    ]
    if freeze:
        cmd.append("--freeze-attitude-after-seed")
    print("RUN", name, flush=True)
    log = subprocess.run(cmd, cwd=str(REPO), check=True, capture_output=True, text=True)
    (arm / "replay.log").write_text(
        (log.stdout or "") + (log.stderr or ""), encoding="utf-8", errors="replace"
    )
    return pipe


def summarize(pipe: Path) -> dict:
    df = pd.read_csv(pipe)
    w = df[(df["timestamp_s"] > T0) & (df["timestamp_s"] <= T1)].copy()
    sp = w[["dv_pred_n", "dv_pred_e"]].sum()
    sn = w[["dv_nhc_n", "dv_nhc_e"]].sum()
    first = w.iloc[0]
    last = w.iloc[-1]
    return {
        "n_ticks": int(len(w)),
        "sum_dv_pred_n": float(sp["dv_pred_n"]),
        "sum_dv_pred_e": float(sp["dv_pred_e"]),
        "sum_dv_pred_h": float(np.hypot(sp["dv_pred_n"], sp["dv_pred_e"])),
        "sum_dv_nhc_n": float(sn["dv_nhc_n"]),
        "sum_dv_nhc_e": float(sn["dv_nhc_e"]),
        "sum_dv_nhc_h": float(np.hypot(sn["dv_nhc_n"], sn["dv_nhc_e"])),
        "v_first_after_nhc": [
            float(first["vel_after_nhc_n"]),
            float(first["vel_after_nhc_e"]),
            float(first["vel_h_mps"]),
        ],
        "v_last_after_nhc": [
            float(last["vel_after_nhc_n"]),
            float(last["vel_after_nhc_e"]),
            float(last["vel_h_mps"]),
        ],
        "nhc_applied_frac": float(w["nhc_applied"].mean()) if len(w) else None,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--skip-build", action="store_true")
    args = ap.parse_args()
    if not args.skip_build:
        subprocess.run(
            ["cmake", "--build", "build", "--target", "NaviCore3D_Replay"],
            cwd=str(REPO),
            check=True,
        )
    OUT.mkdir(parents=True, exist_ok=True)
    h2 = summarize(run_arm("H2", freeze=False))
    frz = summarize(run_arm("Freeze", freeze=True))

    # Hypothesis: freeze kills NHC horizontal drain (<< H2).
    drain_gone = frz["sum_dv_nhc_h"] < 2.0 and frz["sum_dv_nhc_h"] < 0.25 * h2["sum_dv_nhc_h"]
    verdict = {
        "H2": h2,
        "Freeze": frz,
        "hypothesis": "velocity drain requires attitude rotation after seed",
        "PASS_drain_requires_att_rotation": bool(drain_gone),
        "compare_sum_dv_nhc_h": {"H2": h2["sum_dv_nhc_h"], "Freeze": frz["sum_dv_nhc_h"]},
    }
    out = OUT / "verdict.json"
    out.write_text(json.dumps(verdict, indent=2), encoding="utf-8")
    print(json.dumps(verdict, indent=2))
    print(
        "PASS_att_required" if verdict["PASS_drain_requires_att_rotation"] else "FAIL_att_not_required",
        "->",
        out,
    )
    return 0 if verdict["PASS_drain_requires_att_rotation"] else 2


if __name__ == "__main__":
    sys.exit(main())
