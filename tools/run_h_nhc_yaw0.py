#!/usr/bin/env python3
"""H2 vs H2+NHC ATT_Z unobs-immediate: does blocking NHC→yaw stop the 22 ms overwrite?"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
REPLAY = REPO / "build" / "NaviCore3D_Replay.exe"
INPUT = REPO / "docs" / "benchmarks" / "real_run_19082026_baseline" / "real_run_replay.csv"
MOUNT = REPO / "calibration" / "imu_mount.json"
OUT = REPO / "docs" / "benchmarks" / "h_nhc_yaw0"
T_SEED = 4.301

YAW_CHASE_RE = re.compile(
    r"YAW_CHASE deg \| before=([-\d.]+) after_seed=([-\d.]+) after_update=([-\d.]+) "
    r"after_pred1=([-\d.]+) after_pred2=([-\d.]+) \| "
    r"t_seed=([-\d.]+) t_upd=([-\d.]+)\(([^)]+)\) t_p1=([-\d.]+) t_p2=([-\d.]+)"
)


def run_arm(
    name: str,
    *,
    att_z_unobs: bool = False,
    att_unobs: bool = False,
    end_s: float,
) -> tuple[Path, str]:
    arm = OUT / name
    arm.mkdir(parents=True, exist_ok=True)
    tick = arm / "tick_stage_audit.csv"
    pipeline = arm / "constraint_pipeline_audit.csv"
    output = arm / "replay_output.csv"
    log_path = arm / "replay.log"
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
        "--replay-end-s", str(end_s),
        "--tick-stage-audit-csv", str(tick),
        "--gap3-constraint-pipeline-audit-csv", str(pipeline),
        "--output", str(output),
    ]
    if att_unobs:
        cmd.append("--nhc-att-unobs-immediate")
    elif att_z_unobs:
        cmd.append("--nhc-att-z-unobs-immediate")
    print("RUN", name, flush=True)
    proc = subprocess.run(cmd, cwd=str(REPO), check=True, capture_output=True, text=True)
    log = (proc.stdout or "") + (proc.stderr or "")
    log_path.write_text(log, encoding="utf-8")
    return tick, log


def parse_yaw_chase(log: str) -> dict | None:
    m = YAW_CHASE_RE.search(log)
    if not m:
        return None
    return {
        "before": float(m.group(1)),
        "after_seed": float(m.group(2)),
        "after_update": float(m.group(3)),
        "after_pred1": float(m.group(4)),
        "after_pred2": float(m.group(5)),
        "t_seed": float(m.group(6)),
        "t_upd": float(m.group(7)),
        "upd_tag": m.group(8),
        "t_p1": float(m.group(9)),
        "t_p2": float(m.group(10)),
        "dyaw_seed_to_pred1": float(m.group(4)) - float(m.group(2)),
        "dyaw_pred1_to_pred2": float(m.group(5)) - float(m.group(4)),
    }


def t_sep_residual(df: pd.DataFrame, thr: float = 30.0, hold_s: float = 10.0) -> float | None:
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


def eval_arm(tick_csv: Path, log: str, t_max: float = 6.0) -> dict:
    df = pd.read_csv(tick_csv)
    w = df[df["timestamp_s"] <= t_max].copy()
    b = w[w["edge"] == "B_post_nhc_zupt"]
    a = w[w["edge"] == "A_post_predict"].sort_values("timestamp_s")
    out: dict = {"n_B": int(len(b)), "yaw_chase": parse_yaw_chase(log)}

    if "inv_speed_vs_gps_ok" in b.columns:
        valid = b[b["inv_speed_vs_gps_ok"] >= 0]
        out["P1_fail_frac"] = (
            float((valid["inv_speed_vs_gps_ok"] == 0).mean()) if len(valid) else None
        )

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

    # Post-seed B edges: max |dyaw| and course-yaw
    b_post = b[b["timestamp_s"] >= T_SEED]
    if len(b_post) and "dyaw_deg" in b_post.columns:
        out["max_abs_dyaw_B_post_seed"] = float(b_post["dyaw_deg"].abs().max())
    if len(b_post) and "abs_course_yaw_deg" in b_post.columns:
        out["max_abs_course_yaw_B_post_seed_2s"] = float(
            b_post[b_post["timestamp_s"] <= T_SEED + 2.0]["abs_course_yaw_deg"].max()
        )

    if "residual_h_m" in b.columns and len(b):
        out["residual_h_at_6s"] = float(b["residual_h_m"].iloc[-1])
        out["residual_h_max_0_6"] = float(b["residual_h_m"].max())
    out["t_sep_s"] = t_sep_residual(df)

    c = df[df["edge"] == "C_post_gnss"]
    if len(c) and "gnss_accepted" in c.columns:
        out["gnss_accept"] = int((c["gnss_accepted"] == 1).sum())
        out["gnss_reject"] = int((c["gnss_accepted"] == 0).sum())
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--end-s", type=float, default=60.0)
    ap.add_argument(
        "--only-h2ua",
        action="store_true",
        help="run full-attitude unobs arm only; reuse H2/H2u logs if present",
    )
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

    OUT.mkdir(parents=True, exist_ok=True)
    if not args.only_h2ua:
        tick_h2, log_h2 = run_arm("H2", end_s=args.end_s)
        tick_h2u, log_h2u = run_arm("H2u", att_z_unobs=True, end_s=args.end_s)
    else:
        tick_h2 = OUT / "H2" / "tick_stage_audit.csv"
        tick_h2u = OUT / "H2u" / "tick_stage_audit.csv"
        log_h2 = (OUT / "H2" / "replay.log").read_text(encoding="utf-8", errors="replace")
        log_h2u = (OUT / "H2u" / "replay.log").read_text(encoding="utf-8", errors="replace")

    tick_h2ua, log_h2ua = run_arm("H2ua", att_unobs=True, end_s=args.end_s)

    h2 = eval_arm(tick_h2, log_h2)
    h2u = eval_arm(tick_h2u, log_h2u)
    h2ua = eval_arm(tick_h2ua, log_h2ua)

    def yaw_held(arm: dict) -> bool:
        yc = arm.get("yaw_chase") or {}
        return (
            yc.get("dyaw_seed_to_pred1") is not None
            and abs(yc["dyaw_seed_to_pred1"]) < 2.0
            and abs(yc.get("dyaw_pred1_to_pred2", 99.0)) < 2.0
        )

    p3a_ok = not h2ua.get("P3a_immediate_break", True)
    held = yaw_held(h2ua)

    verdict = {
        "H2": h2,
        "H2u": h2u,
        "H2ua": h2ua,
        "gates_H2ua": {
            "yaw_held_after_two_predicts": bool(held),
            "P3a_no_break_in_2s_after_seed": bool(p3a_ok),
        },
        "PASS_H2ua": bool(held and p3a_ok),
        "compare": {
            "yaw_chase": {
                "H2": h2.get("yaw_chase"),
                "H2u": h2u.get("yaw_chase"),
                "H2ua": h2ua.get("yaw_chase"),
            },
            "P3_first_fail_t_s": {
                "H2": h2.get("P3_first_fail_t_s"),
                "H2u": h2u.get("P3_first_fail_t_s"),
                "H2ua": h2ua.get("P3_first_fail_t_s"),
            },
            "residual_h_at_6s": {
                "H2": h2.get("residual_h_at_6s"),
                "H2u": h2u.get("residual_h_at_6s"),
                "H2ua": h2ua.get("residual_h_at_6s"),
            },
            "t_sep_s": {
                "H2": h2.get("t_sep_s"),
                "H2u": h2u.get("t_sep_s"),
                "H2ua": h2ua.get("t_sep_s"),
            },
        },
    }
    out_json = OUT / "verdict.json"
    out_json.write_text(json.dumps(verdict, indent=2), encoding="utf-8")
    print(json.dumps(verdict, indent=2))
    print("PASS_H2ua" if verdict["PASS_H2ua"] else "FAIL_H2ua", "->", out_json)
    return 0 if verdict["PASS_H2ua"] else 2


if __name__ == "__main__":
    sys.exit(main())
