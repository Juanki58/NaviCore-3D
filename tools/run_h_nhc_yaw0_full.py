#!/usr/bin/env python3
"""Full-replay check: H2 vs H2ua (NHC attitude fully unobservable). No permanent policy yet."""

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
OUT = REPO / "docs" / "benchmarks" / "h_nhc_yaw0_full"
T_SEED = 4.301

YAW_CHASE_RE = re.compile(
    r"YAW_CHASE deg \| before=([-\d.]+) after_seed=([-\d.]+) after_update=([-\d.]+) "
    r"after_pred1=([-\d.]+) after_pred2=([-\d.]+)"
)


def run_arm(name: str, att_unobs: bool) -> tuple[Path, str]:
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
        "--tick-stage-audit-csv", str(tick),
        "--gap3-constraint-pipeline-audit-csv", str(pipeline),
        "--output", str(output),
        # no --replay-end-s → full pack (~677 s)
    ]
    if att_unobs:
        cmd.append("--nhc-att-unobs-immediate")
    print("RUN", name, "FULL", flush=True)
    proc = subprocess.run(cmd, cwd=str(REPO), check=True, capture_output=True, text=True)
    log = (proc.stdout or "") + (proc.stderr or "")
    log_path.write_text(log, encoding="utf-8", errors="replace")
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


def eval_full(tick_csv: Path, log: str) -> dict:
    df = pd.read_csv(tick_csv)
    b = df[df["edge"] == "B_post_nhc_zupt"].sort_values("timestamp_s")
    a = df[df["edge"] == "A_post_predict"].sort_values("timestamp_s")
    c = df[df["edge"] == "C_post_gnss"]
    out: dict = {
        "t_end_s": float(df["timestamp_s"].max()) if len(df) else None,
        "n_B": int(len(b)),
        "yaw_chase": parse_yaw_chase(log),
    }

    if len(b) and "residual_h_m" in b.columns:
        r = b["residual_h_m"].to_numpy()
        out["residual_h_final"] = float(r[-1])
        out["residual_h_max"] = float(np.max(r))
        out["residual_h_p50"] = float(np.median(r))
        out["residual_h_p95"] = float(np.percentile(r, 95))
        out["residual_h_at_6s"] = float(b[b["timestamp_s"] <= 6.0]["residual_h_m"].iloc[-1]) if len(b[b["timestamp_s"] <= 6.0]) else None
        out["residual_h_at_60s"] = float(b[b["timestamp_s"] <= 60.0]["residual_h_m"].iloc[-1]) if len(b[b["timestamp_s"] <= 60.0]) else None
        out["residual_h_at_300s"] = float(b[b["timestamp_s"] <= 300.0]["residual_h_m"].iloc[-1]) if len(b[b["timestamp_s"] <= 300.0]) else None
    out["t_sep_s"] = t_sep_residual(df)

    # course-yaw OK→FAIL transitions and fail fraction (A edge, after seed)
    prev_ok = None
    first_fail = None
    n_fail_trans = 0
    a_post = a[a["timestamp_s"] >= T_SEED]
    ok_vals = []
    for _, row in a_post.iterrows():
        v = int(row["inv_course_yaw_ok"])
        if v < 0:
            continue
        ok = v == 1
        ok_vals.append(ok)
        if prev_ok is True and not ok:
            n_fail_trans += 1
            if first_fail is None:
                first_fail = float(row["timestamp_s"])
        prev_ok = ok
    out["P3_first_fail_t_s"] = first_fail
    out["course_yaw_ok_to_fail_count"] = n_fail_trans
    if ok_vals:
        out["course_yaw_fail_frac"] = float(1.0 - (sum(ok_vals) / len(ok_vals)))

    if len(b) and "inv_speed_vs_gps_ok" in b.columns:
        valid = b[b["inv_speed_vs_gps_ok"] >= 0]
        if len(valid):
            out["speed_vs_gps_fail_frac"] = float((valid["inv_speed_vs_gps_ok"] == 0).mean())

    if len(c) and "gnss_accepted" in c.columns:
        n_acc = int((c["gnss_accepted"] == 1).sum())
        n_rej = int((c["gnss_accepted"] == 0).sum())
        out["gnss_accept"] = n_acc
        out["gnss_reject"] = n_rej
        out["gnss_accept_rate"] = float(n_acc / (n_acc + n_rej)) if (n_acc + n_rej) else None

    # Late-window health (t>60s): residual growth and course-yaw
    late = b[b["timestamp_s"] > 60.0]
    if len(late) and "residual_h_m" in late.columns:
        out["residual_h_max_after_60s"] = float(late["residual_h_m"].max())
        out["residual_h_final_vs_at_60s"] = (
            float(late["residual_h_m"].iloc[-1]) - float(b[b["timestamp_s"] <= 60.0]["residual_h_m"].iloc[-1])
            if len(b[b["timestamp_s"] <= 60.0])
            else None
        )
    a_late = a[a["timestamp_s"] > 60.0]
    if len(a_late) and "inv_course_yaw_ok" in a_late.columns:
        vv = a_late[a_late["inv_course_yaw_ok"] >= 0]
        if len(vv):
            out["course_yaw_fail_frac_after_60s"] = float((vv["inv_course_yaw_ok"] == 0).mean())

    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--only-h2ua", action="store_true", help="reuse H2 if present")
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
        tick_h2, log_h2 = run_arm("H2", att_unobs=False)
    else:
        tick_h2 = OUT / "H2" / "tick_stage_audit.csv"
        log_h2 = (OUT / "H2" / "replay.log").read_text(encoding="utf-8", errors="replace")

    tick_h2ua, log_h2ua = run_arm("H2ua", att_unobs=True)

    h2 = eval_full(tick_h2, log_h2)
    h2ua = eval_full(tick_h2ua, log_h2ua)

    def better(key: str, lower_is_better: bool = True) -> str | None:
        a, b = h2.get(key), h2ua.get(key)
        if a is None or b is None:
            return None
        if lower_is_better:
            return "H2ua" if b < a else ("H2" if a < b else "tie")
        return "H2ua" if b > a else ("H2" if a > b else "tie")

    # Secondary-effect flags (not pass/fail gates — inspection)
    side_effects = {
        "gnss_accept_rate_drop_gt_5pp": (
            h2.get("gnss_accept_rate") is not None
            and h2ua.get("gnss_accept_rate") is not None
            and (h2["gnss_accept_rate"] - h2ua["gnss_accept_rate"]) > 0.05
        ),
        "residual_final_worse": (
            h2.get("residual_h_final") is not None
            and h2ua.get("residual_h_final") is not None
            and h2ua["residual_h_final"] > h2["residual_h_final"] * 1.1
        ),
        "late_course_yaw_worse": (
            h2.get("course_yaw_fail_frac_after_60s") is not None
            and h2ua.get("course_yaw_fail_frac_after_60s") is not None
            and h2ua["course_yaw_fail_frac_after_60s"]
            > h2["course_yaw_fail_frac_after_60s"] + 0.1
        ),
    }

    global_better = (
        not side_effects["gnss_accept_rate_drop_gt_5pp"]
        and not side_effects["residual_final_worse"]
        and not side_effects["late_course_yaw_worse"]
        and (
            h2ua.get("residual_h_final") is not None
            and h2.get("residual_h_final") is not None
            and h2ua["residual_h_final"] <= h2["residual_h_final"]
        )
    )

    verdict = {
        "H2": h2,
        "H2ua": h2ua,
        "side_effects": side_effects,
        "FULL_RUN_H2ua_not_worse": bool(global_better),
        "compare": {
            "residual_h_final": {"H2": h2.get("residual_h_final"), "H2ua": h2ua.get("residual_h_final"), "better": better("residual_h_final")},
            "residual_h_max": {"H2": h2.get("residual_h_max"), "H2ua": h2ua.get("residual_h_max"), "better": better("residual_h_max")},
            "t_sep_s": {"H2": h2.get("t_sep_s"), "H2ua": h2ua.get("t_sep_s"), "better": better("t_sep_s", lower_is_better=False)},
            "gnss_accept_rate": {"H2": h2.get("gnss_accept_rate"), "H2ua": h2ua.get("gnss_accept_rate"), "better": better("gnss_accept_rate", lower_is_better=False)},
            "P3_first_fail_t_s": {"H2": h2.get("P3_first_fail_t_s"), "H2ua": h2ua.get("P3_first_fail_t_s")},
            "course_yaw_fail_frac": {"H2": h2.get("course_yaw_fail_frac"), "H2ua": h2ua.get("course_yaw_fail_frac"), "better": better("course_yaw_fail_frac")},
            "course_yaw_fail_frac_after_60s": {
                "H2": h2.get("course_yaw_fail_frac_after_60s"),
                "H2ua": h2ua.get("course_yaw_fail_frac_after_60s"),
                "better": better("course_yaw_fail_frac_after_60s"),
            },
        },
        "interpretation": (
            "Full H2ua (NHC att unobs for whole run) is not worse than H2 on residual/GNSS/late course-yaw; "
            "supports proceeding to init-gate policy (2), not permanent velocity-only (1)."
            if global_better
            else "Full H2ua shows a regression or side effect vs H2 — do not adopt policy yet; inspect side_effects."
        ),
    }
    out_json = OUT / "verdict_full.json"
    out_json.write_text(json.dumps(verdict, indent=2), encoding="utf-8")
    print(json.dumps(verdict, indent=2))
    print(
        "FULL_OK" if verdict["FULL_RUN_H2ua_not_worse"] else "FULL_REGRESSION",
        "->",
        out_json,
    )
    return 0 if verdict["FULL_RUN_H2ua_not_worse"] else 2


if __name__ == "__main__":
    sys.exit(main())
