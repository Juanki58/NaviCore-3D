#!/usr/bin/env python3
"""Full replay: H2 (att always on) vs Gate (NHC att blocked until kinematic coherence)."""

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
OUT = REPO / "docs" / "benchmarks" / "h_nhc_att_gate"
T_SEED = 4.301

YAW_CHASE_RE = re.compile(
    r"YAW_CHASE deg \| before=([-\d.]+) after_seed=([-\d.]+) after_update=([-\d.]+) "
    r"after_pred1=([-\d.]+) after_pred2=([-\d.]+)"
)
GATE_OPEN_RE = re.compile(
    r"NHC_ATT_GATE_OPEN t_s=([-\d.]+) speed=([-\d.]+) \|course-yaw\|=([-\d.]+) deg hold=([-\d.]+)"
)


def run_arm(name: str, *, coherence_gate: bool) -> tuple[Path, str]:
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
    ]
    if coherence_gate:
        cmd.extend(
            [
                "--nhc-att-coherence-gate",
                "--nhc-att-gate-vmin-mps", "5",
                "--nhc-att-gate-yaw-max-deg", "15",
                "--nhc-att-gate-hold-s", "2.5",
            ]
        )
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


def parse_gate_open(log: str) -> dict | None:
    m = GATE_OPEN_RE.search(log)
    if not m:
        return None
    return {
        "t_s": float(m.group(1)),
        "speed": float(m.group(2)),
        "course_yaw_abs_deg": float(m.group(3)),
        "hold_s": float(m.group(4)),
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
        "gate_open": parse_gate_open(log),
    }

    if len(b) and "residual_h_m" in b.columns:
        r = b["residual_h_m"].to_numpy()
        out["residual_h_final"] = float(r[-1])
        out["residual_h_max"] = float(np.max(r))
        out["residual_h_p50"] = float(np.median(r))
        out["residual_h_p95"] = float(np.percentile(r, 95))
        for mark in (6.0, 60.0, 300.0):
            w = b[b["timestamp_s"] <= mark]
            if len(w):
                out[f"residual_h_at_{int(mark)}s"] = float(w["residual_h_m"].iloc[-1])
    out["t_sep_s"] = t_sep_residual(df)

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

    # P3a: break in [t_seed, t_seed+2]
    p3a = False
    prev_ok = None
    for _, row in a.iterrows():
        t = float(row["timestamp_s"])
        if t < T_SEED:
            continue
        if t > T_SEED + 2.0:
            break
        v = int(row["inv_course_yaw_ok"])
        if v < 0:
            continue
        ok = v == 1
        if prev_ok is True and not ok:
            p3a = True
            out["P3a_fail_t_s"] = t
            break
        prev_ok = ok
    out["P3a_immediate_break"] = p3a

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

    late = b[b["timestamp_s"] > 60.0]
    if len(late) and "residual_h_m" in late.columns:
        out["residual_h_max_after_60s"] = float(late["residual_h_m"].max())
    a_late = a[a["timestamp_s"] > 60.0]
    if len(a_late) and "inv_course_yaw_ok" in a_late.columns:
        vv = a_late[a_late["inv_course_yaw_ok"] >= 0]
        if len(vv):
            out["course_yaw_fail_frac_after_60s"] = float((vv["inv_course_yaw_ok"] == 0).mean())

    yc = out.get("yaw_chase") or {}
    out["yaw_held_after_two_predicts"] = (
        yc.get("dyaw_seed_to_pred1") is not None
        and abs(yc["dyaw_seed_to_pred1"]) < 2.0
        and abs(yc.get("dyaw_pred1_to_pred2", 99.0)) < 2.0
    )
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--only-gate", action="store_true")
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
    if not args.only_gate:
        tick_h2, log_h2 = run_arm("H2", coherence_gate=False)
    else:
        tick_h2 = OUT / "H2" / "tick_stage_audit.csv"
        log_h2 = (OUT / "H2" / "replay.log").read_text(encoding="utf-8", errors="replace")

    tick_g, log_g = run_arm("Gate", coherence_gate=True)
    h2 = eval_full(tick_h2, log_h2)
    gate = eval_full(tick_g, log_g)

    early_ok = bool(gate.get("yaw_held_after_two_predicts")) and not gate.get(
        "P3a_immediate_break", True
    )
    late_not_worse = (
        gate.get("course_yaw_fail_frac_after_60s") is not None
        and h2.get("course_yaw_fail_frac_after_60s") is not None
        and gate["course_yaw_fail_frac_after_60s"]
        <= h2["course_yaw_fail_frac_after_60s"] + 0.05
    )
    residual_not_worse = (
        gate.get("residual_h_final") is not None
        and h2.get("residual_h_final") is not None
        and gate["residual_h_final"] <= h2["residual_h_final"] * 1.1
    )
    gnss_ok = gate.get("gnss_accept_rate") == h2.get("gnss_accept_rate") or (
        gate.get("gnss_accept") == h2.get("gnss_accept")
    )
    gate_opened = gate.get("gate_open") is not None

    verdict = {
        "H2": h2,
        "Gate": gate,
        "gates": {
            "gate_opened": gate_opened,
            "early_yaw_held_and_no_P3a": early_ok,
            "late_course_yaw_not_worse": bool(late_not_worse),
            "residual_final_not_worse_10pct": bool(residual_not_worse),
            "gnss_ok": bool(gnss_ok),
        },
        "PASS_GATE": bool(
            gate_opened and early_ok and late_not_worse and residual_not_worse and gnss_ok
        ),
        "compare": {
            "yaw_chase": {"H2": h2.get("yaw_chase"), "Gate": gate.get("yaw_chase")},
            "gate_open": gate.get("gate_open"),
            "P3_first_fail_t_s": {
                "H2": h2.get("P3_first_fail_t_s"),
                "Gate": gate.get("P3_first_fail_t_s"),
            },
            "t_sep_s": {"H2": h2.get("t_sep_s"), "Gate": gate.get("t_sep_s")},
            "residual_h_final": {
                "H2": h2.get("residual_h_final"),
                "Gate": gate.get("residual_h_final"),
            },
            "course_yaw_fail_frac_after_60s": {
                "H2": h2.get("course_yaw_fail_frac_after_60s"),
                "Gate": gate.get("course_yaw_fail_frac_after_60s"),
            },
            "gnss_accept_reject": {
                "H2": [h2.get("gnss_accept"), h2.get("gnss_reject")],
                "Gate": [gate.get("gnss_accept"), gate.get("gnss_reject")],
            },
        },
    }
    out_json = OUT / "verdict_gate_full.json"
    out_json.write_text(json.dumps(verdict, indent=2), encoding="utf-8")
    print(json.dumps(verdict, indent=2))
    print("PASS_GATE" if verdict["PASS_GATE"] else "FAIL_GATE", "->", out_json)
    return 0 if verdict["PASS_GATE"] else 2


if __name__ == "__main__":
    sys.exit(main())
