#!/usr/bin/env python3
"""A/B EKF v1 vs v2 on REF / ALT / JUL17 (NHC-off shell).

Success: v2 accept rate >> v1; final H drift / residual @60s much smaller.
"""

from __future__ import annotations

import json
import math
import re
import subprocess
import sys
from pathlib import Path

import pandas as pd

REPO = Path(__file__).resolve().parents[1]
REPLAY = REPO / "build" / "NaviCore3D_Replay.exe"
MOUNT = REPO / "calibration" / "imu_mount.json"
OUT = REPO / "docs" / "benchmarks" / "ekf_v2_ab_3routes"

ROUTES = {
    "REF_19082026": REPO
    / "docs"
    / "benchmarks"
    / "real_run_19082026_baseline"
    / "real_run_replay.csv",
    "ALT_16072026": REPO / "docs" / "benchmarks" / "real_run_replay.csv",
    "JUL17_20260717": REPO / "docs" / "benchmarks" / "real_run_20260717_replay.csv",
}


def run_arm(route: str, csv: Path, core: str) -> Path:
    arm = OUT / route / core
    arm.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(REPLAY),
        "--input",
        str(csv),
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
        "disabled",
        "--gnss-obs-mode",
        "pos_vel",
        "--p-pv-policy",
        "none",
        "--ekf-core",
        core,
        "--output",
        str(arm / "replay_output.csv"),
        "--gap3-gnss-nis-audit-csv",
        str(arm / "gnss_nis_audit.csv"),
        "--gap3-constraint-pipeline-audit-csv",
        str(arm / "constraint_pipeline_audit.csv"),
    ]
    print("RUN", route, core, flush=True)
    log = subprocess.run(cmd, cwd=str(REPO), check=True, capture_output=True, text=True)
    text = (log.stdout or "") + (log.stderr or "")
    (arm / "replay.log").write_text(text, encoding="utf-8", errors="replace")
    return arm


def parse_drift(log_text: str) -> float | None:
    m = re.search(r"Deriva final H:\s*([0-9.eE+-]+)\s*m", log_text)
    if not m:
        return None
    return float(m.group(1))


def metrics(arm: Path) -> dict:
    gnss = pd.read_csv(arm / "gnss_nis_audit.csv")
    accepts = int((gnss["accepted"] == 1).sum())
    rejects = int((gnss["accepted"] == 0).sum())
    total = accepts + rejects
    rate = accepts / total if total else 0.0
    log_text = (arm / "replay.log").read_text(encoding="utf-8", errors="replace")
    drift = parse_drift(log_text)

    # residual @ ~60s: |ekf_pos - gps| from replay if columns exist
    residual_60 = None
    out_csv = arm / "replay_output.csv"
    if out_csv.exists():
        out = pd.read_csv(out_csv)
        tcol = "timestamp_s" if "timestamp_s" in out.columns else None
        if tcol is not None and "pos_n_m" in out.columns and "gps_n_m" in out.columns:
            near = out[(out[tcol] >= 55.0) & (out[tcol] <= 65.0)]
            if len(near) > 0:
                row = near.iloc[len(near) // 2]
                dn = float(row["pos_n_m"]) - float(row["gps_n_m"])
                de = float(row["pos_e_m"]) - float(row["gps_e_m"])
                residual_60 = math.hypot(dn, de)
        elif tcol is not None and "ekf_pos_n_m" in out.columns and "gps_pos_n_m" in out.columns:
            near = out[(out[tcol] >= 55.0) & (out[tcol] <= 65.0)]
            if len(near) > 0:
                row = near.iloc[len(near) // 2]
                dn = float(row["ekf_pos_n_m"]) - float(row["gps_pos_n_m"])
                de = float(row["ekf_pos_e_m"]) - float(row["gps_pos_e_m"])
                residual_60 = math.hypot(dn, de)

    return {
        "accepts": accepts,
        "rejects": rejects,
        "accept_rate": rate,
        "final_drift_h_m": drift,
        "residual_h_at_60s_m": residual_60,
    }


def main() -> int:
    if not REPLAY.exists():
        print("ERROR: missing", REPLAY, file=sys.stderr)
        return 1

    OUT.mkdir(parents=True, exist_ok=True)
    summary: dict = {"routes": {}}

    for name, csv in ROUTES.items():
        if not csv.exists():
            print("SKIP missing", csv, flush=True)
            continue
        route_sum = {}
        for core in ("v1", "v2"):
            arm = run_arm(name, csv, core)
            route_sum[core] = metrics(arm)
            print(
                f"  {name}/{core}: accept_rate={route_sum[core]['accept_rate']:.4f} "
                f"drift={route_sum[core]['final_drift_h_m']} "
                f"res60={route_sum[core]['residual_h_at_60s_m']}",
                flush=True,
            )
        v1 = route_sum["v1"]
        v2 = route_sum["v2"]
        route_sum["verdict"] = {
            "accept_rate_improved": v2["accept_rate"] > max(0.2, v1["accept_rate"] * 2.0),
            "drift_improved": (
                v1["final_drift_h_m"] is not None
                and v2["final_drift_h_m"] is not None
                and v2["final_drift_h_m"] < v1["final_drift_h_m"] * 0.1
            ),
        }
        summary["routes"][name] = route_sum

    (OUT / "SUMMARY.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )

    lines = [
        "# EKF v1 vs v2 - A/B NHC-off (3 routes)",
        "",
        "| Route | Core | Accept rate | Final drift H [m] | Residual @60s [m] |",
        "|-------|------|-------------|-------------------|-------------------|",
    ]
    all_ok = True
    for name, rs in summary["routes"].items():
        for core in ("v1", "v2"):
            m = rs[core]
            lines.append(
                f"| {name} | {core} | {m['accept_rate']:.4f} | "
                f"{m['final_drift_h_m']} | {m['residual_h_at_60s_m']} |"
            )
        v = rs["verdict"]
        ok = bool(v["accept_rate_improved"] and v["drift_improved"])
        all_ok = all_ok and ok
        lines.append(
            f"| {name} | verdict | accept_up={v['accept_rate_improved']} "
            f"drift_down={v['drift_improved']} | pass={ok} | |"
        )
    lines.append("")
    lines.append(f"**Overall:** {'PASS' if all_ok else 'REVIEW'} - see SUMMARY.json")
    (OUT / "SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines), flush=True)
    return 0 if all_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
