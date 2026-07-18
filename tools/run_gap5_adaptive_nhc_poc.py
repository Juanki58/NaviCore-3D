#!/usr/bin/env python3
"""GAP-5 — Adaptive NHC PoC (B0, B1, P0) per 14-adaptive-nhc-protocol.md v1.0.

BLOCKED: Do not run until GAP-5 v2 observable is preregistered and validated in passive.
See docs/diagnostics/15-gap5-passive-outcome.md (v1 instance closed 2026-07-18).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "docs/benchmarks/gap5_adaptive_nhc"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
DEFAULT_DATA = REPO_ROOT / "data" / "real_run"

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


def controller_stats(audit_csv: Path) -> dict:
    if not audit_csv.is_file():
        return {}
    lines = audit_csv.read_text(encoding="utf-8").splitlines()
    if len(lines) < 2:
        return {}
    header = lines[0].split(",")
    idx = {name: i for i, name in enumerate(header)}
    n1 = n5 = n10 = 0
    transitions = 0
    dwell_sum = {1: 0.0, 5: 0.0, 10: 0.0}
    dwell_count = {1: 0, 5: 0, 10: 0}
    prev_t = None
    for ln in lines[1:]:
        parts = ln.split(",")
        if len(parts) < len(header):
            continue
        t = float(parts[idx["timestamp_s"]])
        state = int(float(parts[idx["controller_state"]]))
        if state == 1:
            n1 += 1
        elif state == 5:
            n5 += 1
        elif state == 10:
            n10 += 1
        if int(float(parts[idx["transition"]])) == 1:
            transitions += 1
        dwell = float(parts[idx["dwell_time_s"]])
        if state in dwell_sum:
            dwell_sum[state] += dwell
            dwell_count[state] += 1
        prev_t = t
    total = max(1, n1 + n5 + n10)
    return {
        "time_frac_N1": n1 / total,
        "time_frac_N5": n5 / total,
        "time_frac_N10": n10 / total,
        "n_transitions": transitions,
        "mean_dwell_N1_s": dwell_sum[1] / max(1, dwell_count[1]),
        "mean_dwell_N5_s": dwell_sum[5] / max(1, dwell_count[5]),
        "mean_dwell_N10_s": dwell_sum[10] / max(1, dwell_count[10]),
        "duration_s": prev_t,
    }


def run_arm(
    replay_exe: Path,
    replay_csv: Path,
    calibration: Path,
    out_subdir: str,
    *,
    nhc_every: int = 1,
    adaptive_nhc: str = "off",
) -> dict:
    out_dir = OUT_DIR / out_subdir
    out_dir.mkdir(parents=True, exist_ok=True)
    gnss_csv = out_dir / "gnss_nis_audit.csv"
    ctrl_csv = out_dir / "controller_audit.csv"
    cov_csv = out_dir / "cov_step_audit.csv"

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
        "--constraint-policy",
        "disabled",
        "--nhc-policy",
        "enabled",
        "--nhc-every-n-ticks",
        str(nhc_every),
        "--gnss-obs-mode",
        "pos_vel",
        "--gnss-vel-std-mps",
        "1.5",
        "--p-pv-policy",
        "none",
        "--gap3-gnss-nis-audit-csv",
        str(gnss_csv),
        "--gap3-cov-step-audit-csv",
        str(cov_csv),
        "--output",
        str(out_dir / "replay_output.csv"),
    ]
    if adaptive_nhc != "off":
        cmd.extend(
            [
                "--adaptive-nhc",
                adaptive_nhc,
                "--adaptive-nhc-controller-audit-csv",
                str(ctrl_csv),
            ]
        )

    print(f"\n=== {out_subdir} (nhc_every={nhc_every}, adaptive={adaptive_nhc}) ===")
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)

    import pandas as pd

    gnss = pd.read_csv(gnss_csv)
    accepted = gnss[gnss["accepted"] == 1]
    pvv_pre_fix3 = None
    cov = pd.read_csv(cov_csv)
    cov_gnss = cov[(cov["update_type"] == "gnss") & (cov["phase"] == "pre")]
    if len(cov_gnss) >= 3:
        pvv_pre_fix3 = float(cov_gnss.iloc[2]["P_vv_frob"])

    row = {
        "arm": out_subdir,
        "nhc_every_n_ticks": nhc_every,
        "adaptive_nhc": adaptive_nhc,
        "gnss_accept_count": int(len(accepted)),
        "gnss_reject_count": int(len(gnss) - len(accepted)),
        "innov_h_mean_accepted": float(accepted["innov_h_m"].mean()) if len(accepted) else None,
        "P_vv_frob_pre_fix3": pvv_pre_fix3,
    }
    if adaptive_nhc != "off":
        row.update(controller_stats(ctrl_csv))
    return row


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-5 adaptive NHC PoC")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-run", action="store_true")
    parser.add_argument("--arm", choices=["all", "B0", "B1", "P0"], default="all")
    parser.add_argument(
        "--force-unblocked",
        action="store_true",
        help="Bypass GAP-5 v1 closure guard (requires GAP-5 v2 preregistration)",
    )
    args = parser.parse_args()

    if not args.force_unblocked:
        print(
            "GAP-5 PoC active run is BLOCKED (v1 instance closed).\n"
            "See docs/diagnostics/15-gap5-passive-outcome.md\n"
            "Use run_gap5_p0_passive_validation.py for passive validation only."
        )
        return 2

    replay_csv = args.replay_csv or resolve_replay_path(None)
    ensure_calibration(args.calibration)
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    arms = []
    if args.arm in ("all", "B0"):
        arms.append(("baseline_n1", {"out_subdir": "baseline_n1", "nhc_every": 1, "adaptive_nhc": "off"}))
    if args.arm in ("all", "B1"):
        arms.append(("baseline_n5", {"out_subdir": "baseline_n5", "nhc_every": 5, "adaptive_nhc": "off"}))
    if args.arm in ("all", "P0"):
        arms.append(
            ("poc_adaptive", {"out_subdir": "poc_adaptive", "nhc_every": 1, "adaptive_nhc": "active"})
        )

    results = []
    if not args.skip_run:
        for _, kw in arms:
            results.append(run_arm(args.replay_exe, replay_csv, args.calibration, **kw))

    report_path = OUT_DIR / "gap5_adaptive_nhc_poc_report.json"
    report = {
        "protocol": "docs/diagnostics/14-adaptive-nhc-protocol.md v1.0",
        "tag": "gap5-preregistration-frozen",
        "note": "Validate controller dynamics before RMSE (gamma_bar, hysteresis, chatter, time in N).",
        "arms": results,
    }
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\nWrote {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
