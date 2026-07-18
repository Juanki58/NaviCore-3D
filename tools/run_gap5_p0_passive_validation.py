#!/usr/bin/env python3
"""Run GAP-5 P0 passive validation replay only (not full PoC)."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PROFILES = {
    "gap5-poc": {
        "out_subdir": "p0_passive_validation",
        "extra_args": [
            "--gnss-obs-mode",
            "pos_vel",
            "--gnss-vel-std-mps",
            "1.5",
            "--p-pv-policy",
            "none",
        ],
    },
    "f1-bridge": {
        "out_subdir": "p0_passive_f1_bridge",
        "extra_args": [],
    },
}
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--replay-end-s", type=float, default=0.0)
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILES),
        default="gap5-poc",
        help="gap5-poc = preregistered PoC config; f1-bridge = F1-equivalent (pos-only) for scale check",
    )
    parser.add_argument("--skip-audit", action="store_true")
    args = parser.parse_args()

    profile = PROFILES[args.profile]
    out_dir = REPO_ROOT / "docs/benchmarks/gap5_adaptive_nhc" / profile["out_subdir"]

    replay_csv = args.replay_csv or resolve_replay_path(None)
    ensure_calibration(args.calibration)
    out_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(args.replay_exe),
        "--input",
        str(replay_csv),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(args.calibration),
        "--yaw-init",
        "zero",
        "--h9a-gravity-tilt-init",
        "--constraint-policy",
        "disabled",
        "--nhc-policy",
        "enabled",
        "--nhc-every-n-ticks",
        "1",
        *profile["extra_args"],
        "--adaptive-nhc",
        "passive",
        "--adaptive-nhc-controller-audit-csv",
        str(out_dir / "controller_audit.csv"),
        "--gap3-cov-step-audit-csv",
        str(out_dir / "cov_step_audit.csv"),
        "--gap3-gnss-nis-audit-csv",
        str(out_dir / "gnss_nis_audit.csv"),
        "--output",
        str(out_dir / "replay_output.csv"),
    ]
    if args.replay_end_s > 0:
        cmd.extend(["--replay-end-s", str(args.replay_end_s)])

    print(f"=== GAP-5 P0 PASSIVE profile={args.profile} (implementation validation only) ===")
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)

    if not args.skip_audit:
        audit = REPO_ROOT / "tools/audit_gap5_passive_controller_validation.py"
        subprocess.run(
            [sys.executable, str(audit), "--run-dir", str(out_dir), "--plot"],
            check=True,
            cwd=REPO_ROOT,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
