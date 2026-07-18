#!/usr/bin/env python3
"""GAP-3.12 / Exp F — NHC cada N ticks: ¿frecuencia domina GNSS accepts?

ZUPT OFF, NHC ON, sweep --nhc-every-n-ticks.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_nhc_decimation"
REPORT_JSON = OUT_DIR / "gap3_nhc_decimation_report.json"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"

N_VALUES = [1, 2, 5, 10, 20, 50, 100]

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


def run_case(replay_exe: Path, replay_csv: Path, calibration: Path, n: int) -> dict:
    out_dir = OUT_DIR / f"nhc_every_{n}"
    out_dir.mkdir(parents=True, exist_ok=True)
    gnss_csv = out_dir / "gnss_nis_audit.csv"
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
        str(n),
        "--output",
        str(out_dir / "replay_output.csv"),
        "--gap3-gnss-nis-audit-csv",
        str(gnss_csv),
    ]
    print(f"\n=== NHC every {n} ticks ===")
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)
    gnss = pd.read_csv(gnss_csv, index_col=False)
    accepted = gnss[gnss["accepted"] == 1]
    return {
        "nhc_every_n_ticks": n,
        "gnss_accept_count": int(len(accepted)),
        "gnss_reject_count": int(len(gnss) - len(accepted)),
        "gnss_innov_h_mean_accepted": float(accepted["innov_h_m"].mean()) if len(accepted) else None,
        "gnss_k_vel_mean_accepted": float(accepted["k_vel_max"].mean()) if len(accepted) else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3.12 NHC decimation sweep")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-run", action="store_true")
    parser.add_argument("--n-values", type=int, nargs="*", default=N_VALUES)
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    ensure_calibration(args.calibration)
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    results = []
    if not args.skip_run:
        for n in args.n_values:
            results.append(run_case(args.replay_exe, replay_csv, args.calibration, n))
    else:
        for n in args.n_values:
            gnss_csv = OUT_DIR / f"nhc_every_{n}" / "gnss_nis_audit.csv"
            if gnss_csv.is_file():
                gnss = pd.read_csv(gnss_csv, index_col=False)
                accepted = gnss[gnss["accepted"] == 1]
                results.append(
                    {
                        "nhc_every_n_ticks": n,
                        "gnss_accept_count": int(len(accepted)),
                        "gnss_reject_count": int(len(gnss) - len(accepted)),
                        "gnss_innov_h_mean_accepted": float(accepted["innov_h_m"].mean())
                        if len(accepted)
                        else None,
                        "gnss_k_vel_mean_accepted": float(accepted["k_vel_max"].mean())
                        if len(accepted)
                        else None,
                    }
                )

    baseline = next((r for r in results if r["nhc_every_n_ticks"] == 1), None)
    report = {
        "experiment": "GAP-3.12 NHC decimation (Exp F)",
        "config": "ZUPT OFF, NHC ON, sweep nhc_every_n_ticks",
        "cases": results,
        "baseline_accepts_n1": baseline["gnss_accept_count"] if baseline else None,
        "verdict": "FREQUENCY_SENSITIVE"
        if any(r["gnss_accept_count"] > (baseline or {}).get("gnss_accept_count", 7) for r in results)
        else "FREQUENCY_NOT_DOMINANT",
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"Wrote {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
