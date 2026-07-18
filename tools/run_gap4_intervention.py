#!/usr/bin/env python3
"""GAP-4 §11 — Run preregistered intervention arms (1a/1b/1d/1d′)."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

INTERVENTION_ARMS = {
    "1a": {"ppv_policy": "gap_le_1s", "subdir": "G1_intervention/arm_1a_gap"},
    "1b": {"ppv_policy": "zero", "subdir": "G1_intervention/arm_1b_unconditional"},
    "1d": {"ppv_policy": "cos_pos", "subdir": "G1_intervention/arm_1d_cos_pos"},
    "1d_prime": {"ppv_policy": "cos_tot", "subdir": "G1_intervention/arm_1d_prime_cos_tot"},
}

G2_REFERENCE = {
    "none": "G1_intervention/G2_reference_posvel",
}


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-4 §11 intervention replay runner")
    parser.add_argument(
        "--intervention",
        choices=sorted(INTERVENTION_ARMS),
        help="Preregistered arm id (1a, 1b, 1d, 1d_prime)",
    )
    parser.add_argument(
        "--ppv-policy",
        choices=["none", "gap_le_1s", "zero", "cos_pos", "cos_tot"],
        help="Override P_pv policy (default from --intervention)",
    )
    parser.add_argument("--arm", choices=["G1", "G2"], default="G1")
    parser.add_argument("--skip-replay", action="store_true")
    parser.add_argument("--run-all", action="store_true", help="Run all four §11 arms on G1 pos+vel")
    parser.add_argument("--g2-reference", action="store_true", help="G2 pos+vel reference for fix#56")
    args, extra = parser.parse_known_args()

    if args.run_all:
        rc = 0
        for arm_id, cfg in INTERVENTION_ARMS.items():
            cmd = [
                sys.executable,
                str(REPO_ROOT / "tools/run_gap4_arm.py"),
                "--arm",
                "G1",
                "--ppv-policy",
                cfg["ppv_policy"],
                "--out-subdir",
                cfg["subdir"],
            ]
            if args.skip_replay:
                cmd.append("--skip-replay")
            print("RUN:", " ".join(cmd))
            if subprocess.run(cmd, cwd=REPO_ROOT).returncode != 0:
                rc = 1
        return rc

    if args.g2_reference:
        cmd = [
            sys.executable,
            str(REPO_ROOT / "tools/run_gap4_arm.py"),
            "--arm",
            "G2",
            "--ppv-policy",
            "none",
            "--out-subdir",
            G2_REFERENCE["none"],
        ]
        if args.skip_replay:
            cmd.append("--skip-replay")
        return subprocess.run(cmd, cwd=REPO_ROOT).returncode

    if not args.intervention and not args.ppv_policy:
        parser.error("Specify --intervention, --ppv-policy, --run-all, or --g2-reference")

    cfg = INTERVENTION_ARMS.get(args.intervention or "", {})
    ppv = args.ppv_policy or cfg.get("ppv_policy", "none")
    subdir = cfg.get("subdir")
    if args.ppv_policy and not subdir:
        subdir = f"G1_intervention/custom_{ppv}"

    cmd = [
        sys.executable,
        str(REPO_ROOT / "tools/run_gap4_arm.py"),
        "--arm",
        args.arm,
        "--ppv-policy",
        ppv,
    ]
    if subdir:
        cmd.extend(["--out-subdir", subdir])
    if args.skip_replay:
        cmd.append("--skip-replay")
    cmd.extend(extra)
    return subprocess.run(cmd, cwd=REPO_ROOT).returncode


if __name__ == "__main__":
    raise SystemExit(main())
