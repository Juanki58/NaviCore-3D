#!/usr/bin/env python3
"""Run integrity-gate experiment (SW injection) and bank evidence under docs/benchmarks."""
from __future__ import annotations

import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "lib"))
from navicore_paths import ensure_tools_path  # noqa: E402

ensure_tools_path(REPO)

BUILD = REPO / "build"
OUT_DIR = REPO / "docs" / "benchmarks" / "integrity_gate_experiment"


def run(cmd: list[str], log, env: dict | None = None) -> int:
    log.write(f"\n$ {' '.join(cmd)}\n")
    log.flush()
    merged = os.environ.copy()
    if env:
        merged.update(env)
    p = subprocess.run(
        cmd,
        cwd=str(REPO),
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        env=merged,
    )
    log.write(f"\n[exit {p.returncode}]\n")
    return p.returncode


def summarize_sweep(sweep: dict) -> dict:
    trials = sweep.get("trials", [])
    short_big = [
        t
        for t in trials
        if t.get("scenario") == "pos_jump"
        and float(t.get("gap_s", 99)) <= 2.0
        and float(t.get("jump_m", 0)) > 120.0
    ]
    short_nudge = [
        t
        for t in trials
        if t.get("scenario") == "pos_jump"
        and float(t.get("gap_s", 99)) <= 2.0
        and float(t.get("jump_m", 99)) <= 2.0
    ]
    long_big = [
        t
        for t in trials
        if t.get("scenario") == "pos_jump"
        and float(t.get("gap_s", 0)) > 2.0
        and float(t.get("jump_m", 0)) > 120.0
    ]
    vel_hard = [
        t
        for t in trials
        if t.get("scenario") == "vel_lie" and float(t.get("speed_lie_mps", 0)) >= 50.0
    ]

    def all_inconsistent(rows: list[dict]) -> bool:
        return bool(rows) and all(
            (not r.get("accepted")) and r.get("reject_name") == "INCONSISTENT" for r in rows
        )

    def none_inconsistent(rows: list[dict]) -> bool:
        return bool(rows) and all(r.get("reject_name") != "INCONSISTENT" for r in rows)

    return {
        "n_trials": len(trials),
        "short_gap_teleport_gt_120m_detected": all_inconsistent(short_big),
        "short_gap_teleport_gt_120m_n": len(short_big),
        "short_gap_nudge_le_2m_not_spoof": none_inconsistent(short_nudge),
        "long_gap_teleport_not_classified_spoof": none_inconsistent(long_big),
        "velocity_lie_60mps_detected": all_inconsistent(vel_hard),
        "claims_ok": all(
            [
                all_inconsistent(short_big),
                none_inconsistent(short_nudge),
                none_inconsistent(long_big),
                all_inconsistent(vel_hard),
            ]
        ),
    }


def main() -> int:
    unit = BUILD / "navicore_unit_tests.exe"
    reg = BUILD / "navicore_regression_test.exe"
    if sys.platform != "win32":
        unit = BUILD / "navicore_unit_tests"
        reg = BUILD / "navicore_regression_test"
    if not unit.is_file():
        print(f"Missing {unit} — build navicore_unit_tests first.", file=sys.stderr)
        return 1

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    sweep_json = OUT_DIR / "sweep.json"
    log_path = OUT_DIR / "host_run_log.txt"
    summary_path = OUT_DIR / "SUMMARY.json"

    with log_path.open("w", encoding="utf-8") as log:
        log.write(f"NaviCore-3D integrity gate experiment\nUTC: {stamp}\n")
        log.write("Policy: software injection only (no RF spoof/jam).\n")

        env = {"NAVICORE_INTEGRITY_SWEEP_OUT": str(sweep_json)}
        rc_sweep = run(
            [str(unit), "[integrity][sweep]", "--reporter", "compact"],
            log,
            env=env,
        )
        rc_props = run(
            [str(unit), "[rapidcheck][integrity]", "--reporter", "compact"],
            log,
        )
        rc_reg = 0
        if reg.is_file():
            rc_reg = run([str(reg), "--safety-inject"], log)
        else:
            log.write("\n[skip] navicore_regression_test not found\n")

        summary = {
            "utc": stamp,
            "exit_codes": {
                "sweep": rc_sweep,
                "rapidcheck_integrity": rc_props,
                "safety_inject": rc_reg,
            },
        }
        if sweep_json.is_file():
            sweep = json.loads(sweep_json.read_text(encoding="utf-8"))
            summary["sweep_claims"] = summarize_sweep(sweep)
            summary["thresholds"] = sweep.get("thresholds", {})
        else:
            summary["sweep_claims"] = {"error": "sweep.json missing"}

        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        log.write(f"\nSUMMARY {json.dumps(summary)}\n")

    print(f"Wrote {log_path}")
    print(f"Wrote {summary_path}")
    if sweep_json.is_file():
        print(f"Wrote {sweep_json}")

    claims = summary.get("sweep_claims", {})
    ok = (
        rc_sweep == 0
        and rc_props == 0
        and rc_reg == 0
        and claims.get("claims_ok") is True
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
