#!/usr/bin/env python3
"""Run host fault-injection / integrity smoke and save log under docs/benchmarks."""
from __future__ import annotations

import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BUILD = REPO / "build"
OUT_DIR = REPO / "docs" / "benchmarks" / "fault_injection" / "20260722_host"


def run(cmd: list[str], log) -> int:
    log.write(f"\n$ {' '.join(cmd)}\n")
    log.flush()
    p = subprocess.run(cmd, cwd=str(REPO), stdout=log, stderr=subprocess.STDOUT, text=True)
    log.write(f"\n[exit {p.returncode}]\n")
    return p.returncode


def main() -> int:
    unit = BUILD / "navicore_unit_tests.exe"
    reg = BUILD / "navicore_regression_test.exe"
    if not unit.is_file() or not reg.is_file():
        print("Build navicore_unit_tests and navicore_regression_test first.", file=sys.stderr)
        return 1

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    log_path = OUT_DIR / "host_smoke_log.txt"
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    with log_path.open("w", encoding="utf-8") as log:
        log.write(f"NaviCore-3D host fault/integrity smoke\nUTC: {stamp}\n")
        rc1 = run(
            [
                str(unit),
                "[fault],[nmea],[ubx],[wt61c],[nhc_ops],[rapidcheck][integrity]",
                "--reporter",
                "compact",
            ],
            log,
        )
        rc2 = run([str(reg), "--safety-inject"], log)
        log.write(f"\nSUMMARY unit={rc1} safety_inject={rc2}\n")

    print(f"Wrote {log_path}")
    return 0 if rc1 == 0 and rc2 == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
