#!/usr/bin/env python3
"""2×2 A/B: NHC Jacobian (correct|legacy) × IMU (ideal|dirty), seed fijo.

NO interpretar hasta tener las cuatro celdas. Lecturas preregistradas en
docs/diagnostics/18-jacobian-imu-ab-protocol.md.
"""

from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "docs" / "benchmarks" / "jacobian_imu_ab"
SEED = 71

sys.path.insert(0, str(REPO_ROOT))
from run_all_benchmarks import (  # noqa: E402
    SLALOM_MAX_LATERAL_DRIFT_M,
    TUNNEL_EXIT_DRIFT_M,
    TUNNEL_ZUPT_MAX_SPEED_MPS,
    run_benchmark,
)

# User matrix:
#              IMU ideal    IMU dirty
# J correct       A              B
# J legacy        C              D
CELLS = (
    ("A", "correct", "ideal"),
    ("B", "correct", "dirty"),
    ("C", "legacy", "ideal"),
    ("D", "legacy", "dirty"),
)


def metric_val(result, name: str):
    for metric in result.metrics:
        if metric.name == name:
            return metric.measured, metric.passed
    return None, False


def run_cell(cell_id: str, nhc_jacobian: str, imu_mode: str) -> dict:
    suffix = f"cell{cell_id}_j{nhc_jacobian}_imu{imu_mode}_s{SEED}"
    print(f"\n=== CELL {cell_id}: jacobian={nhc_jacobian} imu={imu_mode} seed={SEED} ===")

    slalom = run_benchmark(
        f"{cell_id} SLALOM",
        "SLALOM",
        seed=SEED,
        imu_mode=imu_mode,
        nhc_jacobian=nhc_jacobian,
        archive_suffix=suffix,
    )
    tunnel = run_benchmark(
        f"{cell_id} TUNNEL",
        "TUNNEL_STRESS",
        seed=SEED,
        imu_mode=imu_mode,
        nhc_jacobian=nhc_jacobian,
        archive_suffix=suffix,
    )

    slalom_drift, slalom_pass = metric_val(slalom, "max_lateral_drift")
    tunnel_exit, tunnel_exit_pass = metric_val(tunnel, "tunnel_exit_drift")
    zupt_speed, zupt_pass = metric_val(tunnel, "zupt_residual_speed")
    recovery, recovery_pass = metric_val(tunnel, "gps_recovery_time")

    # SLALOM ignores imu-mode (always kinematic ideal); still recorded for completeness.
    return {
        "cell": cell_id,
        "nhc_jacobian": nhc_jacobian,
        "imu_mode": imu_mode,
        "seed": SEED,
        "slalom_note": "SLALOM IMU is always kinematic ideal; imu_mode flag does not apply",
        "slalom_max_lateral_m": slalom_drift,
        "slalom_pass": bool(slalom.passed),
        "tunnel_exit_m": tunnel_exit,
        "tunnel_exit_pass": bool(tunnel_exit_pass),
        "tunnel_zupt_speed_mps": zupt_speed,
        "tunnel_zupt_pass": bool(zupt_pass),
        "tunnel_recovery": recovery,
        "tunnel_recovery_pass": bool(recovery_pass),
        "tunnel_pass": bool(tunnel.passed),
        "overall_pass": bool(slalom.passed and tunnel.passed),
        "errors": {
            "slalom": slalom.error or None,
            "tunnel": tunnel.error or None,
        },
    }


def preregistered_reading(cells: dict[str, dict]) -> dict:
    """Three-path table from the plan — no further narrative."""
    a = cells["A"]
    b = cells["B"]
    c = cells["C"]
    d = cells["D"]

    a_ok = bool(a["tunnel_exit_pass"]) and bool(a["slalom_pass"])
    b_ok = bool(b["tunnel_exit_pass"])  # SLALOM N/A for dirty distinction
    c_ok = bool(c["tunnel_exit_pass"]) and bool(c["slalom_pass"])
    # Also expose tunnel-only for partial reads
    a_tunnel_ok = bool(a["tunnel_exit_pass"])
    b_tunnel_ok = bool(b["tunnel_exit_pass"])
    c_tunnel_ok = bool(c["tunnel_exit_pass"])

    if not a_ok:
        path = "PATH_3_A_ALSO_FAILS"
        instruction = (
            "A también falla: problema posterior/independiente (o no solo Jacobian×dirty). "
            "Acotar con git bisect entre último PASS (~0.12m/11m) y HEAD. "
            "Si C también falla, el signo H no es causa suficiente de este FAIL."
        )
    elif a_ok and not b_tunnel_ok:
        path = "PATH_1_JACOBIAN_X_DIRTY"
        instruction = (
            "A pasa y B falla: interacción Jacobiano-corregido × IMU-dirty. "
            "Abrir investigación tick-a-tick. No tocar §11/H6/Platform."
        )
    elif not c_ok:
        path = "PATH_2_C_ALSO_FAILS"
        instruction = (
            "C también falla (bug de signo restaurado): el Jacobiano no es la causa "
            "de este FAIL; mirar qué más cambió hacia/en bf2bfbd."
        )
    else:
        path = "SEE_TABLE"
        instruction = "Leer solo la tabla numérica; no añadir hipótesis."

    return {
        "path": path,
        "instruction": instruction,
        "a_pass": a_ok,
        "b_tunnel_pass": b_tunnel_ok,
        "c_pass": c_ok,
        "d_tunnel_pass": bool(d["tunnel_exit_pass"]),
        "a_tunnel_exit_pass": a_tunnel_ok,
        "c_tunnel_exit_pass": c_tunnel_ok,
        "session": "NOT_CLOSED — no interpretar más allá de esta tabla",
    }


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cells: dict[str, dict] = {}
    for cell_id, jac, imu in CELLS:
        cells[cell_id] = run_cell(cell_id, jac, imu)

    reading = preregistered_reading(cells)

    # Compact matrix for humans
    matrix = {
        "seed": SEED,
        "limits": {
            "slalom_m": SLALOM_MAX_LATERAL_DRIFT_M,
            "tunnel_exit_m": TUNNEL_EXIT_DRIFT_M,
            "zupt_mps": TUNNEL_ZUPT_MAX_SPEED_MPS,
        },
        "rows": {
            "correct_jacobian": {
                "ideal": {
                    "cell": "A",
                    "slalom_m": cells["A"]["slalom_max_lateral_m"],
                    "tunnel_exit_m": cells["A"]["tunnel_exit_m"],
                    "pass": cells["A"]["overall_pass"],
                },
                "dirty": {
                    "cell": "B",
                    "slalom_m": cells["B"]["slalom_max_lateral_m"],
                    "tunnel_exit_m": cells["B"]["tunnel_exit_m"],
                    "pass": cells["B"]["overall_pass"],
                },
            },
            "legacy_bug_jacobian": {
                "ideal": {
                    "cell": "C",
                    "slalom_m": cells["C"]["slalom_max_lateral_m"],
                    "tunnel_exit_m": cells["C"]["tunnel_exit_m"],
                    "pass": cells["C"]["overall_pass"],
                },
                "dirty": {
                    "cell": "D",
                    "slalom_m": cells["D"]["slalom_max_lateral_m"],
                    "tunnel_exit_m": cells["D"]["tunnel_exit_m"],
                    "pass": cells["D"]["overall_pass"],
                },
            },
        },
    }

    report = {
        "protocol": "docs/diagnostics/18-jacobian-imu-ab-protocol.md",
        "timestamp": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "seed": SEED,
        "cells": cells,
        "matrix": matrix,
        "preregistered_reading": reading,
        "discipline": "No interpretar más allá de preregistered_reading hasta nueva instrucción",
    }

    out_path = OUT_DIR / "ab_2x2_report.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("\n======== MATRIX (seed=%d) ========" % SEED)
    print(
        f"{'':22} | {'IMU ideal':>22} | {'IMU dirty':>22}"
    )
    print("-" * 72)
    for row_name, row in matrix["rows"].items():
        ideal = row["ideal"]
        dirty = row["dirty"]
        print(
            f"{row_name:22} | "
            f"{ideal['cell']}: slalom={ideal['slalom_m']} tun={ideal['tunnel_exit_m']} | "
            f"{dirty['cell']}: slalom={dirty['slalom_m']} tun={dirty['tunnel_exit_m']}"
        )
    print("-" * 72)
    print("PATH:", reading["path"])
    print(reading["instruction"])
    print(f"Wrote {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
