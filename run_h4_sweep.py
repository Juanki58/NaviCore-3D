#!/usr/bin/env python3
"""H4 — Consistency & systematic P0 sweep.

Varies --p0-scale across a log-spaced grid, extracts navigation/consistency
metrics from each replay, and writes a consolidated JSON report plus curves.
"""

from __future__ import annotations

import csv
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY = REPO_ROOT / "docs" / "benchmarks" / "real_run_replay.csv"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
P0_SCALES = (1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0)
MOVING_START_S = 30.0

BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DIAGNOSTICS_CSV = BENCH_DIR / "h4_consistency_diagnostics.csv"
SWEEP_REPORT_JSON = BENCH_DIR / "h4_sweep_report.json"
SWEEP_CURVES_PNG = BENCH_DIR / "h4_sweep_curves.png"

from analyze_real_run import (  # noqa: E402
    analyze,
    discover_input_dir,
    resolve_orientation_path,
    resolve_replay_path,
)


@dataclass(frozen=True)
class SweepPoint:
    p0_scale: float
    horizontal_rmse_m: float
    error_h_final_m: float
    nis_mean: float
    gnss_accept_pct: float
    nees_pos_mean: float
    gnss_accept_count: int
    gnss_reject_count: int
    output_path: Path
    diagnostics_path: Path


def parse_float(text: str | None) -> float | None:
    if text is None:
        return None
    value = text.strip()
    if not value:
        return None
    try:
        out = float(value)
    except ValueError:
        return None
    if not math.isfinite(out):
        return None
    return out


def ensure_calibration(path: Path) -> None:
    if path.is_file():
        return
    subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "audit_imu_chain.py"),
            "--export-calibration",
            str(path),
        ],
        cwd=REPO_ROOT,
        check=True,
    )


def scale_tag(scale: float) -> str:
    return str(scale).replace(".", "p")


def run_replay(
    replay_exe: Path,
    replay_csv: Path,
    output_csv: Path,
    calibration: Path,
    p0_scale: float,
    diagnostics_csv: Path,
) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")

    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(output_csv),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--p0-scale",
        f"{p0_scale}",
        "--consistency-csv",
        str(diagnostics_csv),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def metrics_from_diagnostics(path: Path) -> tuple[int, int, float, float, float]:
    accepted = 0
    rejected = 0
    nis_values: list[float] = []
    nees_values: list[float] = []

    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            flag = parse_float(row.get("gnss_accepted"))
            if flag is not None:
                if int(flag) == 1:
                    accepted += 1
                else:
                    rejected += 1

            t = parse_float(row.get("timestamp_s"))
            if t is None or t <= MOVING_START_S:
                continue

            nis = parse_float(row.get("nis"))
            if nis is not None:
                nis_values.append(nis)

            nees = parse_float(row.get("nees_pos"))
            if nees is not None and nees >= 0.0:
                nees_values.append(nees)

    total = accepted + rejected
    accept_pct = 100.0 * accepted / total if total > 0 else float("nan")
    nis_mean = float(np.mean(nis_values)) if nis_values else float("nan")
    nees_mean = float(np.mean(nees_values)) if nees_values else float("nan")
    return accepted, rejected, accept_pct, nis_mean, nees_mean


def plot_sweep_curves(points: Sequence[SweepPoint], plot_path: Path) -> None:
    scales = [p.p0_scale for p in points]
    rmse = [p.horizontal_rmse_m for p in points]
    accept = [p.gnss_accept_pct for p in points]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("H4 — P0 Sweep: RMSE y aceptacion GNSS", fontsize=13)

    axes[0].plot(scales, rmse, "o-", color="#3498db", linewidth=2.0, markersize=7)
    axes[0].set_xscale("log")
    axes[0].set_xlabel("Escala P0")
    axes[0].set_ylabel("RMSE horizontal (m)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(scales, accept, "o-", color="#e74c3c", linewidth=2.0, markersize=7)
    axes[1].set_xscale("log")
    axes[1].set_xlabel("Escala P0")
    axes[1].set_ylabel("Tasa aceptacion GPS (%)")
    axes[1].grid(True, alpha=0.3)

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def write_report(points: Sequence[SweepPoint]) -> None:
    payload = {
        "p0_scales": list(P0_SCALES),
        "moving_phase_start_s": MOVING_START_S,
        "runs": [
            {
                "p0_scale": p.p0_scale,
                "horizontal_rmse_m": p.horizontal_rmse_m,
                "error_h_final_m": p.error_h_final_m,
                "nis_mean": p.nis_mean,
                "gnss_accept_pct": p.gnss_accept_pct,
                "nees_pos_mean": p.nees_pos_mean,
                "gnss_accept_count": p.gnss_accept_count,
                "gnss_reject_count": p.gnss_reject_count,
                "output_csv": str(p.output_path.relative_to(REPO_ROOT)),
                "diagnostics_csv": str(p.diagnostics_path.relative_to(REPO_ROOT)),
            }
            for p in points
        ],
    }
    SWEEP_REPORT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with SWEEP_REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")


def main() -> int:
    try:
        replay_path = resolve_replay_path(None)
        orientation_path = resolve_orientation_path(None)
        input_dir = discover_input_dir()
        ensure_calibration(DEFAULT_CALIBRATION)

        BENCH_DIR.mkdir(parents=True, exist_ok=True)
        sweep_points: list[SweepPoint] = []

        for scale in P0_SCALES:
            tag = scale_tag(scale)
            output_csv = BENCH_DIR / f"h4_sweep_output_p0_{tag}x.csv"
            diagnostics_csv = (
                DIAGNOSTICS_CSV
                if scale == 1.0
                else BENCH_DIR / f"h4_consistency_diagnostics_p0_{tag}x.csv"
            )

            run_replay(
                DEFAULT_REPLAY_EXE,
                replay_path,
                output_csv,
                DEFAULT_CALIBRATION,
                scale,
                diagnostics_csv,
            )

            if not diagnostics_csv.is_file():
                raise FileNotFoundError(f"Falta diagnostico H4: {diagnostics_csv}")

            analysis = analyze(replay_path, output_csv, orientation_path, input_dir)
            acc, rej, accept_pct, nis_mean, nees_mean = metrics_from_diagnostics(
                diagnostics_csv
            )
            final_error = (
                float(analysis.horizontal_error_m[-1])
                if analysis.horizontal_error_m.size
                else float("nan")
            )

            sweep_points.append(
                SweepPoint(
                    p0_scale=scale,
                    horizontal_rmse_m=analysis.horizontal_rmse_m,
                    error_h_final_m=final_error,
                    nis_mean=nis_mean,
                    gnss_accept_pct=accept_pct,
                    nees_pos_mean=nees_mean,
                    gnss_accept_count=acc,
                    gnss_reject_count=rej,
                    output_path=output_csv,
                    diagnostics_path=diagnostics_csv,
                )
            )

        plot_sweep_curves(sweep_points, SWEEP_CURVES_PNG)
        write_report(sweep_points)

        print("=" * 78)
        print(" H4 — P0 SWEEP")
        print("=" * 78)
        print(
            f"  {'Scale':>8} {'RMSE H':>10} {'Err final':>10} "
            f"{'NIS mean':>10} {'Accept%':>10} {'NEES pos':>10}"
        )
        for p in sweep_points:
            print(
                f"  {p.p0_scale:>7g}x {p.horizontal_rmse_m:>10.1f} "
                f"{p.error_h_final_m:>10.1f} {p.nis_mean:>10.1f} "
                f"{p.gnss_accept_pct:>9.1f}% {p.nees_pos_mean:>10.1f}"
            )
        print("=" * 78)
        print(f"Reporte JSON:  {SWEEP_REPORT_JSON}")
        print(f"Curvas:        {SWEEP_CURVES_PNG}")
        print(f"Diagnostico:   {DIAGNOSTICS_CSV}")
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
