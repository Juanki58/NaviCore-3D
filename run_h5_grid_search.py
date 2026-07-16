#!/usr/bin/env python3
"""H5 — Grid search over IMU process noise Q and NHC observation noise R."""

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

try:
    import seaborn as sns
except ImportError:  # pragma: no cover
    sns = None

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
Q_SCALE_FACTORS = (1.0, 10.0, 100.0, 1000.0)
NHC_SIGMAS = (0.1, 0.5, 1.0, 10.0)
MOVING_START_S = 30.0

BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
GRID_REPORT_JSON = BENCH_DIR / "h5_grid_report.json"
GRID_ANALYSIS_PNG = BENCH_DIR / "h5_grid_analysis.png"

from analyze_real_run import (  # noqa: E402
    analyze,
    discover_input_dir,
    resolve_orientation_path,
    resolve_replay_path,
)


@dataclass(frozen=True)
class GridPoint:
    q_scale: float
    nhc_sigma: float
    horizontal_rmse_m: float
    error_h_final_m: float
    gnss_accept_pct: float
    nis_mean: float
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


def param_tag(value: float) -> str:
    text = f"{value:g}".replace(".", "p")
    return text


def run_replay(
    replay_exe: Path,
    replay_csv: Path,
    output_csv: Path,
    calibration: Path,
    q_scale: float,
    nhc_sigma: float,
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
        "--q-scale",
        f"{q_scale}",
        "--nhc-sigma",
        f"{nhc_sigma}",
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


def build_matrix(
    points: Sequence[GridPoint],
    q_values: Sequence[float],
    nhc_values: Sequence[float],
    attr: str,
) -> np.ndarray:
    lookup = {(p.q_scale, p.nhc_sigma): getattr(p, attr) for p in points}
    matrix = np.full((len(nhc_values), len(q_values)), np.nan, dtype=float)
    for row_idx, nhc in enumerate(nhc_values):
        for col_idx, q_scale in enumerate(q_values):
            matrix[row_idx, col_idx] = lookup[(q_scale, nhc)]
    return matrix


def plot_heatmaps(points: Sequence[GridPoint], plot_path: Path) -> None:
    q_values = list(Q_SCALE_FACTORS)
    nhc_values = list(NHC_SIGMAS)
    accept_matrix = build_matrix(points, q_values, nhc_values, "gnss_accept_pct")
    rmse_matrix = build_matrix(points, q_values, nhc_values, "horizontal_rmse_m")

    q_labels = [f"{q:g}" for q in q_values]
    nhc_labels = [f"{s:g}" for s in nhc_values]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("H5 — Grid Search Q x R_nhc", fontsize=13)

    if sns is not None:
        sns.heatmap(
            accept_matrix,
            ax=axes[0],
            annot=True,
            fmt=".1f",
            cmap="YlGnBu",
            xticklabels=q_labels,
            yticklabels=nhc_labels,
            cbar_kws={"label": "GPS accept (%)"},
        )
        sns.heatmap(
            rmse_matrix,
            ax=axes[1],
            annot=True,
            fmt=".0f",
            cmap="magma_r",
            xticklabels=q_labels,
            yticklabels=nhc_labels,
            cbar_kws={"label": "RMSE horizontal (m)"},
        )
    else:
        for ax, matrix, title, cmap in (
            (axes[0], accept_matrix, "Tasa aceptacion GPS (%)", "YlGnBu"),
            (axes[1], rmse_matrix, "RMSE horizontal (m)", "magma_r"),
        ):
            im = ax.imshow(matrix, aspect="auto", origin="lower", cmap=cmap)
            ax.set_xticks(range(len(q_labels)))
            ax.set_xticklabels(q_labels)
            ax.set_yticks(range(len(nhc_labels)))
            ax.set_yticklabels(nhc_labels)
            ax.set_title(title)
            fig.colorbar(im, ax=ax)

    for ax, title in zip(axes, ("Tasa aceptacion GPS (%)", "RMSE horizontal (m)")):
        ax.set_xlabel("Factor escala Q")
        ax.set_ylabel("nhc_sigma (m/s)")
        ax.set_title(title)

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def write_report(points: Sequence[GridPoint]) -> None:
    payload = {
        "q_scale_factors": list(Q_SCALE_FACTORS),
        "nhc_sigmas": list(NHC_SIGMAS),
        "moving_phase_start_s": MOVING_START_S,
        "runs": [
            {
                "q_scale": p.q_scale,
                "nhc_sigma": p.nhc_sigma,
                "horizontal_rmse_m": p.horizontal_rmse_m,
                "error_h_final_m": p.error_h_final_m,
                "gnss_accept_pct": p.gnss_accept_pct,
                "nis_mean": p.nis_mean,
                "nees_pos_mean": p.nees_pos_mean,
                "gnss_accept_count": p.gnss_accept_count,
                "gnss_reject_count": p.gnss_reject_count,
                "output_csv": str(p.output_path.relative_to(REPO_ROOT)),
                "diagnostics_csv": str(p.diagnostics_path.relative_to(REPO_ROOT)),
            }
            for p in points
        ],
    }
    GRID_REPORT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with GRID_REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")


def main() -> int:
    try:
        replay_path = resolve_replay_path(None)
        orientation_path = resolve_orientation_path(None)
        input_dir = discover_input_dir()
        ensure_calibration(DEFAULT_CALIBRATION)

        BENCH_DIR.mkdir(parents=True, exist_ok=True)
        grid_points: list[GridPoint] = []

        for q_scale in Q_SCALE_FACTORS:
            for nhc_sigma in NHC_SIGMAS:
                tag = f"q{param_tag(q_scale)}_nhc{param_tag(nhc_sigma)}"
                output_csv = BENCH_DIR / f"h5_grid_{tag}.csv"
                diagnostics_csv = BENCH_DIR / f"h5_grid_{tag}_diagnostics.csv"

                run_replay(
                    DEFAULT_REPLAY_EXE,
                    replay_path,
                    output_csv,
                    DEFAULT_CALIBRATION,
                    q_scale,
                    nhc_sigma,
                    diagnostics_csv,
                )

                if not diagnostics_csv.is_file():
                    raise FileNotFoundError(f"Falta diagnostico H5: {diagnostics_csv}")

                analysis = analyze(replay_path, output_csv, orientation_path, input_dir)
                acc, rej, accept_pct, nis_mean, nees_mean = metrics_from_diagnostics(
                    diagnostics_csv
                )
                final_error = (
                    float(analysis.horizontal_error_m[-1])
                    if analysis.horizontal_error_m.size
                    else float("nan")
                )

                grid_points.append(
                    GridPoint(
                        q_scale=q_scale,
                        nhc_sigma=nhc_sigma,
                        horizontal_rmse_m=analysis.horizontal_rmse_m,
                        error_h_final_m=final_error,
                        gnss_accept_pct=accept_pct,
                        nis_mean=nis_mean,
                        nees_pos_mean=nees_mean,
                        gnss_accept_count=acc,
                        gnss_reject_count=rej,
                        output_path=output_csv,
                        diagnostics_path=diagnostics_csv,
                    )
                )

        plot_heatmaps(grid_points, GRID_ANALYSIS_PNG)
        write_report(grid_points)

        print("=" * 88)
        print(" H5 — GRID SEARCH Q x R_nhc")
        print("=" * 88)
        print(
            f"  {'Q scale':>8} {'NHC sigma':>10} {'RMSE H':>10} {'Err final':>10} "
            f"{'Accept%':>10} {'NIS mean':>12} {'NEES pos':>12}"
        )
        for p in grid_points:
            print(
                f"  {p.q_scale:>7g}x {p.nhc_sigma:>10g} {p.horizontal_rmse_m:>10.1f} "
                f"{p.error_h_final_m:>10.1f} {p.gnss_accept_pct:>9.1f}% "
                f"{p.nis_mean:>12.1f} {p.nees_pos_mean:>12.1f}"
            )
        print("=" * 88)
        print(f"Reporte JSON: {GRID_REPORT_JSON}")
        print(f"Heatmaps:     {GRID_ANALYSIS_PNG}")
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
