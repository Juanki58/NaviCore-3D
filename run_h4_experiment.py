#!/usr/bin/env python3
"""H4 — Consistency Test (NEES/NIS) + barrido sistematico de P0.

Parte 1: registro de trace(P), det(P), innovacion, NIS, sqrt(Pnn/ee/dd) vs error real.
Parte 2: P0 x {1,2,5,10,20,50} -> curvas RMSE y tasa aceptacion GNSS.
"""

from __future__ import annotations

import argparse
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
P0_SCALES = (1.0, 2.0, 5.0, 10.0, 20.0, 50.0)
MOVING_START_S = 30.0

CONSISTENCY_CSV = REPO_ROOT / "docs" / "benchmarks" / "h4_consistency.csv"
CONSISTENCY_PLOT = REPO_ROOT / "docs" / "benchmarks" / "h4_consistency_analysis.png"
SWEEP_RMSE_PLOT = REPO_ROOT / "docs" / "benchmarks" / "h4_p0_sweep_rmse.png"
SWEEP_ACCEPT_PLOT = REPO_ROOT / "docs" / "benchmarks" / "h4_p0_sweep_accept.png"
REPORT_JSON = REPO_ROOT / "docs" / "benchmarks" / "h4_experiment_report.json"
REPORT_MD = REPO_ROOT / "docs" / "benchmarks" / "h4_experiment_report.md"

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
    gnss_accept_pct: float
    gnss_accept_count: int
    gnss_reject_count: int
    nis_moving_mean: float
    output_path: Path
    consistency_path: Path


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
        [sys.executable, str(REPO_ROOT / "audit_imu_chain.py"), "--export-calibration", str(path)],
        cwd=REPO_ROOT,
        check=True,
    )


def run_replay(
    replay_exe: Path,
    replay_csv: Path,
    output_csv: Path,
    calibration: Path,
    p0_scale: float,
    consistency_csv: Path | None = None,
) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")
    cmd = [
        str(replay_exe),
        "--input", str(replay_csv),
        "--output", str(output_csv),
        "--mount-mode", "calibration",
        "--mount-calibration", str(calibration),
        "--yaw-init", "zero",
        "--p0-scale", f"{p0_scale}",
    ]
    if consistency_csv is not None:
        cmd.extend(["--consistency-csv", str(consistency_csv)])
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def gnss_stats_from_consistency(path: Path) -> tuple[int, int, float, float]:
    accepted = 0
    rejected = 0
    nis_values: list[float] = []
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
            nis = parse_float(row.get("nis"))
            if t is not None and t > MOVING_START_S and nis is not None:
                nis_values.append(nis)
    total = accepted + rejected
    return (
        accepted,
        rejected,
        100.0 * accepted / total if total > 0 else float("nan"),
        float(np.mean(nis_values)) if nis_values else float("nan"),
    )


def load_consistency_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def analyze_consistency(rows: list[dict[str, str]]) -> dict[str, float]:
    moving = []
    for row in rows:
        t = parse_float(row.get("timestamp_s"))
        if t is None or t <= MOVING_START_S:
            continue
        moving.append(row)

    if not moving:
        return {}

    innov_h = np.array([parse_float(r.get("innovation_h_m")) or 0.0 for r in moving], dtype=float)
    sqrt_pnn = np.array([parse_float(r.get("sqrt_Pnn")) or 0.0 for r in moving], dtype=float)
    sqrt_pee = np.array([parse_float(r.get("sqrt_Pee")) or 0.0 for r in moving], dtype=float)
    sqrt_p_h = np.sqrt(sqrt_pnn * sqrt_pnn + sqrt_pee * sqrt_pee)
    nees_n = np.array([parse_float(r.get("nees_ratio_n")) or 0.0 for r in moving], dtype=float)
    nees_e = np.array([parse_float(r.get("nees_ratio_e")) or 0.0 for r in moving], dtype=float)
    nis = np.array([parse_float(r.get("nis")) or 0.0 for r in moving], dtype=float)

    ratio = innov_h / np.maximum(sqrt_p_h, 1.0e-6)

    return {
        "samples_moving": float(len(moving)),
        "mean_innovation_h_m": float(np.mean(innov_h)),
        "mean_sqrt_P_h_m": float(np.mean(sqrt_p_h)),
        "mean_error_over_sigma_h": float(np.mean(ratio)),
        "median_error_over_sigma_h": float(np.median(ratio)),
        "mean_nees_n": float(np.mean(nees_n)),
        "mean_nees_e": float(np.mean(nees_e)),
        "mean_nis": float(np.mean(nis)),
        "pct_nees_n_gt_11": float(100.0 * np.mean(nees_n > 11.345)),
        "pct_nees_e_gt_11": float(100.0 * np.mean(nees_e > 11.345)),
    }


def plot_consistency(rows: list[dict[str, str]], plot_path: Path) -> None:
    times: list[float] = []
    innov_h: list[float] = []
    sqrt_p_h: list[float] = []
    nis: list[float] = []
    accepted: list[int] = []

    for row in rows:
        t = parse_float(row.get("timestamp_s"))
        if t is None:
            continue
        times.append(t)
        ih = parse_float(row.get("innovation_h_m")) or 0.0
        spn = parse_float(row.get("sqrt_Pnn")) or 0.0
        spe = parse_float(row.get("sqrt_Pee")) or 0.0
        innov_h.append(ih)
        sqrt_p_h.append(math.hypot(spn, spe))
        nis.append(parse_float(row.get("nis")) or 0.0)
        accepted.append(int(parse_float(row.get("gnss_accepted")) or 0))

    t_arr = np.array(times)
    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
    fig.suptitle("H4 — Consistency Test: error real vs incertidumbre declarada", fontsize=13)

    axes[0].plot(t_arr, innov_h, "r-", linewidth=1.0, label="|innovacion| horizontal (error real)")
    axes[0].plot(t_arr, sqrt_p_h, "b--", linewidth=1.0, label="sqrt(Pnn^2+Pee^2) declarado")
    axes[0].set_ylabel("Metros")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    ratio = np.array(innov_h) / np.maximum(np.array(sqrt_p_h), 1e-6)
    axes[1].plot(t_arr, ratio, color="#9b59b6", linewidth=1.0)
    axes[1].axhline(3.0, color="#7f8c8d", linestyle="--", label="3-sigma")
    axes[1].set_ylabel("error / sigma")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t_arr, nis, color="#e67e22", linewidth=1.0, label="NIS")
    axes[2].axhline(11.345, color="#7f8c8d", linestyle="--", label="umbral chi2")
    axes[2].step(t_arr, accepted, where="post", color="#2ecc71", alpha=0.5, label="accepted")
    axes[2].set_xlabel("Tiempo (s)")
    axes[2].set_ylabel("NIS / accept")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_p0_sweep(points: Sequence[SweepPoint]) -> None:
    scales = [p.p0_scale for p in points]
    rmse = [p.horizontal_rmse_m for p in points]
    accept = [p.gnss_accept_pct for p in points]

    SWEEP_RMSE_PLOT.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(scales, rmse, "o-", color="#3498db", linewidth=2.0)
    ax.set_xscale("log")
    ax.set_xlabel("P0 scale factor")
    ax.set_ylabel("RMSE horizontal (m)")
    ax.set_title("H4 — RMSE vs P0 scale")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(SWEEP_RMSE_PLOT, dpi=150, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(scales, accept, "o-", color="#e74c3c", linewidth=2.0)
    ax.set_xscale("log")
    ax.set_xlabel("P0 scale factor")
    ax.set_ylabel("GNSS accept rate (%)")
    ax.set_title("H4 — GNSS accept rate vs P0 scale")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(SWEEP_ACCEPT_PLOT, dpi=150, bbox_inches="tight")
    plt.close(fig)


def write_reports(
    consistency_stats: dict[str, float],
    sweep_points: Sequence[SweepPoint],
) -> None:
    lines = [
        "# H4 — Consistency Test + P0 Sweep",
        "",
        "## Consistency (P0=1x, fase en marcha)",
        "",
    ]
    if consistency_stats:
        lines.extend([
            f"- Muestras en marcha: {consistency_stats['samples_moving']:.0f}",
            f"- Error horizontal medio: {consistency_stats['mean_innovation_h_m']:.1f} m",
            f"- Sigma declarada media (sqrt P_h): {consistency_stats['mean_sqrt_P_h_m']:.1f} m",
            f"- Ratio error/sigma medio: **{consistency_stats['mean_error_over_sigma_h']:.1f}**",
            f"- Ratio error/sigma mediana: {consistency_stats['median_error_over_sigma_h']:.1f}",
            f"- NEES medio (eje N): {consistency_stats['mean_nees_n']:.1f}",
            f"- NEES medio (eje E): {consistency_stats['mean_nees_e']:.1f}",
            f"- % NEES_N > 11.3: {consistency_stats['pct_nees_n_gt_11']:.1f}%",
            "",
            "> Criterio: error/sigma ~ 1 indica consistencia. Valores >> 1 indican P/R optimistas.",
            "",
        ])
    lines.extend([
        "## Barrido P0",
        "",
        "| P0 scale | RMSE H (m) | Error final (m) | GNSS accept % | Rechazos | NIS medio |",
        "|----------|------------|-----------------|---------------|----------|-----------|",
    ])
    for p in sweep_points:
        lines.append(
            f"| {p.p0_scale:g}x | {p.horizontal_rmse_m:.1f} | {p.error_h_final_m:.1f} | "
            f"{p.gnss_accept_pct:.1f}% | {p.gnss_reject_count} | {p.nis_moving_mean:.0f} |"
        )
    lines.extend([
        "",
        f"Graficos: `{SWEEP_RMSE_PLOT.name}`, `{SWEEP_ACCEPT_PLOT.name}`, `{CONSISTENCY_PLOT.name}`",
    ])
    REPORT_MD.parent.mkdir(parents=True, exist_ok=True)
    REPORT_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")

    payload = {
        "consistency": consistency_stats,
        "p0_sweep": [
            {
                "p0_scale": p.p0_scale,
                "horizontal_rmse_m": p.horizontal_rmse_m,
                "error_h_final_m": p.error_h_final_m,
                "gnss_accept_pct": p.gnss_accept_pct,
                "gnss_accept_count": p.gnss_accept_count,
                "gnss_reject_count": p.gnss_reject_count,
                "nis_moving_mean": p.nis_moving_mean,
                "output_csv": str(p.output_path.relative_to(REPO_ROOT)),
            }
            for p in sweep_points
        ],
    }
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="H4 consistency + P0 sweep")
    parser.add_argument("--replay", type=Path, default=None)
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-replay", action="store_true")
    parser.add_argument("--orientation", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        replay_path = resolve_replay_path(args.replay)
        orientation_path = resolve_orientation_path(args.orientation)
        input_dir = discover_input_dir()
        ensure_calibration(args.calibration)

        bench_dir = REPO_ROOT / "docs" / "benchmarks"
        bench_dir.mkdir(parents=True, exist_ok=True)

        if not args.skip_replay:
            baseline_output = bench_dir / "h4_output_p0_1x.csv"
            run_replay(
                args.replay_exe,
                replay_path,
                baseline_output,
                args.calibration,
                1.0,
                CONSISTENCY_CSV,
            )

            for scale in P0_SCALES:
                if scale == 1.0:
                    continue
                scale_tag = str(scale).replace(".", "p")
                out = bench_dir / f"h4_output_p0_{scale_tag}x.csv"
                cons = bench_dir / f"h4_consistency_p0_{scale_tag}x.csv"
                run_replay(args.replay_exe, replay_path, out, args.calibration, scale, cons)

        if not CONSISTENCY_CSV.is_file():
            raise FileNotFoundError(f"Falta {CONSISTENCY_CSV}")

        consistency_rows = load_consistency_rows(CONSISTENCY_CSV)
        consistency_stats = analyze_consistency(consistency_rows)
        plot_consistency(consistency_rows, CONSISTENCY_PLOT)

        sweep_points: list[SweepPoint] = []
        for scale in P0_SCALES:
            scale_tag = str(scale).replace(".", "p")
            output = bench_dir / (f"h4_output_p0_{scale_tag}x.csv" if scale != 1.0 else "h4_output_p0_1x.csv")
            consistency = bench_dir / (f"h4_consistency_p0_{scale_tag}x.csv" if scale != 1.0 else "h4_consistency.csv")
            if not output.is_file():
                raise FileNotFoundError(f"Falta salida para P0={scale}: {output}")
            if not consistency.is_file():
                consistency = CONSISTENCY_CSV if scale == 1.0 else consistency

            analysis = analyze(replay_path, output, orientation_path, input_dir)
            acc, rej, acc_pct, nis_mean = gnss_stats_from_consistency(consistency)
            sweep_points.append(
                SweepPoint(
                    p0_scale=scale,
                    horizontal_rmse_m=analysis.horizontal_rmse_m,
                    error_h_final_m=float(analysis.horizontal_error_m[-1]) if analysis.horizontal_error_m.size else float("nan"),
                    gnss_accept_pct=acc_pct,
                    gnss_accept_count=acc,
                    gnss_reject_count=rej,
                    nis_moving_mean=nis_mean,
                    output_path=output,
                    consistency_path=consistency,
                )
            )

        plot_p0_sweep(sweep_points)
        write_reports(consistency_stats, sweep_points)

        print("=" * 72)
        print(" H4 — CONSISTENCY TEST")
        print("=" * 72)
        if consistency_stats:
            print(f"  Error H medio:        {consistency_stats['mean_innovation_h_m']:.1f} m")
            print(f"  Sigma declarada H:    {consistency_stats['mean_sqrt_P_h_m']:.1f} m")
            print(f"  Ratio error/sigma:    {consistency_stats['mean_error_over_sigma_h']:.1f}x (>>1 = inconsistente)")
            print(f"  NEES medio N/E:       {consistency_stats['mean_nees_n']:.1f} / {consistency_stats['mean_nees_e']:.1f}")
        print("-" * 72)
        print(" P0 SWEEP")
        print(f"  {'Scale':>8} {'RMSE':>10} {'Accept%':>10} {'Rejects':>10}")
        for p in sweep_points:
            print(f"  {p.p0_scale:>7g}x {p.horizontal_rmse_m:>10.1f} {p.gnss_accept_pct:>9.1f}% {p.gnss_reject_count:>10}")
        print("=" * 72)
        print(f"Markdown:     {REPORT_MD}")
        print(f"JSON:         {REPORT_JSON}")
        print(f"Consistency:  {CONSISTENCY_PLOT}")
        print(f"P0 curves:    {SWEEP_RMSE_PLOT}, {SWEEP_ACCEPT_PLOT}")
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
