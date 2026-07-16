#!/usr/bin/env python3
"""Experimento H0 vs H2 vs H3 — comparacion con montaje Rodrigues.

H0: yaw=0
H2: yaw GNSS estable (NIS gate activo)
H3: yaw GNSS estable + reset P pos + 5s gracia (manga ancha)
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

OUTPUT_H0 = REPO_ROOT / "docs" / "benchmarks" / "real_run_output_h0.csv"
OUTPUT_H2 = REPO_ROOT / "docs" / "benchmarks" / "real_run_output_h2.csv"
OUTPUT_H3 = REPO_ROOT / "docs" / "benchmarks" / "real_run_output_h3.csv"
DIAG_H0 = REPO_ROOT / "docs" / "benchmarks" / "h0_diagnostics.csv"
DIAG_H2 = REPO_ROOT / "docs" / "benchmarks" / "h2_diagnostics.csv"
H3_DIAGNOSTICS = REPO_ROOT / "docs" / "benchmarks" / "h3_diagnostics.csv"
REPORT_JSON = REPO_ROOT / "docs" / "benchmarks" / "h3_experiment_report.json"
REPORT_MD = REPO_ROOT / "docs" / "benchmarks" / "h3_experiment_report.md"
TRAJECTORY_PLOT = REPO_ROOT / "docs" / "benchmarks" / "h3_trajectory_comparison.png"

MOVING_START_S = 30.0

from analyze_real_run import (  # noqa: E402
    AnalysisResult,
    analyze,
    discover_input_dir,
    load_replay_gps,
    resolve_orientation_path,
    resolve_replay_path,
)


@dataclass(frozen=True)
class RunMetrics:
    label: str
    mode: str
    output_path: Path
    horizontal_rmse_m: float
    error_h_final_m: float
    nis_moving_mean: float
    att_rmse_roll_deg: float | None
    att_rmse_pitch_deg: float | None
    att_rmse_yaw_deg: float | None
    gnss_accept_pct: float
    gnss_reject_count: int
    gnss_accept_count: int


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


def ensure_calibration(calibration_path: Path) -> None:
    if calibration_path.is_file():
        return
    subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "audit_imu_chain.py"),
            "--export-calibration",
            str(calibration_path),
        ],
        cwd=REPO_ROOT,
        check=True,
    )


def run_replay(
    replay_exe: Path,
    replay_csv: Path,
    output_csv: Path,
    yaw_init: str,
    calibration: Path,
    diagnostics_csv: Path,
) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(
            f"No existe {replay_exe}. Compila con: cmake --build build --target NaviCore3D_Replay"
        )
    output_csv.parent.mkdir(parents=True, exist_ok=True)
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
        yaw_init,
        "--h3-diagnostics-csv",
        str(diagnostics_csv),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def gnss_stats_from_diagnostics(path: Path) -> tuple[int, int, float, float]:
    if not path.is_file():
        raise FileNotFoundError(f"No existe diagnostics: {path}")
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


def metrics_from_analysis(
    label: str,
    mode: str,
    output_path: Path,
    analysis: AnalysisResult,
    gnss_accept: int,
    gnss_reject: int,
    gnss_accept_pct: float,
    nis_moving_mean: float,
) -> RunMetrics:
    att = analysis.att_rmse_deg
    return RunMetrics(
        label=label,
        mode=mode,
        output_path=output_path,
        horizontal_rmse_m=analysis.horizontal_rmse_m,
        error_h_final_m=float(analysis.horizontal_error_m[-1]) if analysis.horizontal_error_m.size else float("nan"),
        nis_moving_mean=nis_moving_mean,
        att_rmse_roll_deg=att[0] if att is not None else None,
        att_rmse_pitch_deg=att[1] if att is not None else None,
        att_rmse_yaw_deg=att[2] if att is not None else None,
        gnss_accept_pct=gnss_accept_pct,
        gnss_reject_count=gnss_reject,
        gnss_accept_count=gnss_accept,
    )


def load_trajectory(output_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times: list[float] = []
    n_vals: list[float] = []
    e_vals: list[float] = []
    with output_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            t = parse_float(row.get("timestamp_s"))
            n = parse_float(row.get("pos_n_m"))
            e = parse_float(row.get("pos_e_m"))
            if t is None or n is None or e is None:
                continue
            times.append(t)
            n_vals.append(n)
            e_vals.append(e)
    return (
        np.array(times, dtype=float),
        np.array(n_vals, dtype=float),
        np.array(e_vals, dtype=float),
    )


def plot_trajectories(
    replay_path: Path,
    h0: RunMetrics,
    h2: RunMetrics,
    h3: RunMetrics,
    plot_path: Path,
) -> None:
    gps = load_replay_gps(replay_path)
    gps_e = np.array([s.pos_e for s in gps], dtype=float)
    gps_n = np.array([s.pos_n for s in gps], dtype=float)

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.plot(gps_e, gps_n, color="#2ecc71", linewidth=2.0, label="GPS referencia", zorder=4)

    for metrics, color, style in (
        (h0, "#95a5a6", "-"),
        (h2, "#3498db", "--"),
        (h3, "#e74c3c", "-."),
    ):
        _, n_arr, e_arr = load_trajectory(metrics.output_path)
        ax.plot(e_arr, n_arr, color=color, linewidth=1.4, linestyle=style, label=metrics.label, alpha=0.9)

    ax.set_xlabel("Este (m)")
    ax.set_ylabel("Norte (m)")
    ax.set_title("H0 / H2 / H3 — Trayectoria horizontal vs GPS")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def fmt_m(v: float) -> str:
    return f"{v:.1f}" if math.isfinite(v) else "n/a"


def fmt_deg(v: float | None) -> str:
    return f"{v:.2f}" if v is not None and math.isfinite(v) else "n/a"


def fmt_pct(v: float) -> str:
    return f"{v:.1f}%" if math.isfinite(v) else "n/a"


def write_markdown_table(h0: RunMetrics, h2: RunMetrics, h3: RunMetrics, path: Path) -> None:
    rows = [
        ("RMSE horizontal (m)", fmt_m(h0.horizontal_rmse_m), fmt_m(h2.horizontal_rmse_m), fmt_m(h3.horizontal_rmse_m)),
        ("Error H final (m)", fmt_m(h0.error_h_final_m), fmt_m(h2.error_h_final_m), fmt_m(h3.error_h_final_m)),
        ("NIS medio en marcha", fmt_m(h0.nis_moving_mean), fmt_m(h2.nis_moving_mean), fmt_m(h3.nis_moving_mean)),
        ("RMSE Roll (deg)", fmt_deg(h0.att_rmse_roll_deg), fmt_deg(h2.att_rmse_roll_deg), fmt_deg(h3.att_rmse_roll_deg)),
        ("RMSE Pitch (deg)", fmt_deg(h0.att_rmse_pitch_deg), fmt_deg(h2.att_rmse_pitch_deg), fmt_deg(h3.att_rmse_pitch_deg)),
        ("RMSE Yaw (deg)", fmt_deg(h0.att_rmse_yaw_deg), fmt_deg(h2.att_rmse_yaw_deg), fmt_deg(h3.att_rmse_yaw_deg)),
        ("GNSS aceptadas (%)", fmt_pct(h0.gnss_accept_pct), fmt_pct(h2.gnss_accept_pct), fmt_pct(h3.gnss_accept_pct)),
        ("GNSS rechazadas (n)", str(h0.gnss_reject_count), str(h2.gnss_reject_count), str(h3.gnss_reject_count)),
    ]
    lines = [
        "# Experimento H0 / H2 / H3",
        "",
        "Montaje: `calibration/imu_mount.json` (Rodrigues) en las tres corridas.",
        "",
        "| Metrica | H0 | H2 | H3 |",
        "|---------|----|----|-----|",
    ]
    for name, v0, v2, v3 in rows:
        lines.append(f"| {name} | {v0} | {v2} | {v3} |")
    lines.extend([
        "",
        f"Grafico trayectorias: `{TRAJECTORY_PLOT.relative_to(REPO_ROOT)}`",
        f"Informe JSON: `{REPORT_JSON.relative_to(REPO_ROOT)}`",
        f"H3 diagnostics: `{H3_DIAGNOSTICS.relative_to(REPO_ROOT)}`",
    ])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_json_report(h0: RunMetrics, h2: RunMetrics, h3: RunMetrics, path: Path) -> None:
    def row(m: RunMetrics) -> dict:
        return {
            "label": m.label,
            "mode": m.mode,
            "output_csv": str(m.output_path.relative_to(REPO_ROOT)),
            "horizontal_rmse_m": m.horizontal_rmse_m,
            "error_h_final_m": m.error_h_final_m,
            "nis_moving_mean": m.nis_moving_mean,
            "att_rmse_deg": {
                "roll": m.att_rmse_roll_deg,
                "pitch": m.att_rmse_pitch_deg,
                "yaw": m.att_rmse_yaw_deg,
            },
            "gnss_accept_pct": m.gnss_accept_pct,
            "gnss_accept_count": m.gnss_accept_count,
            "gnss_reject_count": m.gnss_reject_count,
        }

    payload = {"h0": row(h0), "h2": row(h2), "h3": row(h3)}
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")


def print_table(h0: RunMetrics, h2: RunMetrics, h3: RunMetrics) -> None:
    print()
    print("=" * 88)
    print(" EXPERIMENTO H0 / H2 / H3 (montaje Rodrigues)")
    print("=" * 88)
    print(f"{'Metrica':<28} {'H0':>14} {'H2':>14} {'H3':>14}")
    print("-" * 88)
    rows = [
        ("RMSE horizontal (m)", fmt_m(h0.horizontal_rmse_m), fmt_m(h2.horizontal_rmse_m), fmt_m(h3.horizontal_rmse_m)),
        ("Error H final (m)", fmt_m(h0.error_h_final_m), fmt_m(h2.error_h_final_m), fmt_m(h3.error_h_final_m)),
        ("NIS medio marcha", fmt_m(h0.nis_moving_mean), fmt_m(h2.nis_moving_mean), fmt_m(h3.nis_moving_mean)),
        ("RMSE Roll (deg)", fmt_deg(h0.att_rmse_roll_deg), fmt_deg(h2.att_rmse_roll_deg), fmt_deg(h3.att_rmse_roll_deg)),
        ("RMSE Pitch (deg)", fmt_deg(h0.att_rmse_pitch_deg), fmt_deg(h2.att_rmse_pitch_deg), fmt_deg(h3.att_rmse_pitch_deg)),
        ("RMSE Yaw (deg)", fmt_deg(h0.att_rmse_yaw_deg), fmt_deg(h2.att_rmse_yaw_deg), fmt_deg(h3.att_rmse_yaw_deg)),
        ("GNSS aceptadas", fmt_pct(h0.gnss_accept_pct), fmt_pct(h2.gnss_accept_pct), fmt_pct(h3.gnss_accept_pct)),
        ("GNSS rechazadas", str(h0.gnss_reject_count), str(h2.gnss_reject_count), str(h3.gnss_reject_count)),
    ]
    for name, v0, v2, v3 in rows:
        print(f"{name:<28} {v0:>14} {v2:>14} {v3:>14}")
    print("=" * 88)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Experimento H0/H2/H3")
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

        if not args.skip_replay:
            run_replay(args.replay_exe, replay_path, OUTPUT_H0, "zero", args.calibration, DIAG_H0)
            run_replay(args.replay_exe, replay_path, OUTPUT_H2, "h2", args.calibration, DIAG_H2)
            run_replay(args.replay_exe, replay_path, OUTPUT_H3, "h3", args.calibration, H3_DIAGNOSTICS)

        h0_analysis = analyze(replay_path, OUTPUT_H0, orientation_path, input_dir)
        h2_analysis = analyze(replay_path, OUTPUT_H2, orientation_path, input_dir)
        h3_analysis = analyze(replay_path, OUTPUT_H3, orientation_path, input_dir)

        h0 = metrics_from_analysis("H0 yaw=0", "h0", OUTPUT_H0, h0_analysis, *gnss_stats_from_diagnostics(DIAG_H0))
        h2 = metrics_from_analysis("H2 yaw estable", "h2", OUTPUT_H2, h2_analysis, *gnss_stats_from_diagnostics(DIAG_H2))
        h3 = metrics_from_analysis("H3 manga ancha", "h3", OUTPUT_H3, h3_analysis, *gnss_stats_from_diagnostics(H3_DIAGNOSTICS))

        print_table(h0, h2, h3)
        write_markdown_table(h0, h2, h3, REPORT_MD)
        write_json_report(h0, h2, h3, REPORT_JSON)
        plot_trajectories(replay_path, h0, h2, h3, TRAJECTORY_PLOT)

        print(f"Markdown:  {REPORT_MD}")
        print(f"JSON:      {REPORT_JSON}")
        print(f"Grafico:   {TRAJECTORY_PLOT}")
        if H3_DIAGNOSTICS.is_file():
            print(f"H3 diag:   {H3_DIAGNOSTICS}")
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
