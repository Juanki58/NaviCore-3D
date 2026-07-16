#!/usr/bin/env python3
"""Experimento H0 vs H1 — montaje IMU legacy vs calibracion Rodrigues.

H0: montaje Euler legacy (transpuesta) en real_run_replay
H1: matriz documentada en calibration/imu_mount.json

Compara las mismas metricas objetivas que analyze_real_run.py:
  - RMSE horizontal
  - Error H final
  - NIS medio en marcha
  - RMSE Roll / Pitch / Yaw (vs Orientation.csv)
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY = REPO_ROOT / "docs" / "benchmarks" / "real_run_replay.csv"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
OUTPUT_H0 = REPO_ROOT / "docs" / "benchmarks" / "real_run_output_h0.csv"
OUTPUT_H1 = REPO_ROOT / "docs" / "benchmarks" / "real_run_output_h1.csv"
REPORT_JSON = REPO_ROOT / "docs" / "benchmarks" / "mount_experiment_report.json"

from analyze_real_run import (  # noqa: E402
    AnalysisResult,
    analyze,
    discover_input_dir,
    resolve_orientation_path,
    resolve_replay_path,
)


@dataclass(frozen=True)
class HypothesisMetrics:
    label: str
    mount_mode: str
    output_path: Path
    horizontal_rmse_m: float
    error_h_final_m: float
    nis_moving_mean: float
    att_rmse_roll_deg: float | None
    att_rmse_pitch_deg: float | None
    att_rmse_yaw_deg: float | None


def fmt_m(value: float) -> str:
    if not (value == value):  # NaN
        return "n/a"
    return f"{value:.1f} m"


def fmt_nis(value: float) -> str:
    if not (value == value):
        return "n/a"
    return f"{value:.1f}"


def fmt_deg(value: float | None) -> str:
    if value is None or not (value == value):
        return "n/a"
    return f"{value:.2f} deg"


def ensure_calibration(calibration_path: Path, replay_path: Path) -> None:
    if calibration_path.is_file():
        return
    print(f"Calibracion no encontrada; generando {calibration_path} ...")
    cmd = [
        sys.executable,
        str(REPO_ROOT / "audit_imu_chain.py"),
        "--replay",
        str(replay_path),
        "--export-calibration",
        str(calibration_path),
    ]
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def run_replay(
    replay_exe: Path,
    replay_csv: Path,
    output_csv: Path,
    mount_mode: str,
    calibration_path: Path,
) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(
            f"No se encontro {replay_exe}. Compila con: cmake --build build --target NaviCore3D_Replay"
        )
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(output_csv),
        "--mount-mode",
        mount_mode,
        "--mount-calibration",
        str(calibration_path),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def metrics_from_analysis(
    label: str,
    mount_mode: str,
    output_path: Path,
    result: AnalysisResult,
) -> HypothesisMetrics:
    att = result.att_rmse_deg
    return HypothesisMetrics(
        label=label,
        mount_mode=mount_mode,
        output_path=output_path,
        horizontal_rmse_m=result.horizontal_rmse_m,
        error_h_final_m=float(result.horizontal_error_m[-1]) if result.horizontal_error_m.size else float("nan"),
        nis_moving_mean=result.nis_moving_mean,
        att_rmse_roll_deg=att[0] if att is not None else None,
        att_rmse_pitch_deg=att[1] if att is not None else None,
        att_rmse_yaw_deg=att[2] if att is not None else None,
    )


def print_comparison(h0: HypothesisMetrics, h1: HypothesisMetrics) -> None:
    def delta(a: float, b: float) -> str:
        if not (a == a) or not (b == b):
            return "n/a"
        change = b - a
        pct = (change / a * 100.0) if abs(a) > 1e-9 else float("inf")
        sign = "+" if change >= 0 else ""
        return f"{sign}{change:.1f} ({sign}{pct:.1f}%)"

    print()
    print("=" * 72)
    print(" EXPERIMENTO MONTAJE IMU — H0 vs H1")
    print("=" * 72)
    print(f"  H0 ({h0.mount_mode}): {h0.output_path.name}")
    print(f"  H1 ({h1.mount_mode}): {h1.output_path.name}")
    print("-" * 72)
    print(f"{'Metrica':<24} {'H0':>14} {'H1':>14} {'Delta H1-H0':>16}")
    print("-" * 72)
    rows = [
        ("RMSE horizontal", fmt_m(h0.horizontal_rmse_m), fmt_m(h1.horizontal_rmse_m),
         delta(h0.horizontal_rmse_m, h1.horizontal_rmse_m)),
        ("Error H final", fmt_m(h0.error_h_final_m), fmt_m(h1.error_h_final_m),
         delta(h0.error_h_final_m, h1.error_h_final_m)),
        ("NIS medio (marcha)", fmt_nis(h0.nis_moving_mean), fmt_nis(h1.nis_moving_mean),
         delta(h0.nis_moving_mean, h1.nis_moving_mean)),
        ("RMSE Roll", fmt_deg(h0.att_rmse_roll_deg), fmt_deg(h1.att_rmse_roll_deg),
         delta(h0.att_rmse_roll_deg or float('nan'), h1.att_rmse_roll_deg or float('nan'))),
        ("RMSE Pitch", fmt_deg(h0.att_rmse_pitch_deg), fmt_deg(h1.att_rmse_pitch_deg),
         delta(h0.att_rmse_pitch_deg or float('nan'), h1.att_rmse_pitch_deg or float('nan'))),
        ("RMSE Yaw", fmt_deg(h0.att_rmse_yaw_deg), fmt_deg(h1.att_rmse_yaw_deg),
         delta(h0.att_rmse_yaw_deg or float('nan'), h1.att_rmse_yaw_deg or float('nan'))),
    ]
    for name, v0, v1, d in rows:
        print(f"{name:<24} {v0:>14} {v1:>14} {d:>16}")
    print("-" * 72)

    rmse_ratio = h0.horizontal_rmse_m / h1.horizontal_rmse_m if h1.horizontal_rmse_m > 1e-9 else float("inf")
    if h1.horizontal_rmse_m < h0.horizontal_rmse_m * 0.5 and h1.horizontal_rmse_m < 500.0:
        print(" CONCLUSION: mejora fuerte en posicion — el cuello de botella era probablemente R_mount.")
    elif h1.horizontal_rmse_m < h0.horizontal_rmse_m * 0.85:
        print(" CONCLUSION: mejora moderada — R_mount contribuye, pero revisar otros factores.")
    else:
        print(" CONCLUSION: mejora marginal o nula — descartar R_mount como causa principal;")
        print("             centrar en sincronizacion, modelo de sensor, bias, etc.")
    print("=" * 72)


def write_report(h0: HypothesisMetrics, h1: HypothesisMetrics, calibration_path: Path) -> None:
    payload = {
        "h0": {
            "label": h0.label,
            "mount_mode": h0.mount_mode,
            "output_csv": str(h0.output_path.relative_to(REPO_ROOT)),
            "horizontal_rmse_m": h0.horizontal_rmse_m,
            "error_h_final_m": h0.error_h_final_m,
            "nis_moving_mean": h0.nis_moving_mean,
            "att_rmse_deg": {
                "roll": h0.att_rmse_roll_deg,
                "pitch": h0.att_rmse_pitch_deg,
                "yaw": h0.att_rmse_yaw_deg,
            },
        },
        "h1": {
            "label": h1.label,
            "mount_mode": h1.mount_mode,
            "output_csv": str(h1.output_path.relative_to(REPO_ROOT)),
            "calibration": str(calibration_path.relative_to(REPO_ROOT)),
            "horizontal_rmse_m": h1.horizontal_rmse_m,
            "error_h_final_m": h1.error_h_final_m,
            "nis_moving_mean": h1.nis_moving_mean,
            "att_rmse_deg": {
                "roll": h1.att_rmse_roll_deg,
                "pitch": h1.att_rmse_pitch_deg,
                "yaw": h1.att_rmse_yaw_deg,
            },
        },
    }
    REPORT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    print(f"Informe JSON: {REPORT_JSON}")


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Experimento montaje IMU H0 vs H1")
    parser.add_argument("--replay", type=Path, default=None, help="CSV parseado de entrada")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--output-h0", type=Path, default=OUTPUT_H0)
    parser.add_argument("--output-h1", type=Path, default=OUTPUT_H1)
    parser.add_argument("--skip-replay", action="store_true", help="Solo analizar CSVs existentes")
    parser.add_argument("--orientation", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    try:
        replay_path = resolve_replay_path(args.replay)
        orientation_path = resolve_orientation_path(args.orientation)
        input_dir = discover_input_dir()

        ensure_calibration(args.calibration, replay_path)

        if not args.skip_replay:
            run_replay(args.replay_exe, replay_path, args.output_h0, "legacy", args.calibration)
            run_replay(args.replay_exe, replay_path, args.output_h1, "calibration", args.calibration)

        h0_result = analyze(replay_path, args.output_h0, orientation_path, input_dir)
        h1_result = analyze(replay_path, args.output_h1, orientation_path, input_dir)

        h0 = metrics_from_analysis("H0 legacy Euler", "legacy_euler", args.output_h0, h0_result)
        h1 = metrics_from_analysis("H1 Rodrigues", "calibration_file", args.output_h1, h1_result)

        print_comparison(h0, h1)
        write_report(h0, h1, args.calibration)
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
