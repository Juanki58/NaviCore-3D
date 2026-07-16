#!/usr/bin/env python3
"""Experimento H0 vs H2 — yaw inicial cero vs rumbo GNSS estable.

Ambas corridas usan montaje calibrado (roll/pitch validados en H1).
H0: yaw=0 al arrancar (comportamiento actual)
H2: yaw desde heading GNSS cuando speed>=3 m/s durante N muestras estables

Metricas:
  RMSE horizontal, error final, NIS medio, RMSE Roll/Pitch/Yaw,
  tiempo hasta convergencia, % GNSS aceptadas, rechazos totales.
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

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY = REPO_ROOT / "docs" / "benchmarks" / "real_run_replay.csv"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
OUTPUT_H0 = REPO_ROOT / "docs" / "benchmarks" / "real_run_output_yaw_h0.csv"
OUTPUT_H2 = REPO_ROOT / "docs" / "benchmarks" / "real_run_output_yaw_h2.csv"
INSTR_H0 = REPO_ROOT / "docs" / "benchmarks" / "yaw_init_instrumentation_h0.csv"
INSTR_H2 = REPO_ROOT / "docs" / "benchmarks" / "yaw_init_instrumentation_h2.csv"
REPORT_JSON = REPO_ROOT / "docs" / "benchmarks" / "yaw_init_experiment_report.json"

MOVING_START_S = 30.0
CONVERGENCE_ERROR_M = 100.0
NIS_THRESHOLD = 11.345

from analyze_real_run import (  # noqa: E402
    AnalysisResult,
    analyze,
    discover_input_dir,
    interpolate_series,
    load_replay_gps,
    resolve_orientation_path,
    resolve_replay_path,
)


@dataclass(frozen=True)
class InstrumentationStats:
    gnss_rows: int
    gnss_accepted: int
    gnss_rejected: int
    gnss_accept_pct: float
    nis_moving_mean: float
    mean_abs_delta_yaw_deg: float | None
    convergence_time_s: float | None


@dataclass(frozen=True)
class HypothesisMetrics:
    label: str
    yaw_init_mode: str
    output_path: Path
    instrumentation_path: Path
    horizontal_rmse_m: float
    error_h_final_m: float
    nis_moving_mean: float
    att_rmse_roll_deg: float | None
    att_rmse_pitch_deg: float | None
    att_rmse_yaw_deg: float | None
    gnss_accept_pct: float
    gnss_reject_count: int
    convergence_time_s: float | None
    yaw_init_applied: bool | None
    yaw_init_heading_deg: float | None


def fmt_m(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.1f} m"


def fmt_nis(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.1f}"


def fmt_deg(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.2f} deg"


def fmt_pct(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.1f}%"


def fmt_time(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.1f} s"


def ensure_calibration(calibration_path: Path, replay_path: Path) -> None:
    if calibration_path.is_file():
        return
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
    instrumentation_csv: Path,
    yaw_init_mode: str,
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
        "calibration",
        "--mount-calibration",
        str(calibration_path),
        "--yaw-init",
        yaw_init_mode,
        "--instrumentation-csv",
        str(instrumentation_csv),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def load_instrumentation(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def parse_optional_float(text: str | None) -> float | None:
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


def compute_convergence_time(
    replay_path: Path,
    output_path: Path,
    threshold_m: float = CONVERGENCE_ERROR_M,
    moving_start_s: float = MOVING_START_S,
) -> float | None:
    gps_samples = load_replay_gps(replay_path)
    if not gps_samples:
        return None

    gps_times = np.array([s.timestamp_s for s in gps_samples], dtype=float)
    gps_n = np.array([s.pos_n for s in gps_samples], dtype=float)
    gps_e = np.array([s.pos_e for s in gps_samples], dtype=float)

    out_times: list[float] = []
    out_n: list[float] = []
    out_e: list[float] = []
    with output_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            t = parse_optional_float(row.get("timestamp_s"))
            n = parse_optional_float(row.get("pos_n_m"))
            e = parse_optional_float(row.get("pos_e_m"))
            if t is None or n is None or e is None:
                continue
            out_times.append(t)
            out_n.append(n)
            out_e.append(e)

    if not out_times:
        return None

    out_times_arr = np.array(out_times, dtype=float)
    out_n_arr = np.array(out_n, dtype=float)
    out_e_arr = np.array(out_e, dtype=float)

    filt_n = interpolate_series(gps_times, out_times_arr, out_n_arr)
    filt_e = interpolate_series(gps_times, out_times_arr, out_e_arr)
    horizontal_error = np.hypot(filt_n - gps_n, filt_e - gps_e)

    for t, err in zip(gps_times, horizontal_error):
        if t >= moving_start_s and err <= threshold_m:
            return float(t)
    return None


def analyze_instrumentation(
    path: Path,
    replay_path: Path,
    output_path: Path,
) -> InstrumentationStats:
    rows = load_instrumentation(path)
    if not rows:
        return InstrumentationStats(0, 0, 0, float("nan"), float("nan"), None, None)

    accepted = 0
    rejected = 0
    nis_values: list[float] = []
    delta_yaw_values: list[float] = []

    for row in rows:
        accepted_flag = parse_optional_float(row.get("gnss_accepted"))
        if accepted_flag is not None:
            if int(accepted_flag) == 1:
                accepted += 1
            else:
                rejected += 1

        t = parse_optional_float(row.get("timestamp_s"))
        nis = parse_optional_float(row.get("nis"))
        if t is not None and t > MOVING_START_S and nis is not None:
            nis_values.append(nis)

        delta_yaw = parse_optional_float(row.get("delta_yaw_deg"))
        if delta_yaw is not None:
            delta_yaw_values.append(abs(delta_yaw))

    total = accepted + rejected
    accept_pct = (100.0 * accepted / total) if total > 0 else float("nan")
    nis_mean = float(np.mean(nis_values)) if nis_values else float("nan")
    mean_abs_delta_yaw = float(np.mean(delta_yaw_values)) if delta_yaw_values else None
    convergence_s = compute_convergence_time(replay_path, output_path)

    return InstrumentationStats(
        gnss_rows=total,
        gnss_accepted=accepted,
        gnss_rejected=rejected,
        gnss_accept_pct=accept_pct,
        nis_moving_mean=nis_mean,
        mean_abs_delta_yaw_deg=mean_abs_delta_yaw,
        convergence_time_s=convergence_s,
    )


def metrics_from_analysis(
    label: str,
    yaw_init_mode: str,
    output_path: Path,
    instrumentation_path: Path,
    analysis: AnalysisResult,
    instr: InstrumentationStats,
) -> HypothesisMetrics:
    att = analysis.att_rmse_deg
    return HypothesisMetrics(
        label=label,
        yaw_init_mode=yaw_init_mode,
        output_path=output_path,
        instrumentation_path=instrumentation_path,
        horizontal_rmse_m=analysis.horizontal_rmse_m,
        error_h_final_m=float(analysis.horizontal_error_m[-1]) if analysis.horizontal_error_m.size else float("nan"),
        nis_moving_mean=instr.nis_moving_mean,
        att_rmse_roll_deg=att[0] if att is not None else None,
        att_rmse_pitch_deg=att[1] if att is not None else None,
        att_rmse_yaw_deg=att[2] if att is not None else None,
        gnss_accept_pct=instr.gnss_accept_pct,
        gnss_reject_count=instr.gnss_rejected,
        convergence_time_s=instr.convergence_time_s,
        yaw_init_applied=None,
        yaw_init_heading_deg=None,
    )


def print_comparison(h0: HypothesisMetrics, h2: HypothesisMetrics) -> None:
    def delta(a: float, b: float) -> str:
        if not math.isfinite(a) or not math.isfinite(b):
            return "n/a"
        change = b - a
        pct = (change / a * 100.0) if abs(a) > 1e-9 else float("inf")
        sign = "+" if change >= 0 else ""
        return f"{sign}{change:.1f} ({sign}{pct:.1f}%)"

    print()
    print("=" * 78)
    print(" EXPERIMENTO YAW INICIAL — H0 vs H2")
    print(" (montaje: calibration/imu_mount.json en ambas corridas)")
    print("=" * 78)
    print(f"  H0 ({h0.yaw_init_mode}): {h0.output_path.name}")
    print(f"  H2 ({h2.yaw_init_mode}): {h2.output_path.name}")
    print("-" * 78)
    print(f"{'Metrica':<28} {'H0':>14} {'H2':>14} {'Delta H2-H0':>16}")
    print("-" * 78)
    rows = [
        ("RMSE horizontal", fmt_m(h0.horizontal_rmse_m), fmt_m(h2.horizontal_rmse_m),
         delta(h0.horizontal_rmse_m, h2.horizontal_rmse_m)),
        ("Error H final", fmt_m(h0.error_h_final_m), fmt_m(h2.error_h_final_m),
         delta(h0.error_h_final_m, h2.error_h_final_m)),
        ("NIS medio (marcha)", fmt_nis(h0.nis_moving_mean), fmt_nis(h2.nis_moving_mean),
         delta(h0.nis_moving_mean, h2.nis_moving_mean)),
        ("RMSE Roll", fmt_deg(h0.att_rmse_roll_deg), fmt_deg(h2.att_rmse_roll_deg),
         delta(h0.att_rmse_roll_deg or float("nan"), h2.att_rmse_roll_deg or float("nan"))),
        ("RMSE Pitch", fmt_deg(h0.att_rmse_pitch_deg), fmt_deg(h2.att_rmse_pitch_deg),
         delta(h0.att_rmse_pitch_deg or float("nan"), h2.att_rmse_pitch_deg or float("nan"))),
        ("RMSE Yaw", fmt_deg(h0.att_rmse_yaw_deg), fmt_deg(h2.att_rmse_yaw_deg),
         delta(h0.att_rmse_yaw_deg or float("nan"), h2.att_rmse_yaw_deg or float("nan"))),
        ("Tiempo convergencia", fmt_time(h0.convergence_time_s), fmt_time(h2.convergence_time_s),
         delta(h0.convergence_time_s or float("nan"), h2.convergence_time_s or float("nan"))),
        ("GNSS aceptadas", fmt_pct(h0.gnss_accept_pct), fmt_pct(h2.gnss_accept_pct),
         delta(h0.gnss_accept_pct, h2.gnss_accept_pct)),
        ("GNSS rechazadas", f"{h0.gnss_reject_count}", f"{h2.gnss_reject_count}",
         delta(float(h0.gnss_reject_count), float(h2.gnss_reject_count))),
    ]
    for name, v0, v1, d in rows:
        print(f"{name:<28} {v0:>14} {v1:>14} {d:>16}")
    print("-" * 78)

    yaw_improved = (
        h2.att_rmse_yaw_deg is not None
        and h0.att_rmse_yaw_deg is not None
        and h2.att_rmse_yaw_deg < h0.att_rmse_yaw_deg * 0.5
    )
    pos_improved_strong = (
        math.isfinite(h0.horizontal_rmse_m)
        and math.isfinite(h2.horizontal_rmse_m)
        and h2.horizontal_rmse_m < h0.horizontal_rmse_m * 0.5
        and h2.horizontal_rmse_m < 500.0
    )
    pos_improved_moderate = (
        math.isfinite(h0.horizontal_rmse_m)
        and math.isfinite(h2.horizontal_rmse_m)
        and h2.horizontal_rmse_m < h0.horizontal_rmse_m * 0.85
    )
    nis_improved = (
        math.isfinite(h0.nis_moving_mean)
        and math.isfinite(h2.nis_moving_mean)
        and h2.nis_moving_mean < h0.nis_moving_mean * 0.5
    )

    if yaw_improved and pos_improved_strong and nis_improved:
        print(" CONCLUSION (Escenario A): yaw inicial era el gran culpable.")
    elif yaw_improved and (pos_improved_moderate or nis_improved):
        print(" CONCLUSION (Escenario B): yaw contribuye; persisten otros factores")
        print("             (sincronizacion, ruido GNSS, sesgos, etc.).")
    elif yaw_improved and not pos_improved_moderate:
        print(" CONCLUSION (Escenario C): yaw mejora en actitud pero no en posicion.")
        print("             Descartar yaw inicial como causa principal del drift.")
    else:
        print(" CONCLUSION: H2 no refuta H0 de forma clara — revisar instrumentacion CSV.")
    print(f"  Instrumentacion H0: {h0.instrumentation_path}")
    print(f"  Instrumentacion H2: {h2.instrumentation_path}")
    print("=" * 78)


def write_report(h0: HypothesisMetrics, h2: HypothesisMetrics) -> None:
    def row(metrics: HypothesisMetrics) -> dict:
        return {
            "label": metrics.label,
            "yaw_init_mode": metrics.yaw_init_mode,
            "output_csv": str(metrics.output_path.relative_to(REPO_ROOT)),
            "instrumentation_csv": str(metrics.instrumentation_path.relative_to(REPO_ROOT)),
            "horizontal_rmse_m": metrics.horizontal_rmse_m,
            "error_h_final_m": metrics.error_h_final_m,
            "nis_moving_mean": metrics.nis_moving_mean,
            "att_rmse_deg": {
                "roll": metrics.att_rmse_roll_deg,
                "pitch": metrics.att_rmse_pitch_deg,
                "yaw": metrics.att_rmse_yaw_deg,
            },
            "gnss_accept_pct": metrics.gnss_accept_pct,
            "gnss_reject_count": metrics.gnss_reject_count,
            "convergence_time_s": metrics.convergence_time_s,
        }

    payload = {"h0": row(h0), "h2": row(h2)}
    REPORT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    print(f"Informe JSON: {REPORT_JSON}")


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Experimento yaw inicial H0 vs H2")
    parser.add_argument("--replay", type=Path, default=None)
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--output-h0", type=Path, default=OUTPUT_H0)
    parser.add_argument("--output-h2", type=Path, default=OUTPUT_H2)
    parser.add_argument("--instrumentation-h0", type=Path, default=INSTR_H0)
    parser.add_argument("--instrumentation-h2", type=Path, default=INSTR_H2)
    parser.add_argument("--skip-replay", action="store_true")
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
            run_replay(
                args.replay_exe,
                replay_path,
                args.output_h0,
                args.instrumentation_h0,
                "zero",
                args.calibration,
            )
            run_replay(
                args.replay_exe,
                replay_path,
                args.output_h2,
                args.instrumentation_h2,
                "gnss_stable",
                args.calibration,
            )

        h0_analysis = analyze(replay_path, args.output_h0, orientation_path, input_dir)
        h2_analysis = analyze(replay_path, args.output_h2, orientation_path, input_dir)
        h0_instr = analyze_instrumentation(args.instrumentation_h0, replay_path, args.output_h0)
        h2_instr = analyze_instrumentation(args.instrumentation_h2, replay_path, args.output_h2)

        h0 = metrics_from_analysis(
            "H0 yaw=0",
            "zero_yaw",
            args.output_h0,
            args.instrumentation_h0,
            h0_analysis,
            h0_instr,
        )
        h2 = metrics_from_analysis(
            "H2 GNSS stable heading",
            "gnss_stable_heading",
            args.output_h2,
            args.instrumentation_h2,
            h2_analysis,
            h2_instr,
        )

        print_comparison(h0, h2)
        write_report(h0, h2)
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
