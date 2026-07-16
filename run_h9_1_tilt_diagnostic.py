#!/usr/bin/env python3
"""H9.1 — Diagnostico observacional de error de inclinacion (sin cambiar el EKF).

Registra los primeros 30 s: actitud EKF vs Orientation, gravedad en body/nav,
a_lin horizontal, y prueba si a_lin_h ~ g*sin(tilt_error) explica ~0.52 m/s^2.
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

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
DEFAULT_INPUT_DIR = REPO_ROOT / "data" / "real_run"
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DEFAULT_REPLAY = BENCH_DIR / "real_run_replay.csv"
TILT_CSV = BENCH_DIR / "h9_1_tilt_audit.csv"
MERGED_CSV = BENCH_DIR / "h9_1_tilt_merged.csv"
REPORT_JSON = BENCH_DIR / "h9_1_tilt_diagnostic_report.json"
ANALYSIS_PNG = BENCH_DIR / "h9_1_tilt_diagnostic_analysis.png"

STATIC_PHASE_END_S = 30.0
GRAVITY_MPS2 = 9.80665
TILT_MATCH_TOLERANCE_DEG = 1.0
ALIN_MATCH_TOLERANCE_MPS2 = 0.10

from analyze_real_run import (  # noqa: E402
    discover_t0_ns,
    estimate_mount_offset_deg,
    interpolate_series,
    load_orientation,
    resolve_orientation_path,
    resolve_replay_path,
    wrap_angle_deg,
)


@dataclass
class TiltSample:
    timestamp_s: float
    roll_ekf_deg: float
    pitch_ekf_deg: float
    yaw_ekf_deg: float
    g_body: tuple[float, float, float]
    g_nav: tuple[float, float, float]
    a_nav: tuple[float, float, float]
    a_lin: tuple[float, float, float]
    a_lin_h: float
    constraint_mode: int


def parse_float(value: str | None) -> float | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        out = float(text)
    except ValueError:
        return None
    if not math.isfinite(out):
        return None
    return out


def load_tilt_csv(path: Path) -> list[TiltSample]:
    rows: list[TiltSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t = parse_float(raw.get("timestamp_s"))
            if t is None:
                continue
            rows.append(
                TiltSample(
                    timestamp_s=t,
                    roll_ekf_deg=parse_float(raw.get("roll_ekf_deg")) or 0.0,
                    pitch_ekf_deg=parse_float(raw.get("pitch_ekf_deg")) or 0.0,
                    yaw_ekf_deg=parse_float(raw.get("yaw_ekf_deg")) or 0.0,
                    g_body=(
                        parse_float(raw.get("g_body_x")) or 0.0,
                        parse_float(raw.get("g_body_y")) or 0.0,
                        parse_float(raw.get("g_body_z")) or 0.0,
                    ),
                    g_nav=(
                        parse_float(raw.get("g_nav_n")) or 0.0,
                        parse_float(raw.get("g_nav_e")) or 0.0,
                        parse_float(raw.get("g_nav_d")) or GRAVITY_MPS2,
                    ),
                    a_nav=(
                        parse_float(raw.get("a_nav_n")) or 0.0,
                        parse_float(raw.get("a_nav_e")) or 0.0,
                        parse_float(raw.get("a_nav_d")) or 0.0,
                    ),
                    a_lin=(
                        parse_float(raw.get("a_lin_n")) or 0.0,
                        parse_float(raw.get("a_lin_e")) or 0.0,
                        parse_float(raw.get("a_lin_d")) or 0.0,
                    ),
                    a_lin_h=parse_float(raw.get("a_lin_h")) or 0.0,
                    constraint_mode=int(parse_float(raw.get("constraint_mode")) or 0),
                )
            )
    if not rows:
        raise ValueError(f"CSV vacio: {path}")
    rows.sort(key=lambda sample: sample.timestamp_s)
    return rows


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


def run_h9_replay(replay_csv: Path, replay_exe: Path, calibration: Path) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")

    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(BENCH_DIR / "h9_1_replay_output.csv"),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--h9-tilt-audit-csv",
        str(TILT_CSV),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def predicted_a_lin_h_from_tilt_deg(roll_err_deg: np.ndarray, pitch_err_deg: np.ndarray) -> np.ndarray:
    roll_rad = np.deg2rad(roll_err_deg)
    pitch_rad = np.deg2rad(pitch_err_deg)
    return GRAVITY_MPS2 * np.sqrt(
        np.sin(roll_rad) ** 2 + np.sin(pitch_rad) ** 2
    )


def predicted_a_lin_h_single_axis_deg(tilt_err_deg: np.ndarray) -> np.ndarray:
    return GRAVITY_MPS2 * np.abs(np.sin(np.deg2rad(tilt_err_deg)))


def analyze_tilt_diagnostic(
    samples: list[TiltSample],
    orientation_path: Path,
    input_dir: Path | None,
) -> dict:
    t0_ns = discover_t0_ns(input_dir)
    orientation = load_orientation(orientation_path, t0_ns)

    times = np.array([s.timestamp_s for s in samples], dtype=float)
    roll_ekf = np.array([s.roll_ekf_deg for s in samples], dtype=float)
    pitch_ekf = np.array([s.pitch_ekf_deg for s in samples], dtype=float)
    yaw_ekf = np.array([s.yaw_ekf_deg for s in samples], dtype=float)
    a_lin_h = np.array([s.a_lin_h for s in samples], dtype=float)

    orient_times = np.array([s.timestamp_s for s in orientation], dtype=float)
    orient_roll = np.array([s.roll_deg for s in orientation], dtype=float)
    orient_pitch = np.array([s.pitch_deg for s in orientation], dtype=float)
    orient_yaw = np.array([s.yaw_deg for s in orientation], dtype=float)

    roll_orient = interpolate_series(times, orient_times, orient_roll)
    pitch_orient = interpolate_series(times, orient_times, orient_pitch)
    yaw_orient = interpolate_series(times, orient_times, orient_yaw)

    roll_off, pitch_off, yaw_off = estimate_mount_offset_deg(
        roll_orient,
        pitch_orient,
        yaw_orient,
        roll_ekf,
        pitch_ekf,
        yaw_ekf,
        times,
        static_end_s=STATIC_PHASE_END_S,
    )

    roll_orient_aligned = roll_orient - roll_off
    pitch_orient_aligned = pitch_orient - pitch_off

    tilt_error_roll = wrap_angle_deg(roll_ekf - roll_orient_aligned)
    tilt_error_pitch = wrap_angle_deg(pitch_ekf - pitch_orient_aligned)

    tilt_mag_deg = np.sqrt(tilt_error_roll ** 2 + tilt_error_pitch ** 2)
    predicted_a_h = predicted_a_lin_h_from_tilt_deg(tilt_error_roll, tilt_error_pitch)
    predicted_a_h_roll = predicted_a_lin_h_single_axis_deg(tilt_error_roll)
    predicted_a_h_pitch = predicted_a_lin_h_single_axis_deg(tilt_error_pitch)

    implied_tilt_deg = np.rad2deg(np.arcsin(np.clip(a_lin_h / GRAVITY_MPS2, -1.0, 1.0)))

    static_mask = times <= STATIC_PHASE_END_S
    if not np.any(static_mask):
        static_mask = np.ones_like(times, dtype=bool)

    def stat(name: str, values: np.ndarray) -> dict[str, float]:
        subset = values[static_mask]
        return {
            f"{name}_mean": float(np.mean(subset)),
            f"{name}_median": float(np.median(subset)),
            f"{name}_std": float(np.std(subset)),
            f"{name}_p95": float(np.percentile(subset, 95)),
        }

    a_lin_h_stats = stat("a_lin_h", a_lin_h)
    tilt_mag_stats = stat("tilt_mag_deg", tilt_mag_deg)
    predicted_stats = stat("predicted_a_h", predicted_a_h)
    implied_tilt_stats = stat("implied_tilt_deg", implied_tilt_deg)
    roll_err_stats = stat("tilt_error_roll_deg", tilt_error_roll)
    pitch_err_stats = stat("tilt_error_pitch_deg", tilt_error_pitch)

    residual = a_lin_h - predicted_a_h
    residual_stats = stat("residual_a_h", residual)

    corr = float(np.corrcoef(a_lin_h[static_mask], predicted_a_h[static_mask])[0, 1])

    median_a_lin_h = a_lin_h_stats["a_lin_h_median"]
    mean_a_lin_h = a_lin_h_stats["a_lin_h_mean"]
    median_tilt_mag = tilt_mag_stats["tilt_mag_deg_median"]
    mean_tilt_mag = tilt_mag_stats["tilt_mag_deg_mean"]
    median_implied_tilt = implied_tilt_stats["implied_tilt_deg_median"]
    mean_implied_tilt = implied_tilt_stats["implied_tilt_deg_mean"]
    median_predicted = predicted_stats["predicted_a_h_median"]
    mean_predicted = predicted_stats["predicted_a_h_mean"]
    median_roll_err = roll_err_stats["tilt_error_roll_deg_median"]
    median_pitch_err = pitch_err_stats["tilt_error_pitch_deg_median"]
    rms_roll_err = float(np.sqrt(np.mean(tilt_error_roll[static_mask] ** 2)))
    rms_pitch_err = float(np.sqrt(np.mean(tilt_error_pitch[static_mask] ** 2)))
    rms_tilt_err = float(np.sqrt(np.mean(tilt_mag_deg[static_mask] ** 2)))

    g_sin_3deg = GRAVITY_MPS2 * math.sin(math.radians(3.0))

    magnitude_consistent_with_3deg = (
        abs(mean_a_lin_h - g_sin_3deg) <= ALIN_MATCH_TOLERANCE_MPS2
        and abs(mean_implied_tilt - 3.0) <= TILT_MATCH_TOLERANCE_DEG
    )
    orientation_explains_residual = (
        abs(mean_a_lin_h - mean_predicted) <= ALIN_MATCH_TOLERANCE_MPS2
        and corr >= 0.7
    )

    hypothesis_confirmed = magnitude_consistent_with_3deg and orientation_explains_residual

    merged_rows: list[dict[str, float | int]] = []
    for idx, sample in enumerate(samples):
        merged_rows.append(
            {
                "timestamp_s": sample.timestamp_s,
                "roll_ekf_deg": roll_ekf[idx],
                "pitch_ekf_deg": pitch_ekf[idx],
                "roll_orientation_deg": roll_orient[idx],
                "pitch_orientation_deg": pitch_orient[idx],
                "roll_orientation_aligned_deg": roll_orient_aligned[idx],
                "pitch_orientation_aligned_deg": pitch_orient_aligned[idx],
                "tilt_error_roll_deg": float(tilt_error_roll[idx]),
                "tilt_error_pitch_deg": float(tilt_error_pitch[idx]),
                "tilt_mag_deg": float(tilt_mag_deg[idx]),
                "predicted_a_lin_h_mps2": float(predicted_a_h[idx]),
                "a_lin_h_mps2": sample.a_lin_h,
                "implied_tilt_deg": float(implied_tilt_deg[idx]),
                "g_body_x": sample.g_body[0],
                "g_body_y": sample.g_body[1],
                "g_body_z": sample.g_body[2],
                "constraint_mode": sample.constraint_mode,
            }
        )

    with MERGED_CSV.open("w", newline="", encoding="utf-8") as handle:
        if merged_rows:
            writer = csv.DictWriter(handle, fieldnames=list(merged_rows[0].keys()))
            writer.writeheader()
            writer.writerows(merged_rows)

    report = {
        "experiment": "H9.1_tilt_diagnostic",
        "description": (
            "Diagnostico observacional: ¿a_lin_h ~ g*sin(tilt_error) explica ~0.52 m/s^2?"
        ),
        "static_phase_end_s": STATIC_PHASE_END_S,
        "gravity_mps2": GRAVITY_MPS2,
        "g_sin_3deg_mps2": g_sin_3deg,
        "samples": int(len(samples)),
        "mount_offset_deg": {
            "roll": roll_off,
            "pitch": pitch_off,
            "yaw": yaw_off,
        },
        "static_phase": {
            **a_lin_h_stats,
            **tilt_mag_stats,
            **predicted_stats,
            **implied_tilt_stats,
            **roll_err_stats,
            **pitch_err_stats,
            **residual_stats,
            "correlation_a_lin_h_vs_predicted": corr,
        },
        "hypothesis_test": {
            "question": "Es a_lin_h horizontal el resultado de ~3 deg de error de inclinacion?",
            "mean_a_lin_h_mps2": mean_a_lin_h,
            "median_a_lin_h_mps2": median_a_lin_h,
            "mean_predicted_a_h_mps2": mean_predicted,
            "median_predicted_a_h_mps2": median_predicted,
            "mean_tilt_error_mag_deg": mean_tilt_mag,
            "rms_tilt_error_deg": rms_tilt_err,
            "rms_tilt_error_roll_deg": rms_roll_err,
            "rms_tilt_error_pitch_deg": rms_pitch_err,
            "mean_implied_tilt_from_a_lin_deg": mean_implied_tilt,
            "median_implied_tilt_from_a_lin_deg": median_implied_tilt,
            "magnitude_consistent_with_3deg_tilt": magnitude_consistent_with_3deg,
            "orientation_explains_residual": orientation_explains_residual,
            "hypothesis_confirmed": hypothesis_confirmed,
        },
        "interpretation": (
            "Magnitud coherente con ~3 deg (g*sin(3) ~ a_lin_h), pero Orientation no predice "
            "el residual tras alinear montaje. El leak horizontal existe en predict aunque "
            "roll/pitch EKF coincidan con Orientation: revisar R_bn vs acelerometro, o H9 predict-only."
            if magnitude_consistent_with_3deg and not orientation_explains_residual
            else (
                "Hipotesis tilt confirmada: magnitud y diferencia EKF/Orientation coherentes. "
                "Proceder a H9.2 (init gravedad, yaw GNSS, montajes)."
                if hypothesis_confirmed
                else (
                    "El residual horizontal NO se explica por tilt ~3 deg ni por Orientation. "
                    "Priorizar consistencia propagacion/actualizaciones (ZUPT-NHC) o convencion de ejes."
                )
            )
        ),
        "recommended_next_experiments": [
            {
                "id": "H9_predict_only_isolation",
                "description": (
                    "60 s: solo IMU predict, sin GPS/NHC/ZUPT. "
                    "Si a_lin_h persiste, origen en propagacion; si desaparece, en actualizaciones."
                ),
            },
            {
                "id": "H9_zupt_nhc_transition",
                "description": (
                    "Medir si el residual en t=34.3 s (ZUPT-NHC) existia antes (ZUPT ocultaba) "
                    "o lo introduce el cambio de regimen."
                ),
            },
        ],
        "outputs": {
            "tilt_audit_csv": str(TILT_CSV),
            "merged_csv": str(MERGED_CSV),
            "plot_png": str(ANALYSIS_PNG),
        },
    }
    return report, times, roll_ekf, pitch_ekf, roll_orient_aligned, pitch_orient_aligned, (
        tilt_error_roll,
        tilt_error_pitch,
        a_lin_h,
        predicted_a_h,
    )


def plot_diagnostic(
    times: np.ndarray,
    roll_ekf: np.ndarray,
    pitch_ekf: np.ndarray,
    roll_orient_aligned: np.ndarray,
    pitch_orient_aligned: np.ndarray,
    tilt_error_roll: np.ndarray,
    tilt_error_pitch: np.ndarray,
    a_lin_h: np.ndarray,
    predicted_a_h: np.ndarray,
    report: dict,
) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
    fig.suptitle("H9.1 - Diagnostico de inclinacion (primeros 30 s)", fontsize=14)

    axes[0].plot(times, roll_ekf, label="Roll EKF", linewidth=1.2)
    axes[0].plot(times, roll_orient_aligned, label="Roll Orientation (alineado)", linewidth=1.0, alpha=0.8)
    axes[0].plot(times, tilt_error_roll, label="Roll Difference", linewidth=0.9, alpha=0.7)
    axes[0].set_ylabel("Roll [deg]")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, pitch_ekf, label="Pitch EKF", linewidth=1.2)
    axes[1].plot(times, pitch_orient_aligned, label="Pitch Orientation (alineado)", linewidth=1.0, alpha=0.8)
    axes[1].plot(times, tilt_error_pitch, label="Pitch Difference", linewidth=0.9, alpha=0.7)
    axes[1].set_ylabel("Pitch [deg]")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times, a_lin_h, label="a_lin_h medido", linewidth=1.2)
    axes[2].plot(times, predicted_a_h, label="g*sin(tilt) predicho", linewidth=1.0, alpha=0.85)
    axes[2].axhline(
        GRAVITY_MPS2 * math.sin(math.radians(3.0)),
        color="red",
        linestyle="--",
        linewidth=0.9,
        label="g*sin(3 deg) ref",
    )
    axes[2].set_ylabel("a_lin_h [m/s2]")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(True, alpha=0.3)

    hypothesis = report["hypothesis_test"]
    summary = (
        f"mean a_lin_h={hypothesis['mean_a_lin_h_mps2']:.3f} m/s2 | "
        f"implied tilt={hypothesis['mean_implied_tilt_from_a_lin_deg']:.2f} deg | "
        f"mag~3deg={hypothesis['magnitude_consistent_with_3deg_tilt']} | "
        f"orient_ok={hypothesis['orientation_explains_residual']}"
    )
    axes[3].plot(times, tilt_error_roll, label="tilt roll", alpha=0.7)
    axes[3].plot(times, tilt_error_pitch, label="tilt pitch", alpha=0.7)
    axes[3].plot(
        times,
        np.sqrt(tilt_error_roll ** 2 + tilt_error_pitch ** 2),
        label="|tilt| combined",
        linewidth=1.2,
    )
    axes[3].set_ylabel("Tilt error [deg]")
    axes[3].set_xlabel("Tiempo [s]")
    axes[3].legend(loc="upper right", fontsize=8)
    axes[3].grid(True, alpha=0.3)
    axes[3].text(0.02, 0.95, summary, transform=axes[3].transAxes, fontsize=9, va="top")

    fig.tight_layout()
    fig.savefig(ANALYSIS_PNG, dpi=150)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="H9.1 — Diagnostico observacional de tilt")
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--orientation", type=Path, default=None)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()

    replay_csv = resolve_replay_path(args.replay_csv)
    orientation_path = resolve_orientation_path(args.orientation)
    if orientation_path is None:
        print("ERROR: no se encontro Orientation.csv", file=sys.stderr)
        return 1

    BENCH_DIR.mkdir(parents=True, exist_ok=True)

    if not args.skip_replay:
        ensure_calibration(args.calibration)
        run_h9_replay(replay_csv, args.replay_exe, args.calibration)

    if not TILT_CSV.is_file():
        print(f"ERROR: falta {TILT_CSV}", file=sys.stderr)
        return 1

    samples = load_tilt_csv(TILT_CSV)
    report, times, roll_ekf, pitch_ekf, roll_orient_aligned, pitch_orient_aligned, series = (
        analyze_tilt_diagnostic(samples, orientation_path, args.input_dir)
    )
    tilt_error_roll, tilt_error_pitch, a_lin_h, predicted_a_h = series

    plot_diagnostic(
        times,
        roll_ekf,
        pitch_ekf,
        roll_orient_aligned,
        pitch_orient_aligned,
        tilt_error_roll,
        tilt_error_pitch,
        a_lin_h,
        predicted_a_h,
        report,
    )

    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)

    ht = report["hypothesis_test"]
    print("=" * 64)
    print("H9.1 - Diagnostico de inclinacion")
    print("=" * 64)
    print(f"  Muestras (0-{STATIC_PHASE_END_S} s): {report['samples']}")
    print(f"  Offset montaje [deg]: roll={report['mount_offset_deg']['roll']:.2f}, "
          f"pitch={report['mount_offset_deg']['pitch']:.2f}")
    print(f"  a_lin_h medio:       {ht['mean_a_lin_h_mps2']:.4f} m/s2")
    print(f"  a_lin_h mediano:     {ht['median_a_lin_h_mps2']:.4f} m/s2")
    print(f"  g*sin(3 deg) ref:    {report['g_sin_3deg_mps2']:.4f} m/s2")
    print(f"  Tilt implied (mean): {ht['mean_implied_tilt_from_a_lin_deg']:.2f} deg")
    print(f"  Tilt err RMS:        {ht['rms_tilt_error_deg']:.2f} deg "
          f"(roll={ht['rms_tilt_error_roll_deg']:.2f}, "
          f"pitch={ht['rms_tilt_error_pitch_deg']:.2f})")
    print(f"  Predicho g*sin(tilt): mean={ht['mean_predicted_a_h_mps2']:.4f} m/s2")
    print(f"  Correlacion:         {report['static_phase']['correlation_a_lin_h_vs_predicted']:.3f}")
    print(f"  Magnitud ~3 deg:     {ht['magnitude_consistent_with_3deg_tilt']}")
    print(f"  Orientation explica: {ht['orientation_explains_residual']}")
    print(f"  Hipotesis completa:  {ht['hypothesis_confirmed']}")
    print(f"  -> {report['interpretation']}")
    print(f"  Informe:  {REPORT_JSON}")
    print(f"  Grafica:  {ANALYSIS_PNG}")
    print(f"  Merged:   {MERGED_CSV}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
