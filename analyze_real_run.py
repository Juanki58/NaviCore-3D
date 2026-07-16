#!/usr/bin/env python3
"""Análisis y visualización del replay de trayectoria real (NaviCore3D_Replay).

Lee la entrada parseada, la salida del EKF y Orientation.csv (Sensor Logger),
calcula métricas de posición/actitud/integridad y genera un informe con gráficos.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY = REPO_ROOT / "docs" / "benchmarks" / "real_run_replay.csv"
DEFAULT_OUTPUT = REPO_ROOT / "docs" / "benchmarks" / "real_run_output.csv"
DEFAULT_BASELINE_OUTPUT = REPO_ROOT / "docs" / "benchmarks" / "real_run_output_base.csv"
DEFAULT_PLOT = REPO_ROOT / "docs" / "benchmarks" / "real_run_analysis.png"
ORIENTATION_NAME = "Orientation.csv"

SEARCH_DIRS = (
    REPO_ROOT / "data" / "real_run",
    REPO_ROOT,
    REPO_ROOT / "docs",
)

NIS_THRESHOLD = 11.345  # Chi-cuadrado 3 DoF, p=0.99 (ins_ekf.hpp)
STATIC_PHASE_END_S = 30.0
MOVING_NIS_START_S = 30.0

DEG_PER_RAD = 180.0 / math.pi


@dataclass(frozen=True)
class GpsSample:
    timestamp_s: float
    pos_n: float
    pos_e: float
    pos_d: float


@dataclass(frozen=True)
class FilterSample:
    timestamp_s: float
    pos_n: float
    pos_e: float
    pos_d: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    nis: float
    row_type: str
    accel_bias: tuple[float, float, float]
    gyro_bias: tuple[float, float, float]
    cov_pos: tuple[float, float, float]
    cov_att: tuple[float, float, float]


@dataclass(frozen=True)
class OrientationSample:
    timestamp_s: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass
class AnalysisResult:
    gps_times: np.ndarray
    gps_n: np.ndarray
    gps_e: np.ndarray
    filt_n_at_gps: np.ndarray
    filt_e_at_gps: np.ndarray
    horizontal_error_m: np.ndarray
    horizontal_rmse_m: float
    position_rmse_m: float
    nis_moving_mean: float
    nis_moving_std: float
    nis_moving_count: int
    mount_offset_deg: tuple[float, float, float]
    att_rmse_deg: tuple[float, float, float] | None
    att_times: np.ndarray
    filt_roll: np.ndarray
    filt_pitch: np.ndarray
    filt_yaw: np.ndarray
    ref_roll: np.ndarray | None
    ref_pitch: np.ndarray | None
    ref_yaw: np.ndarray | None
    output_times: np.ndarray
    output_n: np.ndarray
    output_e: np.ndarray
    baseline_n: np.ndarray | None
    baseline_e: np.ndarray | None
    nis_times: np.ndarray
    nis_values: np.ndarray
    final_state: FilterSample | None


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


def discover_file(name: str, explicit: Path | None = None) -> Path | None:
    if explicit is not None:
        return explicit if explicit.is_file() else None
    for directory in SEARCH_DIRS:
        candidate = directory / name
        if candidate.is_file():
            return candidate
    return None


def discover_input_dir() -> Path | None:
    for candidate in SEARCH_DIRS:
        if not candidate.is_dir():
            continue
        if all((candidate / fname).is_file() for fname in (
            "AccelerometerUncalibrated.csv",
            "Gyroscope.csv",
            "Location.csv",
        )):
            return candidate
    return None


def discover_t0_ns(input_dir: Path | None) -> int | None:
    if input_dir is None:
        return None
    try:
        from parse_mobile_log import (
            ACCEL_FILE,
            GYRO_FILE,
            LOCATION_FILE,
            load_location_csv,
            load_vec3_csv,
        )
    except ImportError:
        return None

    try:
        accel = load_vec3_csv(input_dir / ACCEL_FILE)
        gyro = load_vec3_csv(input_dir / GYRO_FILE)
        locations = load_location_csv(input_dir / LOCATION_FILE)
    except (FileNotFoundError, ValueError):
        return None

    return min(accel[0].time_ns, gyro[0].time_ns, locations[0].time_ns)


def load_replay_gps(path: Path) -> list[GpsSample]:
    samples: list[GpsSample] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV sin cabecera: {path}")

        for row in reader:
            if (row.get("type") or "").strip().upper() != "GPS":
                continue
            timestamp_s = parse_float(row.get("timestamp_s"))
            pos_n = parse_float(row.get("pos_n"))
            pos_e = parse_float(row.get("pos_e"))
            pos_d = parse_float(row.get("pos_d"))
            if None in (timestamp_s, pos_n, pos_e, pos_d):
                continue
            samples.append(
                GpsSample(
                    timestamp_s=timestamp_s,
                    pos_n=pos_n,
                    pos_e=pos_e,
                    pos_d=pos_d,
                )
            )

    if not samples:
        raise ValueError(f"No hay filas GPS en {path}")
    samples.sort(key=lambda sample: sample.timestamp_s)
    return samples


def build_replay_from_mobile(output_path: Path) -> Path:
    from parse_mobile_log import (
        ACCEL_FILE,
        GYRO_FILE,
        LOCATION_FILE,
        build_gnss_rows,
        build_imu_rows,
        discover_input_dir as mobile_discover,
        load_location_csv,
        load_vec3_csv,
        normalize_timestamps,
        write_replay_csv,
    )

    input_dir = mobile_discover(None)
    if input_dir is None:
        raise FileNotFoundError(
            "No existe docs/benchmarks/real_run_replay.csv y no se encontraron "
            "los CSV originales de Sensor Logger (AccelerometerUncalibrated, "
            "Gyroscope, Location)."
        )

    accel = load_vec3_csv(input_dir / ACCEL_FILE)
    gyro = load_vec3_csv(input_dir / GYRO_FILE)
    locations = load_location_csv(input_dir / LOCATION_FILE)
    ref = locations[0]

    imu_rows = build_imu_rows(accel, gyro)
    gnss_rows = build_gnss_rows(
        locations,
        ref.latitude,
        ref.longitude,
        ref.altitude,
    )
    t0_ns = min(accel[0].time_ns, gyro[0].time_ns, locations[0].time_ns)
    rows = normalize_timestamps(imu_rows + gnss_rows, t0_ns)
    write_replay_csv(output_path, rows)
    print(f"Generado replay temporal: {output_path}")
    return output_path


def resolve_replay_path(explicit: Path | None) -> Path:
    if explicit is not None:
        if not explicit.is_file():
            raise FileNotFoundError(f"Replay no encontrado: {explicit}")
        return explicit

    if DEFAULT_REPLAY.is_file():
        return DEFAULT_REPLAY

    DEFAULT_REPLAY.parent.mkdir(parents=True, exist_ok=True)
    return build_replay_from_mobile(DEFAULT_REPLAY)


def load_filter_output(path: Path) -> list[FilterSample]:
    samples: list[FilterSample] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV sin cabecera: {path}")

        for row in reader:
            timestamp_s = parse_float(row.get("timestamp_s"))
            pos_n = parse_float(row.get("pos_n_m"))
            pos_e = parse_float(row.get("pos_e_m"))
            pos_d = parse_float(row.get("pos_d_m"))
            roll_deg = parse_float(row.get("roll_deg"))
            pitch_deg = parse_float(row.get("pitch_deg"))
            yaw_deg = parse_float(row.get("yaw_deg"))
            nis = parse_float(row.get("nis")) or 0.0
            row_type = (row.get("row_type") or "").strip().upper()
            if None in (timestamp_s, pos_n, pos_e, pos_d, roll_deg, pitch_deg, yaw_deg):
                continue

            accel_bias = (
                parse_float(row.get("accel_bias_x")) or 0.0,
                parse_float(row.get("accel_bias_y")) or 0.0,
                parse_float(row.get("accel_bias_z")) or 0.0,
            )
            gyro_bias = (
                parse_float(row.get("gyro_bias_x")) or 0.0,
                parse_float(row.get("gyro_bias_y")) or 0.0,
                parse_float(row.get("gyro_bias_z")) or 0.0,
            )
            cov_pos = (
                parse_float(row.get("cov_pos_n")) or 0.0,
                parse_float(row.get("cov_pos_e")) or 0.0,
                parse_float(row.get("cov_pos_d")) or 0.0,
            )
            cov_att = (
                parse_float(row.get("cov_att_roll")) or 0.0,
                parse_float(row.get("cov_att_pitch")) or 0.0,
                parse_float(row.get("cov_att_yaw")) or 0.0,
            )
            samples.append(
                FilterSample(
                    timestamp_s=timestamp_s,
                    pos_n=pos_n,
                    pos_e=pos_e,
                    pos_d=pos_d,
                    roll_deg=roll_deg,
                    pitch_deg=pitch_deg,
                    yaw_deg=yaw_deg,
                    nis=nis,
                    row_type=row_type,
                    accel_bias=accel_bias,
                    gyro_bias=gyro_bias,
                    cov_pos=cov_pos,
                    cov_att=cov_att,
                )
            )

    if not samples:
        raise ValueError(f"No hay muestras validas en {path}")
    samples.sort(key=lambda sample: sample.timestamp_s)
    return samples


def euler_columns(fieldnames: Sequence[str]) -> tuple[str, str, str] | None:
    lower = {name.lower(): name for name in fieldnames}
    if all(key in lower for key in ("roll", "pitch", "yaw")):
        return lower["roll"], lower["pitch"], lower["yaw"]
    return None


def orientation_to_deg(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        return values
    if np.nanmax(np.abs(values)) <= (2.0 * math.pi + 0.5):
        return np.rad2deg(values)
    return values


def load_orientation(path: Path, t0_ns: int | None) -> list[OrientationSample]:
    samples: list[OrientationSample] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV sin cabecera: {path}")

        euler_cols = euler_columns(reader.fieldnames)
        if euler_cols is None:
            raise ValueError(
                f"{path.name}: se requieren columnas roll, pitch, yaw "
                f"(encontradas: {reader.fieldnames})"
            )
        roll_col, pitch_col, yaw_col = euler_cols
        lower = {name.lower(): name for name in reader.fieldnames}
        time_col = lower.get("time")
        seconds_col = lower.get("seconds_elapsed")

        for row in reader:
            roll = parse_float(row.get(roll_col))
            pitch = parse_float(row.get(pitch_col))
            yaw = parse_float(row.get(yaw_col))
            if None in (roll, pitch, yaw):
                continue

            timestamp_s: float | None = None
            if time_col is not None:
                time_ns = parse_float(row.get(time_col))
                if time_ns is not None:
                    if t0_ns is not None:
                        timestamp_s = (time_ns - t0_ns) * 1e-9
                    else:
                        timestamp_s = time_ns * 1e-9
            if timestamp_s is None and seconds_col is not None:
                timestamp_s = parse_float(row.get(seconds_col))

            if timestamp_s is None:
                continue

            roll_deg, pitch_deg, yaw_deg = orientation_to_deg(
                np.array([roll, pitch, yaw], dtype=float)
            )
            samples.append(
                OrientationSample(
                    timestamp_s=timestamp_s,
                    roll_deg=float(roll_deg),
                    pitch_deg=float(pitch_deg),
                    yaw_deg=float(yaw_deg),
                )
            )

    if not samples:
        raise ValueError(f"No hay orientacion valida en {path}")
    samples.sort(key=lambda sample: sample.timestamp_s)
    return samples


def resolve_orientation_path(explicit: Path | None) -> Path | None:
    if explicit is not None:
        return explicit if explicit.is_file() else None
    return discover_file(ORIENTATION_NAME)


def wrap_angle_deg(angle: float | np.ndarray) -> float | np.ndarray:
    return (angle + 180.0) % 360.0 - 180.0


def circular_mean_deg(values: np.ndarray) -> float:
    radians = np.deg2rad(values)
    return float(np.rad2deg(np.arctan2(np.mean(np.sin(radians)), np.mean(np.cos(radians)))))


def interpolate_series(
    query_times: np.ndarray,
    source_times: np.ndarray,
    source_values: np.ndarray,
) -> np.ndarray:
    if source_times.size == 0:
        return np.full(query_times.shape, np.nan)
    if source_times.size == 1:
        return np.full(query_times.shape, source_values[0])
    return np.interp(query_times, source_times, source_values)


def estimate_mount_offset_deg(
    ref_roll: np.ndarray,
    ref_pitch: np.ndarray,
    ref_yaw: np.ndarray,
    est_roll: np.ndarray,
    est_pitch: np.ndarray,
    est_yaw: np.ndarray,
    times: np.ndarray,
    static_end_s: float = STATIC_PHASE_END_S,
) -> tuple[float, float, float]:
    mask = times <= static_end_s
    if not np.any(mask):
        mask = np.ones_like(times, dtype=bool)

    roll_off = float(np.median(wrap_angle_deg(ref_roll[mask] - est_roll[mask])))
    pitch_off = float(np.median(wrap_angle_deg(ref_pitch[mask] - est_pitch[mask])))
    yaw_off = circular_mean_deg(wrap_angle_deg(ref_yaw[mask] - est_yaw[mask]))
    return roll_off, pitch_off, yaw_off


def compute_attitude_rmse(
    ref: np.ndarray,
    est: np.ndarray,
) -> float:
    diff = wrap_angle_deg(ref - est)
    return float(np.sqrt(np.mean(diff * diff)))


def analyze(
    replay_path: Path,
    output_path: Path,
    orientation_path: Path | None,
    input_dir: Path | None,
) -> AnalysisResult:
    gps_samples = load_replay_gps(replay_path)
    filter_samples = load_filter_output(output_path)
    t0_ns = discover_t0_ns(input_dir)

    gps_times = np.array([sample.timestamp_s for sample in gps_samples], dtype=float)
    gps_n = np.array([sample.pos_n for sample in gps_samples], dtype=float)
    gps_e = np.array([sample.pos_e for sample in gps_samples], dtype=float)
    gps_d = np.array([sample.pos_d for sample in gps_samples], dtype=float)

    out_times = np.array([sample.timestamp_s for sample in filter_samples], dtype=float)
    out_n = np.array([sample.pos_n for sample in filter_samples], dtype=float)
    out_e = np.array([sample.pos_e for sample in filter_samples], dtype=float)
    out_d = np.array([sample.pos_d for sample in filter_samples], dtype=float)
    out_roll = np.array([sample.roll_deg for sample in filter_samples], dtype=float)
    out_pitch = np.array([sample.pitch_deg for sample in filter_samples], dtype=float)
    out_yaw = np.array([sample.yaw_deg for sample in filter_samples], dtype=float)

    filt_n_at_gps = interpolate_series(gps_times, out_times, out_n)
    filt_e_at_gps = interpolate_series(gps_times, out_times, out_e)
    filt_d_at_gps = interpolate_series(gps_times, out_times, out_d)

    horizontal_error = np.hypot(filt_n_at_gps - gps_n, filt_e_at_gps - gps_e)
    position_error = np.sqrt(
        (filt_n_at_gps - gps_n) ** 2
        + (filt_e_at_gps - gps_e) ** 2
        + (filt_d_at_gps - gps_d) ** 2
    )
    horizontal_rmse = float(np.sqrt(np.mean(horizontal_error ** 2)))
    position_rmse = float(np.sqrt(np.mean(position_error ** 2)))

    gps_rows = [
        sample for sample in filter_samples
        if sample.row_type == "GPS" and sample.timestamp_s > MOVING_NIS_START_S
    ]
    nis_values_moving = np.array([sample.nis for sample in gps_rows], dtype=float)
    if nis_values_moving.size == 0:
        nis_moving_mean = float("nan")
        nis_moving_std = float("nan")
        nis_moving_count = 0
    else:
        nis_moving_mean = float(np.mean(nis_values_moving))
        nis_moving_std = float(np.std(nis_values_moving))
        nis_moving_count = int(nis_values_moving.size)

    nis_gps = [sample for sample in filter_samples if sample.row_type == "GPS"]
    nis_times = np.array([sample.timestamp_s for sample in nis_gps], dtype=float)
    nis_values = np.array([sample.nis for sample in nis_gps], dtype=float)

    att_times = out_times
    ref_roll = ref_pitch = ref_yaw = None
    mount_offset = (0.0, 0.0, 0.0)
    att_rmse: tuple[float, float, float] | None = None

    if orientation_path is not None:
        orientation = load_orientation(orientation_path, t0_ns)
        ori_times = np.array([sample.timestamp_s for sample in orientation], dtype=float)
        ori_roll = np.array([sample.roll_deg for sample in orientation], dtype=float)
        ori_pitch = np.array([sample.pitch_deg for sample in orientation], dtype=float)
        ori_yaw = np.array([sample.yaw_deg for sample in orientation], dtype=float)

        ref_roll = interpolate_series(att_times, ori_times, ori_roll)
        ref_pitch = interpolate_series(att_times, ori_times, ori_pitch)
        ref_yaw = interpolate_series(att_times, ori_times, ori_yaw)

        mount_offset = estimate_mount_offset_deg(
            ref_roll,
            ref_pitch,
            ref_yaw,
            out_roll,
            out_pitch,
            out_yaw,
            att_times,
        )
        comp_roll = out_roll + mount_offset[0]
        comp_pitch = out_pitch + mount_offset[1]
        comp_yaw = wrap_angle_deg(out_yaw + mount_offset[2])

        att_rmse = (
            compute_attitude_rmse(ref_roll, comp_roll),
            compute_attitude_rmse(ref_pitch, comp_pitch),
            compute_attitude_rmse(ref_yaw, comp_yaw),
        )
        out_roll, out_pitch, out_yaw = comp_roll, comp_pitch, comp_yaw

    baseline_n: np.ndarray | None = None
    baseline_e: np.ndarray | None = None
    if DEFAULT_BASELINE_OUTPUT.is_file():
        baseline_samples = load_filter_output(DEFAULT_BASELINE_OUTPUT)
        baseline_n = np.array([sample.pos_n for sample in baseline_samples], dtype=float)
        baseline_e = np.array([sample.pos_e for sample in baseline_samples], dtype=float)

    return AnalysisResult(
        gps_times=gps_times,
        gps_n=gps_n,
        gps_e=gps_e,
        filt_n_at_gps=filt_n_at_gps,
        filt_e_at_gps=filt_e_at_gps,
        horizontal_error_m=horizontal_error,
        horizontal_rmse_m=horizontal_rmse,
        position_rmse_m=position_rmse,
        nis_moving_mean=nis_moving_mean,
        nis_moving_std=nis_moving_std,
        nis_moving_count=nis_moving_count,
        mount_offset_deg=mount_offset,
        att_rmse_deg=att_rmse,
        att_times=att_times,
        filt_roll=out_roll,
        filt_pitch=out_pitch,
        filt_yaw=out_yaw,
        ref_roll=ref_roll,
        ref_pitch=ref_pitch,
        ref_yaw=ref_yaw,
        output_times=out_times,
        output_n=out_n,
        output_e=out_e,
        baseline_n=baseline_n,
        baseline_e=baseline_e,
        nis_times=nis_times,
        nis_values=nis_values,
        final_state=filter_samples[-1],
    )


def plot_analysis(result: AnalysisResult, plot_path: Path) -> None:
    plot_path.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(3, 1, figsize=(11, 12), sharex=False)
    fig.suptitle("Análisis real run — NaviCore3D Replay", fontsize=13, fontweight="bold")

    ax_traj = axes[0]
    ax_traj.plot(result.gps_e, result.gps_n, color="#2ecc71", linewidth=1.8, label="GPS referencia")
    ax_traj.plot(result.output_e, result.output_n, color="#3498db", linewidth=1.2, alpha=0.85, label="EKF estimado")
    if result.baseline_n is not None and result.baseline_e is not None:
        ax_traj.plot(
            result.baseline_e,
            result.baseline_n,
            color="#95a5a6",
            linewidth=1.2,
            linestyle="--",
            alpha=0.85,
            label="EKF (Corrida Anterior)",
        )
    ax_traj.set_xlabel("Este (m)")
    ax_traj.set_ylabel("Norte (m)")
    ax_traj.set_title("Trayectoria horizontal (NED)")
    ax_traj.set_aspect("equal", adjustable="datalim")
    ax_traj.grid(True, alpha=0.3)
    ax_traj.legend(loc="best")

    ax_att = axes[1]
    ax_att.plot(result.att_times, result.filt_roll, color="#e74c3c", linewidth=1.0, label="Roll EKF")
    ax_att.plot(result.att_times, result.filt_pitch, color="#9b59b6", linewidth=1.0, label="Pitch EKF")
    ax_att.plot(result.att_times, result.filt_yaw, color="#f39c12", linewidth=1.0, label="Yaw EKF")
    if result.ref_roll is not None:
        ax_att.plot(result.att_times, result.ref_roll, color="#e74c3c", linewidth=0.9, linestyle="--", alpha=0.7, label="Roll ref.")
        ax_att.plot(result.att_times, result.ref_pitch, color="#9b59b6", linewidth=0.9, linestyle="--", alpha=0.7, label="Pitch ref.")
        ax_att.plot(result.att_times, result.ref_yaw, color="#f39c12", linewidth=0.9, linestyle="--", alpha=0.7, label="Yaw ref.")
    ax_att.set_xlabel("Tiempo (s)")
    ax_att.set_ylabel("Actitud (deg)")
    ax_att.set_title("Evolución de actitud (montaje compensado en EKF)")
    ax_att.grid(True, alpha=0.3)
    ax_att.legend(loc="best", ncol=2, fontsize=8)

    ax_nis = axes[2]
    if result.nis_times.size > 0:
        ax_nis.plot(result.nis_times, result.nis_values, color="#c0392b", linewidth=1.0, label="NIS GNSS")
    ax_nis.axhline(
        NIS_THRESHOLD,
        color="#7f8c8d",
        linestyle="--",
        linewidth=1.2,
        label=f"Umbral χ² (3 DoF) = {NIS_THRESHOLD:.3f}",
    )
    ax_nis.set_xlabel("Tiempo (s)")
    ax_nis.set_ylabel("NIS")
    ax_nis.set_title("Integridad — Normalized Innovation Squared")
    ax_nis.grid(True, alpha=0.3)
    ax_nis.legend(loc="best")

    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def print_report(
    result: AnalysisResult,
    replay_path: Path,
    output_path: Path,
    orientation_path: Path | None,
    plot_path: Path,
) -> None:
    final = result.final_state
    duration_s = float(result.output_times[-1]) if result.output_times.size else 0.0
    max_horizontal = float(np.max(result.horizontal_error_m)) if result.horizontal_error_m.size else 0.0
    final_horizontal = float(result.horizontal_error_m[-1]) if result.horizontal_error_m.size else 0.0

    print("=" * 64)
    print(" REAL RUN — Informe de análisis")
    print("=" * 64)
    print(f"  Entrada replay:   {replay_path}")
    print(f"  Salida EKF:       {output_path}")
    if DEFAULT_BASELINE_OUTPUT.is_file():
        print(f"  Baseline EKF:     {DEFAULT_BASELINE_OUTPUT}")
    print(f"  Orientación:      {orientation_path or '(no disponible)'}")
    print(f"  Gráfico:          {plot_path}")
    print(f"  Duración:         {duration_s:.2f} s")
    print("-" * 64)
    print(" Posición (sincronizada en fixes GNSS)")
    print(f"  RMSE horizontal:  {result.horizontal_rmse_m:.3f} m")
    print(f"  RMSE 3D:          {result.position_rmse_m:.3f} m")
    print(f"  Error H máximo:   {max_horizontal:.3f} m")
    print(f"  Error H final:    {final_horizontal:.3f} m")
    print("-" * 64)
    print(f" NIS en marcha (t > {MOVING_NIS_START_S:.0f} s, filas GPS)")
    if result.nis_moving_count > 0:
        print(f"  Muestras:         {result.nis_moving_count}")
        print(f"  Media:            {result.nis_moving_mean:.4f}")
        print(f"  Desv. estándar:   {result.nis_moving_std:.4f}")
    else:
        print("  (sin actualizaciones GNSS tras la fase estática)")
    print("-" * 64)
    if result.att_rmse_deg is not None:
        print(" Actitud vs Orientation.csv (offset de montaje compensado)")
        print(
            f"  Offset Roll/Pitch/Yaw: "
            f"{result.mount_offset_deg[0]:+.2f} / "
            f"{result.mount_offset_deg[1]:+.2f} / "
            f"{result.mount_offset_deg[2]:+.2f} deg"
        )
        print(
            f"  RMSE Roll/Pitch/Yaw: "
            f"{result.att_rmse_deg[0]:.3f} / "
            f"{result.att_rmse_deg[1]:.3f} / "
            f"{result.att_rmse_deg[2]:.3f} deg"
        )
    else:
        print(" Actitud: Orientation.csv no encontrado — comparación omitida")
    print("-" * 64)
    if final is not None:
        print(" Estado final del filtro")
        print(
            f"  Posición NED:     ({final.pos_n:.3f}, {final.pos_e:.3f}, {final.pos_d:.3f}) m"
        )
        print(
            f"  Actitud R/P/Y:    ({final.roll_deg:.2f}, {final.pitch_deg:.2f}, {final.yaw_deg:.2f}) deg"
        )
        print(
            f"  Bias accel:       ({final.accel_bias[0]:+.4f}, {final.accel_bias[1]:+.4f}, "
            f"{final.accel_bias[2]:+.4f}) m/s²"
        )
        print(
            f"  Bias gyro:        ({final.gyro_bias[0]:+.6f}, {final.gyro_bias[1]:+.6f}, "
            f"{final.gyro_bias[2]:+.6f}) rad/s"
        )
        print(
            f"  Cov pos N/E/D:    ({final.cov_pos[0]:.3f}, {final.cov_pos[1]:.3f}, {final.cov_pos[2]:.3f})"
        )
        print(
            f"  Cov att R/P/Y:    ({final.cov_att[0]:.4f}, {final.cov_att[1]:.4f}, {final.cov_att[2]:.4f})"
        )
        print(f"  Último NIS:       {final.nis:.4f} ({final.row_type})")
    print("=" * 64)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analiza y visualiza el replay de trayectoria real NaviCore3D."
    )
    parser.add_argument(
        "--replay",
        type=Path,
        default=None,
        help=f"CSV de entrada parseado (por defecto: {DEFAULT_REPLAY})",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"CSV de salida del EKF (por defecto: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--orientation",
        type=Path,
        default=None,
        help=f"Orientation.csv de Sensor Logger (busca en {', '.join(str(p) for p in SEARCH_DIRS)})",
    )
    parser.add_argument(
        "--plot",
        type=Path,
        default=DEFAULT_PLOT,
        help=f"Imagen de salida (por defecto: {DEFAULT_PLOT})",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    try:
        replay_path = resolve_replay_path(args.replay)
        if not args.output.is_file():
            raise FileNotFoundError(
                f"No se encontró la salida del EKF: {args.output}\n"
                "Ejecuta primero NaviCore3D_Replay para generar real_run_output.csv."
            )

        orientation_path = resolve_orientation_path(args.orientation)
        input_dir = discover_input_dir()
        result = analyze(replay_path, args.output, orientation_path, input_dir)
        plot_analysis(result, args.plot)
        print_report(result, replay_path, args.output, orientation_path, args.plot)
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
