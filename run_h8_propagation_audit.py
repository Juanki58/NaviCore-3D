#!/usr/bin/env python3
"""H8 - Auditoria de la cadena de propagacion inercial (magnitudes fisicas internas).

Registra y analiza: a_sensor -> R_mount -> a_body -> R_bn -> a_nav -> a_lin -> vel -> pos.
Compara con expectativas fisicas en reposo y en crucero (velocidad ~constante).
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
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DEFAULT_REPLAY = BENCH_DIR / "real_run_replay.csv"
PROPAGATION_CSV = BENCH_DIR / "h8_propagation_audit.csv"
REPORT_JSON = BENCH_DIR / "h8_propagation_audit_report.json"
ANALYSIS_PNG = BENCH_DIR / "h8_propagation_audit_analysis.png"

STATIC_PHASE_END_S = 30.0
CRUISE_MIN_SPEED_MPS = 5.0
CRUISE_MAX_SPEED_STD_MPS = 1.5
RESIDUAL_ALERT_MPS2 = 0.05
SUSTAINED_WINDOW_S = 5.0

from analyze_real_run import resolve_replay_path  # noqa: E402


@dataclass
class PropagationSample:
    timestamp_s: float
    dt_s: float
    a_sens: tuple[float, float, float]
    a_body: tuple[float, float, float]
    a_corr: tuple[float, float, float]
    a_nav: tuple[float, float, float]
    a_lin: tuple[float, float, float]
    vel_pre: tuple[float, float, float]
    vel_post: tuple[float, float, float]
    pos: tuple[float, float, float]
    yaw_deg: float
    gps_speed_mps: float
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


def load_propagation_csv(path: Path) -> list[PropagationSample]:
    rows: list[PropagationSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t = parse_float(raw.get("timestamp_s"))
            if t is None:
                continue
            rows.append(
                PropagationSample(
                    timestamp_s=t,
                    dt_s=parse_float(raw.get("dt_s")) or 0.0,
                    a_sens=(
                        parse_float(raw.get("a_sens_x")) or 0.0,
                        parse_float(raw.get("a_sens_y")) or 0.0,
                        parse_float(raw.get("a_sens_z")) or 0.0,
                    ),
                    a_body=(
                        parse_float(raw.get("a_body_x")) or 0.0,
                        parse_float(raw.get("a_body_y")) or 0.0,
                        parse_float(raw.get("a_body_z")) or 0.0,
                    ),
                    a_corr=(
                        parse_float(raw.get("a_corr_x")) or 0.0,
                        parse_float(raw.get("a_corr_y")) or 0.0,
                        parse_float(raw.get("a_corr_z")) or 0.0,
                    ),
                    a_nav=(
                        parse_float(raw.get("a_nav_x")) or 0.0,
                        parse_float(raw.get("a_nav_y")) or 0.0,
                        parse_float(raw.get("a_nav_z")) or 0.0,
                    ),
                    a_lin=(
                        parse_float(raw.get("a_lin_x")) or 0.0,
                        parse_float(raw.get("a_lin_y")) or 0.0,
                        parse_float(raw.get("a_lin_z")) or 0.0,
                    ),
                    vel_pre=(
                        parse_float(raw.get("vel_pre_n")) or 0.0,
                        parse_float(raw.get("vel_pre_e")) or 0.0,
                        parse_float(raw.get("vel_pre_d")) or 0.0,
                    ),
                    vel_post=(
                        parse_float(raw.get("vel_post_n")) or 0.0,
                        parse_float(raw.get("vel_post_e")) or 0.0,
                        parse_float(raw.get("vel_post_d")) or 0.0,
                    ),
                    pos=(
                        parse_float(raw.get("pos_n")) or 0.0,
                        parse_float(raw.get("pos_e")) or 0.0,
                        parse_float(raw.get("pos_d")) or 0.0,
                    ),
                    yaw_deg=parse_float(raw.get("yaw_deg")) or 0.0,
                    gps_speed_mps=parse_float(raw.get("gps_speed_mps")) or 0.0,
                    constraint_mode=int(parse_float(raw.get("constraint_mode")) or 0),
                )
            )
    if not rows:
        raise ValueError(f"CSV vacio: {path}")
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


def run_h8_replay(replay_csv: Path, replay_exe: Path, calibration: Path) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")

    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(BENCH_DIR / "h8_replay_output.csv"),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--h8-propagation-audit-csv",
        str(PROPAGATION_CSV),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def vec_mag3(v: tuple[float, float, float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def a_lin_horizontal(sample: PropagationSample) -> float:
    return math.hypot(sample.a_lin[0], sample.a_lin[1])


def a_lin_along_track(sample: PropagationSample) -> float:
    yaw = math.radians(sample.yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    return sample.a_lin[0] * c + sample.a_lin[1] * s


def a_lin_lateral(sample: PropagationSample) -> float:
    yaw = math.radians(sample.yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    return -sample.a_lin[0] * s + sample.a_lin[1] * c


def summarize_phase(
    samples: list[PropagationSample],
    label: str,
) -> dict[str, float]:
    if not samples:
        return {"label": label, "samples": 0.0}

    a_lin_h = np.array([a_lin_horizontal(s) for s in samples], dtype=float)
    a_lin_n = np.array([s.a_lin[0] for s in samples], dtype=float)
    a_lin_e = np.array([s.a_lin[1] for s in samples], dtype=float)
    a_lin_d = np.array([s.a_lin[2] for s in samples], dtype=float)
    a_body_mag = np.array([vec_mag3(s.a_body) for s in samples], dtype=float)
    a_nav_d = np.array([s.a_nav[2] for s in samples], dtype=float)
    a_along = np.array([a_lin_along_track(s) for s in samples], dtype=float)
    a_lat = np.array([a_lin_lateral(s) for s in samples], dtype=float)
    vel_h = np.array(
        [math.hypot(s.vel_post[0], s.vel_post[1]) for s in samples],
        dtype=float,
    )
    gps_speed = np.array([s.gps_speed_mps for s in samples], dtype=float)

    return {
        "label": label,
        "samples": float(len(samples)),
        "t_start_s": samples[0].timestamp_s,
        "t_end_s": samples[-1].timestamp_s,
        "a_lin_h_mean_mps2": float(np.mean(a_lin_h)),
        "a_lin_h_std_mps2": float(np.std(a_lin_h)),
        "a_lin_n_mean_mps2": float(np.mean(a_lin_n)),
        "a_lin_e_mean_mps2": float(np.mean(a_lin_e)),
        "a_lin_d_mean_mps2": float(np.mean(a_lin_d)),
        "a_lin_along_mean_mps2": float(np.mean(a_along)),
        "a_lin_lateral_mean_mps2": float(np.mean(a_lat)),
        "a_body_mag_mean_mps2": float(np.mean(a_body_mag)),
        "a_nav_d_mean_mps2": float(np.mean(a_nav_d)),
        "vel_h_mean_mps": float(np.mean(vel_h)),
        "gps_speed_mean_mps": float(np.mean(gps_speed)),
        "vel_minus_gps_mean_mps": float(np.mean(vel_h - gps_speed)),
    }


def find_cruise_windows(
    samples: list[PropagationSample],
    window_s: float = 10.0,
) -> list[tuple[float, float]]:
    if len(samples) < 20:
        return []

    times = np.array([s.timestamp_s for s in samples], dtype=float)
    speeds = np.array([s.gps_speed_mps for s in samples], dtype=float)
    moving = np.array([s.constraint_mode == 1 for s in samples], dtype=bool)

    windows: list[tuple[float, float]] = []
    start_idx = 0
    while start_idx < len(samples):
        t0 = times[start_idx]
        mask = (
            (times >= t0)
            & (times <= t0 + window_s)
            & moving
            & (speeds >= CRUISE_MIN_SPEED_MPS)
        )
        idx = np.where(mask)[0]
        if idx.size >= 50:
            seg_speed = speeds[idx]
            if float(np.std(seg_speed)) <= CRUISE_MAX_SPEED_STD_MPS:
                windows.append((float(times[idx[0]]), float(times[idx[-1]])))
                start_idx = int(idx[-1]) + 1
                continue
        start_idx += 1
    return windows


def find_first_sustained_residual(
    samples: list[PropagationSample],
    window_s: float = SUSTAINED_WINDOW_S,
    threshold_mps2: float = RESIDUAL_ALERT_MPS2,
) -> dict[str, float] | None:
    if len(samples) < 10:
        return None

    times = np.array([s.timestamp_s for s in samples], dtype=float)
    a_h = np.array([a_lin_horizontal(s) for s in samples], dtype=float)
    moving = np.array(
        [s.constraint_mode == 1 and s.gps_speed_mps >= CRUISE_MIN_SPEED_MPS for s in samples],
        dtype=bool,
    )

    for i in range(len(samples)):
        if not moving[i]:
            continue
        t0 = times[i]
        mask = (times >= t0) & (times <= t0 + window_s) & moving
        if int(np.sum(mask)) < 20:
            continue
        seg = a_h[mask]
        mean_h = float(np.mean(seg))
        if abs(mean_h) >= threshold_mps2:
            return {
                "timestamp_s": float(t0),
                "window_s": window_s,
                "a_lin_h_mean_mps2": mean_h,
                "a_lin_h_std_mps2": float(np.std(seg)),
            }
    return None


def gravity_mismatch_estimate(static_stats: dict[str, float]) -> dict[str, float]:
    g = 9.80665
    body_mag = static_stats.get("a_body_mag_mean_mps2", float("nan"))
    nav_d = static_stats.get("a_nav_d_mean_mps2", float("nan"))
    return {
        "expected_gravity_mps2": g,
        "a_body_mag_error_mps2": body_mag - g if math.isfinite(body_mag) else float("nan"),
        "a_nav_d_before_comp_mean_mps2": nav_d,
        "a_lin_d_mean_mps2": static_stats.get("a_lin_d_mean_mps2", float("nan")),
    }


def plot_analysis(samples: list[PropagationSample], plot_path: Path) -> None:
    times = np.array([s.timestamp_s for s in samples], dtype=float)
    a_lin_h = np.array([a_lin_horizontal(s) for s in samples], dtype=float)
    a_along = np.array([a_lin_along_track(s) for s in samples], dtype=float)
    a_lat = np.array([a_lin_lateral(s) for s in samples], dtype=float)
    vel_h = np.array([math.hypot(s.vel_post[0], s.vel_post[1]) for s in samples], dtype=float)
    gps_speed = np.array([s.gps_speed_mps for s in samples], dtype=float)
    pos_h = np.array([math.hypot(s.pos[0], s.pos[1]) for s in samples], dtype=float)

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle("H8 - Cadena propagacion inercial (magnitudes fisicas)", fontsize=13)

    axes[0, 0].plot(times, a_lin_h, color="#e74c3c", linewidth=0.6)
    axes[0, 0].axhline(RESIDUAL_ALERT_MPS2, color="#7f8c8d", linestyle="--", linewidth=0.8)
    axes[0, 0].axhline(-RESIDUAL_ALERT_MPS2, color="#7f8c8d", linestyle="--", linewidth=0.8)
    axes[0, 0].set_title("Aceleracion lineal horizontal |a_lin|_h")
    axes[0, 0].set_ylabel("m/s^2")
    axes[0, 0].grid(True, alpha=0.25)

    axes[0, 1].plot(times, a_along, label="along-track", linewidth=0.6)
    axes[0, 1].plot(times, a_lat, label="lateral", linewidth=0.6, alpha=0.8)
    axes[0, 1].set_title("A_lin descompuesta (yaw EKF)")
    axes[0, 1].set_ylabel("m/s^2")
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True, alpha=0.25)

    axes[1, 0].plot(times, vel_h, label="EKF |v|", linewidth=0.8)
    axes[1, 0].plot(times, gps_speed, label="GPS speed", linewidth=0.8, alpha=0.7)
    axes[1, 0].set_title("Velocidad horizontal EKF vs GPS")
    axes[1, 0].set_ylabel("m/s")
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.25)

    axes[1, 1].plot(times, vel_h - gps_speed, color="#8e44ad", linewidth=0.7)
    axes[1, 1].set_title("Delta velocidad (EKF - GPS)")
    axes[1, 1].set_ylabel("m/s")
    axes[1, 1].grid(True, alpha=0.25)

    body_mag = np.array([vec_mag3(s.a_body) for s in samples], dtype=float)
    axes[2, 0].plot(times, body_mag, color="#2980b9", linewidth=0.6)
    axes[2, 0].axhline(9.80665, color="#7f8c8d", linestyle="--", label="g")
    axes[2, 0].set_title("Magnitud aceleracion body (post-mount)")
    axes[2, 0].set_ylabel("m/s^2")
    axes[2, 0].legend(fontsize=8)
    axes[2, 0].grid(True, alpha=0.25)

    axes[2, 1].plot(times, pos_h, color="#27ae60", linewidth=0.8)
    axes[2, 1].set_title("Posicion horizontal EKF (integrada)")
    axes[2, 1].set_xlabel("Tiempo (s)")
    axes[2, 1].set_ylabel("m")
    axes[2, 1].grid(True, alpha=0.25)

    for ax in axes.flat:
        if ax is not axes[2, 0] and ax is not axes[2, 1]:
            ax.set_xlabel("Tiempo (s)")

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="H8 inertial propagation audit")
    parser.add_argument("--skip-replay", action="store_true")
    parser.add_argument("--csv", type=Path, default=PROPAGATION_CSV)
    args = parser.parse_args()

    try:
        replay_csv = resolve_replay_path(None)
        ensure_calibration(DEFAULT_CALIBRATION)
        BENCH_DIR.mkdir(parents=True, exist_ok=True)

        if not args.skip_replay:
            run_h8_replay(replay_csv, DEFAULT_REPLAY_EXE, DEFAULT_CALIBRATION)

        if not args.csv.is_file():
            raise FileNotFoundError(f"Falta {args.csv}")

        samples = load_propagation_csv(args.csv)

        static_samples = [
            s
            for s in samples
            if s.timestamp_s <= STATIC_PHASE_END_S and s.constraint_mode == 0
        ]
        moving_samples = [s for s in samples if s.constraint_mode == 1]
        cruise_windows = find_cruise_windows(samples)
        cruise_samples = [
            s
            for s in samples
            if s.constraint_mode == 1
            and s.gps_speed_mps >= CRUISE_MIN_SPEED_MPS
            and any(t0 <= s.timestamp_s <= t1 for t0, t1 in cruise_windows)
        ]
        if not cruise_samples:
            cruise_samples = [
                s for s in moving_samples if s.gps_speed_mps >= CRUISE_MIN_SPEED_MPS
            ]

        static_stats = summarize_phase(static_samples, "static_zupt")
        cruise_stats = summarize_phase(cruise_samples, "cruise")
        full_moving_stats = summarize_phase(moving_samples, "moving_nhc")
        gravity_check = gravity_mismatch_estimate(static_stats)
        first_sustained = find_first_sustained_residual(samples)

        plot_analysis(samples, ANALYSIS_PNG)

        payload = {
            "experiment": "H8_propagation_audit",
            "samples_total": len(samples),
            "static_phase": static_stats,
            "cruise_phase": cruise_stats,
            "moving_nhc_phase": full_moving_stats,
            "gravity_check": gravity_check,
            "cruise_windows_s": [{"t0": w[0], "t1": w[1]} for w in cruise_windows[:10]],
            "residual_alert_threshold_mps2": RESIDUAL_ALERT_MPS2,
            "first_sustained_residual": first_sustained,
            "artifacts": {
                "propagation_csv": str(args.csv),
                "plot_png": str(ANALYSIS_PNG),
            },
        }
        with REPORT_JSON.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
            handle.write("\n")

        print("=" * 78)
        print(" H8 - AUDITORIA CADENA PROPAGACION INERCIAL")
        print("=" * 78)
        print(f"  Muestras IMU auditadas: {len(samples)}")
        print("-" * 78)
        print("  Fase estatica (ZUPT, t<=30 s):")
        print(
            f"    |a_body| mean={static_stats.get('a_body_mag_mean_mps2', float('nan')):.3f} m/s^2  "
            f"(esperado ~9.81)"
        )
        print(
            f"    a_lin_h mean={static_stats.get('a_lin_h_mean_mps2', float('nan')):.4f} m/s^2  "
            f"std={static_stats.get('a_lin_h_std_mps2', float('nan')):.4f}"
        )
        print(
            f"    a_nav_d mean={static_stats.get('a_nav_d_mean_mps2', float('nan')):.3f} m/s^2  "
            f"(antes restar g; esperado ~+9.81 en reposo nivelado)"
        )
        print("-" * 78)
        print("  Fase crucero (NHC, velocidad ~constante):")
        print(
            f"    ventanas detectadas: {len(cruise_windows)}  "
            f"muestras={int(cruise_stats.get('samples', 0))}"
        )
        print(
            f"    a_lin along mean={cruise_stats.get('a_lin_along_mean_mps2', float('nan')):.4f} m/s^2  "
            f"lateral mean={cruise_stats.get('a_lin_lateral_mean_mps2', float('nan')):.4f} m/s^2"
        )
        print(
            f"    |a_lin|_h mean={cruise_stats.get('a_lin_h_mean_mps2', float('nan')):.4f} m/s^2"
        )
        print(
            f"    |v_ekf| mean={cruise_stats.get('vel_h_mean_mps', float('nan')):.2f} m/s  "
            f"gps={cruise_stats.get('gps_speed_mean_mps', float('nan')):.2f} m/s  "
            f"delta={cruise_stats.get('vel_minus_gps_mean_mps', float('nan')):.2f} m/s"
        )
        print("-" * 78)
        if first_sustained is not None:
            print(
                f"  ALERTA: aceleracion residual sostenida >= {RESIDUAL_ALERT_MPS2} m/s^2 "
                f"desde t={first_sustained['timestamp_s']:.1f} s  "
                f"mean={first_sustained['a_lin_h_mean_mps2']:.4f} m/s^2"
            )
            drift_100s = first_sustained["a_lin_h_mean_mps2"] * 100.0
            print(f"    Extrapolacion: {drift_100s:.1f} m/s de error velocidad en 100 s")
            print(f"    Extrapolacion: {0.5 * first_sustained['a_lin_h_mean_mps2'] * 100.0 * 100.0:.0f} m posicion en 100 s")
        else:
            print(
                f"  No se detecto sesgo sostenido >= {RESIDUAL_ALERT_MPS2} m/s^2 "
                f"en ventanas de {SUSTAINED_WINDOW_S:.0f} s en crucero"
            )
        print("-" * 78)
        print(f"  CSV:     {args.csv}")
        print(f"  Grafico: {ANALYSIS_PNG}")
        print(f"  JSON:    {REPORT_JSON}")
        print("=" * 78)
        return 0
    except (FileNotFoundError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
