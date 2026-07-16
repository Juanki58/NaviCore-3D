#!/usr/bin/env python3
"""H5 — Auditoria de sincronizacion: histograma dt y scatter ratio_n/ratio_e."""

from __future__ import annotations

import argparse
import csv
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
MIN_COMBINED_SPEED_MPS = 3.0

SYNC_AUDIT_CSV = REPO_ROOT / "docs" / "benchmarks" / "h5_sync_audit.csv"
ANALYSIS_PNG = REPO_ROOT / "docs" / "benchmarks" / "h5_sync_analysis.png"

from analyze_real_run import resolve_replay_path  # noqa: E402


@dataclass
class SyncSample:
    t_ekf: float
    t_gps_raw: float
    t_imu_last: float
    innov_n: float
    innov_e: float
    v_n: float
    v_e: float
    ratio_n: float
    ratio_e: float
    dt_predict_update: float
    gps_accepted: bool


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


def run_replay(replay_csv: Path, replay_exe: Path, calibration: Path) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")

    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(REPO_ROOT / "docs" / "benchmarks" / "h5_sync_replay_output.csv"),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--h5-sync-audit-csv",
        str(SYNC_AUDIT_CSV),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def load_samples(path: Path) -> list[SyncSample]:
    rows: list[SyncSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t_gps = parse_float(raw.get("t_gps_raw"))
            if t_gps is None:
                continue
            rows.append(
                SyncSample(
                    t_ekf=parse_float(raw.get("t_ekf")) or t_gps,
                    t_gps_raw=t_gps,
                    t_imu_last=parse_float(raw.get("t_imu_last")) or t_gps,
                    innov_n=parse_float(raw.get("innov_n")) or 0.0,
                    innov_e=parse_float(raw.get("innov_e")) or 0.0,
                    v_n=parse_float(raw.get("v_n")) or 0.0,
                    v_e=parse_float(raw.get("v_e")) or 0.0,
                    ratio_n=parse_float(raw.get("ratio_n")) or 0.0,
                    ratio_e=parse_float(raw.get("ratio_e")) or 0.0,
                    dt_predict_update=parse_float(raw.get("dt_predict_update")) or 0.0,
                    gps_accepted=bool(int(parse_float(raw.get("gps_accepted")) or 0)),
                )
            )
    return rows


def filter_moving(samples: list[SyncSample]) -> list[SyncSample]:
    moving: list[SyncSample] = []
    for sample in samples:
        speed_h = math.hypot(sample.v_n, sample.v_e)
        if speed_h > MIN_COMBINED_SPEED_MPS:
            moving.append(sample)
    return moving


def summarize_ratios(values: np.ndarray) -> tuple[float, float, float]:
    clean = values[np.isfinite(values)]
    if clean.size == 0:
        return float("nan"), float("nan"), float("nan")
    return float(np.mean(clean)), float(np.median(clean)), float(np.std(clean))


def plot_analysis(samples: list[SyncSample], plot_path: Path) -> None:
    times = np.array([s.t_gps_raw for s in samples], dtype=float)
    dt_ms = np.array([s.dt_predict_update * 1000.0 for s in samples], dtype=float)
    ratio_n = np.array([s.ratio_n for s in samples], dtype=float)
    ratio_e = np.array([s.ratio_e for s in samples], dtype=float)

    median_n = float(np.median(ratio_n[np.isfinite(ratio_n)])) if ratio_n.size else 0.0
    median_e = float(np.median(ratio_e[np.isfinite(ratio_e)])) if ratio_e.size else 0.0

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("H5 — Auditoria de sincronizacion (v combinada > 3 m/s)", fontsize=13)

    axes[0].hist(dt_ms, bins=40, color="#3498db", edgecolor="white", alpha=0.9)
    axes[0].set_xlabel("dt_predict_update (ms)")
    axes[0].set_ylabel("Recuento")
    axes[0].set_title("Histograma desfase predict -> update")
    axes[0].grid(True, alpha=0.25)

    axes[1].scatter(times, ratio_n, s=12, alpha=0.55, color="#e74c3c", label="ratio_n")
    axes[1].scatter(times, ratio_e, s=12, alpha=0.55, color="#2ecc71", label="ratio_e")
    axes[1].axhline(median_n, color="#e74c3c", linestyle="--", linewidth=1.2, label=f"mediana N={median_n:.2f} s")
    axes[1].axhline(median_e, color="#2ecc71", linestyle="--", linewidth=1.2, label=f"mediana E={median_e:.2f} s")
    axes[1].set_xlabel("Tiempo simulacion (s)")
    axes[1].set_ylabel("Ratio innov / v (s)")
    axes[1].set_title("Scatter ratio vs tiempo")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(fontsize=8)

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="H5 sync analysis")
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()

    try:
        replay_path = resolve_replay_path(None)
        ensure_calibration(DEFAULT_CALIBRATION)
        SYNC_AUDIT_CSV.parent.mkdir(parents=True, exist_ok=True)

        if not args.skip_replay:
            run_replay(replay_path, DEFAULT_REPLAY_EXE, DEFAULT_CALIBRATION)

        if not SYNC_AUDIT_CSV.is_file():
            raise FileNotFoundError(f"Falta {SYNC_AUDIT_CSV}")

        all_samples = load_samples(SYNC_AUDIT_CSV)
        if not all_samples:
            raise ValueError("CSV H5 vacio")

        moving = filter_moving(all_samples)
        if not moving:
            raise ValueError("No hay muestras con velocidad combinada > 3 m/s")

        ratio_n = np.array([s.ratio_n for s in moving], dtype=float)
        ratio_e = np.array([s.ratio_e for s in moving], dtype=float)
        lag_values = np.concatenate([ratio_n[np.isfinite(ratio_n)], ratio_e[np.isfinite(ratio_e)]])

        mean_n, median_n, std_n = summarize_ratios(ratio_n)
        mean_e, median_e, std_e = summarize_ratios(ratio_e)
        mean_all, median_all, std_all = summarize_ratios(lag_values)

        plot_analysis(moving, ANALYSIS_PNG)

        print("=" * 72)
        print(" H5 - AUDITORIA DE SINCRONIZACION")
        print("=" * 72)
        print(f"  Muestras totales (v>2 m/s en replay): {len(all_samples)}")
        print(f"  Muestras analisis (v>3 m/s):          {len(moving)}")
        print("-" * 72)
        print("  Ratios de lag (innov/v, segundos):")
        print(f"    ratio_n  media={mean_n:+.3f}  mediana={median_n:+.3f}  std={std_n:.3f}")
        print(f"    ratio_e  media={mean_e:+.3f}  mediana={median_e:+.3f}  std={std_e:.3f}")
        print(f"    combinada media={mean_all:+.3f}  mediana={median_all:+.3f}  std={std_all:.3f}")
        print("-" * 72)
        dt_ms = np.array([s.dt_predict_update * 1000.0 for s in moving], dtype=float)
        print(
            f"  dt_predict_update (ms): mediana={float(np.median(dt_ms)):.2f}  "
            f"p05={float(np.percentile(dt_ms, 5)):.2f}  p95={float(np.percentile(dt_ms, 95)):.2f}"
        )
        print("=" * 72)
        print(f"CSV:      {SYNC_AUDIT_CSV}")
        print(f"Grafico:  {ANALYSIS_PNG}")
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
