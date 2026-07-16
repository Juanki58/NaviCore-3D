#!/usr/bin/env python3
"""H6 — Auditoría completa de sincronización temporal (sin tocar EKF/Q/R/P).

Registra desfases GPS–IMU, EKF–GPS y predict→update por cada update GNSS,
genera histogramas (no solo medias) y estima desfases equivalentes ν/v.
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
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
MOVING_START_S = 30.0
MIN_SPEED_MPS = 5.0

BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
SYNC_AUDIT_CSV = BENCH_DIR / "h6_sync_audit.csv"
SYNC_REPORT_JSON = BENCH_DIR / "h6_sync_audit_report.json"
SYNC_ANALYSIS_PNG = BENCH_DIR / "h6_sync_audit_analysis.png"

from analyze_real_run import resolve_replay_path  # noqa: E402


@dataclass
class SyncRow:
    gps_timestamp_s: float
    imu_timestamp_s: float
    ekf_timestamp_s: float
    dt_gps_imu_s: float
    dt_ekf_gps_s: float
    dt_predict_update_s: float
    innovation_n_m: float
    innovation_e_m: float
    innovation_d_m: float
    vel_n_mps: float
    vel_e_mps: float
    vel_d_mps: float
    gps_speed_mps: float
    gnss_accepted: bool


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


def run_sync_audit_replay(
    replay_exe: Path,
    replay_csv: Path,
    sync_csv: Path,
    calibration: Path,
) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")

    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(BENCH_DIR / "h6_sync_audit_replay_output.csv"),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--sync-audit-csv",
        str(sync_csv),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def load_sync_rows(path: Path) -> list[SyncRow]:
    rows: list[SyncRow] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            gps_ts = parse_float(raw.get("gps_timestamp_s"))
            if gps_ts is None:
                continue
            rows.append(
                SyncRow(
                    gps_timestamp_s=gps_ts,
                    imu_timestamp_s=parse_float(raw.get("imu_timestamp_s")) or gps_ts,
                    ekf_timestamp_s=parse_float(raw.get("ekf_timestamp_s")) or gps_ts,
                    dt_gps_imu_s=parse_float(raw.get("dt_gps_imu_s")) or 0.0,
                    dt_ekf_gps_s=parse_float(raw.get("dt_ekf_gps_s")) or 0.0,
                    dt_predict_update_s=parse_float(raw.get("dt_predict_update_s")) or 0.0,
                    innovation_n_m=parse_float(raw.get("innovation_n_m")) or 0.0,
                    innovation_e_m=parse_float(raw.get("innovation_e_m")) or 0.0,
                    innovation_d_m=parse_float(raw.get("innovation_d_m")) or 0.0,
                    vel_n_mps=parse_float(raw.get("vel_n_mps")) or 0.0,
                    vel_e_mps=parse_float(raw.get("vel_e_mps")) or 0.0,
                    vel_d_mps=parse_float(raw.get("vel_d_mps")) or 0.0,
                    gps_speed_mps=parse_float(raw.get("gps_speed_mps")) or 0.0,
                    gnss_accepted=bool(int(parse_float(raw.get("gnss_accepted")) or 0)),
                )
            )
    return rows


def histogram_payload(values: np.ndarray, bins: int = 40) -> dict[str, list[float] | int]:
    clean = values[np.isfinite(values)]
    if clean.size == 0:
        return {"count": 0, "bin_edges": [], "bin_counts": []}
    counts, edges = np.histogram(clean, bins=bins)
    return {
        "count": int(clean.size),
        "bin_edges": edges.tolist(),
        "bin_counts": counts.astype(int).tolist(),
        "median": float(np.median(clean)),
        "p05": float(np.percentile(clean, 5)),
        "p95": float(np.percentile(clean, 95)),
    }


def peak_bin_center(payload: dict[str, list[float] | int | float]) -> float | None:
    edges = payload.get("bin_edges")
    counts = payload.get("bin_counts")
    if not isinstance(edges, list) or not isinstance(counts, list) or not counts:
        return None
    idx = int(np.argmax(np.asarray(counts, dtype=float)))
    if idx >= len(edges) - 1:
        return None
    return float(0.5 * (edges[idx] + edges[idx + 1]))


def implied_delay_s(innovation_m: float, velocity_mps: float) -> float:
    if abs(velocity_mps) < 0.5:
        return float("nan")
    return innovation_m / velocity_mps


def moving_constant_speed_rows(rows: Sequence[SyncRow]) -> list[SyncRow]:
    selected: list[SyncRow] = []
    for row in rows:
        if row.gps_timestamp_s <= MOVING_START_S:
            continue
        speed_h = math.hypot(row.vel_n_mps, row.vel_e_mps)
        ref_speed = row.gps_speed_mps if row.gps_speed_mps > 0.0 else speed_h
        if ref_speed < MIN_SPEED_MPS:
            continue
        selected.append(row)
    return selected


def analyze_rows(rows: Sequence[SyncRow]) -> dict[str, object]:
    dt_gps_imu_ms = np.array([r.dt_gps_imu_s * 1000.0 for r in rows], dtype=float)
    dt_ekf_gps_ms = np.array([r.dt_ekf_gps_s * 1000.0 for r in rows], dtype=float)
    dt_predict_update_ms = np.array([r.dt_predict_update_s * 1000.0 for r in rows], dtype=float)

    moving = [r for r in rows if r.gps_timestamp_s > MOVING_START_S]
    constant_speed = moving_constant_speed_rows(rows)

    implied_n: list[float] = []
    implied_e: list[float] = []
    implied_h: list[float] = []
    for row in constant_speed:
        delay_n = implied_delay_s(row.innovation_n_m, row.vel_n_mps)
        delay_e = implied_delay_s(row.innovation_e_m, row.vel_e_mps)
        speed_h = math.hypot(row.vel_n_mps, row.vel_e_mps)
        innov_h = math.hypot(row.innovation_n_m, row.innovation_e_m)
        delay_h = implied_delay_s(innov_h, speed_h)
        if math.isfinite(delay_n):
            implied_n.append(delay_n)
        if math.isfinite(delay_e):
            implied_e.append(delay_e)
        if math.isfinite(delay_h):
            implied_h.append(delay_h)

    hist_gps_imu = histogram_payload(dt_gps_imu_ms)
    hist_ekf_gps = histogram_payload(dt_ekf_gps_ms)
    hist_predict_update = histogram_payload(dt_predict_update_ms)
    hist_implied_n = histogram_payload(np.asarray(implied_n, dtype=float))
    hist_implied_e = histogram_payload(np.asarray(implied_e, dtype=float))
    hist_implied_h = histogram_payload(np.asarray(implied_h, dtype=float))

    return {
        "samples_total": len(rows),
        "samples_moving": len(moving),
        "samples_constant_speed": len(constant_speed),
        "histograms_ms": {
            "dt_gps_imu": hist_gps_imu,
            "dt_ekf_gps": hist_ekf_gps,
            "dt_predict_update": hist_predict_update,
        },
        "histograms_implied_delay_s": {
            "innovation_n_over_vel_n": hist_implied_n,
            "innovation_e_over_vel_e": hist_implied_e,
            "innovation_h_over_speed_h": hist_implied_h,
        },
        "peak_implied_delay_s": {
            "innovation_n_over_vel_n": peak_bin_center(hist_implied_n),
            "innovation_e_over_vel_e": peak_bin_center(hist_implied_e),
            "innovation_h_over_speed_h": peak_bin_center(hist_implied_h),
        },
        "interpretation": {
            "note": (
                "Si el error posicional proviene de un desfase temporal constante, "
                "ν/v debería concentrarse en un pico estrecho (p.ej. 80 m / 20 m/s = 4 s)."
            ),
            "example_80m_at_20mps_s": 4.0,
        },
    }


def plot_histogram(
    ax: plt.Axes,
    values_ms: np.ndarray,
    title: str,
    xlabel: str,
) -> None:
    clean = values_ms[np.isfinite(values_ms)]
    if clean.size == 0:
        ax.set_title(f"{title} (sin datos)")
        return
    ax.hist(clean, bins=40, color="#3498db", edgecolor="white", alpha=0.9)
    median = float(np.median(clean))
    ax.axvline(median, color="#e74c3c", linestyle="--", linewidth=1.5, label=f"mediana={median:.1f}")
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Recuento")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)


def plot_implied_delay_histogram(
    ax: plt.Axes,
    delays_s: np.ndarray,
    title: str,
) -> None:
    clean = delays_s[np.isfinite(delays_s)]
    if clean.size == 0:
        ax.set_title(f"{title} (sin datos)")
        return
    ax.hist(clean, bins=40, color="#9b59b6", edgecolor="white", alpha=0.9)
    median = float(np.median(clean))
    ax.axvline(median, color="#e74c3c", linestyle="--", linewidth=1.5, label=f"mediana={median:.2f} s")
    ax.axvline(4.0, color="#7f8c8d", linestyle=":", linewidth=1.2, label="ref 4 s")
    ax.set_title(title)
    ax.set_xlabel("Desfase equivalente (s)")
    ax.set_ylabel("Recuento")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)


def plot_analysis(rows: Sequence[SyncRow], plot_path: Path, report: dict[str, object]) -> None:
    dt_gps_imu_ms = np.array([r.dt_gps_imu_s * 1000.0 for r in rows], dtype=float)
    dt_ekf_gps_ms = np.array([r.dt_ekf_gps_s * 1000.0 for r in rows], dtype=float)
    dt_predict_update_ms = np.array([r.dt_predict_update_s * 1000.0 for r in rows], dtype=float)

    constant_speed = moving_constant_speed_rows(rows)
    implied_n = np.array(
        [implied_delay_s(r.innovation_n_m, r.vel_n_mps) for r in constant_speed],
        dtype=float,
    )
    implied_e = np.array(
        [implied_delay_s(r.innovation_e_m, r.vel_e_mps) for r in constant_speed],
        dtype=float,
    )
    implied_h = np.array(
        [
            implied_delay_s(
                math.hypot(r.innovation_n_m, r.innovation_e_m),
                math.hypot(r.vel_n_mps, r.vel_e_mps),
            )
            for r in constant_speed
        ],
        dtype=float,
    )

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle(
        "H6 — Auditoria de sincronizacion (histogramas, fase en marcha resaltada en nu/v)",
        fontsize=13,
    )

    plot_histogram(axes[0, 0], dt_gps_imu_ms, "dt GPS - IMU", "Milisegundos")
    plot_histogram(axes[0, 1], dt_ekf_gps_ms, "dt EKF - GPS", "Milisegundos")
    plot_histogram(axes[0, 2], dt_predict_update_ms, "dt predict -> update", "Milisegundos")

    plot_implied_delay_histogram(axes[1, 0], implied_n, "nu_N / v_N  (marcha, v>=5 m/s)")
    plot_implied_delay_histogram(axes[1, 1], implied_e, "nu_E / v_E  (marcha, v>=5 m/s)")
    plot_implied_delay_histogram(axes[1, 2], implied_h, "||nu_h|| / ||v_h||  (marcha)")

    peak_h = report.get("peak_implied_delay_s", {})
    if isinstance(peak_h, dict):
        peak = peak_h.get("innovation_h_over_speed_h")
        if isinstance(peak, float) and math.isfinite(peak):
            fig.text(
                0.5,
                0.01,
                f"Pico histograma desfase horizontal equivalente ~ {peak:.2f} s "
                f"(referencia ejemplo: 80 m / 20 m/s = 4.0 s)",
                ha="center",
                fontsize=10,
            )

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="H6 sync audit")
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()

    try:
        replay_path = resolve_replay_path(None)
        ensure_calibration(DEFAULT_CALIBRATION)
        BENCH_DIR.mkdir(parents=True, exist_ok=True)

        if not args.skip_replay:
            run_sync_audit_replay(
                DEFAULT_REPLAY_EXE,
                replay_path,
                SYNC_AUDIT_CSV,
                DEFAULT_CALIBRATION,
            )

        if not SYNC_AUDIT_CSV.is_file():
            raise FileNotFoundError(f"Falta {SYNC_AUDIT_CSV}")

        rows = load_sync_rows(SYNC_AUDIT_CSV)
        if not rows:
            raise ValueError("CSV de sincronización vacío")

        report = analyze_rows(rows)
        plot_analysis(rows, SYNC_ANALYSIS_PNG, report)

        payload = {
            "experiment": "H6_sync_audit",
            "input_csv": str(SYNC_AUDIT_CSV.relative_to(REPO_ROOT)),
            "moving_phase_start_s": MOVING_START_S,
            "constant_speed_min_mps": MIN_SPEED_MPS,
            **report,
        }
        with SYNC_REPORT_JSON.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
            handle.write("\n")

        peaks = report["peak_implied_delay_s"]
        hists = report["histograms_ms"]
        print("=" * 72)
        print(" H6 - AUDITORIA DE SINCRONIZACION")
        print("=" * 72)
        print(f"  Muestras GPS:              {report['samples_total']}")
        print(f"  En marcha (t>{MOVING_START_S}s):     {report['samples_moving']}")
        print(f"  Velocidad >= {MIN_SPEED_MPS} m/s:        {report['samples_constant_speed']}")
        print("-" * 72)
        print("  Histogramas dt (mediana, ms):")
        for key, label in (
            ("dt_gps_imu", "GPS-IMU"),
            ("dt_ekf_gps", "EKF-GPS"),
            ("dt_predict_update", "predict->update"),
        ):
            hist = hists[key]
            print(f"    {label:16} mediana={hist['median']:.1f} ms  p05={hist['p05']:.1f}  p95={hist['p95']:.1f}")
        print("-" * 72)
        print("  Desfase equivalente nu/v (pico histograma, s):")
        print(f"    Norte:       {peaks['innovation_n_over_vel_n']}")
        print(f"    Este:        {peaks['innovation_e_over_vel_e']}")
        print(f"    Horizontal:  {peaks['innovation_h_over_speed_h']}")
        print("=" * 72)
        print(f"CSV:      {SYNC_AUDIT_CSV}")
        print(f"JSON:     {SYNC_REPORT_JSON}")
        print(f"Graficos: {SYNC_ANALYSIS_PNG}")
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
