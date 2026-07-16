#!/usr/bin/env python3
"""H3a — Auditoria completa de innovaciones GNSS (solo instrumentacion replay).

Genera CSV detallado y graficos quicklook. No modifica el EKF.
"""

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
DEFAULT_REPLAY = REPO_ROOT / "docs" / "benchmarks" / "real_run_replay.csv"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_AUDIT = REPO_ROOT / "docs" / "benchmarks" / "gnss_innovation_audit.csv"
DEFAULT_PLOT = REPO_ROOT / "docs" / "benchmarks" / "gnss_innovation_audit.png"
NIS_THRESHOLD = 11.345


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


@dataclass
class AuditSummary:
    rows: int
    accepted: int
    rejected: int
    mean_innov_n: float
    mean_innov_e: float
    std_innov_n: float
    std_innov_e: float
    mean_mahal_n: float
    mean_mahal_e: float
    mean_mahal_d: float
    mean_nis: float
    mean_latency_s: float


def run_audit_replay(
    replay_exe: Path,
    replay_csv: Path,
    audit_csv: Path,
    calibration: Path,
) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")
    cmd = [
        str(replay_exe),
        "--input", str(replay_csv),
        "--output", str(REPO_ROOT / "docs" / "benchmarks" / "gnss_audit_replay_output.csv"),
        "--mount-mode", "calibration",
        "--mount-calibration", str(calibration),
        "--yaw-init", "zero",
        "--gnss-audit-csv", str(audit_csv),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def load_audit(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def summarize(rows: list[dict[str, str]]) -> AuditSummary:
    accepted = 0
    innov_n: list[float] = []
    innov_e: list[float] = []
    mahal_n: list[float] = []
    mahal_e: list[float] = []
    mahal_d: list[float] = []
    nis_vals: list[float] = []
    latency: list[float] = []

    for row in rows:
        if parse_float(row.get("gnss_accepted")) == 1.0:
            accepted += 1
        n = parse_float(row.get("innovation_n_m"))
        e = parse_float(row.get("innovation_e_m"))
        if n is not None:
            innov_n.append(n)
        if e is not None:
            innov_e.append(e)
        for key, bucket in (
            ("mahalanobis_n", mahal_n),
            ("mahalanobis_e", mahal_e),
            ("mahalanobis_d", mahal_d),
        ):
            v = parse_float(row.get(key))
            if v is not None:
                bucket.append(v)
        nis = parse_float(row.get("nis"))
        if nis is not None:
            nis_vals.append(nis)
        lat = parse_float(row.get("latency_imu_gps_s"))
        if lat is not None:
            latency.append(lat)

    rejected = len(rows) - accepted
    return AuditSummary(
        rows=len(rows),
        accepted=accepted,
        rejected=rejected,
        mean_innov_n=float(np.mean(innov_n)) if innov_n else float("nan"),
        mean_innov_e=float(np.mean(innov_e)) if innov_e else float("nan"),
        std_innov_n=float(np.std(innov_n)) if innov_n else float("nan"),
        std_innov_e=float(np.std(innov_e)) if innov_e else float("nan"),
        mean_mahal_n=float(np.mean(np.abs(mahal_n))) if mahal_n else float("nan"),
        mean_mahal_e=float(np.mean(np.abs(mahal_e))) if mahal_e else float("nan"),
        mean_mahal_d=float(np.mean(np.abs(mahal_d))) if mahal_d else float("nan"),
        mean_nis=float(np.mean(nis_vals)) if nis_vals else float("nan"),
        mean_latency_s=float(np.mean(latency)) if latency else float("nan"),
    )


def plot_audit(rows: list[dict[str, str]], plot_path: Path) -> None:
    times: list[float] = []
    innov_n: list[float] = []
    innov_e: list[float] = []
    mahal_n: list[float] = []
    mahal_e: list[float] = []
    nis: list[float] = []
    accepted: list[int] = []

    for row in rows:
        t = parse_float(row.get("timestamp_s"))
        if t is None:
            continue
        times.append(t)
        innov_n.append(parse_float(row.get("innovation_n_m")) or float("nan"))
        innov_e.append(parse_float(row.get("innovation_e_m")) or float("nan"))
        mahal_n.append(parse_float(row.get("mahalanobis_n")) or float("nan"))
        mahal_e.append(parse_float(row.get("mahalanobis_e")) or float("nan"))
        nis.append(parse_float(row.get("nis")) or float("nan"))
        accepted.append(int(parse_float(row.get("gnss_accepted")) or 0))

    t_arr = np.array(times)
    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
    fig.suptitle("H3a — Auditoria innovaciones GNSS", fontsize=13, fontweight="bold")

    axes[0].plot(t_arr, innov_n, "r-", linewidth=1.0, label="innov_n")
    axes[0].plot(t_arr, innov_e, "b-", linewidth=1.0, label="innov_e")
    axes[0].axhline(0, color="#7f8c8d", linestyle="--", linewidth=0.8)
    axes[0].set_ylabel("Innov (m)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_arr, mahal_n, "r-", linewidth=1.0, label="|mahal_n|")
    axes[1].plot(t_arr, mahal_e, "b-", linewidth=1.0, label="|mahal_e|")
    axes[1].axhline(3.0, color="#7f8c8d", linestyle="--", label="3 sigma")
    axes[1].set_ylabel("Mahalanobis")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t_arr, nis, color="#9b59b6", linewidth=1.0)
    axes[2].axhline(NIS_THRESHOLD, color="#7f8c8d", linestyle="--")
    axes[2].set_ylabel("NIS")
    axes[2].grid(True, alpha=0.3)

    acc = np.array(accepted, dtype=float)
    axes[3].step(t_arr, acc, where="post", color="#2ecc71", linewidth=1.2)
    axes[3].set_ylim(-0.1, 1.1)
    axes[3].set_xlabel("Tiempo (s)")
    axes[3].set_ylabel("Aceptado")
    axes[3].grid(True, alpha=0.3)

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def print_summary(summary: AuditSummary, audit_path: Path, plot_path: Path) -> None:
    accept_pct = 100.0 * summary.accepted / summary.rows if summary.rows else 0.0
    print("=" * 72)
    print(" H3a — RESUMEN AUDITORIA GNSS")
    print("=" * 72)
    print(f"  CSV:              {audit_path}")
    print(f"  Grafico:          {plot_path}")
    print(f"  Updates GNSS:     {summary.rows}")
    print(f"  Aceptadas:        {summary.accepted} ({accept_pct:.1f}%)")
    print(f"  Rechazadas:       {summary.rejected}")
    print(f"  Innov N:          media={summary.mean_innov_n:.1f}  std={summary.std_innov_n:.1f}")
    print(f"  Innov E:          media={summary.mean_innov_e:.1f}  std={summary.std_innov_e:.1f}")
    print(f"  |Mahal| N/E/D:    {summary.mean_mahal_n:.2f} / {summary.mean_mahal_e:.2f} / {summary.mean_mahal_d:.2f}")
    print(f"  NIS medio:        {summary.mean_nis:.1f}")
    print(f"  Latencia IMU-GPS: {summary.mean_latency_s*1000:.1f} ms (media)")
    print("-" * 72)

    if summary.std_innov_n < 10 and abs(summary.mean_innov_n) > 30:
        print(" Patron North: OFFSET constante")
    elif summary.std_innov_n > 30:
        print(" Patron North: DERIVA u oscilacion")
    else:
        print(" Patron North: mixto")

    if summary.std_innov_e < 10 and abs(summary.mean_innov_e) > 30:
        print(" Patron East:  OFFSET constante")
    elif summary.std_innov_e > 30:
        print(" Patron East:  DERIVA u oscilacion")
    else:
        print(" Patron East:  mixto")

    if summary.mean_mahal_n < 5 and summary.mean_mahal_e > 15:
        print(" Diagnostico: North razonable, East desastroso -> revisar eje E / convencion NED")
    elif summary.mean_mahal_n > 15 and summary.mean_mahal_e > 15:
        print(" Diagnostico: ambos ejes mal -> deriva posicion global o P/R mal modelados")
    print("=" * 72)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="H3a auditoria innovaciones GNSS")
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--audit-csv", type=Path, default=DEFAULT_AUDIT)
    parser.add_argument("--plot", type=Path, default=DEFAULT_PLOT)
    parser.add_argument("--skip-replay", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        if not args.skip_replay:
            if not args.calibration.is_file():
                subprocess.run(
                    [sys.executable, str(REPO_ROOT / "audit_imu_chain.py"),
                     "--export-calibration", str(args.calibration)],
                    cwd=REPO_ROOT,
                    check=True,
                )
            run_audit_replay(args.replay_exe, args.replay, args.audit_csv, args.calibration)

        if not args.audit_csv.is_file():
            raise FileNotFoundError(f"No existe auditoria: {args.audit_csv}")

        rows = load_audit(args.audit_csv)
        summary = summarize(rows)
        plot_audit(rows, args.plot)
        print_summary(summary, args.audit_csv, args.plot)
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
