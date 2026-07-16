#!/usr/bin/env python3
"""Quicklook: innovation_n / innovation_e vs tiempo.

Uso inmediato sobre CSV de instrumentacion existente (sin recompilar).
Detecta patron: offset constante, deriva, o oscilacion (sincronizacion).
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_INPUT = REPO_ROOT / "docs" / "benchmarks" / "yaw_init_instrumentation_h2.csv"
DEFAULT_PLOT = REPO_ROOT / "docs" / "benchmarks" / "gnss_innovation_quicklook.png"


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


def load_innovations(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    times: list[float] = []
    innov_n: list[float] = []
    innov_e: list[float] = []
    nis: list[float] = []

    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            t = parse_float(row.get("timestamp_s"))
            n = parse_float(row.get("innovation_n_m"))
            e = parse_float(row.get("innovation_e_m"))
            if t is None or n is None or e is None:
                continue
            times.append(t)
            innov_n.append(n)
            innov_e.append(e)
            nis.append(parse_float(row.get("nis")) or float("nan"))

    return (
        np.array(times, dtype=float),
        np.array(innov_n, dtype=float),
        np.array(innov_e, dtype=float),
        np.array(nis, dtype=float),
    )


def classify_pattern(innov_n: np.ndarray, innov_e: np.ndarray) -> str:
    if innov_n.size < 5:
        return "pocas muestras"

    def axis_pattern(values: np.ndarray, name: str) -> str:
        mean = float(np.mean(values))
        std = float(np.std(values))
        slope = float(np.polyfit(np.arange(values.size), values, 1)[0]) if values.size >= 3 else 0.0
        sign_changes = int(np.sum(np.diff(np.sign(values)) != 0))

        if std < 5.0 and abs(mean) > 20.0:
            return f"{name}: offset constante (~{mean:.0f} m, std={std:.1f})"
        if abs(slope) > 2.0:
            return f"{name}: deriva (pendiente~{slope:.1f} m/fix)"
        if sign_changes >= max(3, values.size // 4):
            return f"{name}: oscilacion ({sign_changes} cambios de signo)"
        return f"{name}: mixto (media={mean:.1f}, std={std:.1f})"

    return axis_pattern(innov_n, "North") + " | " + axis_pattern(innov_e, "East")


def plot_quicklook(
    times: np.ndarray,
    innov_n: np.ndarray,
    innov_e: np.ndarray,
    nis: np.ndarray,
    plot_path: Path,
    title_suffix: str,
) -> None:
    plot_path.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"Innovaciones GNSS vs tiempo {title_suffix}", fontsize=13, fontweight="bold")

    axes[0].plot(times, innov_n, color="#e74c3c", linewidth=1.2, label="innovation_n")
    axes[0].axhline(0.0, color="#7f8c8d", linestyle="--", linewidth=0.8)
    axes[0].set_ylabel("Innov N (m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].plot(times, innov_e, color="#3498db", linewidth=1.2, label="innovation_e")
    axes[1].axhline(0.0, color="#7f8c8d", linestyle="--", linewidth=0.8)
    axes[1].set_ylabel("Innov E (m)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    axes[2].plot(times, nis, color="#9b59b6", linewidth=1.0, label="NIS")
    axes[2].axhline(11.345, color="#7f8c8d", linestyle="--", linewidth=1.0, label="umbral chi2")
    axes[2].set_xlabel("Tiempo (s)")
    axes[2].set_ylabel("NIS")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")

    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Quicklook innovaciones GNSS")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--plot", type=Path, default=DEFAULT_PLOT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    if not args.input.is_file():
        print(f"ERROR: no existe {args.input}", file=sys.stderr)
        return 1

    times, innov_n, innov_e, nis = load_innovations(args.input)
    if times.size == 0:
        print("ERROR: sin filas de innovacion", file=sys.stderr)
        return 1

    pattern = classify_pattern(innov_n, innov_e)
    plot_quicklook(times, innov_n, innov_e, nis, args.plot, f"({args.input.name})")

    print("=" * 64)
    print(" GNSS INNOVATION QUICKLOOK")
    print("=" * 64)
    print(f"  Entrada:  {args.input}")
    print(f"  Grafico:  {args.plot}")
    print(f"  Muestras: {times.size}")
    print(f"  Innov N:  media={np.mean(innov_n):.1f}  std={np.std(innov_n):.1f}  "
          f"min={np.min(innov_n):.1f}  max={np.max(innov_n):.1f}")
    print(f"  Innov E:  media={np.mean(innov_e):.1f}  std={np.std(innov_e):.1f}  "
          f"min={np.min(innov_e):.1f}  max={np.max(innov_e):.1f}")
    print(f"  Patron:   {pattern}")
    print("=" * 64)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
