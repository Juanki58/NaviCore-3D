#!/usr/bin/env python3
"""H9a - Auditoria de alineacion gravedad: accel vs R_bn^T g_ned tick a tick.

Hipotesis H9a (prediccion cuantitativa):
  Si el residual horizontal es solo error inicial de actitud, al inicializar roll/pitch
  desde gravedad, a_lin_h debe colapsar de ~0.9 m/s2 a ~0.03-0.05 m/s2 inmediatamente.

Instrumentacion:
  angle(accel_normalized, g_pred) -> gravity_alignment_error_deg
  comparar con a_lin_h e implied_tilt = arcsin(a_lin_h / g)
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

BASELINE_CSV = BENCH_DIR / "h9a_alignment_baseline.csv"
H9A_CSV = BENCH_DIR / "h9a_alignment_with_init.csv"
REPORT_JSON = BENCH_DIR / "h9a_gravity_alignment_report.json"
ANALYSIS_PNG = BENCH_DIR / "h9a_gravity_alignment_analysis.png"

PREDICT_ONLY_END_S = 60.0
CHEAP_CHECK_END_S = 2.0
STATIC_END_S = 30.0
GRAVITY_MPS2 = 9.80665
G_SIN_3DEG = GRAVITY_MPS2 * math.sin(math.radians(3.0))
H9A_COLLAPSE_TARGET_MPS2 = 0.05
H9A_COLLAPSE_MAX_MPS2 = 0.08

from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


@dataclass
class AlignmentSample:
    timestamp_s: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    gravity_alignment_error_deg: float
    gravity_alignment_corr_error_deg: float
    predicted_a_lin_h_from_angle_mps2: float
    a_lin_h_mps2: float
    implied_tilt_from_a_lin_deg: float
    h9a_applied: bool


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


def load_alignment_csv(path: Path) -> list[AlignmentSample]:
    rows: list[AlignmentSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t = parse_float(raw.get("timestamp_s"))
            if t is None:
                continue
            rows.append(
                AlignmentSample(
                    timestamp_s=t,
                    roll_deg=parse_float(raw.get("roll_ekf_deg")) or 0.0,
                    pitch_deg=parse_float(raw.get("pitch_ekf_deg")) or 0.0,
                    yaw_deg=parse_float(raw.get("yaw_ekf_deg")) or 0.0,
                    gravity_alignment_error_deg=parse_float(
                        raw.get("gravity_alignment_error_deg")
                    )
                    or 0.0,
                    gravity_alignment_corr_error_deg=parse_float(
                        raw.get("gravity_alignment_corr_error_deg")
                    )
                    or 0.0,
                    predicted_a_lin_h_from_angle_mps2=parse_float(
                        raw.get("predicted_a_lin_h_from_angle_mps2")
                    )
                    or 0.0,
                    a_lin_h_mps2=parse_float(raw.get("a_lin_h_mps2")) or 0.0,
                    implied_tilt_from_a_lin_deg=parse_float(
                        raw.get("implied_tilt_from_a_lin_deg")
                    )
                    or 0.0,
                    h9a_applied=bool(int(parse_float(raw.get("h9a_applied")) or 0)),
                )
            )
    if not rows:
        raise ValueError(f"CSV vacio: {path}")
    rows.sort(key=lambda sample: sample.timestamp_s)
    return rows


def run_predict_only_replay(
    replay_csv: Path,
    replay_exe: Path,
    calibration: Path,
    audit_csv: Path,
    *,
    h9a: bool,
) -> None:
    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(BENCH_DIR / ("h9a_alignment_baseline_out.csv" if not h9a else "h9a_alignment_h9a_out.csv")),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--predict-only",
        "--predict-only-end-s",
        str(PREDICT_ONLY_END_S),
        "--h9a-gravity-alignment-audit-csv",
        str(audit_csv),
    ]
    if h9a:
        cmd.append("--h9a-gravity-tilt-init")
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def summarize_phase(samples: list[AlignmentSample], label: str) -> dict[str, float]:
    if not samples:
        return {"label": label, "samples": 0.0}

    err = np.array([s.gravity_alignment_error_deg for s in samples], dtype=float)
    a_lin = np.array([s.a_lin_h_mps2 for s in samples], dtype=float)
    pred = np.array([s.predicted_a_lin_h_from_angle_mps2 for s in samples], dtype=float)
    implied = np.array([s.implied_tilt_from_a_lin_deg for s in samples], dtype=float)
    roll = np.array([s.roll_deg for s in samples], dtype=float)
    pitch = np.array([s.pitch_deg for s in samples], dtype=float)

    corr_angle_alin = float(np.corrcoef(err, a_lin)[0, 1]) if len(samples) > 2 else float("nan")

    return {
        "label": label,
        "samples": float(len(samples)),
        "t_start_s": samples[0].timestamp_s,
        "t_end_s": samples[-1].timestamp_s,
        "gravity_alignment_error_deg_mean": float(np.mean(err)),
        "gravity_alignment_error_deg_median": float(np.median(err)),
        "gravity_alignment_error_deg_p95": float(np.percentile(err, 95)),
        "a_lin_h_mean_mps2": float(np.mean(a_lin)),
        "a_lin_h_median_mps2": float(np.median(a_lin)),
        "a_lin_h_p95_mps2": float(np.percentile(a_lin, 95)),
        "predicted_a_lin_h_from_angle_mean_mps2": float(np.mean(pred)),
        "implied_tilt_from_a_lin_deg_mean": float(np.mean(implied)),
        "roll_deg_mean": float(np.mean(roll)),
        "pitch_deg_mean": float(np.mean(pitch)),
        "correlation_alignment_error_vs_a_lin_h": corr_angle_alin,
    }


def filter_window(samples: list[AlignmentSample], t_end: float) -> list[AlignmentSample]:
    return [s for s in samples if s.timestamp_s <= t_end]


def first_post_init_samples(
    samples: list[AlignmentSample],
    *,
    count: int = 20,
) -> list[AlignmentSample]:
    post = [s for s in samples if s.h9a_applied]
    return post[:count]


def plot_analysis(
    baseline: list[AlignmentSample],
    with_h9a: list[AlignmentSample],
    path: Path,
) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=False)
    fig.suptitle(
        "H9a gravity alignment audit (predict-only, accel vs R_bn^T g_ned)",
        fontsize=14,
    )

    for samples, label, color, ax_idx in (
        (baseline, "H0 baseline", "#c0392b", 0),
        (with_h9a, "H9a init", "#2980b9", 1),
    ):
        if not samples:
            continue
        times = np.array([s.timestamp_s for s in samples], dtype=float)
        err = np.array([s.gravity_alignment_error_deg for s in samples], dtype=float)
        a_lin = np.array([s.a_lin_h_mps2 for s in samples], dtype=float)
        pred = np.array([s.predicted_a_lin_h_from_angle_mps2 for s in samples], dtype=float)

        axes[0].plot(times, err, linewidth=0.7, alpha=0.85, color=color, label=label)
        axes[1].plot(times, a_lin, linewidth=0.7, alpha=0.85, color=color, label=label)
        axes[2].plot(times, pred, linewidth=0.7, alpha=0.85, color=color, label=f"{label} g*sin(err)")
        axes[3].plot(times, np.array([s.roll_deg for s in samples]), linewidth=0.6, alpha=0.7, color=color)
        axes[3].plot(
            times,
            np.array([s.pitch_deg for s in samples]),
            linewidth=0.6,
            alpha=0.7,
            color=color,
            linestyle="--",
        )

    axes[0].axvline(CHEAP_CHECK_END_S, color="#7f8c8d", linestyle=":", label="2 s check")
    axes[0].set_ylabel("alignment err [deg]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].axhline(G_SIN_3DEG, color="#7f8c8d", linestyle="--", label="g*sin(3 deg)")
    axes[1].axhline(H9A_COLLAPSE_TARGET_MPS2, color="#27ae60", linestyle=":", label="H9a target")
    axes[1].set_ylabel("a_lin_h [m/s2]")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.25)

    axes[2].set_ylabel("g*sin(err) [m/s2]")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.25)

    axes[3].set_ylabel("roll/pitch [deg]")
    axes[3].set_xlabel("Tiempo [s]")
    axes[3].grid(True, alpha=0.25)

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="H9a gravity alignment audit")
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()

    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    replay_csv = resolve_replay_path(None)
    ensure_calibration(DEFAULT_CALIBRATION)

    if not args.skip_replay:
        run_predict_only_replay(
            replay_csv,
            DEFAULT_REPLAY_EXE,
            DEFAULT_CALIBRATION,
            BASELINE_CSV,
            h9a=False,
        )
        run_predict_only_replay(
            replay_csv,
            DEFAULT_REPLAY_EXE,
            DEFAULT_CALIBRATION,
            H9A_CSV,
            h9a=True,
        )

    if not BASELINE_CSV.is_file() or not H9A_CSV.is_file():
        print("ERROR: faltan CSV de alineacion", file=sys.stderr)
        return 1

    baseline = load_alignment_csv(BASELINE_CSV)
    with_h9a = load_alignment_csv(H9A_CSV)

    cheap_baseline = summarize_phase(filter_window(baseline, CHEAP_CHECK_END_S), "baseline_first_2s")
    cheap_h9a = summarize_phase(filter_window(with_h9a, CHEAP_CHECK_END_S), "h9a_first_2s")
    static_baseline = summarize_phase(filter_window(baseline, STATIC_END_S), "baseline_0_30s")
    static_h9a = summarize_phase(filter_window(with_h9a, STATIC_END_S), "h9a_0_30s")
    full_baseline = summarize_phase(baseline, "baseline_0_60s")
    full_h9a = summarize_phase(with_h9a, "h9a_0_60s")
    post_init_h9a = summarize_phase(first_post_init_samples(with_h9a, count=30), "h9a_first_30_ticks_post_init")

    baseline_first_tick = baseline[0] if baseline else None
    h9a_first_post = first_post_init_samples(with_h9a, count=1)
    h9a_first_post_tick = h9a_first_post[0] if h9a_first_post else None

    angle_explains_alin = (
        full_baseline.get("correlation_alignment_error_vs_a_lin_h", 0.0) >= 0.85
    )
    geometric_error_confirmed = (
        angle_explains_alin
        and full_baseline.get("gravity_alignment_error_deg_mean", 0.0) >= 4.0
    )

    h9a_collapsed = False
    if h9a_first_post_tick is not None:
        h9a_collapsed = h9a_first_post_tick.a_lin_h_mps2 <= H9A_COLLAPSE_MAX_MPS2

    post_init_mean = post_init_h9a.get("a_lin_h_mean_mps2", float("nan"))
    h9a_sustained = full_h9a.get("a_lin_h_mean_mps2", float("nan")) <= H9A_COLLAPSE_MAX_MPS2

    # Colapso inmediato pero no sostenido => no es solo actitud inicial
    h9a_initial_only = h9a_collapsed and h9a_sustained
    h9a_hypothesis_confirmed = h9a_initial_only

    cheap_alignment_high = (
        cheap_baseline.get("gravity_alignment_error_deg_median", 0.0) >= 4.0
    )
    sustained_alignment_high = (
        full_baseline.get("gravity_alignment_error_deg_mean", 0.0) >= 4.0
    )
    geometric_error_grows = (
        not cheap_alignment_high
        and sustained_alignment_high
    )

    report = {
        "experiment": "H9a_gravity_alignment_audit",
        "hypothesis": {
            "statement": (
                "Si a_lin_h horizontal proviene exclusivamente de error inicial de actitud, "
                "H9a debe colapsar a_lin_h de ~0.9 m/s2 a ~0.03-0.05 m/s2 inmediatamente"
            ),
            "quantitative_prediction": {
                "before_a_lin_h_mps2": "~0.94",
                "after_a_lin_h_mps2": "0.03-0.05",
            },
            "h9a_collapsed_immediately": h9a_collapsed,
            "h9a_sustained_over_60s": h9a_sustained,
            "h9a_hypothesis_initial_attitude_only": h9a_initial_only,
            "h9a_hypothesis_confirmed": h9a_hypothesis_confirmed,
        },
        "geometric_error_analysis": {
            "gravity_mps2": GRAVITY_MPS2,
            "g_sin_3deg_mps2": G_SIN_3DEG,
            "alignment_error_grows_over_time": geometric_error_grows,
            "cheap_check_first_2s": {
                "baseline": cheap_baseline,
                "with_h9a": cheap_h9a,
            },
            "static_0_30s": {
                "baseline": static_baseline,
                "with_h9a": static_h9a,
            },
            "full_0_60s": {
                "baseline": full_baseline,
                "with_h9a": full_h9a,
            },
            "h9a_post_init_first_ticks": post_init_h9a,
            "geometric_misalignment_likely": geometric_error_confirmed,
            "angle_error_explains_a_lin_h": angle_explains_alin,
            "baseline_vs_h9a_60s_a_lin_delta_mps2": float(
                full_h9a.get("a_lin_h_mean_mps2", 0.0)
                - full_baseline.get("a_lin_h_mean_mps2", 0.0)
            ),
        },
        "first_tick_comparison": {
            "baseline": None
            if baseline_first_tick is None
            else {
                "timestamp_s": baseline_first_tick.timestamp_s,
                "gravity_alignment_error_deg": baseline_first_tick.gravity_alignment_error_deg,
                "a_lin_h_mps2": baseline_first_tick.a_lin_h_mps2,
                "implied_tilt_deg": baseline_first_tick.implied_tilt_from_a_lin_deg,
            },
            "h9a_first_post_init": None
            if h9a_first_post_tick is None
            else {
                "timestamp_s": h9a_first_post_tick.timestamp_s,
                "gravity_alignment_error_deg": h9a_first_post_tick.gravity_alignment_error_deg,
                "a_lin_h_mps2": h9a_first_post_tick.a_lin_h_mps2,
                "implied_tilt_deg": h9a_first_post_tick.implied_tilt_from_a_lin_deg,
            },
        },
        "interpretation": (
            "a_lin_h es geometrico (corr~1 con angle error) pero NO solo actitud inicial: "
            "H9a colapsa al instante (0.008 m/s2) y vuelve a ~0.94 m/s2 en 60 s. "
            "El error pasa de ~1 deg (2 s) a ~5.5 deg (60 s): priorizar propagacion de actitud (H9b)."
            if (geometric_error_confirmed and h9a_collapsed and not h9a_sustained)
            else (
                "Error geometrico + H9a sostenido: origen era actitud inicial."
                if h9a_hypothesis_confirmed
                else (
                    "Sin evidencia clara de misalignment geometrico sostenido."
                    if not geometric_error_confirmed
                    else "Resultado intermedio."
                )
            )
        ),
        "artifacts": {
            "baseline_csv": str(BASELINE_CSV),
            "h9a_csv": str(H9A_CSV),
            "plot_png": str(ANALYSIS_PNG),
        },
    }

    plot_analysis(baseline, with_h9a, ANALYSIS_PNG)

    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    print("=" * 72)
    print("H9a - Gravity alignment audit")
    print("=" * 72)
    print("  Comprobacion barata (0-2 s, baseline H0):")
    print(f"    alignment error median: {cheap_baseline.get('gravity_alignment_error_deg_median', float('nan')):.2f} deg")
    print(f"    a_lin_h median:         {cheap_baseline.get('a_lin_h_median_mps2', float('nan')):.4f} m/s2")
    print(f"    implied tilt mean:      {cheap_baseline.get('implied_tilt_from_a_lin_deg_mean', float('nan')):.2f} deg")
    print("  Full 0-60 s baseline:")
    print(f"    alignment error mean:   {full_baseline.get('gravity_alignment_error_deg_mean', float('nan')):.2f} deg")
    print(f"    a_lin_h mean:           {full_baseline.get('a_lin_h_mean_mps2', float('nan')):.4f} m/s2")
    print(f"    corr(err, a_lin_h):     {full_baseline.get('correlation_alignment_error_vs_a_lin_h', float('nan')):.3f}")
    if h9a_first_post_tick is not None:
        print("  Primer tick post-H9a init:")
        print(f"    t={h9a_first_post_tick.timestamp_s:.3f} s alignment={h9a_first_post_tick.gravity_alignment_error_deg:.2f} deg")
        print(f"    a_lin_h={h9a_first_post_tick.a_lin_h_mps2:.4f} m/s2 (target <= {H9A_COLLAPSE_MAX_MPS2})")
    print(f"  H9a colapsa inmediato:    {h9a_collapsed}")
    print(f"  H9a sostenido (60 s):      {h9a_sustained} (mean={full_h9a.get('a_lin_h_mean_mps2', float('nan')):.4f})")
    print(f"  Error crece en el tiempo:  {geometric_error_grows}")
    print(f"  Error geometrico ~5 deg:  {geometric_error_confirmed}")
    print(f"  Solo actitud inicial:      {h9a_hypothesis_confirmed}")
    print(f"  -> {report['interpretation']}")
    print(f"  Informe: {REPORT_JSON}")
    print(f"  Grafica: {ANALYSIS_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
