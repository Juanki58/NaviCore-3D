#!/usr/bin/env python3
"""H9a - Inicializacion roll/pitch desde vector gravedad (acelerometro estatico).

Compara a_lin_h y deriva vs baseline H0 y vs predict-only sin H9a.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"

FULL_AUDIT_CSV = BENCH_DIR / "h9a_full_audit.csv"
PREDICT_AUDIT_CSV = BENCH_DIR / "h9a_predict_only_audit.csv"
REPORT_JSON = BENCH_DIR / "h9a_gravity_init_report.json"
ANALYSIS_PNG = BENCH_DIR / "h9a_gravity_init_analysis.png"

STATIC_PHASE_END_S = 30.0
PREDICT_ONLY_END_S = 60.0
GRAVITY_MPS2 = 9.80665

from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import (  # noqa: E402
    a_lin_horizontal,
    ensure_calibration,
    load_propagation_csv,
    summarize_phase,
)


def load_json_metric(path: Path, *keys: str) -> float | None:
    if not path.is_file():
        return None
    with path.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    node = payload
    for key in keys:
        if not isinstance(node, dict):
            return None
        node = node.get(key)
    if node is None:
        return None
    try:
        return float(node)
    except (TypeError, ValueError):
        return None


def run_replay(
    replay_csv: Path,
    replay_exe: Path,
    calibration: Path,
    output_csv: Path,
    audit_csv: Path,
    *,
    predict_only: bool,
    h9a: bool,
) -> None:
    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(output_csv),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--h8-propagation-audit-csv",
        str(audit_csv),
    ]
    if h9a:
        cmd.append("--h9a-gravity-tilt-init")
    if predict_only:
        cmd.extend(["--predict-only", "--predict-only-end-s", str(PREDICT_ONLY_END_S)])

    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def phase_stats(samples, t_end: float, constraint_mode: int | None = None) -> dict[str, float]:
    rows = [s for s in samples if s.timestamp_s <= t_end]
    if constraint_mode is not None:
        rows = [s for s in rows if s.constraint_mode == constraint_mode]
    return summarize_phase(rows, f"t<={t_end}")


def plot_comparison(
    full_samples,
    predict_samples,
    report: dict,
    path: Path,
) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=False)

    for ax, samples, title, t_end in (
        (axes[0], full_samples, "Full replay + H9a (0-30 s)", STATIC_PHASE_END_S),
        (axes[1], predict_samples, "Predict-only + H9a (0-60 s)", PREDICT_ONLY_END_S),
    ):
        window = [s for s in samples if s.timestamp_s <= t_end]
        if not window:
            continue
        times = np.array([s.timestamp_s for s in window], dtype=float)
        a_lin_h = np.array([a_lin_horizontal(s) for s in window], dtype=float)
        ax.plot(times, a_lin_h, linewidth=0.7, color="#c0392b")
        ax.axhline(report["g_sin_3deg_mps2"], color="#7f8c8d", linestyle="--", label="g*sin(3 deg)")
        ax.set_title(title)
        ax.set_ylabel("a_lin_h [m/s2]")
        ax.set_xlabel("Tiempo [s]")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8)

    fig.suptitle("H9a gravity tilt init vs baselines", fontsize=14)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="H9a gravity tilt initialization experiment")
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()

    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    replay_csv = resolve_replay_path(None)
    ensure_calibration(DEFAULT_CALIBRATION)

    if not args.skip_replay:
        run_replay(
            replay_csv,
            DEFAULT_REPLAY_EXE,
            DEFAULT_CALIBRATION,
            BENCH_DIR / "h9a_full_output.csv",
            FULL_AUDIT_CSV,
            predict_only=False,
            h9a=True,
        )
        run_replay(
            replay_csv,
            DEFAULT_REPLAY_EXE,
            DEFAULT_CALIBRATION,
            BENCH_DIR / "h9a_predict_only_output.csv",
            PREDICT_AUDIT_CSV,
            predict_only=True,
            h9a=True,
        )

    if not FULL_AUDIT_CSV.is_file() or not PREDICT_AUDIT_CSV.is_file():
        print("ERROR: faltan CSV de auditoria H9a", file=sys.stderr)
        return 1

    full_samples = load_propagation_csv(FULL_AUDIT_CSV)
    predict_samples = load_propagation_csv(PREDICT_AUDIT_CSV)

    full_static = phase_stats(full_samples, STATIC_PHASE_END_S, constraint_mode=0)
    full_first_5s = phase_stats(full_samples, 5.0, constraint_mode=0)
    predict_window = phase_stats(predict_samples, PREDICT_ONLY_END_S)
    predict_first_5s = phase_stats(predict_samples, 5.0)

    g_sin_3 = GRAVITY_MPS2 * math.sin(math.radians(3.0))

    baseline_h9_1 = load_json_metric(
        BENCH_DIR / "h9_1_tilt_diagnostic_report.json",
        "static_phase",
        "a_lin_h_mean",
    )
    baseline_predict = load_json_metric(
        BENCH_DIR / "h9_predict_only_report.json",
        "predict_only_phase",
        "a_lin_h_mean_mps2",
    )

    full_mean = float(full_static.get("a_lin_h_mean_mps2", float("nan")))
    predict_mean = float(predict_window.get("a_lin_h_mean_mps2", float("nan")))

    def improvement_pct(baseline: float | None, current: float) -> float | None:
        if baseline is None or baseline <= 1e-6:
            return None
        return 100.0 * (baseline - current) / baseline

    full_improved = full_mean < 0.15 if math.isfinite(full_mean) else False
    predict_improved = predict_mean < 0.15 if math.isfinite(predict_mean) else False

    report = {
        "experiment": "H9a_gravity_tilt_init",
        "description": "Inicializar roll/pitch desde media de acelerometro estatico antes del primer predict",
        "g_sin_3deg_mps2": g_sin_3,
        "full_replay_static_30s": full_static,
        "full_replay_first_5s": full_first_5s,
        "predict_only_60s": predict_window,
        "predict_only_first_5s": predict_first_5s,
        "baselines": {
            "h9_1_a_lin_h_mean_mps2": baseline_h9_1,
            "predict_only_a_lin_h_mean_mps2": baseline_predict,
        },
        "improvement_pct": {
            "full_vs_h9_1": improvement_pct(baseline_h9_1, full_mean),
            "predict_vs_baseline": improvement_pct(baseline_predict, predict_mean),
        },
        "success_criteria": {
            "full_static_a_lin_h_below_0_15": full_improved,
            "predict_only_a_lin_h_below_0_15": predict_improved,
        },
        "hypothesis_confirmed": full_improved or predict_improved,
        "interpretation": (
            "Init desde gravedad reduce a_lin_h por debajo de 0.15 m/s2: el leak era tilt inicial H0."
            if (full_improved or predict_improved)
            else (
                "Init desde gravedad no elimina a_lin_h: revisar convencion accel/body, "
                "orden de montaje vs tilt, o error distinto a roll/pitch inicial."
            )
        ),
        "artifacts": {
            "full_audit_csv": str(FULL_AUDIT_CSV),
            "predict_audit_csv": str(PREDICT_AUDIT_CSV),
            "plot_png": str(ANALYSIS_PNG),
        },
    }

    plot_comparison(full_samples, predict_samples, report, ANALYSIS_PNG)

    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    print("=" * 72)
    print("H9a - Gravity tilt initialization")
    print("=" * 72)
    print(f"  Full replay static (0-30 s) a_lin_h mean: {full_mean:.4f} m/s2")
    print(f"  Full replay first 5 s a_lin_h mean:       {full_first_5s.get('a_lin_h_mean_mps2', float('nan')):.4f} m/s2")
    if baseline_h9_1 is not None:
        print(f"  H9.1 baseline (H0 init):                {baseline_h9_1:.4f} m/s2")
    print(f"  Predict-only (0-60 s) a_lin_h mean:       {predict_mean:.4f} m/s2")
    if baseline_predict is not None:
        print(f"  Predict-only baseline (sin H9a):          {baseline_predict:.4f} m/s2")
    print(f"  g*sin(3 deg) ref:                         {g_sin_3:.4f} m/s2")
    print(f"  Hipotesis confirmada:                     {report['hypothesis_confirmed']}")
    print(f"  -> {report['interpretation']}")
    print(f"  Informe: {REPORT_JSON}")
    print(f"  Grafica: {ANALYSIS_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
