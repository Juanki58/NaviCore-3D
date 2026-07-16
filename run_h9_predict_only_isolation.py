#!/usr/bin/env python3
"""H9 predict-only isolation: IMU -> predict -> log, sin GPS/NHC/ZUPT (60 s).

Si a_lin_h persiste ~0.5 m/s2, el origen esta en propagacion.
Si desaparece, el problema esta en alguna actualizacion o cambio de regimen.
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
PROPAGATION_CSV = BENCH_DIR / "h9_predict_only_audit.csv"
REPORT_JSON = BENCH_DIR / "h9_predict_only_report.json"
ANALYSIS_PNG = BENCH_DIR / "h9_predict_only_analysis.png"
H9_1_REPORT = BENCH_DIR / "h9_1_tilt_diagnostic_report.json"
H8_REPORT = BENCH_DIR / "h8_propagation_audit_report.json"

PREDICT_ONLY_END_S = 60.0
GRAVITY_MPS2 = 9.80665
G_SIN_3DEG = GRAVITY_MPS2 * math.sin(math.radians(3.0))
PERSIST_THRESHOLD_MPS2 = 0.35
GONE_THRESHOLD_MPS2 = 0.15

from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import (  # noqa: E402
    a_lin_horizontal,
    ensure_calibration,
    load_propagation_csv,
    summarize_phase,
)


def run_predict_only_replay(replay_csv: Path, replay_exe: Path, calibration: Path) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")

    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(BENCH_DIR / "h9_predict_only_output.csv"),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--predict-only",
        "--predict-only-end-s",
        str(PREDICT_ONLY_END_S),
        "--h8-propagation-audit-csv",
        str(PROPAGATION_CSV),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def load_baseline_a_lin_h() -> dict[str, float | None]:
    out: dict[str, float | None] = {
        "h9_1_mean_mps2": None,
        "h9_1_median_mps2": None,
        "h8_static_mean_mps2": None,
    }
    if H9_1_REPORT.is_file():
        with H9_1_REPORT.open(encoding="utf-8") as handle:
            h9 = json.load(handle)
        sp = h9.get("static_phase", {})
        out["h9_1_mean_mps2"] = sp.get("a_lin_h_mean")
        out["h9_1_median_mps2"] = sp.get("a_lin_h_median")
    if H8_REPORT.is_file():
        with H8_REPORT.open(encoding="utf-8") as handle:
            h8 = json.load(handle)
        static = h8.get("static_phase", {})
        out["h8_static_mean_mps2"] = static.get("a_lin_h_mean_mps2")
    return out


def classify_verdict(mean_a_lin_h: float) -> str:
    if mean_a_lin_h >= PERSIST_THRESHOLD_MPS2:
        return "propagation_only"
    if mean_a_lin_h <= GONE_THRESHOLD_MPS2:
        return "updates_or_regime"
    return "inconclusive"


def verdict_text(verdict: str) -> str:
    if verdict == "propagation_only":
        return (
            "a_lin_h persiste sin GNSS/ZUPT/NHC: el leak horizontal nace en predict "
            "(R_bn, gravedad, montaje o bias en propagacion)."
        )
    if verdict == "updates_or_regime":
        return (
            "a_lin_h desaparece en predict-only: el problema esta en actualizaciones "
            "GNSS, ZUPT, NHC o transicion de regimen."
        )
    return (
        "Resultado intermedio: no conclusivo; revisar ventanas temporales o umbrales."
    )


def plot_predict_only(
    times: np.ndarray,
    a_lin_h: np.ndarray,
    vel_h: np.ndarray,
    mean_a_lin_h: float,
    verdict: str,
    path: Path,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("H9 predict-only isolation (60 s, sin GNSS/ZUPT/NHC)", fontsize=14)

    axes[0].plot(times, a_lin_h, color="#c0392b", linewidth=0.7)
    axes[0].axhline(G_SIN_3DEG, color="#7f8c8d", linestyle="--", label="g*sin(3 deg)")
    axes[0].axhline(mean_a_lin_h, color="#2980b9", linestyle=":", label=f"mean={mean_a_lin_h:.3f}")
    axes[0].set_ylabel("a_lin_h [m/s2]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, vel_h, color="#27ae60", linewidth=0.7)
    axes[1].set_ylabel("vel horizontal [m/s]")
    axes[1].grid(True, alpha=0.25)

    implied_tilt = np.rad2deg(np.arcsin(np.clip(a_lin_h / GRAVITY_MPS2, -1.0, 1.0)))
    axes[2].plot(times, implied_tilt, color="#8e44ad", linewidth=0.7)
    axes[2].axhline(3.0, color="#7f8c8d", linestyle="--", label="3 deg ref")
    axes[2].set_ylabel("tilt implied [deg]")
    axes[2].set_xlabel("Tiempo [s]")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.25)
    axes[2].text(
        0.02,
        0.95,
        f"verdict={verdict}",
        transform=axes[2].transAxes,
        fontsize=9,
        va="top",
    )

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="H9 predict-only isolation experiment")
    parser.add_argument("--skip-replay", action="store_true")
    parser.add_argument("--csv", type=Path, default=PROPAGATION_CSV)
    args = parser.parse_args()

    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    replay_csv = resolve_replay_path(None)
    ensure_calibration(DEFAULT_CALIBRATION)

    if not args.skip_replay:
        run_predict_only_replay(replay_csv, DEFAULT_REPLAY_EXE, DEFAULT_CALIBRATION)

    if not args.csv.is_file():
        print(f"ERROR: falta {args.csv}", file=sys.stderr)
        return 1

    samples = load_propagation_csv(args.csv)
    window_samples = [s for s in samples if s.timestamp_s <= PREDICT_ONLY_END_S]
    if not window_samples:
        print("ERROR: no hay muestras en ventana predict-only", file=sys.stderr)
        return 1

    stats = summarize_phase(window_samples, "predict_only_60s")
    mean_a_lin_h = float(stats.get("a_lin_h_mean_mps2", float("nan")))
    a_lin_h_arr = np.array([a_lin_horizontal(s) for s in window_samples], dtype=float)
    median_a_lin_h = float(np.median(a_lin_h_arr))
    implied_tilt_mean = math.degrees(math.asin(min(1.0, mean_a_lin_h / GRAVITY_MPS2)))

    baseline = load_baseline_a_lin_h()
    verdict = classify_verdict(mean_a_lin_h)

    times = np.array([s.timestamp_s for s in window_samples], dtype=float)
    a_lin_h = a_lin_h_arr
    vel_h = np.array(
        [math.hypot(s.vel_post[0], s.vel_post[1]) for s in window_samples],
        dtype=float,
    )
    plot_predict_only(times, a_lin_h, vel_h, mean_a_lin_h, verdict, ANALYSIS_PNG)

    stats["a_lin_h_median_mps2"] = median_a_lin_h
    report = {
        "experiment": "H9_predict_only_isolation",
        "description": "60 s IMU predict only, sin GNSS update, ZUPT ni NHC",
        "predict_only_end_s": PREDICT_ONLY_END_S,
        "samples": len(window_samples),
        "predict_only_phase": stats,
        "implied_tilt_from_mean_a_lin_deg": implied_tilt_mean,
        "g_sin_3deg_mps2": G_SIN_3DEG,
        "baseline_comparison": baseline,
        "verdict": verdict,
        "interpretation": verdict_text(verdict),
        "thresholds_mps2": {
            "persist": PERSIST_THRESHOLD_MPS2,
            "gone": GONE_THRESHOLD_MPS2,
        },
        "artifacts": {
            "audit_csv": str(args.csv),
            "plot_png": str(ANALYSIS_PNG),
        },
    }
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    print("=" * 72)
    print("H9 predict-only isolation")
    print("=" * 72)
    print(f"  Muestras (0-{PREDICT_ONLY_END_S} s): {len(window_samples)}")
    print(f"  a_lin_h mean:   {mean_a_lin_h:.4f} m/s2")
    print(f"  a_lin_h median: {median_a_lin_h:.4f} m/s2")
    print(f"  g*sin(3 deg):   {G_SIN_3DEG:.4f} m/s2")
    print(f"  tilt implied:   {implied_tilt_mean:.2f} deg")
    if baseline["h9_1_mean_mps2"] is not None:
        print(f"  H9.1 baseline:  {baseline['h9_1_mean_mps2']:.4f} m/s2 (con ZUPT)")
    if baseline["h8_static_mean_mps2"] is not None:
        print(f"  H8 static:      {baseline['h8_static_mean_mps2']:.4f} m/s2")
    print(f"  Verdict:        {verdict}")
    print(f"  -> {report['interpretation']}")
    print(f"  Informe:  {REPORT_JSON}")
    print(f"  Grafica:  {ANALYSIS_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
