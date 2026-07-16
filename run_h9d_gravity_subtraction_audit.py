#!/usr/bin/env python3
"""H9d - Auditoria cadena a_body -> R_bn -> restar gravedad -> a_lin.

Instrumenta cada eslabon durante predict-only + H9a init para localizar
donde la aceleracion longitudinal se proyecta como componente horizontal ficticia.
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

H9D_CSV = BENCH_DIR / "h9d_gravity_subtraction.csv"
H9C_MERGED = BENCH_DIR / "h9c_orientation_merged.csv"
REPORT_JSON = BENCH_DIR / "h9d_gravity_subtraction_report.json"
ANALYSIS_PNG = BENCH_DIR / "h9d_gravity_subtraction_analysis.png"

PREDICT_ONLY_END_S = 60.0
STATIC_END_S = 2.0
MOTION_END_S = 10.0
GRAVITY_MPS2 = 9.80665

from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


@dataclass
class GravityChainSample:
    timestamp_s: float
    a_body: np.ndarray
    a_corr: np.ndarray
    g_body_pred: np.ndarray
    a_nav_pre_g: np.ndarray
    a_lin: np.ndarray
    a_nav_pre_g_h: float
    a_lin_h: float
    residual_body: np.ndarray
    residual_body_h: float
    alignment_error_deg: float
    body_axis_to_nav_h: np.ndarray
    roll_deg: float
    pitch_deg: float
    gps_speed_mps: float
    h9a_applied: bool


def parse_vec3(row: dict, prefix: str) -> np.ndarray:
    return np.array(
        [float(row.get(f"{prefix}_x", 0.0) or 0.0),
         float(row.get(f"{prefix}_y", 0.0) or 0.0),
         float(row.get(f"{prefix}_z", 0.0) or 0.0)],
        dtype=float,
    )


def load_h9d_csv(path: Path) -> list[GravityChainSample]:
    rows: list[GravityChainSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t_text = raw.get("timestamp_s")
            if not t_text:
                continue
            rows.append(
                GravityChainSample(
                    timestamp_s=float(t_text),
                    a_body=parse_vec3(raw, "a_body"),
                    a_corr=parse_vec3(raw, "a_corr"),
                    g_body_pred=parse_vec3(raw, "g_body_pred"),
                    a_nav_pre_g=np.array(
                        [
                            float(raw.get("a_nav_pre_g_n") or 0.0),
                            float(raw.get("a_nav_pre_g_e") or 0.0),
                            float(raw.get("a_nav_pre_g_d") or 0.0),
                        ],
                        dtype=float,
                    ),
                    a_lin=parse_vec3(raw, "a_lin"),
                    a_nav_pre_g_h=float(raw.get("a_nav_pre_g_h") or 0.0),
                    a_lin_h=float(raw.get("a_lin_h") or 0.0),
                    residual_body=parse_vec3(raw, "residual_body"),
                    residual_body_h=float(raw.get("residual_body_h") or 0.0),
                    alignment_error_deg=float(raw.get("gravity_alignment_error_deg") or 0.0),
                    body_axis_to_nav_h=np.array(
                        [
                            float(raw.get("body_axis0_to_nav_h_mps2") or 0.0),
                            float(raw.get("body_axis1_to_nav_h_mps2") or 0.0),
                            float(raw.get("body_axis2_to_nav_h_mps2") or 0.0),
                        ],
                        dtype=float,
                    ),
                    roll_deg=float(raw.get("roll_deg") or 0.0),
                    pitch_deg=float(raw.get("pitch_deg") or 0.0),
                    gps_speed_mps=float(raw.get("gps_speed_mps") or 0.0),
                    h9a_applied=bool(int(float(raw.get("h9a_applied") or 0))),
                )
            )
    if not rows:
        raise ValueError(f"CSV vacio: {path}")
    rows.sort(key=lambda sample: sample.timestamp_s)
    return rows


def run_h9d_replay(replay_csv: Path, replay_exe: Path, calibration: Path) -> None:
    cmd = [
        str(replay_exe),
        "--input", str(replay_csv),
        "--output", str(BENCH_DIR / "h9d_replay_output.csv"),
        "--mount-mode", "calibration",
        "--mount-calibration", str(calibration),
        "--yaw-init", "zero",
        "--predict-only", "--predict-only-end-s", str(PREDICT_ONLY_END_S),
        "--h9a-gravity-tilt-init",
        "--h9d-gravity-subtraction-audit-csv", str(H9D_CSV),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def filter_window(samples: list[GravityChainSample], t_end: float) -> list[GravityChainSample]:
    return [s for s in samples if s.timestamp_s <= t_end]


def filter_between(samples: list[GravityChainSample], t0: float, t1: float) -> list[GravityChainSample]:
    return [s for s in samples if t0 <= s.timestamp_s <= t1]


def post_init(samples: list[GravityChainSample]) -> list[GravityChainSample]:
    post = [s for s in samples if s.h9a_applied]
    return post if post else samples


def summarize(samples: list[GravityChainSample], label: str) -> dict:
    if not samples:
        return {"label": label, "samples": 0}

    a_lin_h = np.array([s.a_lin_h for s in samples], dtype=float)
    a_nav_h = np.array([s.a_nav_pre_g_h for s in samples], dtype=float)
    res_h = np.array([s.residual_body_h for s in samples], dtype=float)
    align = np.array([s.alignment_error_deg for s in samples], dtype=float)
    axis_h = np.array([s.body_axis_to_nav_h for s in samples], dtype=float)
    pitch = np.array([s.pitch_deg for s in samples], dtype=float)

    out = {
        "label": label,
        "samples": len(samples),
        "t_start_s": samples[0].timestamp_s,
        "t_end_s": samples[-1].timestamp_s,
        "a_lin_h_mean_mps2": float(np.mean(a_lin_h)),
        "a_nav_pre_g_h_mean_mps2": float(np.mean(a_nav_h)),
        "residual_body_h_mean_mps2": float(np.mean(res_h)),
        "gravity_alignment_error_deg_mean": float(np.mean(align)),
        "body_axis0_to_nav_h_mean_mps2": float(np.mean(axis_h[:, 0])),
        "body_axis1_to_nav_h_mean_mps2": float(np.mean(axis_h[:, 1])),
        "body_axis2_to_nav_h_mean_mps2": float(np.mean(axis_h[:, 2])),
        "pitch_deg_mean": float(np.mean(pitch)),
    }
    if len(samples) > 2:
        out["corr_a_lin_h_vs_a_nav_pre_g_h"] = float(np.corrcoef(a_lin_h, a_nav_h)[0, 1])
        out["corr_a_lin_h_vs_alignment_error"] = float(np.corrcoef(a_lin_h, align)[0, 1])
        out["corr_a_lin_h_vs_body_axis0_nav_h"] = float(np.corrcoef(a_lin_h, axis_h[:, 0])[0, 1])
        out["corr_a_lin_h_vs_body_axis1_nav_h"] = float(np.corrcoef(a_lin_h, axis_h[:, 1])[0, 1])
        out["corr_a_lin_h_vs_body_axis2_nav_h"] = float(np.corrcoef(a_lin_h, axis_h[:, 2])[0, 1])
    return out


def load_h9c_delta_pitch(path: Path) -> dict[float, float]:
    out: dict[float, float] = {}
    if not path.is_file():
        return out
    with path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            t = raw.get("timestamp_s")
            if not t:
                continue
            out[float(t)] = abs(float(raw.get("delta_pitch_deg") or 0.0))
    return out


def diagnose(samples: list[GravityChainSample], h9c_delta_pitch: dict[float, float]) -> dict:
    post = post_init(samples)
    static = filter_window(post, STATIC_END_S)
    motion = filter_between(post, STATIC_END_S, MOTION_END_S)
    static_sum = summarize(static, "static_0_2s")
    motion_sum = summarize(motion, "motion_2_10s")
    full_sum = summarize(post, "full_0_60s")

    alin_jump = motion_sum.get("a_lin_h_mean_mps2", 0.0) - static_sum.get("a_lin_h_mean_mps2", 0.0)
    nav_h_jump = (
        motion_sum.get("a_nav_pre_g_h_mean_mps2", 0.0)
        - static_sum.get("a_nav_pre_g_h_mean_mps2", 0.0)
    )
    align_jump = (
        motion_sum.get("gravity_alignment_error_deg_mean", 0.0)
        - static_sum.get("gravity_alignment_error_deg_mean", 0.0)
    )

    axis_corrs = {
        "body_axis0": motion_sum.get("corr_a_lin_h_vs_body_axis0_nav_h", float("nan")),
        "body_axis1": motion_sum.get("corr_a_lin_h_vs_body_axis1_nav_h", float("nan")),
        "body_axis2": motion_sum.get("corr_a_lin_h_vs_body_axis2_nav_h", float("nan")),
    }
    dominant_axis = max(axis_corrs, key=lambda k: abs(axis_corrs[k]) if math.isfinite(axis_corrs[k]) else -1.0)

    corr_nav = motion_sum.get("corr_a_lin_h_vs_a_nav_pre_g_h", float("nan"))
    mechanism = "inconclusive"
    if math.isfinite(corr_nav) and corr_nav > 0.95 and nav_h_jump > 0.4:
        mechanism = "horizontal_a_nav_before_gravity_subtraction_dominates"
    elif abs(axis_corrs.get("body_axis0", 0.0)) >= 0.75:
        mechanism = "body_axis0_projects_into_nav_horizontal_via_R_bn"
    elif abs(axis_corrs.get("body_axis1", 0.0)) >= 0.75:
        mechanism = "body_axis1_projects_into_nav_horizontal_via_R_bn"
    elif align_jump > 2.0 and alin_jump > 0.4:
        mechanism = "attitude_misprojects_gravity_in_body_frame"

    h9c_corr = float("nan")
    if h9c_delta_pitch and motion:
        pairs = []
        for s in motion:
            keys = np.array(sorted(h9c_delta_pitch.keys()), dtype=float)
            idx = int(np.argmin(np.abs(keys - s.timestamp_s)))
            if abs(keys[idx] - s.timestamp_s) <= 0.05:
                pairs.append((s.a_lin_h, h9c_delta_pitch[float(keys[idx])]))
        if len(pairs) > 10:
            a = np.array([p[0] for p in pairs])
            b = np.array([p[1] for p in pairs])
            h9c_corr = float(np.corrcoef(a, b)[0, 1])

    return {
        "static_0_2s": static_sum,
        "motion_2_10s": motion_sum,
        "full_0_60s": full_sum,
        "jumps_static_to_motion": {
            "a_lin_h_mps2": alin_jump,
            "a_nav_pre_g_h_mps2": nav_h_jump,
            "gravity_alignment_error_deg": align_jump,
        },
        "body_axis_correlations_motion_2_10s": axis_corrs,
        "dominant_body_axis_in_motion": dominant_axis,
        "corr_h9c_delta_pitch_vs_a_lin_h_motion": h9c_corr,
        "likely_mechanism": mechanism,
    }


def plot_analysis(samples: list[GravityChainSample], path: Path) -> None:
    post = post_init(samples)
    times = np.array([s.timestamp_s for s in post], dtype=float)
    a_lin_h = np.array([s.a_lin_h for s in post], dtype=float)
    a_nav_h = np.array([s.a_nav_pre_g_h for s in post], dtype=float)
    align = np.array([s.alignment_error_deg for s in post], dtype=float)
    axis_h = np.array([s.body_axis_to_nav_h for s in post], dtype=float)

    fig, axes = plt.subplots(5, 1, figsize=(12, 15), sharex=True)
    fig.suptitle("H9d gravity subtraction chain (a_body -> R_bn -> a_lin)", fontsize=14)

    axes[0].plot(times, a_lin_h, label="a_lin_h", color="#8e44ad", linewidth=0.8)
    axes[0].plot(times, a_nav_h, label="a_nav_pre_g_h", color="#2980b9", linewidth=0.8)
    axes[0].axvline(STATIC_END_S, color="#7f8c8d", linestyle=":", label="2 s")
    axes[0].set_ylabel("[m/s2]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, align, color="#c0392b", linewidth=0.8)
    axes[1].set_ylabel("align err [deg]")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(times, axis_h[:, 0], label="body axis 0 -> nav H", linewidth=0.7)
    axes[2].plot(times, axis_h[:, 1], label="body axis 1 -> nav H", linewidth=0.7)
    axes[2].plot(times, axis_h[:, 2], label="body axis 2 -> nav H", linewidth=0.7)
    axes[2].set_ylabel("[m/s2]")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.25)

    g_pred = np.array([s.g_body_pred for s in post], dtype=float)
    a_corr = np.array([s.a_corr for s in post], dtype=float)
    axes[3].plot(times, a_corr[:, 0], label="a_corr x", linewidth=0.6)
    axes[3].plot(times, g_pred[:, 0], label="g_pred x", linewidth=0.6, linestyle="--")
    axes[3].plot(times, a_corr[:, 1], label="a_corr y", linewidth=0.6)
    axes[3].plot(times, g_pred[:, 1], label="g_pred y", linewidth=0.6, linestyle="--")
    axes[3].set_ylabel("body [m/s2]")
    axes[3].legend(fontsize=7, ncol=2)
    axes[3].grid(True, alpha=0.25)

    axes[4].plot(times, np.array([s.gps_speed_mps for s in post]), color="#27ae60", linewidth=0.8)
    axes[4].set_ylabel("GPS speed")
    axes[4].set_xlabel("Tiempo [s]")
    axes[4].grid(True, alpha=0.25)

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="H9d gravity subtraction chain audit")
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()

    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    replay_csv = resolve_replay_path(None)
    ensure_calibration(DEFAULT_CALIBRATION)

    if not args.skip_replay:
        run_h9d_replay(replay_csv, DEFAULT_REPLAY_EXE, DEFAULT_CALIBRATION)

    if not H9D_CSV.is_file():
        print("ERROR: falta CSV H9d", file=sys.stderr)
        return 1

    samples = load_h9d_csv(H9D_CSV)
    h9c_delta = load_h9c_delta_pitch(H9C_MERGED)
    diagnosis = diagnose(samples, h9c_delta)
    plot_analysis(samples, ANALYSIS_PNG)

    report = {
        "experiment": "H9d_gravity_subtraction_chain_audit",
        "question": "Donde a_body -> R_bn -> restar g introduce a_lin_h horizontal ficticio?",
        "diagnosis": diagnosis,
        "interpretation": diagnosis["likely_mechanism"],
        "artifacts": {"csv": str(H9D_CSV), "plot_png": str(ANALYSIS_PNG)},
    }
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    motion = diagnosis["motion_2_10s"]
    jumps = diagnosis["jumps_static_to_motion"]
    print("=" * 72)
    print("H9d - Gravity subtraction chain audit")
    print("=" * 72)
    print(f"  Static a_lin_h:     {diagnosis['static_0_2s'].get('a_lin_h_mean_mps2', float('nan')):.4f} m/s2")
    print(f"  Motion a_lin_h:     {motion.get('a_lin_h_mean_mps2', float('nan')):.4f} m/s2")
    print(f"  Salto a_lin_h:      {jumps.get('a_lin_h_mps2', float('nan')):.4f} m/s2")
    print(f"  Salto a_nav_pre_h:  {jumps.get('a_nav_pre_g_h_mps2', float('nan')):.4f} m/s2")
    print(f"  corr(a_lin_h, a_nav_pre_h) [2-10s]: {motion.get('corr_a_lin_h_vs_a_nav_pre_g_h', float('nan')):.3f}")
    print(f"  Eje body dominante: {diagnosis.get('dominant_body_axis_in_motion')}")
    print(f"  Mecanismo:          {diagnosis.get('likely_mechanism')}")
    print(f"  Informe: {REPORT_JSON}")
    print(f"  Grafica: {ANALYSIS_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
