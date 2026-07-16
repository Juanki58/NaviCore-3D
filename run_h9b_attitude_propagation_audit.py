#!/usr/bin/env python3
"""H9b - Auditoria completa de propagacion de actitud (predict-only + H9a init).

Instrumenta la cadena:
  gyro_raw -> gyro_bias -> gyro_corr -> delta_theta_int -> quat integrate
  vs observacion por gravedad: delta_theta_gravity

Objetivo: identificar que mecanismo introduce ~4.5 deg adicionales en 60 s
con actitud inicialmente correcta (H9a).
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

H9B_CSV = BENCH_DIR / "h9b_attitude_propagation.csv"
REPORT_JSON = BENCH_DIR / "h9b_attitude_propagation_report.json"
ANALYSIS_PNG = BENCH_DIR / "h9b_attitude_propagation_analysis.png"

PREDICT_ONLY_END_S = 60.0
CHEAP_CHECK_END_S = 2.0
STATIC_END_S = 30.0
MOTION_ONSET_END_S = 10.0
GRAVITY_MPS2 = 9.80665

from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


@dataclass
class PropSample:
    timestamp_s: float
    dt_s: float
    gyro_raw: np.ndarray
    gyro_bias: np.ndarray
    gyro_corr: np.ndarray
    delta_theta_int: np.ndarray
    delta_theta_int_mag_deg: float
    delta_theta_gravity: np.ndarray
    delta_theta_gravity_mag_deg: float
    delta_theta_gravity_step: np.ndarray
    delta_theta_gravity_step_mag_deg: float
    int_dot_gravity_step: float
    gravity_alignment_error_deg: float
    a_lin_h_mps2: float
    roll_after_deg: float
    pitch_after_deg: float
    h9a_applied: bool


def parse_vec3(row: dict, prefix: str) -> np.ndarray:
    return np.array(
        [
            float(row.get(f"{prefix}_x", 0.0) or 0.0),
            float(row.get(f"{prefix}_y", 0.0) or 0.0),
            float(row.get(f"{prefix}_z", 0.0) or 0.0),
        ],
        dtype=float,
    )


def load_h9b_csv(path: Path) -> list[PropSample]:
    rows: list[PropSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t_text = raw.get("timestamp_s")
            if not t_text:
                continue
            t = float(t_text)
            rows.append(
                PropSample(
                    timestamp_s=t,
                    dt_s=float(raw.get("dt_s") or 0.0),
                    gyro_raw=parse_vec3(raw, "gyro_raw"),
                    gyro_bias=parse_vec3(raw, "gyro_bias"),
                    gyro_corr=parse_vec3(raw, "gyro_corr"),
                    delta_theta_int=parse_vec3(raw, "delta_theta_int"),
                    delta_theta_int_mag_deg=float(raw.get("delta_theta_int_mag_deg") or 0.0),
                    delta_theta_gravity=parse_vec3(raw, "delta_theta_gravity"),
                    delta_theta_gravity_mag_deg=float(
                        raw.get("delta_theta_gravity_mag_deg") or 0.0
                    ),
                    delta_theta_gravity_step=parse_vec3(raw, "delta_theta_gravity_step"),
                    delta_theta_gravity_step_mag_deg=float(
                        raw.get("delta_theta_gravity_step_mag_deg") or 0.0
                    ),
                    int_dot_gravity_step=float(raw.get("delta_theta_int_vs_gravity_step_dot") or 0.0),
                    gravity_alignment_error_deg=float(
                        raw.get("gravity_alignment_error_deg") or 0.0
                    ),
                    a_lin_h_mps2=float(raw.get("a_lin_h_mps2") or 0.0),
                    roll_after_deg=float(raw.get("roll_after_deg") or 0.0),
                    pitch_after_deg=float(raw.get("pitch_after_deg") or 0.0),
                    h9a_applied=bool(int(float(raw.get("h9a_applied") or 0))),
                )
            )
    if not rows:
        raise ValueError(f"CSV vacio: {path}")
    rows.sort(key=lambda sample: sample.timestamp_s)
    return rows


def run_h9b_replay(replay_csv: Path, replay_exe: Path, calibration: Path, audit_csv: Path) -> None:
    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(BENCH_DIR / "h9b_attitude_propagation_out.csv"),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--predict-only",
        "--predict-only-end-s",
        str(PREDICT_ONLY_END_S),
        "--h9a-gravity-tilt-init",
        "--h9b-attitude-propagation-audit-csv",
        str(audit_csv),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def filter_window(samples: list[PropSample], t_end: float) -> list[PropSample]:
    return [s for s in samples if s.timestamp_s <= t_end]


def post_init_samples(samples: list[PropSample]) -> list[PropSample]:
    post = [s for s in samples if s.h9a_applied]
    return post if post else samples


def cumulative_rotation(samples: list[PropSample], field: str) -> np.ndarray:
    if field == "integrated":
        vecs = np.array([s.delta_theta_int for s in samples], dtype=float)
    else:
        vecs = np.array([s.delta_theta_gravity_step for s in samples], dtype=float)
    return np.cumsum(vecs, axis=0)


def summarize(samples: list[PropSample], label: str) -> dict:
    if not samples:
        return {"label": label, "samples": 0}

    err = np.array([s.gravity_alignment_error_deg for s in samples], dtype=float)
    int_mag = np.array([s.delta_theta_int_mag_deg for s in samples], dtype=float)
    grav_step = np.array([s.delta_theta_gravity_step_mag_deg for s in samples], dtype=float)
    dots = np.array([s.int_dot_gravity_step for s in samples], dtype=float)
    gyro_corr = np.array([s.gyro_corr for s in samples], dtype=float)
    dt = np.array([s.dt_s for s in samples], dtype=float)

    cum_int = cumulative_rotation(samples, "integrated")
    cum_obs_step = cumulative_rotation(samples, "gravity_step")
    cum_int_mag = np.linalg.norm(cum_int, axis=1)
    cum_obs_mag = np.linalg.norm(cum_obs_step, axis=1)

    valid_dots = dots[np.isfinite(dots)]
    dot_mean = float(np.mean(valid_dots)) if len(valid_dots) else float("nan")
    dot_pos_frac = float(np.mean(valid_dots > 0.0)) if len(valid_dots) else float("nan")

    return {
        "label": label,
        "samples": len(samples),
        "t_start_s": samples[0].timestamp_s,
        "t_end_s": samples[-1].timestamp_s,
        "gravity_alignment_error_deg_mean": float(np.mean(err)),
        "gravity_alignment_error_deg_median": float(np.median(err)),
        "gravity_alignment_error_deg_at_end": float(err[-1]),
        "a_lin_h_mean_mps2": float(np.mean([s.a_lin_h_mps2 for s in samples])),
        "delta_theta_int_mag_deg_mean": float(np.mean(int_mag)),
        "delta_theta_gravity_step_mag_deg_mean": float(np.mean(grav_step)),
        "gyro_corr_radps_mean": [float(x) for x in np.mean(gyro_corr, axis=0)],
        "gyro_corr_radps_std": [float(x) for x in np.std(gyro_corr, axis=0)],
        "dt_s_mean": float(np.mean(dt)),
        "dt_s_median": float(np.median(dt)),
        "cum_integrated_mag_deg_at_end": float(cum_int_mag[-1] * 180.0 / math.pi),
        "cum_observed_step_mag_deg_at_end": float(cum_obs_mag[-1] * 180.0 / math.pi),
        "int_vs_gravity_step_dot_mean": dot_mean,
        "int_vs_gravity_step_dot_positive_fraction": dot_pos_frac,
    }


def audit_conventions(samples: list[PropSample]) -> dict:
    post = post_init_samples(samples)
    gyro_corr_mean = np.mean(np.array([s.gyro_corr for s in post], dtype=float), axis=0)
    gyro_bias_mean = np.mean(np.array([s.gyro_bias for s in post], dtype=float), axis=0)
    dt_mean = float(np.mean([s.dt_s for s in post]))

    int_vecs = np.array([s.delta_theta_int for s in post], dtype=float)
    step_vecs = np.array([s.delta_theta_gravity_step for s in post], dtype=float)
    valid = np.linalg.norm(step_vecs, axis=1) > 1e-9
    if np.any(valid):
        int_n = int_vecs[valid] / np.linalg.norm(int_vecs[valid], axis=1, keepdims=True)
        step_n = step_vecs[valid] / np.linalg.norm(step_vecs[valid], axis=1, keepdims=True)
        cos_align = np.sum(int_n * step_n, axis=1)
        mean_cos = float(np.mean(cos_align))
    else:
        mean_cos = float("nan")

    max_int_step_deg = float(np.max(np.linalg.norm(int_vecs, axis=1)) * 180.0 / math.pi)
    units_suspicious = max_int_step_deg > 30.0

    return {
        "gyro_bias_mean_radps": [float(x) for x in gyro_bias_mean],
        "gyro_corr_mean_radps": [float(x) for x in gyro_corr_mean],
        "dt_mean_s": dt_mean,
        "integrated_vs_gravity_step_direction_cos_mean": mean_cos,
        "max_per_tick_delta_theta_int_deg": max_int_step_deg,
        "units_deg_rad_mix_suspected": units_suspicious,
        "notes": {
            "gyro_sign": (
                "Revisar si signo de omega produce deriva coherente con delta_theta_gravity."
                if not math.isnan(mean_cos)
                else "Sin pasos observables."
            ),
            "quaternion_order": (
                "Si cos(int, gravity_step) ~ -1 de forma sistematica, sospechar dq*q vs q*dq."
                if mean_cos < -0.5
                else "Sin evidencia fuerte de orden de cuaternion invertido."
            ),
            "gyro_frame": "Confirmar que gyro_corr esta en body frame antes de integrar.",
            "units": (
                "Posible mezcla deg/rad en integracion."
                if units_suspicious
                else "Magnitudes por tick compatibles con rad/s * s."
            ),
        },
    }


def filter_between(samples: list[PropSample], t_start: float, t_end: float) -> list[PropSample]:
    return [s for s in samples if t_start <= s.timestamp_s <= t_end]


def diagnose_mechanism(samples: list[PropSample]) -> dict:
    post = post_init_samples(samples)
    cheap = filter_window(post, CHEAP_CHECK_END_S)
    motion_onset = filter_between(post, CHEAP_CHECK_END_S, MOTION_ONSET_END_S)
    full = post

    cheap_err = cheap[-1].gravity_alignment_error_deg if cheap else float("nan")
    motion_err = motion_onset[-1].gravity_alignment_error_deg if motion_onset else float("nan")
    full_err = full[-1].gravity_alignment_error_deg if full else float("nan")
    drift_deg = full_err - cheap_err if cheap and full else float("nan")
    drift_duration = full[-1].timestamp_s - cheap[0].timestamp_s if cheap and full else float("nan")
    drift_rate_deg_s = drift_deg / drift_duration if drift_duration > 1e-6 else float("nan")
    motion_jump_deg = motion_err - cheap_err if cheap and motion_onset else float("nan")

    static_summary = summarize(cheap, "static_post_init_0_2s") if cheap else {}
    motion_summary = summarize(motion_onset, "motion_onset_2_10s") if motion_onset else {}
    full_summary = summarize(full, "post_init_0_60s")
    cos_mean = full_summary.get("int_vs_gravity_step_dot_mean", float("nan"))
    static_cos = static_summary.get("int_vs_gravity_step_dot_mean", float("nan"))
    cum_int = full_summary.get("cum_integrated_mag_deg_at_end", float("nan"))
    cum_obs = full_summary.get("cum_observed_step_mag_deg_at_end", float("nan"))

    integration_dominant = False
    mechanism = "inconclusive"
    if math.isfinite(static_cos) and static_summary.get("gravity_alignment_error_deg_at_end", 99.0) < 0.5:
        if math.isfinite(motion_jump_deg) and motion_jump_deg > 2.0:
            mechanism = "dynamic_accel_contaminates_gravity_observation"
        elif math.isfinite(cos_mean) and cos_mean > 0.05:
            integration_dominant = True
            mechanism = "gyro_integration_bias_or_convention"
        else:
            mechanism = "static_propagation_stable_no_integration_drift_detected"
    elif math.isfinite(cos_mean) and math.isfinite(drift_rate_deg_s):
        same_sign_systematic = cos_mean > 0.0 and drift_rate_deg_s > 0.01
        if same_sign_systematic:
            integration_dominant = True
            mechanism = "gyro_integration_partial_match"
        elif cos_mean < -0.1:
            mechanism = "integration_vs_observation_sign_mismatch"
        else:
            mechanism = "observation_drift_not_tracking_integration"

    return {
        "alignment_error_at_2s_deg": cheap_err,
        "alignment_error_at_10s_deg": motion_err,
        "alignment_error_at_60s_deg": full_err,
        "additional_drift_deg_2s_to_60s": drift_deg,
        "error_jump_static_to_10s_deg": motion_jump_deg,
        "drift_rate_deg_per_s_2s_to_60s": drift_rate_deg_s,
        "expected_drift_rate_from_h9a_deg_per_s": 0.075,
        "integration_matches_observation_drift": integration_dominant,
        "likely_mechanism": mechanism,
        "corr_int_step_dot_mean_full": cos_mean,
        "corr_int_step_dot_mean_static_0_2s": static_cos,
        "cum_integrated_vs_observed_ratio": (
            float(cum_int / cum_obs) if cum_obs > 1e-6 else float("nan")
        ),
        "static_window": static_summary,
        "motion_onset_window": motion_summary,
    }


def plot_analysis(samples: list[PropSample], path: Path) -> None:
    post = post_init_samples(samples)
    times = np.array([s.timestamp_s for s in post], dtype=float)
    err = np.array([s.gravity_alignment_error_deg for s in post], dtype=float)
    int_mag = np.array([s.delta_theta_int_mag_deg for s in post], dtype=float)
    grav_step = np.array([s.delta_theta_gravity_step_mag_deg for s in post], dtype=float)
    a_lin = np.array([s.a_lin_h_mps2 for s in post], dtype=float)

    cum_int = cumulative_rotation(post, "integrated")
    cum_obs = cumulative_rotation(post, "gravity_step")
    cum_int_deg = np.linalg.norm(cum_int, axis=1) * 180.0 / math.pi
    cum_obs_deg = np.linalg.norm(cum_obs, axis=1) * 180.0 / math.pi

    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle("H9b attitude propagation audit (predict-only + H9a init)", fontsize=14)

    axes[0].plot(times, err, color="#c0392b", linewidth=0.8)
    axes[0].axvline(CHEAP_CHECK_END_S, color="#7f8c8d", linestyle=":", label="2 s")
    axes[0].set_ylabel("gravity err [deg]")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(fontsize=8)

    axes[1].plot(times, int_mag, label="|dtheta_int|", color="#2980b9", linewidth=0.7)
    axes[1].plot(times, grav_step, label="|dtheta_grav step|", color="#27ae60", linewidth=0.7)
    axes[1].set_ylabel("per-tick [deg]")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(times, cum_int_deg, label="cum |dtheta_int|", color="#2980b9", linewidth=0.9)
    axes[2].plot(times, cum_obs_deg, label="cum |dtheta_grav step|", color="#27ae60", linewidth=0.9)
    axes[2].set_ylabel("cumulative [deg]")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(times, a_lin, color="#8e44ad", linewidth=0.8)
    axes[3].set_ylabel("a_lin_h [m/s2]")
    axes[3].grid(True, alpha=0.25)

    gyro_corr = np.array([s.gyro_corr for s in post], dtype=float)
    axes[4].plot(times, gyro_corr[:, 0], label="wx", linewidth=0.6)
    axes[4].plot(times, gyro_corr[:, 1], label="wy", linewidth=0.6)
    axes[4].plot(times, gyro_corr[:, 2], label="wz", linewidth=0.6)
    axes[4].set_ylabel("gyro_corr [rad/s]")
    axes[4].set_xlabel("Tiempo [s]")
    axes[4].legend(fontsize=8, ncol=3)
    axes[4].grid(True, alpha=0.25)

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="H9b attitude propagation audit")
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()

    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    replay_csv = resolve_replay_path(None)
    ensure_calibration(DEFAULT_CALIBRATION)

    if not args.skip_replay:
        run_h9b_replay(replay_csv, DEFAULT_REPLAY_EXE, DEFAULT_CALIBRATION, H9B_CSV)

    if not H9B_CSV.is_file():
        print("ERROR: falta CSV H9b", file=sys.stderr)
        return 1

    samples = load_h9b_csv(H9B_CSV)
    post = post_init_samples(samples)

    cheap = summarize(filter_window(post, CHEAP_CHECK_END_S), "post_init_first_2s")
    full = summarize(post, "post_init_0_60s")
    conventions = audit_conventions(samples)
    mechanism = diagnose_mechanism(samples)

    report = {
        "experiment": "H9b_attitude_propagation_audit",
        "question": (
            "Que mecanismo concreto hace que una actitud inicialmente correcta "
            "derive ~4.5 deg adicionales en 60 s durante propagacion pura?"
        ),
        "configuration": {
            "predict_only_end_s": PREDICT_ONLY_END_S,
            "h9a_gravity_tilt_init": True,
            "mount": "calibration/imu_mount.json",
        },
        "windows": {
            "post_init_first_2s": cheap,
            "motion_onset_2_10s": summarize(
                filter_between(post, CHEAP_CHECK_END_S, MOTION_ONSET_END_S),
                "motion_onset_2_10s",
            ),
            "post_init_0_60s": full,
        },
        "convention_audit": conventions,
        "mechanism_diagnosis": mechanism,
        "interpretation": (
            f"Mecanismo mas probable: {mechanism['likely_mechanism']}. "
            f"Estatico 0-2s: error={mechanism.get('alignment_error_at_2s_deg', float('nan')):.3f} deg. "
            f"Salto 2-10s: +{mechanism.get('error_jump_static_to_10s_deg', float('nan')):.2f} deg. "
            f"60s: {mechanism.get('alignment_error_at_60s_deg', float('nan')):.2f} deg. "
            f"Integracion vs observacion (60s): "
            f"{'coinciden' if mechanism.get('integration_matches_observation_drift') else 'no coinciden'}."
        ),
        "artifacts": {
            "csv": str(H9B_CSV),
            "plot_png": str(ANALYSIS_PNG),
        },
    }

    plot_analysis(samples, ANALYSIS_PNG)

    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    print("=" * 72)
    print("H9b - Attitude propagation audit")
    print("=" * 72)
    print(f"  Post-init error @2s:  {mechanism.get('alignment_error_at_2s_deg', float('nan')):.3f} deg")
    print(f"  Post-init error @10s: {mechanism.get('alignment_error_at_10s_deg', float('nan')):.3f} deg")
    print(f"  Post-init error @60s: {mechanism.get('alignment_error_at_60s_deg', float('nan')):.3f} deg")
    print(f"  Salto estatico->10s:  {mechanism.get('error_jump_static_to_10s_deg', float('nan')):.3f} deg")
    print(f"  Deriva 2s->60s:       {mechanism.get('additional_drift_deg_2s_to_60s', float('nan')):.3f} deg")
    print(f"  Ritmo deriva:         {mechanism.get('drift_rate_deg_per_s_2s_to_60s', float('nan')):.4f} deg/s")
    print(f"  Mecanismo:            {mechanism.get('likely_mechanism')}")
    print(f"  Int vs grav dot mean: {full.get('int_vs_gravity_step_dot_mean', float('nan')):.6e}")
    print(f"  Cum int @60s:         {full.get('cum_integrated_mag_deg_at_end', float('nan')):.3f} deg")
    print(f"  Cum obs step @60s:    {full.get('cum_observed_step_mag_deg_at_end', float('nan')):.3f} deg")
    print(f"  -> {report['interpretation']}")
    print(f"  Informe: {REPORT_JSON}")
    print(f"  Grafica: {ANALYSIS_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
