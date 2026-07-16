#!/usr/bin/env python3
"""H9c - Referencia externa de actitud via Orientation.csv (todo el replay).

Compara R_bn_EKF vs R_bn_ref (Orientation + offset de montaje estatico):
  delta_roll, delta_pitch, delta_yaw vs a_lin_h

Prueba barata de crucero (Location.csv):
  tramo recto, velocidad ~constante, sin acelerar (15-20 s).
  Si a_body ~ gravedad otra vez:
    - error accel-based desaparece pero delta_orient persiste -> actitud
    - ambos desaparecen -> el acelerometro no era referencia valida en dinamica

Elimina la ambiguedad de H9b donde accel deja de ser ground truth en movimiento.
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
DEFAULT_INPUT_DIR = REPO_ROOT / "data" / "real_run"
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"

EKF_CSV = BENCH_DIR / "h9c_ekf_state.csv"
ACCEL_GRAV_CSV = BENCH_DIR / "h9c_accel_gravity.csv"
MERGED_CSV = BENCH_DIR / "h9c_orientation_merged.csv"
REPORT_JSON = BENCH_DIR / "h9c_orientation_ref_report.json"
ANALYSIS_PNG = BENCH_DIR / "h9c_orientation_ref_analysis.png"

PREDICT_ONLY_END_S = 60.0
STATIC_OFFSET_END_S = 2.0
MOTION_ONSET_END_S = 10.0
GRAVITY_MPS2 = 9.80665

CRUISE_MIN_SPEED_MPS = 5.0
CRUISE_MAX_SPEED_STD_MPS = 0.65
CRUISE_MAX_BEARING_STD_DEG = 8.0
CRUISE_MAX_ACCEL_MPS2 = 0.35
CRUISE_MIN_DURATION_S = 10.0
CRUISE_TARGET_DURATION_S = 18.0

from analyze_real_run import (  # noqa: E402
    discover_t0_ns,
    estimate_mount_offset_deg,
    interpolate_series,
    load_orientation,
    resolve_orientation_path,
    resolve_replay_path,
    wrap_angle_deg,
)
from run_h8_propagation_audit import (  # noqa: E402
    PropagationSample,
    a_lin_horizontal,
    ensure_calibration,
    load_propagation_csv,
)


@dataclass
class LocationSample:
    timestamp_s: float
    speed_mps: float
    bearing_deg: float


@dataclass
class MergedSample:
    timestamp_s: float
    roll_ekf_deg: float
    pitch_ekf_deg: float
    yaw_ekf_deg: float
    roll_ref_deg: float
    pitch_ref_deg: float
    yaw_ref_deg: float
    delta_roll_deg: float
    delta_pitch_deg: float
    delta_yaw_deg: float
    delta_tilt_mag_deg: float
    a_lin_h_mps2: float
    gps_speed_mps: float
    gravity_alignment_error_deg: float | None


def load_location_csv(path: Path, t0_ns: int | None) -> list[LocationSample]:
    rows: list[LocationSample] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            speed = raw.get("speed")
            bearing = raw.get("bearing")
            if speed is None or bearing is None:
                continue
            try:
                speed_mps = float(speed)
                bearing_deg = float(bearing)
            except ValueError:
                continue

            timestamp_s: float | None = None
            if "seconds_elapsed" in raw and raw["seconds_elapsed"]:
                timestamp_s = float(raw["seconds_elapsed"])
            elif "time" in raw and raw["time"]:
                time_ns = float(raw["time"])
                timestamp_s = (time_ns - t0_ns) * 1e-9 if t0_ns is not None else time_ns * 1e-9

            if timestamp_s is None:
                continue

            rows.append(
                LocationSample(
                    timestamp_s=timestamp_s,
                    speed_mps=speed_mps,
                    bearing_deg=bearing_deg,
                )
            )
    rows.sort(key=lambda sample: sample.timestamp_s)
    return rows


def load_gravity_alignment_csv(path: Path) -> dict[float, float]:
    out: dict[float, float] = {}
    if not path.is_file():
        return out
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t_text = raw.get("timestamp_s")
            err_text = raw.get("gravity_alignment_error_deg")
            if not t_text or not err_text:
                continue
            out[float(t_text)] = float(err_text)
    return out


def nearest_gravity_error(t: float, gravity_map: dict[float, float]) -> float | None:
    if not gravity_map:
        return None
    keys = np.array(sorted(gravity_map.keys()), dtype=float)
    idx = int(np.argmin(np.abs(keys - t)))
    if abs(keys[idx] - t) > 0.05:
        return None
    return gravity_map[float(keys[idx])]


def run_h9c_replay(replay_csv: Path, replay_exe: Path, calibration: Path) -> None:
    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(BENCH_DIR / "h9c_replay_output.csv"),
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
        "--h8-propagation-audit-csv",
        str(EKF_CSV),
        "--h9a-gravity-alignment-audit-csv",
        str(ACCEL_GRAV_CSV),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def merge_ekf_orientation(
    ekf_samples: list[PropagationSample],
    orientation_path: Path,
    input_dir: Path | None,
    gravity_map: dict[float, float],
) -> list[MergedSample]:
    t0_ns = discover_t0_ns(input_dir)
    orient = load_orientation(orientation_path, t0_ns)
    if not orient:
        raise ValueError("Orientation.csv vacio")

    times = np.array([s.timestamp_s for s in ekf_samples], dtype=float)

    roll_ekf: list[float] = []
    pitch_ekf = []
    yaw_ekf = []
    with EKF_CSV.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            roll_ekf.append(float(raw["roll_deg"]))
            pitch_ekf.append(float(raw["pitch_deg"]))
            yaw_ekf.append(float(raw["yaw_deg"]))
    roll_ekf = np.array(roll_ekf, dtype=float)
    pitch_ekf = np.array(pitch_ekf, dtype=float)
    yaw_ekf = np.array(yaw_ekf, dtype=float)

    o_times = np.array([s.timestamp_s for s in orient], dtype=float)
    roll_o = np.array([s.roll_deg for s in orient], dtype=float)
    pitch_o = np.array([s.pitch_deg for s in orient], dtype=float)
    yaw_o = np.array([s.yaw_deg for s in orient], dtype=float)

    roll_ref = interpolate_series(times, o_times, roll_o)
    pitch_ref = interpolate_series(times, o_times, pitch_o)
    yaw_ref = interpolate_series(times, o_times, yaw_o)

    offset_mask = times <= STATIC_OFFSET_END_S
    if not np.any(offset_mask):
        offset_mask = times <= 30.0

    roll_off, pitch_off, yaw_off = estimate_mount_offset_deg(
        roll_ref,
        pitch_ref,
        yaw_ref,
        roll_ekf,
        pitch_ekf,
        yaw_ekf,
        times,
        static_end_s=float(np.max(times[offset_mask])),
    )

    merged: list[MergedSample] = []
    for idx, sample in enumerate(ekf_samples):
        roll_aligned = roll_ref[idx] - roll_off
        pitch_aligned = pitch_ref[idx] - pitch_off
        yaw_aligned = wrap_angle_deg(yaw_ref[idx] - yaw_off)

        d_roll = float(wrap_angle_deg(roll_ekf[idx] - roll_aligned))
        d_pitch = float(wrap_angle_deg(pitch_ekf[idx] - pitch_aligned))
        d_yaw = float(wrap_angle_deg(yaw_ekf[idx] - yaw_aligned))
        tilt_mag = float(math.sqrt(d_roll * d_roll + d_pitch * d_pitch))

        merged.append(
            MergedSample(
                timestamp_s=sample.timestamp_s,
                roll_ekf_deg=float(roll_ekf[idx]),
                pitch_ekf_deg=float(pitch_ekf[idx]),
                yaw_ekf_deg=float(yaw_ekf[idx]),
                roll_ref_deg=float(roll_aligned),
                pitch_ref_deg=float(pitch_aligned),
                yaw_ref_deg=float(yaw_aligned),
                delta_roll_deg=d_roll,
                delta_pitch_deg=d_pitch,
                delta_yaw_deg=d_yaw,
                delta_tilt_mag_deg=tilt_mag,
                a_lin_h_mps2=a_lin_horizontal(sample),
                gps_speed_mps=sample.gps_speed_mps,
                gravity_alignment_error_deg=nearest_gravity_error(
                    sample.timestamp_s, gravity_map
                ),
            )
        )

    return merged


def summarize_window(samples: list[MergedSample], label: str) -> dict:
    if not samples:
        return {"label": label, "samples": 0}

    a_lin = np.array([s.a_lin_h_mps2 for s in samples], dtype=float)
    d_roll = np.array([s.delta_roll_deg for s in samples], dtype=float)
    d_pitch = np.array([s.delta_pitch_deg for s in samples], dtype=float)
    d_tilt = np.array([s.delta_tilt_mag_deg for s in samples], dtype=float)
    grav = np.array(
        [s.gravity_alignment_error_deg for s in samples if s.gravity_alignment_error_deg is not None],
        dtype=float,
    )

    out = {
        "label": label,
        "samples": len(samples),
        "t_start_s": samples[0].timestamp_s,
        "t_end_s": samples[-1].timestamp_s,
        "a_lin_h_mean_mps2": float(np.mean(a_lin)),
        "a_lin_h_median_mps2": float(np.median(a_lin)),
        "delta_roll_deg_mean": float(np.mean(d_roll)),
        "delta_pitch_deg_mean": float(np.mean(d_pitch)),
        "delta_tilt_mag_deg_mean": float(np.mean(d_tilt)),
        "delta_tilt_mag_deg_at_end": float(d_tilt[-1]),
    }
    if grav.size:
        out["gravity_alignment_error_deg_mean"] = float(np.mean(grav))
        out["gravity_alignment_error_deg_median"] = float(np.median(grav))
    if len(samples) > 2:
        out["corr_delta_pitch_vs_a_lin_h"] = float(np.corrcoef(d_pitch, a_lin)[0, 1])
        out["corr_delta_tilt_vs_a_lin_h"] = float(np.corrcoef(d_tilt, a_lin)[0, 1])
        if grav.size == len(samples):
            g_arr = np.array([s.gravity_alignment_error_deg or 0.0 for s in samples])
            out["corr_gravity_err_vs_a_lin_h"] = float(np.corrcoef(g_arr, a_lin)[0, 1])
            out["corr_gravity_err_vs_delta_pitch"] = float(np.corrcoef(g_arr, d_pitch)[0, 1])
    return out


def find_cruise_windows_merged(
    samples: list[MergedSample],
    *,
    window_s: float = CRUISE_TARGET_DURATION_S,
) -> list[dict]:
    if len(samples) < 100:
        return []

    times = np.array([s.timestamp_s for s in samples], dtype=float)
    speeds = np.array([s.gps_speed_mps for s in samples], dtype=float)
    candidates: list[dict] = []

    start_idx = 0
    while start_idx < len(samples):
        t0 = times[start_idx]
        end_idx = int(np.searchsorted(times, t0 + window_s, side="right")) - 1
        if end_idx <= start_idx + 50:
            start_idx += 10
            continue

        seg_speed = speeds[start_idx : end_idx + 1]
        duration = float(times[end_idx] - times[start_idx])
        if duration < CRUISE_MIN_DURATION_S:
            start_idx += 10
            continue

        speed_mean = float(np.mean(seg_speed))
        speed_std = float(np.std(seg_speed))
        if speed_mean < CRUISE_MIN_SPEED_MPS or speed_std > CRUISE_MAX_SPEED_STD_MPS:
            start_idx += 10
            continue

        seg_times = times[start_idx : end_idx + 1]
        accel = np.diff(seg_speed) / np.maximum(np.diff(seg_times), 1e-3)
        if accel.size and float(np.max(np.abs(accel))) > CRUISE_MAX_ACCEL_MPS2:
            start_idx += 10
            continue

        candidates.append(
            {
                "t_start_s": float(times[start_idx]),
                "t_end_s": float(times[end_idx]),
                "duration_s": duration,
                "speed_mean_mps": speed_mean,
                "speed_std_mps": speed_std,
                "bearing_std_deg": float("nan"),
                "source": "merged_gps_speed",
                "score": duration - 2.0 * speed_std,
            }
        )
        start_idx = end_idx + 1

    candidates.sort(key=lambda item: item["score"], reverse=True)
    deduped: list[dict] = []
    for cand in candidates:
        if any(abs(cand["t_start_s"] - kept["t_start_s"]) < 5.0 for kept in deduped):
            continue
        deduped.append(cand)
        if len(deduped) >= 3:
            break
    return deduped


def find_cruise_windows_location(
    location: list[LocationSample],
    *,
    min_duration_s: float = CRUISE_MIN_DURATION_S,
    target_duration_s: float = CRUISE_TARGET_DURATION_S,
) -> list[dict]:
    if len(location) < 5:
        return []

    times = np.array([s.timestamp_s for s in location], dtype=float)
    speeds = np.array([s.speed_mps for s in location], dtype=float)
    bearings = np.array([s.bearing_deg for s in location], dtype=float)

    candidates: list[dict] = []
    n = len(location)
    for start in range(n):
        t0 = times[start]
        if speeds[start] < CRUISE_MIN_SPEED_MPS:
            continue
        for end in range(start + 2, n):
            t1 = times[end]
            duration = t1 - t0
            if duration < min_duration_s:
                continue
            if duration > target_duration_s + 4.0:
                break

            seg_speed = speeds[start : end + 1]
            seg_bearing = bearings[start : end + 1]
            seg_times = times[start : end + 1]

            speed_std = float(np.std(seg_speed))
            if speed_std > CRUISE_MAX_SPEED_STD_MPS:
                continue

            bearing_diff = wrap_angle_deg(np.diff(seg_bearing))
            bearing_std = float(np.std(bearing_diff))
            if bearing_std > CRUISE_MAX_BEARING_STD_DEG:
                continue

            if len(seg_times) >= 3:
                accel = np.diff(seg_speed) / np.diff(seg_times)
                if float(np.max(np.abs(accel))) > CRUISE_MAX_ACCEL_MPS2:
                    continue

            score = duration - speed_std - 0.1 * bearing_std
            candidates.append(
                {
                    "t_start_s": float(t0),
                    "t_end_s": float(t1),
                    "duration_s": float(duration),
                    "speed_mean_mps": float(np.mean(seg_speed)),
                    "speed_std_mps": speed_std,
                    "bearing_std_deg": bearing_std,
                    "source": "location_csv",
                    "score": float(score),
                }
            )

    candidates.sort(key=lambda item: item["score"], reverse=True)
    deduped: list[dict] = []
    for cand in candidates:
        if any(
            abs(cand["t_start_s"] - kept["t_start_s"]) < 5.0 for kept in deduped
        ):
            continue
        deduped.append(cand)
        if len(deduped) >= 3:
            break
    return deduped


def filter_time(samples: list[MergedSample], t0: float, t1: float) -> list[MergedSample]:
    return [s for s in samples if t0 <= s.timestamp_s <= t1]


def filter_time_max(samples: list[MergedSample], t_max: float) -> list[MergedSample]:
    return [s for s in samples if s.timestamp_s <= t_max]


def diagnose(samples: list[MergedSample], cruise_windows: list[dict]) -> dict:
    static = filter_time_max(samples, STATIC_OFFSET_END_S)
    motion = [s for s in samples if STATIC_OFFSET_END_S < s.timestamp_s <= MOTION_ONSET_END_S]
    full = samples

    static_sum = summarize_window(static, "static_0_2s")
    motion_sum = summarize_window(motion, "motion_2_10s")
    full_sum = summarize_window(full, "full_0_60s")

    static_tilt = static_sum.get("delta_tilt_mag_deg_mean", 0.0)
    motion_tilt = motion_sum.get("delta_tilt_mag_deg_mean", 0.0)
    static_alin = static_sum.get("a_lin_h_mean_mps2", 0.0)
    motion_alin = motion_sum.get("a_lin_h_mean_mps2", 0.0)
    static_grav = static_sum.get("gravity_alignment_error_deg_mean", 0.0)
    motion_grav = motion_sum.get("gravity_alignment_error_deg_mean", 0.0)

    orient_jump = motion_tilt - static_tilt
    alin_jump = motion_alin - static_alin
    grav_jump = motion_grav - static_grav if motion_grav else float("nan")

    corr_pitch_alin = motion_sum.get("corr_delta_pitch_vs_a_lin_h", float("nan"))
    corr_tilt_alin = motion_sum.get("corr_delta_tilt_vs_a_lin_h", float("nan"))
    corr_grav_pitch = motion_sum.get("corr_gravity_err_vs_delta_pitch", float("nan"))

    cruise_analysis: list[dict] = []
    for idx, window in enumerate(cruise_windows):
        seg = filter_time(samples, window["t_start_s"], window["t_end_s"])
        seg_sum = summarize_window(seg, f"cruise_{idx}")
        cruise_analysis.append({**window, "metrics": seg_sum})

    best_cruise = cruise_analysis[0] if cruise_analysis else None
    cruise_verdict = "no_cruise_segment_found"
    if best_cruise is not None:
        m = best_cruise["metrics"]
        grav_mean = m.get("gravity_alignment_error_deg_mean", float("nan"))
        tilt_mean = m.get("delta_tilt_mag_deg_mean", float("nan"))
        alin_mean = m.get("a_lin_h_mean_mps2", float("nan"))
        if math.isfinite(grav_mean) and math.isfinite(tilt_mean):
            if grav_mean < 1.0 and tilt_mean >= 3.0:
                cruise_verdict = "attitude_error_persists_in_cruise"
            elif grav_mean < 1.0 and tilt_mean < 1.5:
                cruise_verdict = "both_small_in_cruise"
            elif grav_mean >= 3.0 and tilt_mean < 1.5:
                cruise_verdict = "accel_metric_only_in_cruise"
            elif grav_mean >= 3.0 and tilt_mean >= 3.0:
                cruise_verdict = "both_large_in_cruise"
            else:
                cruise_verdict = "mixed_cruise"

    if (
        orient_jump >= 2.0
        and math.isfinite(corr_pitch_alin)
        and abs(corr_pitch_alin) >= 0.75
    ):
        primary = "attitude_diverges_from_orientation_when_a_lin_appears"
    elif orient_jump < 1.0 and alin_jump >= 0.5:
        primary = "a_lin_projection_not_attitude_drift"
    elif best_cruise and cruise_verdict == "attitude_error_persists_in_cruise":
        primary = "attitude_error_confirmed_in_constant_speed_cruise"
    elif best_cruise and cruise_verdict in ("both_small_in_cruise", "accel_metric_only_in_cruise"):
        primary = "dynamic_accel_contamination_was_dominant_not_attitude"
    elif best_cruise and cruise_verdict == "both_large_in_cruise":
        primary = "attitude_and_accel_metric_both_elevated_in_cruise"
    else:
        primary = "inconclusive_review_merged_csv"

    return {
        "static_0_2s": static_sum,
        "motion_2_10s": motion_sum,
        "full_0_60s": full_sum,
        "jumps_static_to_motion": {
            "delta_tilt_mag_deg": orient_jump,
            "a_lin_h_mps2": alin_jump,
            "gravity_alignment_error_deg": grav_jump,
        },
        "correlations_motion_2_10s": {
            "delta_pitch_vs_a_lin_h": corr_pitch_alin,
            "delta_tilt_vs_a_lin_h": corr_tilt_alin,
            "gravity_err_vs_delta_pitch": corr_grav_pitch,
        },
        "cruise_windows": cruise_analysis,
        "cruise_verdict": cruise_verdict,
        "likely_mechanism": primary,
    }


def write_merged_csv(samples: list[MergedSample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "timestamp_s",
                "roll_ekf_deg",
                "pitch_ekf_deg",
                "yaw_ekf_deg",
                "roll_ref_deg",
                "pitch_ref_deg",
                "yaw_ref_deg",
                "delta_roll_deg",
                "delta_pitch_deg",
                "delta_yaw_deg",
                "delta_tilt_mag_deg",
                "a_lin_h_mps2",
                "gps_speed_mps",
                "gravity_alignment_error_deg",
            ],
        )
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {
                    "timestamp_s": sample.timestamp_s,
                    "roll_ekf_deg": sample.roll_ekf_deg,
                    "pitch_ekf_deg": sample.pitch_ekf_deg,
                    "yaw_ekf_deg": sample.yaw_ekf_deg,
                    "roll_ref_deg": sample.roll_ref_deg,
                    "pitch_ref_deg": sample.pitch_ref_deg,
                    "yaw_ref_deg": sample.yaw_ref_deg,
                    "delta_roll_deg": sample.delta_roll_deg,
                    "delta_pitch_deg": sample.delta_pitch_deg,
                    "delta_yaw_deg": sample.delta_yaw_deg,
                    "delta_tilt_mag_deg": sample.delta_tilt_mag_deg,
                    "a_lin_h_mps2": sample.a_lin_h_mps2,
                    "gps_speed_mps": sample.gps_speed_mps,
                    "gravity_alignment_error_deg": sample.gravity_alignment_error_deg,
                }
            )


def plot_analysis(samples: list[MergedSample], cruise_windows: list[dict], path: Path) -> None:
    times = np.array([s.timestamp_s for s in samples], dtype=float)
    d_pitch = np.array([s.delta_pitch_deg for s in samples], dtype=float)
    d_roll = np.array([s.delta_roll_deg for s in samples], dtype=float)
    d_tilt = np.array([s.delta_tilt_mag_deg for s in samples], dtype=float)
    a_lin = np.array([s.a_lin_h_mps2 for s in samples], dtype=float)
    grav = np.array(
        [s.gravity_alignment_error_deg if s.gravity_alignment_error_deg is not None else np.nan
         for s in samples],
        dtype=float,
    )

    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
    fig.suptitle("H9c EKF vs Orientation.csv (predict-only + H9a init)", fontsize=14)

    for win in cruise_windows[:2]:
        axes[0].axvspan(win["t_start_s"], win["t_end_s"], color="#f9e79f", alpha=0.35)
    axes[0].plot(times, d_pitch, label="delta pitch", linewidth=0.8)
    axes[0].plot(times, d_roll, label="delta roll", linewidth=0.8, alpha=0.8)
    axes[0].plot(times, d_tilt, label="|delta tilt|", linewidth=0.9, color="#c0392b")
    axes[0].axvline(STATIC_OFFSET_END_S, color="#7f8c8d", linestyle=":", label="2 s")
    axes[0].set_ylabel("deg vs Orient")
    axes[0].legend(fontsize=8, ncol=2)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, a_lin, color="#8e44ad", linewidth=0.8)
    axes[1].set_ylabel("a_lin_h [m/s2]")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(times, grav, color="#27ae60", linewidth=0.8)
    axes[2].set_ylabel("accel grav err [deg]")
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(times, np.array([s.gps_speed_mps for s in samples]), color="#2980b9", linewidth=0.8)
    axes[3].set_ylabel("GPS speed [m/s]")
    axes[3].set_xlabel("Tiempo [s]")
    axes[3].grid(True, alpha=0.25)

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="H9c orientation reference audit")
    parser.add_argument("--skip-replay", action="store_true")
    parser.add_argument("--orientation", type=Path, default=None)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    args = parser.parse_args()

    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    replay_csv = resolve_replay_path(None)
    orientation_path = resolve_orientation_path(args.orientation)
    if orientation_path is None:
        print("ERROR: no se encontro Orientation.csv", file=sys.stderr)
        return 1

    ensure_calibration(DEFAULT_CALIBRATION)

    if not args.skip_replay:
        run_h9c_replay(replay_csv, DEFAULT_REPLAY_EXE, DEFAULT_CALIBRATION)

    if not EKF_CSV.is_file():
        print(f"ERROR: falta {EKF_CSV}", file=sys.stderr)
        return 1

    ekf_samples = load_propagation_csv(EKF_CSV)
    gravity_map = load_gravity_alignment_csv(ACCEL_GRAV_CSV)
    merged = merge_ekf_orientation(
        ekf_samples,
        orientation_path,
        args.input_dir if args.input_dir.is_dir() else None,
        gravity_map,
    )
    write_merged_csv(merged, MERGED_CSV)

    location_path = args.input_dir / "Location.csv"
    cruise_windows: list[dict] = find_cruise_windows_merged(merged)
    if location_path.is_file():
        t0_ns = discover_t0_ns(args.input_dir if args.input_dir.is_dir() else None)
        location = load_location_csv(location_path, t0_ns)
        loc_windows = find_cruise_windows_location(location)
        for win in loc_windows:
            if win["t_end_s"] <= PREDICT_ONLY_END_S + 1.0:
                win["source"] = "location_csv"
                cruise_windows.append(win)
    cruise_windows.sort(key=lambda item: item.get("score", 0.0), reverse=True)
    # dedupe by start time
    deduped: list[dict] = []
    for win in cruise_windows:
        if any(abs(win["t_start_s"] - kept["t_start_s"]) < 3.0 for kept in deduped):
            continue
        deduped.append(win)
    cruise_windows = deduped[:3]
    if not cruise_windows:
        # Fallback: ventana manual con velocidad relativamente estable en 0-60 s.
        cruise_windows = [
            {
                "t_start_s": 11.4,
                "t_end_s": 25.4,
                "duration_s": 14.0,
                "speed_mean_mps": 6.03,
                "speed_std_mps": 0.61,
                "bearing_std_deg": 1.44,
                "source": "manual_stable_speed_11_25s",
                "score": 0.0,
            }
        ]

    diagnosis = diagnose(merged, cruise_windows)
    plot_analysis(merged, cruise_windows, ANALYSIS_PNG)

    report = {
        "experiment": "H9c_orientation_reference_audit",
        "question": (
            "Cuando aparece a_lin_h ~0.9 m/s2, se desvia el EKF de Orientation.csv "
            "(actitud) o solo la metrica basada en acelerometro?"
        ),
        "configuration": {
            "predict_only_end_s": PREDICT_ONLY_END_S,
            "h9a_gravity_tilt_init": True,
            "reference": str(orientation_path),
            "mount_offset_window_s": STATIC_OFFSET_END_S,
        },
        "diagnosis": diagnosis,
        "interpretation": diagnosis["likely_mechanism"],
        "artifacts": {
            "merged_csv": str(MERGED_CSV),
            "ekf_csv": str(EKF_CSV),
            "accel_gravity_csv": str(ACCEL_GRAV_CSV),
            "plot_png": str(ANALYSIS_PNG),
        },
    }

    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    jumps = diagnosis["jumps_static_to_motion"]
    print("=" * 72)
    print("H9c - Orientation reference audit")
    print("=" * 72)
    print(f"  Static 0-2s  |delta_tilt| mean: {diagnosis['static_0_2s'].get('delta_tilt_mag_deg_mean', float('nan')):.3f} deg")
    print(f"  Static 0-2s  a_lin_h mean:       {diagnosis['static_0_2s'].get('a_lin_h_mean_mps2', float('nan')):.4f} m/s2")
    print(f"  Motion 2-10s |delta_tilt| mean: {diagnosis['motion_2_10s'].get('delta_tilt_mag_deg_mean', float('nan')):.3f} deg")
    print(f"  Motion 2-10s a_lin_h mean:       {diagnosis['motion_2_10s'].get('a_lin_h_mean_mps2', float('nan')):.4f} m/s2")
    print(f"  Salto tilt (2-10s):              {jumps.get('delta_tilt_mag_deg', float('nan')):.3f} deg")
    print(f"  Salto a_lin_h:                   {jumps.get('a_lin_h_mps2', float('nan')):.4f} m/s2")
    print(f"  corr(delta_pitch, a_lin) [2-10s]: {diagnosis['correlations_motion_2_10s'].get('delta_pitch_vs_a_lin_h', float('nan')):.3f}")
    if cruise_windows:
        best = cruise_windows[0]
        print(
            f"  Crucero ({best.get('source', '?')}): t={best['t_start_s']:.1f}-{best['t_end_s']:.1f}s "
            f"v={best.get('speed_mean_mps', float('nan')):.1f} m/s"
        )
        print(f"  Veredicto crucero:               {diagnosis['cruise_verdict']}")
    else:
        print("  Crucero: no se encontro tramo valido")
    print(f"  Mecanismo:                       {diagnosis['likely_mechanism']}")
    print(f"  Informe: {REPORT_JSON}")
    print(f"  Grafica: {ANALYSIS_PNG}")
    print(f"  Merged:  {MERGED_CSV}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
