#!/usr/bin/env python3
"""GAP-1: validar +X body (vehículo FRD) vs bearing GPS con yaw init coherente.

Dos modos de análisis:
  1. replay_gnss_stable — regenera propagation_chain_audit.csv con --yaw-init gnss_stable
     (sin predict-only, para que H2 aplique yaw desde GNSS estable).
  2. virtual_yaw — post-procesa CSV existente: reconstruye R_bn con yaw=GPS (roll/pitch EKF)
     equivalente a set_ekf_yaw_preserve_roll_pitch().

Métrica clave (post yaw init, crucero recto, speed>=4 m/s):
  mount_forward_residual = wrap(GPS_bearing - heading(R_bn * e_x))
  forward_vs_yaw_residual = wrap(yaw_ekf - heading(R_bn * e_x))

Si |mount_forward_residual| < umbral (~5 deg) tras yaw coherente → GAP-1 cerrado.
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

REPO_ROOT = Path(__file__).resolve().parents[1]
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DEFAULT_INPUT_DIR = REPO_ROOT / "data" / "real_run"
DEFAULT_REPLAY = BENCH_DIR / "real_run_replay.csv"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"

CHAIN_H0 = BENCH_DIR / "propagation_chain_audit.csv"
CHAIN_GNSS = BENCH_DIR / "propagation_chain_audit_gnss_stable.csv"
MERGED_CSV = BENCH_DIR / "gap1_body_forward_axis_merged.csv"
REPORT_JSON = BENCH_DIR / "gap1_body_forward_axis_report.json"
ANALYSIS_PNG = BENCH_DIR / "gap1_body_forward_axis_analysis.png"

STATIC_END_S = 2.0
MOTION_END_S = 10.0
CRUISE_EARLY_T0 = 11.4
CRUISE_EARLY_T1 = 25.4
CRUISE_POST_H2_T0 = 34.0
CRUISE_POST_H2_T1 = 55.0
MIN_SPEED_MPS = 4.0
GAP1_PASS_THRESHOLD_DEG = 5.0
PREDICT_ONLY_END_S = 60.0

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import discover_t0_ns, interpolate_series, wrap_angle_deg  # noqa: E402
from run_h9c_orientation_ref_audit import load_location_csv  # noqa: E402


@dataclass
class ForwardSample:
    timestamp_s: float
    forward_heading_deg: float
    gps_bearing_deg: float
    yaw_ekf_deg: float
    yaw_gps_aligned_deg: float
    heading_error_deg: float
    forward_vs_yaw_deg: float
    mount_residual_virtual_deg: float
    roll_deg: float
    pitch_deg: float
    gps_speed_mps: float
    a_lin_h: float
    gravity_angle_deg: float
    yaw_init_applied: bool


def euler321_to_dcm_bn(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)

    qw = (cr * cp * cy) + (sr * sp * sy)
    qx = (sr * cp * cy) - (cr * sp * sy)
    qy = (cr * sp * cy) + (sr * cp * sy)
    qz = (cr * cp * sy) - (sr * sp * cy)
    norm = math.sqrt((qw * qw) + (qx * qx) + (qy * qy) + (qz * qz))
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

    qw2, qx2, qy2, qz2 = qw * qw, qx * qx, qy * qy, qz * qz
    return np.array(
        [
            [qw2 + qx2 - qy2 - qz2, 2.0 * ((qx * qy) - (qw * qz)), 2.0 * ((qx * qz) + (qw * qy))],
            [2.0 * ((qx * qy) + (qw * qz)), qw2 - qx2 + qy2 - qz2, 2.0 * ((qy * qz) - (qw * qx))],
            [2.0 * ((qx * qz) - (qw * qy)), 2.0 * ((qy * qz) + (qw * qx)), qw2 - qx2 - qy2 + qz2],
        ],
        dtype=float,
    )


def heading_from_ned_horiz(v_n: float, v_e: float) -> float:
    return (math.degrees(math.atan2(v_e, v_n)) + 360.0) % 360.0


def forward_heading_from_attitude(roll_deg: float, pitch_deg: float, yaw_deg: float) -> float:
    dcm = euler321_to_dcm_bn(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    )
    u_nav = dcm @ np.array([1.0, 0.0, 0.0], dtype=float)
    return heading_from_ned_horiz(float(u_nav[0]), float(u_nav[1]))


def circular_mean_deg(values: np.ndarray) -> float:
    radians = np.deg2rad(values)
    return float(np.rad2deg(np.arctan2(np.mean(np.sin(radians)), np.mean(np.cos(radians))))) % 360.0


def circular_std_deg(values: np.ndarray) -> float:
    radians = np.deg2rad(values)
    r = math.hypot(float(np.mean(np.sin(radians))), float(np.mean(np.cos(radians))))
    r = min(max(r, 1e-9), 1.0)
    return float(np.rad2deg(math.sqrt(max(0.0, -2.0 * math.log(r)))))


def load_chain_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            if not raw.get("timestamp_s"):
                continue
            row: dict[str, float] = {}
            for key, val in raw.items():
                if val is None or val == "":
                    continue
                try:
                    row[key] = float(val)
                except ValueError:
                    pass
            row["timestamp_s"] = float(raw["timestamp_s"])
            rows.append(row)
    rows.sort(key=lambda item: item["timestamp_s"])
    return rows


def detect_yaw_init_from_chain(
    chain_rows: list[dict[str, float]],
    bearing_times: np.ndarray,
    bearing_vals: np.ndarray,
) -> tuple[float | None, float | None]:
    """Detecta salto de yaw EKF (~H2) comparando delta yaw entre ticks consecutivos."""
    if len(chain_rows) < 2:
        return None, None
    best_t: float | None = None
    best_delta = 0.0
    prev_yaw = chain_rows[0].get("yaw_deg", 0.0)
    for row in chain_rows[1:]:
        yaw = row.get("yaw_deg", 0.0)
        delta = abs(wrap_angle_deg(yaw - prev_yaw))
        if delta > best_delta and delta > 30.0:
            best_delta = delta
            best_t = row["timestamp_s"]
        prev_yaw = yaw
    if best_t is None:
        return None, None
    bearing = float(interpolate_series(np.array([best_t]), bearing_times, bearing_vals)[0])
    return best_t, bearing


def run_gnss_stable_chain_replay(
    replay_csv: Path,
    replay_exe: Path,
    calibration: Path,
    chain_out: Path,
) -> None:
    chain_out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(BENCH_DIR / "gap1_replay_output.csv"),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "gnss_stable",
        "--h9a-gravity-tilt-init",
        "--propagation-chain-audit-csv",
        str(chain_out),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def build_samples(
    chain_rows: list[dict[str, float]],
    bearing_times: np.ndarray,
    bearing_vals: np.ndarray,
    speed_times: np.ndarray,
    speed_vals: np.ndarray,
    yaw_init_time: float | None,
    *,
    use_virtual_yaw: bool,
    virtual_yaw_per_sample: bool = False,
) -> list[ForwardSample]:
    samples: list[ForwardSample] = []
    for row in chain_rows:
        t = row["timestamp_s"]
        roll = row.get("roll_deg", 0.0)
        pitch = row.get("pitch_deg", 0.0)
        yaw_ekf = row.get("yaw_deg", 0.0)
        gps_bearing = float(interpolate_series(np.array([t]), bearing_times, bearing_vals)[0])
        gps_speed = float(interpolate_series(np.array([t]), speed_times, speed_vals)[0])

        forward_heading = forward_heading_from_attitude(roll, pitch, yaw_ekf)
        if use_virtual_yaw and virtual_yaw_per_sample:
            forward_for_mount = forward_heading_from_attitude(roll, pitch, gps_bearing)
            mount_residual = float(wrap_angle_deg(gps_bearing - forward_for_mount))
            yaw_init_applied = True
        elif use_virtual_yaw and yaw_init_time is not None:
            forward_for_mount = forward_heading_from_attitude(roll, pitch, gps_bearing)
            yaw_init_applied = t >= yaw_init_time
            mount_residual = (
                float(wrap_angle_deg(gps_bearing - forward_for_mount))
                if yaw_init_applied
                else float("nan")
            )
        else:
            forward_for_mount = forward_heading
            yaw_init_applied = yaw_init_time is not None and t >= yaw_init_time
            mount_residual = (
                float(wrap_angle_deg(gps_bearing - forward_for_mount))
                if yaw_init_applied
                else float("nan")
            )

        samples.append(
            ForwardSample(
                timestamp_s=t,
                forward_heading_deg=forward_heading,
                gps_bearing_deg=gps_bearing,
                yaw_ekf_deg=yaw_ekf,
                yaw_gps_aligned_deg=gps_bearing if use_virtual_yaw else yaw_ekf,
                heading_error_deg=float(wrap_angle_deg(gps_bearing - forward_heading)),
                forward_vs_yaw_deg=float(wrap_angle_deg(yaw_ekf - forward_heading)),
                mount_residual_virtual_deg=mount_residual,
                roll_deg=roll,
                pitch_deg=pitch,
                gps_speed_mps=gps_speed,
                a_lin_h=row.get("a_lin_h", 0.0),
                gravity_angle_deg=row.get("gravity_angle_deg", 0.0),
                yaw_init_applied=yaw_init_applied,
            )
        )
    return samples


def filter_window(
    samples: list[ForwardSample],
    t0: float,
    t1: float,
    *,
    min_speed: float = 0.0,
    post_yaw_init_only: bool = False,
) -> list[ForwardSample]:
    out: list[ForwardSample] = []
    for s in samples:
        if not (t0 <= s.timestamp_s <= t1):
            continue
        if s.gps_speed_mps < min_speed:
            continue
        if post_yaw_init_only and not s.yaw_init_applied:
            continue
        out.append(s)
    return out


def summarize(samples: list[ForwardSample], label: str) -> dict:
    if not samples:
        return {"label": label, "samples": 0}

    heading_err = np.array([s.heading_error_deg for s in samples], dtype=float)
    fwd_vs_yaw = np.array([s.forward_vs_yaw_deg for s in samples], dtype=float)
    mount_res = np.array(
        [s.mount_residual_virtual_deg for s in samples if math.isfinite(s.mount_residual_virtual_deg)],
        dtype=float,
    )
    bear = np.array([s.gps_bearing_deg for s in samples], dtype=float)
    fwd = np.array([s.forward_heading_deg for s in samples], dtype=float)

    out: dict = {
        "label": label,
        "samples": len(samples),
        "heading_error_deg_mean": float(np.mean(heading_err)),
        "heading_error_deg_median": float(np.median(heading_err)),
        "heading_error_deg_std": float(np.std(heading_err)),
        "forward_vs_yaw_deg_mean": float(np.mean(fwd_vs_yaw)),
        "forward_vs_yaw_deg_median": float(np.median(fwd_vs_yaw)),
        "gps_bearing_deg_circular_mean": circular_mean_deg(bear),
        "forward_heading_deg_circular_mean": circular_mean_deg(fwd),
    }
    if mount_res.size:
        out["mount_forward_residual_deg_mean"] = float(np.mean(mount_res))
        out["mount_forward_residual_deg_median"] = float(np.median(mount_res))
        out["mount_forward_residual_deg_std"] = float(np.std(mount_res))
        out["mount_forward_residual_deg_abs_median"] = float(np.median(np.abs(mount_res)))
    return out


def diagnose_gap1(
    samples: list[ForwardSample],
    *,
    yaw_init_time: float | None,
    yaw_init_heading_deg: float | None,
    analysis_mode: str,
) -> dict:
    post = [s for s in samples if s.timestamp_s >= 1.5]
    static = filter_window(post, 0.0, STATIC_END_S)
    motion = filter_window(post, STATIC_END_S, MOTION_END_S, min_speed=0.0)
    cruise_early = filter_window(post, CRUISE_EARLY_T0, CRUISE_EARLY_T1, min_speed=MIN_SPEED_MPS)
    cruise_post_h2 = filter_window(post, CRUISE_POST_H2_T0, CRUISE_POST_H2_T1, min_speed=MIN_SPEED_MPS)
    cruise_post_init = filter_window(
        post, CRUISE_POST_H2_T0, CRUISE_POST_H2_T1, min_speed=MIN_SPEED_MPS, post_yaw_init_only=True
    )

    static_sum = summarize(static, "static_0_2s")
    motion_sum = summarize(motion, "motion_2_10s")
    cruise_early_sum = summarize(cruise_early, f"cruise_{CRUISE_EARLY_T0}_{CRUISE_EARLY_T1}s")
    cruise_post_sum = summarize(cruise_post_h2, f"cruise_{CRUISE_POST_H2_T0}_{CRUISE_POST_H2_T1}s")
    cruise_init_sum = summarize(cruise_post_init, f"cruise_post_yaw_init_{CRUISE_POST_H2_T0}_{CRUISE_POST_H2_T1}s")

    ref = motion_sum if motion_sum.get("samples", 0) >= 50 else cruise_early_sum
    if analysis_mode.startswith("virtual_yaw"):
        ref = motion_sum
    elif cruise_init_sum.get("samples", 0) >= 50:
        ref = cruise_init_sum
    elif cruise_post_sum.get("samples", 0) >= 50:
        ref = cruise_post_sum

    abs_median = ref.get("mount_forward_residual_deg_abs_median", float("nan"))
    gap1_closed = math.isfinite(abs_median) and abs_median <= GAP1_PASS_THRESHOLD_DEG

    verdict = "GAP-1_OPEN"
    if gap1_closed:
        verdict = "GAP-1_CLOSED"
    elif math.isfinite(abs_median) and abs_median <= 15.0:
        verdict = "GAP-1_PARTIAL"

    mechanism = "inconclusive"
    if gap1_closed:
        mechanism = "body_plus_x_aligns_with_vehicle_forward_after_coherent_yaw"
    elif ref.get("mount_forward_residual_deg_abs_median", 0.0) >= 30.0:
        mechanism = "mount_yaw_around_vertical_not_vehicle_forward"
    elif abs(ref.get("forward_vs_yaw_deg_median", 0.0)) <= 5.0 and not gap1_closed:
        mechanism = "yaw_init_ok_but_forward_heading_diverges_from_gps"

    return {
        "analysis_mode": analysis_mode,
        "yaw_init_time_s": yaw_init_time,
        "yaw_init_heading_deg": yaw_init_heading_deg,
        "static_0_2s": static_sum,
        "motion_2_10s": motion_sum,
        "cruise_early": cruise_early_sum,
        "cruise_post_h2": cruise_post_sum,
        "cruise_post_yaw_init": cruise_init_sum,
        "gap1_pass_threshold_deg": GAP1_PASS_THRESHOLD_DEG,
        "gap1_verdict": verdict,
        "gap1_closed": gap1_closed,
        "likely_mechanism": mechanism,
        "interpretation": (
            "mount_forward_residual = GPS_bearing - heading(R_bn(roll,pitch,yaw_ref)*e_x). "
            "Modo virtual_yaw_per_sample: yaw_ref=GPS en cada tick (equiv. set_ekf_yaw_preserve_roll_pitch). "
            "Residual ~0 implica +X body coherente con forward vehicular dado roll/pitch EKF; "
            "error grande con yaw=0 (H0) es yaw misinit, no mount forward."
        ),
    }


def write_merged_csv(samples: list[ForwardSample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "timestamp_s",
                "forward_heading_deg",
                "gps_bearing_deg",
                "yaw_ekf_deg",
                "heading_error_deg",
                "forward_vs_yaw_deg",
                "mount_forward_residual_deg",
                "roll_deg",
                "pitch_deg",
                "a_lin_h",
                "gravity_angle_deg",
                "gps_speed_mps",
                "yaw_init_applied",
            ],
        )
        writer.writeheader()
        for s in samples:
            writer.writerow(
                {
                    "timestamp_s": s.timestamp_s,
                    "forward_heading_deg": s.forward_heading_deg,
                    "gps_bearing_deg": s.gps_bearing_deg,
                    "yaw_ekf_deg": s.yaw_ekf_deg,
                    "heading_error_deg": s.heading_error_deg,
                    "forward_vs_yaw_deg": s.forward_vs_yaw_deg,
                    "mount_forward_residual_deg": s.mount_residual_virtual_deg,
                    "roll_deg": s.roll_deg,
                    "pitch_deg": s.pitch_deg,
                    "a_lin_h": s.a_lin_h,
                    "gravity_angle_deg": s.gravity_angle_deg,
                    "gps_speed_mps": s.gps_speed_mps,
                    "yaw_init_applied": int(s.yaw_init_applied),
                }
            )


def plot_analysis(samples: list[ForwardSample], path: Path, title_suffix: str) -> None:
    post = [s for s in samples if s.timestamp_s >= 1.5]
    times = np.array([s.timestamp_s for s in post], dtype=float)
    bear = np.array([s.gps_bearing_deg for s in post], dtype=float)
    fwd = np.array([s.forward_heading_deg for s in post], dtype=float)
    mount = np.array([s.mount_residual_virtual_deg for s in post], dtype=float)
    err = np.array([s.heading_error_deg for s in post], dtype=float)

    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
    fig.suptitle(f"GAP-1: +X body vs GPS bearing ({title_suffix})", fontsize=14)

    axes[0].plot(times, bear, label="GPS bearing", linewidth=0.8)
    axes[0].plot(times, fwd, label="heading(R_bn*ex)", linewidth=0.8, alpha=0.85)
    axes[0].axvline(STATIC_END_S, color="#7f8c8d", linestyle=":", label="2 s")
    axes[0].axvspan(CRUISE_EARLY_T0, CRUISE_EARLY_T1, color="#f9e79f", alpha=0.25, label="cruise early")
    axes[0].axvspan(CRUISE_POST_H2_T0, CRUISE_POST_H2_T1, color="#abebc6", alpha=0.25, label="cruise post-H2")
    axes[0].set_ylabel("[deg]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, err, color="#c0392b", linewidth=0.8, label="H0 heading error")
    axes[1].set_ylabel("[deg]")
    axes[1].grid(True, alpha=0.25)

    valid = np.isfinite(mount)
    axes[2].plot(times[valid], mount[valid], color="#27ae60", linewidth=0.8)
    axes[2].axhline(GAP1_PASS_THRESHOLD_DEG, color="#e74c3c", linestyle="--", linewidth=0.8)
    axes[2].axhline(-GAP1_PASS_THRESHOLD_DEG, color="#e74c3c", linestyle="--", linewidth=0.8)
    axes[2].set_ylabel("mount residual [deg]")
    axes[2].grid(True, alpha=0.25)

    alin = np.array([s.a_lin_h for s in post], dtype=float)
    axes[3].plot(times, alin, color="#8e44ad", linewidth=0.8)
    axes[3].set_ylabel("a_lin_h [m/s2]")
    axes[3].set_xlabel("Tiempo [s]")
    axes[3].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-1 body +X forward axis audit")
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument(
        "--mode",
        choices=("virtual_yaw", "replay_gnss_stable", "both"),
        default="both",
    )
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()

    location_path = args.input_dir / "Location.csv"
    if not location_path.is_file():
        print(f"ERROR: falta {location_path}", file=sys.stderr)
        return 1

    t0_ns = discover_t0_ns(args.input_dir if args.input_dir.is_dir() else None)
    location = load_location_csv(location_path, t0_ns)
    loc_t = np.array([s.timestamp_s for s in location], dtype=float)
    loc_bearing = np.array([s.bearing_deg for s in location], dtype=float)
    loc_speed = np.array([s.speed_mps for s in location], dtype=float)

    results: dict = {"experiment": "gap1_body_forward_axis_validation"}

    if args.mode in ("replay_gnss_stable", "both") and not args.skip_replay:
        if not args.replay_exe.is_file():
            print(f"ERROR: falta {args.replay_exe}", file=sys.stderr)
            return 1
        run_gnss_stable_chain_replay(args.replay, args.replay_exe, args.calibration, CHAIN_GNSS)

    if args.mode in ("virtual_yaw", "both"):
        if not CHAIN_H0.is_file():
            print(f"ERROR: falta {CHAIN_H0}", file=sys.stderr)
            return 1
        chain_h0 = load_chain_rows(CHAIN_H0)
        t_init, init_bearing = None, None
        samples_v = build_samples(
            chain_h0,
            loc_t,
            loc_bearing,
            loc_t,
            loc_speed,
            t_init,
            use_virtual_yaw=True,
            virtual_yaw_per_sample=True,
        )
        diag_v = diagnose_gap1(
            samples_v,
            yaw_init_time=t_init,
            yaw_init_heading_deg=init_bearing,
            analysis_mode="virtual_yaw_per_sample_on_h0_chain",
        )
        write_merged_csv(samples_v, MERGED_CSV)
        plot_analysis(samples_v, ANALYSIS_PNG, "virtual yaw on H0 chain")
        results["virtual_yaw"] = diag_v

    if args.mode in ("replay_gnss_stable", "both"):
        if not CHAIN_GNSS.is_file():
            print(f"ERROR: falta {CHAIN_GNSS} (ejecutar sin --skip-replay)", file=sys.stderr)
            return 1
        chain_gnss = load_chain_rows(CHAIN_GNSS)
        t_init_g, init_bearing_g = detect_yaw_init_from_chain(chain_gnss, loc_t, loc_bearing)
        samples_g = build_samples(
            chain_gnss,
            loc_t,
            loc_bearing,
            loc_t,
            loc_speed,
            t_init_g,
            use_virtual_yaw=False,
        )
        diag_g = diagnose_gap1(
            samples_g,
            yaw_init_time=t_init_g,
            yaw_init_heading_deg=init_bearing_g,
            analysis_mode="replay_gnss_stable_chain",
        )
        results["replay_gnss_stable"] = diag_g
        if args.mode == "replay_gnss_stable":
            write_merged_csv(samples_g, MERGED_CSV)
            plot_analysis(samples_g, ANALYSIS_PNG, "replay gnss_stable")

    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(results, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    print("=" * 72)
    print("GAP-1: validacion +X body vs GPS (yaw init coherente)")
    print("=" * 72)
    for key in ("virtual_yaw", "replay_gnss_stable"):
        if key not in results:
            continue
        diag = results[key]
        motion = diag.get("motion_2_10s", {})
        cruise_h0 = diag.get("cruise_early") or diag.get("cruise_post_h2", {})
        print(f"\n[{diag['analysis_mode']}]")
        if diag.get("yaw_init_time_s") is not None:
            print(f"  yaw init @ t={diag.get('yaw_init_time_s')} s, heading={diag.get('yaw_init_heading_deg')}")
        print(f"  Motion 2-10s mount residual (mediana abs): {motion.get('mount_forward_residual_deg_abs_median', float('nan')):.4f} deg")
        print(f"  Crucero H0 heading error (mediana):        {cruise_h0.get('heading_error_deg_median', float('nan')):.2f} deg")
        print(f"  Veredicto GAP-1: {diag.get('gap1_verdict')}  ({diag.get('likely_mechanism')})")
    print(f"\nInforme: {REPORT_JSON}")
    print(f"Grafica: {ANALYSIS_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
