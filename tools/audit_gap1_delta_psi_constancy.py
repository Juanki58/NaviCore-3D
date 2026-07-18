#!/usr/bin/env python3
"""GAP-1 — constancia de Δψ = heading(R_bn·e_x) − bearing_GPS.

Criterio de éxito (no coincidencia absoluta):
  Si Δψ es ~constante (p.ej. 39°, 39.2°, 38.8°) → offset fijo de yaw de montaje;
  incorporar en calibración y GAP-1 cerrado.
  Si Δψ varía fuertemente (39°, 52°, 21°, …) → mecanismo dinámico; GAP-1 abierto.

Registra por tick: Δψ, Δheading, Δroll, Δpitch, speed, ωz, ax, ay.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DEFAULT_INPUT_DIR = REPO_ROOT / "data" / "real_run"

CHAIN_CSV = BENCH_DIR / "propagation_chain_audit.csv"
H9B_CSV = BENCH_DIR / "h9b_attitude_propagation.csv"
MERGED_CSV = BENCH_DIR / "gap1_delta_psi_merged.csv"
REPORT_JSON = BENCH_DIR / "gap1_delta_psi_constancy_report.json"
ANALYSIS_PNG = BENCH_DIR / "gap1_delta_psi_constancy_analysis.png"

STATIC_END_S = 2.0
MOTION_END_S = 10.0
CRUISE_T0 = 11.4
CRUISE_T1 = 25.4
CRUISE_STRAIGHT_T0 = 12.0
CRUISE_STRAIGHT_T1 = 24.0
MIN_SPEED_MPS = 4.0
CONSTANT_STD_PASS_DEG = 5.0
CONSTANT_RANGE_PASS_DEG = 10.0

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import discover_t0_ns, interpolate_series, wrap_angle_deg  # noqa: E402
from run_h9c_orientation_ref_audit import load_location_csv  # noqa: E402


@dataclass
class DeltaPsiSample:
    timestamp_s: float
    delta_psi_deg: float
    forward_heading_deg: float
    gps_bearing_deg: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    delta_heading_deg: float
    delta_roll_deg: float
    delta_pitch_deg: float
    gps_speed_mps: float
    omega_z_rad_s: float
    ax_body_mps2: float
    ay_body_mps2: float
    a_lin_h: float
    gravity_angle_deg: float


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


def forward_heading_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> float:
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
    r = min(max(r, 1e-12), 1.0)
    return float(np.rad2deg(math.sqrt(max(0.0, -2.0 * math.log(r)))))


def load_chain_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            if not raw.get("timestamp_s"):
                continue
            row: dict[str, float] = {"timestamp_s": float(raw["timestamp_s"])}
            for key in (
                "roll_deg", "pitch_deg", "yaw_deg", "a_body_x", "a_body_y",
                "a_lin_h", "gravity_angle_deg", "gps_speed_mps",
            ):
                if raw.get(key) not in (None, ""):
                    row[key] = float(raw[key])
            rows.append(row)
    rows.sort(key=lambda item: item["timestamp_s"])
    return rows


def load_h9b_gyro_z(path: Path) -> tuple[np.ndarray, np.ndarray]:
    times: list[float] = []
    wz: list[float] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            if not raw.get("timestamp_s"):
                continue
            times.append(float(raw["timestamp_s"]))
            wz.append(float(raw.get("gyro_corr_z") or raw.get("gyro_raw_z") or 0.0))
    return np.array(times, dtype=float), np.array(wz, dtype=float)


def build_samples(
    chain_rows: list[dict[str, float]],
    bearing_times: np.ndarray,
    bearing_vals: np.ndarray,
    speed_times: np.ndarray,
    speed_vals: np.ndarray,
    gyro_times: np.ndarray,
    gyro_wz: np.ndarray,
) -> list[DeltaPsiSample]:
    samples: list[DeltaPsiSample] = []
    prev_fwd: float | None = None
    prev_roll: float | None = None
    prev_pitch: float | None = None

    for row in chain_rows:
        t = row["timestamp_s"]
        roll = row.get("roll_deg", 0.0)
        pitch = row.get("pitch_deg", 0.0)
        yaw = row.get("yaw_deg", 0.0)
        gps_bearing = float(interpolate_series(np.array([t]), bearing_times, bearing_vals)[0])
        gps_speed = float(interpolate_series(np.array([t]), speed_times, speed_vals)[0])
        if row.get("gps_speed_mps") is not None and gps_speed <= 0.0:
            gps_speed = float(row["gps_speed_mps"])
        omega_z = float(interpolate_series(np.array([t]), gyro_times, gyro_wz)[0])

        fwd = forward_heading_deg(roll, pitch, yaw)
        delta_psi = float(wrap_angle_deg(fwd - gps_bearing))

        if prev_fwd is None:
            d_heading = 0.0
            d_roll = 0.0
            d_pitch = 0.0
        else:
            d_heading = float(wrap_angle_deg(fwd - prev_fwd))
            d_roll = roll - prev_roll
            d_pitch = pitch - prev_pitch

        prev_fwd, prev_roll, prev_pitch = fwd, roll, pitch

        samples.append(
            DeltaPsiSample(
                timestamp_s=t,
                delta_psi_deg=delta_psi,
                forward_heading_deg=fwd,
                gps_bearing_deg=gps_bearing,
                roll_deg=roll,
                pitch_deg=pitch,
                yaw_deg=yaw,
                delta_heading_deg=d_heading,
                delta_roll_deg=d_roll,
                delta_pitch_deg=d_pitch,
                gps_speed_mps=gps_speed,
                omega_z_rad_s=omega_z,
                ax_body_mps2=row.get("a_body_x", 0.0),
                ay_body_mps2=row.get("a_body_y", 0.0),
                a_lin_h=row.get("a_lin_h", 0.0),
                gravity_angle_deg=row.get("gravity_angle_deg", 0.0),
            )
        )
    return samples


def filter_window(
    samples: list[DeltaPsiSample],
    t0: float,
    t1: float,
    *,
    min_speed: float = 0.0,
) -> list[DeltaPsiSample]:
    return [s for s in samples if t0 <= s.timestamp_s <= t1 and s.gps_speed_mps >= min_speed]


def safe_corr(a: np.ndarray, b: np.ndarray) -> float | None:
    if a.size < 3 or b.size < 3:
        return None
    if float(np.std(a)) < 1e-12 or float(np.std(b)) < 1e-12:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def summarize_window(samples: list[DeltaPsiSample], label: str) -> dict:
    if not samples:
        return {"label": label, "samples": 0}

    dpsi = np.array([s.delta_psi_deg for s in samples], dtype=float)
    mean = circular_mean_deg(dpsi)
    std = circular_std_deg(dpsi)
    dev = np.array([abs(wrap_angle_deg(v - mean)) for v in dpsi], dtype=float)
    fwd = np.array([s.forward_heading_deg for s in samples], dtype=float)
    bear = np.array([s.gps_bearing_deg for s in samples], dtype=float)
    # delta_psi + bearing = forward_heading (identidad); estable si fwd estable
    psi_plus_bear = np.array([wrap_angle_deg(s.delta_psi_deg + s.gps_bearing_deg) for s in samples], dtype=float)

    d_roll = np.array([s.delta_roll_deg for s in samples], dtype=float)
    d_pitch = np.array([s.delta_pitch_deg for s in samples], dtype=float)
    ax = np.array([s.ax_body_mps2 for s in samples], dtype=float)
    ay = np.array([s.ay_body_mps2 for s in samples], dtype=float)
    wz = np.array([s.omega_z_rad_s for s in samples], dtype=float)
    speed = np.array([s.gps_speed_mps for s in samples], dtype=float)
    alin = np.array([s.a_lin_h for s in samples], dtype=float)

    out: dict = {
        "label": label,
        "samples": len(samples),
        "delta_psi_circular_mean_deg": mean,
        "delta_psi_circular_std_deg": std,
        "delta_psi_abs_dev_median_deg": float(np.median(dev)),
        "delta_psi_abs_dev_p95_deg": float(np.percentile(dev, 95)),
        "delta_psi_range_p5_p95_deg": float(np.percentile(dpsi, 95) - np.percentile(dpsi, 5)),
        "delta_psi_is_constant": bool(std <= CONSTANT_STD_PASS_DEG and float(np.percentile(dev, 95)) <= CONSTANT_RANGE_PASS_DEG),
        "forward_heading_circular_mean_deg": circular_mean_deg(fwd),
        "forward_heading_circular_std_deg": circular_std_deg(fwd),
        "gps_bearing_circular_mean_deg": circular_mean_deg(bear),
        "gps_bearing_circular_std_deg": circular_std_deg(bear),
        "delta_psi_plus_bearing_circular_mean_deg": circular_mean_deg(psi_plus_bear),
        "delta_psi_plus_bearing_circular_std_deg": circular_std_deg(psi_plus_bear),
        "roll_deg_mean": float(np.mean([s.roll_deg for s in samples])),
        "pitch_deg_mean": float(np.mean([s.pitch_deg for s in samples])),
        "speed_mps_mean": float(np.mean(speed)),
    }

    corr = {
        "corr_delta_psi_vs_delta_pitch": safe_corr(dpsi, d_pitch),
        "corr_delta_psi_vs_ax_body": safe_corr(dpsi, ax),
        "corr_delta_psi_vs_ay_body": safe_corr(dpsi, ay),
        "corr_delta_psi_vs_omega_z": safe_corr(dpsi, wz),
        "corr_delta_psi_vs_a_lin_h": safe_corr(dpsi, alin),
        "corr_delta_pitch_vs_ax_body": safe_corr(d_pitch, ax),
        "corr_delta_pitch_vs_a_lin_h": safe_corr(d_pitch, alin),
        "corr_delta_psi_vs_delta_roll": safe_corr(dpsi, d_roll),
    }
    out["correlations"] = {k: v for k, v in corr.items() if v is not None}
    return out


def diagnose_gap1(windows: dict[str, dict]) -> dict:
    ref = windows.get("cruise_straight") or windows.get("cruise_early") or windows.get("motion_2_10s", {})
    static = windows.get("static_0_2s", {})
    motion = windows.get("motion_2_10s", {})

    std = ref.get("delta_psi_circular_std_deg", float("nan"))
    mean = ref.get("delta_psi_circular_mean_deg", float("nan"))
    is_const_straight = ref.get("delta_psi_is_constant", False)

    static_mean = static.get("delta_psi_circular_mean_deg", float("nan"))
    cruise_mean = ref.get("delta_psi_circular_mean_deg", float("nan"))
    mean_delta_across_regimes = float("nan")
    if math.isfinite(static_mean) and math.isfinite(cruise_mean):
        mean_delta_across_regimes = abs(wrap_angle_deg(cruise_mean - static_mean))

    # Si delta cambia ~como el bearing (yaw=0), no es offset de montaje independiente
    bearing_invariant = (
        math.isfinite(mean_delta_across_regimes) and mean_delta_across_regimes <= 15.0
    )
    fwd_stable_in_cruise = ref.get("forward_heading_circular_std_deg", 999.0) <= 5.0

    if is_const_straight and not bearing_invariant and fwd_stable_in_cruise:
        verdict = "GAP-1_CLOSED_YAW_INIT_REQUIRED"
        mechanism = "delta_psi_constant_in_straight_cruise_but_tracks_minus_bearing"
        action = (
            f"En crucero recto: delta_psi estable (std={std:.1f} deg, media={mean:.1f} deg). "
            "No es offset de montaje independiente del bearing: forward_heading ~ 0 deg (yaw=0). "
            "Corregir con yaw_init GPS (H2) o R_z(-delta_psi) en calibracion."
        )
        gap1_closed = True
    elif is_const_straight and bearing_invariant:
        verdict = "GAP-1_CLOSED_FIXED_MOUNT_YAW"
        mechanism = "constant_delta_psi_independent_of_bearing"
        action = f"Incorporar yaw de montaje fijo ~ {mean:.1f} deg en R_mount (Rodrigues + R_z)."
        gap1_closed = True
    elif math.isfinite(std) and std > CONSTANT_STD_PASS_DEG and motion.get("delta_psi_circular_std_deg", 0) > 30:
        verdict = "GAP-1_OPEN_DYNAMIC"
        mechanism = "delta_psi_varies_during_motion_onset"
        action = "Delta no constante en giro 2-10 s; investigar R_bn dinamico (GAP-2)."
        gap1_closed = False
    else:
        verdict = "GAP-1_PARTIAL"
        mechanism = "inconclusive"
        action = "Ampliar ventanas rectas o repetir con yaw_init aplicado."
        gap1_closed = False

    corrs = ref.get("correlations", {})
    tilt_decoupled = (
        abs(corrs.get("corr_delta_pitch_vs_ax_body", 0.0) or 0.0) > 0.5
        and abs(corrs.get("corr_delta_psi_vs_delta_pitch", 0.0) or 0.0) < 0.3
    )

    return {
        "reference_window": ref.get("label"),
        "gap1_verdict": verdict,
        "gap1_closed": gap1_closed,
        "likely_mechanism": mechanism,
        "recommended_action": action,
        "delta_psi_offset_deg_cruise_straight": mean,
        "delta_psi_offset_deg_static": static_mean,
        "delta_psi_mean_shift_static_to_cruise_deg": mean_delta_across_regimes,
        "bearing_invariant_mount_offset": bearing_invariant,
        "delta_psi_circular_std_deg": std,
        "motion_delta_psi_std_deg": motion.get("delta_psi_circular_std_deg"),
        "tilt_pattern_decoupled_from_delta_psi": tilt_decoupled,
        "interpretation": (
            "delta_psi = heading(R_bn*e_x) - GPS_bearing con actitud EKF real (predict-only, yaw=0). "
            "Constante en recto + forward_heading estable => yaw misinit calibrable. "
            "Constante con distinto bearing entre regimenes => offset mount fijo."
        ),
    }


def write_merged_csv(samples: list[DeltaPsiSample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "timestamp_s", "delta_psi_deg", "forward_heading_deg", "gps_bearing_deg",
                "roll_deg", "pitch_deg", "yaw_deg",
                "delta_heading_deg", "delta_roll_deg", "delta_pitch_deg",
                "gps_speed_mps", "omega_z_rad_s", "ax_body_mps2", "ay_body_mps2",
                "a_lin_h", "gravity_angle_deg",
            ],
        )
        writer.writeheader()
        for s in samples:
            writer.writerow(
                {
                    "timestamp_s": s.timestamp_s,
                    "delta_psi_deg": s.delta_psi_deg,
                    "forward_heading_deg": s.forward_heading_deg,
                    "gps_bearing_deg": s.gps_bearing_deg,
                    "roll_deg": s.roll_deg,
                    "pitch_deg": s.pitch_deg,
                    "yaw_deg": s.yaw_deg,
                    "delta_heading_deg": s.delta_heading_deg,
                    "delta_roll_deg": s.delta_roll_deg,
                    "delta_pitch_deg": s.delta_pitch_deg,
                    "gps_speed_mps": s.gps_speed_mps,
                    "omega_z_rad_s": s.omega_z_rad_s,
                    "ax_body_mps2": s.ax_body_mps2,
                    "ay_body_mps2": s.ay_body_mps2,
                    "a_lin_h": s.a_lin_h,
                    "gravity_angle_deg": s.gravity_angle_deg,
                }
            )


def plot_analysis(samples: list[DeltaPsiSample], windows: dict[str, dict], path: Path) -> None:
    post = [s for s in samples if s.timestamp_s >= 1.5]
    times = np.array([s.timestamp_s for s in post], dtype=float)
    dpsi = np.array([s.delta_psi_deg for s in post], dtype=float)
    d_pitch = np.array([s.delta_pitch_deg for s in post], dtype=float)
    ax = np.array([s.ax_body_mps2 for s in post], dtype=float)
    wz = np.array([s.omega_z_rad_s for s in post], dtype=float)
    speed = np.array([s.gps_speed_mps for s in post], dtype=float)
    alin = np.array([s.a_lin_h for s in post], dtype=float)

    ref = windows.get("cruise_straight") or {}
    mean = ref.get("delta_psi_circular_mean_deg", float("nan"))

    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    fig.suptitle("GAP-1: constancia de Δψ = heading(R_bn·e_x) − bearing_GPS", fontsize=14)

    axes[0].plot(times, dpsi, color="#2980b9", linewidth=0.8, label="Δψ")
    if math.isfinite(mean):
        axes[0].axhline(mean, color="#27ae60", linestyle="--", linewidth=0.9, label=f"media={mean:.1f}°")
        axes[0].axhline(mean + CONSTANT_STD_PASS_DEG, color="#e74c3c", linestyle=":", linewidth=0.7)
        axes[0].axhline(mean - CONSTANT_STD_PASS_DEG, color="#e74c3c", linestyle=":", linewidth=0.7)
    axes[0].axvspan(CRUISE_STRAIGHT_T0, CRUISE_STRAIGHT_T1, color="#f9e79f", alpha=0.35, label="crucero recto")
    axes[0].set_ylabel("Δψ [deg]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, d_pitch, color="#8e44ad", linewidth=0.8)
    axes[1].set_ylabel("Δpitch [deg/tick]")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(times, ax, color="#c0392b", linewidth=0.8, label="ax body")
    axes[2].plot(times, ay := np.array([s.ay_body_mps2 for s in post]), color="#d35400", linewidth=0.8, alpha=0.7, label="ay body")
    axes[2].set_ylabel("a_body [m/s²]")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(times, wz, color="#16a085", linewidth=0.8)
    axes[3].set_ylabel("ωz [rad/s]")
    axes[3].grid(True, alpha=0.25)

    axes[4].plot(times, speed, color="#2c3e50", linewidth=0.8, label="speed")
    axes[4].plot(times, alin, color="#7f8c8d", linewidth=0.8, alpha=0.8, label="a_lin_h")
    axes[4].set_ylabel("speed / a_lin_h")
    axes[4].set_xlabel("Tiempo [s]")
    axes[4].legend(fontsize=8)
    axes[4].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-1 Δψ constancy audit")
    parser.add_argument("--chain-csv", type=Path, default=CHAIN_CSV)
    parser.add_argument("--h9b-csv", type=Path, default=H9B_CSV)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    args = parser.parse_args()

    if not args.chain_csv.is_file():
        print(f"ERROR: falta {args.chain_csv}", file=sys.stderr)
        return 1
    location_path = args.input_dir / "Location.csv"
    if not location_path.is_file():
        print(f"ERROR: falta {location_path}", file=sys.stderr)
        return 1

    chain = load_chain_rows(args.chain_csv)
    t0_ns = discover_t0_ns(args.input_dir if args.input_dir.is_dir() else None)
    location = load_location_csv(location_path, t0_ns)
    loc_t = np.array([s.timestamp_s for s in location], dtype=float)
    loc_bearing = np.array([s.bearing_deg for s in location], dtype=float)
    loc_speed = np.array([s.speed_mps for s in location], dtype=float)

    if args.h9b_csv.is_file():
        gyro_t, gyro_wz = load_h9b_gyro_z(args.h9b_csv)
    else:
        gyro_t, gyro_wz = np.array([0.0]), np.array([0.0])

    samples = build_samples(chain, loc_t, loc_bearing, loc_t, loc_speed, gyro_t, gyro_wz)
    post = [s for s in samples if s.timestamp_s >= 1.5]

    windows = {
        "static_0_2s": summarize_window(filter_window(post, 0.0, STATIC_END_S), "static_0_2s"),
        "motion_2_10s": summarize_window(filter_window(post, STATIC_END_S, MOTION_END_S), "motion_2_10s"),
        "cruise_early": summarize_window(
            filter_window(post, CRUISE_T0, CRUISE_T1, min_speed=MIN_SPEED_MPS), "cruise_11_25s"
        ),
        "cruise_straight": summarize_window(
            filter_window(post, CRUISE_STRAIGHT_T0, CRUISE_STRAIGHT_T1, min_speed=MIN_SPEED_MPS),
            "cruise_straight_12_24s",
        ),
    }
    diagnosis = diagnose_gap1(windows)

    write_merged_csv(samples, MERGED_CSV)
    plot_analysis(samples, windows, ANALYSIS_PNG)

    report = {
        "experiment": "gap1_delta_psi_constancy",
        "formula": "delta_psi = heading(R_bn * e_x) - GPS_bearing",
        "success_criterion": (
            f"circular_std(Δψ) <= {CONSTANT_STD_PASS_DEG}° AND p95(|Δψ-mean|) <= {CONSTANT_RANGE_PASS_DEG}° "
            "→ offset fijo de yaw de montaje"
        ),
        "windows": windows,
        "diagnosis": diagnosis,
        "artifacts": {
            "merged_csv": str(MERGED_CSV),
            "plot_png": str(ANALYSIS_PNG),
        },
    }
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    ref = windows.get(diagnosis["reference_window"], {})
    print("=" * 72)
    print("GAP-1: constancia de delta_psi = heading(R_bn*e_x) - bearing_GPS")
    print("=" * 72)
    for key in ("static_0_2s", "motion_2_10s", "cruise_straight", "cruise_early"):
        w = windows.get(key, {})
        if w.get("samples", 0) == 0:
            continue
        print(f"\n  [{w['label']}] n={w['samples']}")
        print(f"    delta_psi media (circular): {w['delta_psi_circular_mean_deg']:.2f} deg")
        print(f"    delta_psi std (circular):   {w['delta_psi_circular_std_deg']:.2f} deg")
        print(f"    delta_psi p95 desv:         {w['delta_psi_abs_dev_p95_deg']:.2f} deg")
        print(f"    constante:                  {'SI' if w['delta_psi_is_constant'] else 'NO'}")
        if w.get("correlations"):
            c = w["correlations"]
            if "corr_delta_psi_vs_delta_pitch" in c:
                print(f"    corr(delta_psi, delta_pitch): {c['corr_delta_psi_vs_delta_pitch']:.3f}")
            if "corr_delta_pitch_vs_ax_body" in c:
                print(f"    corr(delta_pitch, ax):       {c['corr_delta_pitch_vs_ax_body']:.3f}")

    print(f"\n  Veredicto: {diagnosis['gap1_verdict']}")
    print(f"  Accion:    {diagnosis['recommended_action']}")
    print(f"  Informe:   {REPORT_JSON}")
    print(f"  Gráfica:   {ANALYSIS_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
