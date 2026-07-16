#!/usr/bin/env python3
"""Verificacion matematica: u_body=(1,0,0) -> u_nav = R_bn * u vs heading GPS.

Sin nueva instrumentacion C++. Usa propagation_chain_audit.csv + Location.csv.
Responde: que componente de R_bn falla durante el salto 2-10 s?
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

REPO_ROOT = Path(__file__).resolve().parent
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DEFAULT_INPUT_DIR = REPO_ROOT / "data" / "real_run"

CHAIN_CSV = BENCH_DIR / "propagation_chain_audit.csv"
MERGED_CSV = BENCH_DIR / "rb_forward_heading_merged.csv"
REPORT_JSON = BENCH_DIR / "rb_forward_heading_report.json"
ANALYSIS_PNG = BENCH_DIR / "rb_forward_heading_analysis.png"

STATIC_END_S = 2.0
MOTION_END_S = 10.0
CRUISE_T0 = 11.4
CRUISE_T1 = 25.4
MIN_SPEED_FOR_HEADING_MPS = 4.0

from analyze_real_run import discover_t0_ns, interpolate_series, wrap_angle_deg  # noqa: E402
from run_h9c_orientation_ref_audit import load_location_csv  # noqa: E402


@dataclass
class ForwardHeadingSample:
    timestamp_s: float
    u_nav: np.ndarray
    u_nav_h_unit: np.ndarray
    forward_heading_deg: float
    gps_bearing_deg: float
    gps_speed_mps: float
    heading_error_deg: float
    yaw_ekf_deg: float
    roll_deg: float
    pitch_deg: float
    a_lin_h: float
    gravity_angle_deg: float


def euler321_to_dcm_bn(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """Replica ins_ekf.cpp: euler321_to_quat + quat_to_dcm_bn."""
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


def body_to_ned(dcm_bn: np.ndarray, body: np.ndarray) -> np.ndarray:
    return dcm_bn @ body


def heading_from_ned_horiz(v_n: float, v_e: float) -> float:
    deg = math.degrees(math.atan2(v_e, v_n))
    return (deg + 360.0) % 360.0


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


def build_samples(
    chain_rows: list[dict[str, float]],
    bearing_times: np.ndarray,
    bearing_vals: np.ndarray,
    speed_times: np.ndarray,
    speed_vals: np.ndarray,
) -> list[ForwardHeadingSample]:
    samples: list[ForwardHeadingSample] = []
    u_body = np.array([1.0, 0.0, 0.0], dtype=float)

    for row in chain_rows:
        t = row["timestamp_s"]
        roll = math.radians(row.get("roll_deg", 0.0))
        pitch = math.radians(row.get("pitch_deg", 0.0))
        yaw = math.radians(row.get("yaw_deg", 0.0))
        dcm = euler321_to_dcm_bn(roll, pitch, yaw)
        u_nav = body_to_ned(dcm, u_body)
        horiz_norm = math.hypot(u_nav[0], u_nav[1])
        if horiz_norm < 1e-9:
            u_h_unit = np.array([1.0, 0.0], dtype=float)
        else:
            u_h_unit = np.array([u_nav[0] / horiz_norm, u_nav[1] / horiz_norm], dtype=float)

        fwd_heading = heading_from_ned_horiz(u_nav[0], u_nav[1])
        gps_bearing = float(interpolate_series(np.array([t]), bearing_times, bearing_vals)[0])
        gps_speed = float(interpolate_series(np.array([t]), speed_times, speed_vals)[0])
        heading_err = float(wrap_angle_deg(gps_bearing - fwd_heading))

        samples.append(
            ForwardHeadingSample(
                timestamp_s=t,
                u_nav=u_nav,
                u_nav_h_unit=u_h_unit,
                forward_heading_deg=fwd_heading,
                gps_bearing_deg=gps_bearing,
                gps_speed_mps=gps_speed,
                heading_error_deg=heading_err,
                yaw_ekf_deg=row.get("yaw_deg", 0.0),
                roll_deg=row.get("roll_deg", 0.0),
                pitch_deg=row.get("pitch_deg", 0.0),
                a_lin_h=row.get("a_lin_h", 0.0),
                gravity_angle_deg=row.get("gravity_angle_deg", 0.0),
            )
        )
    return samples


def filter_window(
    samples: list[ForwardHeadingSample],
    t0: float,
    t1: float,
    *,
    min_speed: float = 0.0,
) -> list[ForwardHeadingSample]:
    return [
        s
        for s in samples
        if t0 <= s.timestamp_s <= t1 and s.gps_speed_mps >= min_speed
    ]


def circular_mean_deg(values: np.ndarray) -> float:
    radians = np.deg2rad(values)
    return float(np.rad2deg(np.arctan2(np.mean(np.sin(radians)), np.mean(np.cos(radians))))) % 360.0


def summarize(samples: list[ForwardHeadingSample], label: str) -> dict:
    if not samples:
        return {"label": label, "samples": 0}

    err = np.array([s.heading_error_deg for s in samples], dtype=float)
    yaw_err = np.array(
        [wrap_angle_deg(s.gps_bearing_deg - s.yaw_ekf_deg) for s in samples], dtype=float
    )
    alin = np.array([s.a_lin_h for s in samples], dtype=float)
    grav = np.array([s.gravity_angle_deg for s in samples], dtype=float)
    fwd = np.array([s.forward_heading_deg for s in samples], dtype=float)
    bear = np.array([s.gps_bearing_deg for s in samples], dtype=float)
    tilt_from_horiz = np.array(
        [math.degrees(math.atan2(abs(s.u_nav[2]), math.hypot(s.u_nav[0], s.u_nav[1]))) for s in samples],
        dtype=float,
    )

    out = {
        "label": label,
        "samples": len(samples),
        "heading_error_deg_mean": float(np.mean(err)),
        "heading_error_deg_median": float(np.median(err)),
        "heading_error_deg_std": float(np.std(err)),
        "yaw_vs_bearing_error_deg_mean": float(np.mean(yaw_err)),
        "forward_tilt_from_horizontal_deg_mean": float(np.mean(tilt_from_horiz)),
        "a_lin_h_mean": float(np.mean(alin)),
        "gravity_angle_deg_mean": float(np.mean(grav)),
        "gps_bearing_deg_circular_mean": circular_mean_deg(bear),
        "forward_heading_deg_circular_mean": circular_mean_deg(fwd),
    }
    if len(samples) > 2:
        out["corr_heading_error_vs_a_lin_h"] = float(np.corrcoef(err, alin)[0, 1])
        out["corr_heading_error_vs_gravity_angle"] = float(np.corrcoef(err, grav)[0, 1])
    return out


def diagnose(samples: list[ForwardHeadingSample]) -> dict:
    post = [s for s in samples if s.timestamp_s >= 1.5]
    static = filter_window(post, 0.0, STATIC_END_S, min_speed=0.0)
    motion = filter_window(post, STATIC_END_S, MOTION_END_S, min_speed=0.0)
    cruise = filter_window(post, CRUISE_T0, CRUISE_T1, min_speed=MIN_SPEED_FOR_HEADING_MPS)

    static_sum = summarize(static, "static_0_2s")
    motion_sum = summarize(motion, "motion_2_10s")
    cruise_sum = summarize(cruise, f"cruise_{CRUISE_T0}_{CRUISE_T1}s")

    def jump(key: str) -> float:
        return motion_sum.get(key, 0.0) - static_sum.get(key, 0.0)

    jumps = {
        "heading_error_deg": jump("heading_error_deg_mean"),
        "yaw_vs_bearing_error_deg": jump("yaw_vs_bearing_error_deg_mean"),
        "forward_tilt_from_horizontal_deg": jump("forward_tilt_from_horizontal_deg_mean"),
        "gravity_angle_deg": jump("gravity_angle_deg_mean"),
        "a_lin_h": jump("a_lin_h_mean"),
    }

    mechanism = "inconclusive"
    heading_jump = abs(jumps["heading_error_deg"])
    grav_jump = jumps["gravity_angle_deg"]
    alin_jump = jumps["a_lin_h"]

    # Salto 2-10 s: si gravity crece pero heading_error no, fallo en tilt de R_bn no en yaw.
    if grav_jump >= 2.0 and heading_jump < 5.0:
        mechanism = "rbn_tilt_error_not_forward_heading_during_motion_onset"
    elif heading_jump >= 5.0 and cruise_sum.get("samples", 0) > 50:
        if abs(cruise_sum.get("heading_error_deg_mean", 0.0)) >= 20.0:
            mechanism = "rbn_yaw_uninitialized_vs_gps_bearing"
        else:
            mechanism = "rbn_forward_axis_misaligned_with_vehicle_heading"
    elif heading_jump >= 5.0:
        mechanism = "rbn_forward_axis_misaligned_at_motion_onset"

    forward_vs_yaw = (
        abs(static_sum.get("heading_error_deg_mean", 0.0) - static_sum.get("yaw_vs_bearing_error_deg_mean", 0.0))
        < 2.0
    )

    return {
        "static_0_2s": static_sum,
        "motion_2_10s": motion_sum,
        "cruise_straight": cruise_sum,
        "jumps_static_to_motion": jumps,
        "forward_axis_equals_yaw_axis_in_static": forward_vs_yaw,
        "likely_mechanism": mechanism,
        "interpretation_notes": (
            "heading_error = GPS_bearing - heading(R_bn * e_x). "
            "Si crece en dinamica mientras bearing es estable, el eje longitudinal "
            "body no coincide con la direccion fisica del vehiculo segun R_bn."
        ),
    }


def write_merged_csv(samples: list[ForwardHeadingSample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "timestamp_s",
                "u_nav_n",
                "u_nav_e",
                "u_nav_d",
                "forward_heading_deg",
                "gps_bearing_deg",
                "heading_error_deg",
                "yaw_ekf_deg",
                "yaw_vs_bearing_error_deg",
                "roll_deg",
                "pitch_deg",
                "a_lin_h",
                "gravity_angle_deg",
                "gps_speed_mps",
            ],
        )
        writer.writeheader()
        for s in samples:
            writer.writerow(
                {
                    "timestamp_s": s.timestamp_s,
                    "u_nav_n": s.u_nav[0],
                    "u_nav_e": s.u_nav[1],
                    "u_nav_d": s.u_nav[2],
                    "forward_heading_deg": s.forward_heading_deg,
                    "gps_bearing_deg": s.gps_bearing_deg,
                    "heading_error_deg": s.heading_error_deg,
                    "yaw_ekf_deg": s.yaw_ekf_deg,
                    "yaw_vs_bearing_error_deg": wrap_angle_deg(s.gps_bearing_deg - s.yaw_ekf_deg),
                    "roll_deg": s.roll_deg,
                    "pitch_deg": s.pitch_deg,
                    "a_lin_h": s.a_lin_h,
                    "gravity_angle_deg": s.gravity_angle_deg,
                    "gps_speed_mps": s.gps_speed_mps,
                }
            )


def plot_analysis(samples: list[ForwardHeadingSample], path: Path) -> None:
    post = [s for s in samples if s.timestamp_s >= 1.5]
    times = np.array([s.timestamp_s for s in post], dtype=float)
    err = np.array([s.heading_error_deg for s in post], dtype=float)
    bearing = np.array([s.gps_bearing_deg for s in post], dtype=float)
    fwd = np.array([s.forward_heading_deg for s in post], dtype=float)
    alin = np.array([s.a_lin_h for s in post], dtype=float)
    grav = np.array([s.gravity_angle_deg for s in post], dtype=float)

    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
    fig.suptitle("R_bn forward axis vs GPS heading (u_nav = R_bn * e_x)", fontsize=14)

    axes[0].plot(times, bearing, label="GPS bearing", linewidth=0.8)
    axes[0].plot(times, fwd, label="heading(R_bn*ex)", linewidth=0.8, alpha=0.85)
    axes[0].axvline(STATIC_END_S, color="#7f8c8d", linestyle=":", label="2 s")
    axes[0].axvspan(CRUISE_T0, CRUISE_T1, color="#f9e79f", alpha=0.3, label="cruise")
    axes[0].set_ylabel("[deg]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, err, color="#c0392b", linewidth=0.8)
    axes[1].set_ylabel("heading error [deg]")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(times, grav, color="#2980b9", linewidth=0.8, label="gravity angle")
    axes[2].set_ylabel("[deg]")
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(times, alin, color="#8e44ad", linewidth=0.8)
    axes[3].set_ylabel("a_lin_h [m/s2]")
    axes[3].set_xlabel("Tiempo [s]")
    axes[3].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="R_bn forward axis vs GPS heading audit")
    parser.add_argument("--chain-csv", type=Path, default=CHAIN_CSV)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    args = parser.parse_args()

    if not args.chain_csv.is_file():
        print(f"ERROR: falta {args.chain_csv} (ejecutar run_propagation_chain_audit.py)", file=sys.stderr)
        return 1

    location_path = args.input_dir / "Location.csv"
    if not location_path.is_file():
        print("ERROR: falta Location.csv", file=sys.stderr)
        return 1

    chain_rows = load_chain_rows(args.chain_csv)
    t0_ns = discover_t0_ns(args.input_dir if args.input_dir.is_dir() else None)
    location = load_location_csv(location_path, t0_ns)
    loc_t = np.array([s.timestamp_s for s in location], dtype=float)
    loc_bearing = np.array([s.bearing_deg for s in location], dtype=float)
    loc_speed = np.array([s.speed_mps for s in location], dtype=float)

    samples = build_samples(chain_rows, loc_t, loc_bearing, loc_t, loc_speed)
    diagnosis = diagnose(samples)
    write_merged_csv(samples, MERGED_CSV)
    plot_analysis(samples, ANALYSIS_PNG)

    report = {
        "experiment": "rb_forward_heading_verification",
        "formula": "u_body=(1,0,0), u_nav=R_bn*u_body, heading_error=GPS_bearing-heading(u_nav_h)",
        "diagnosis": diagnosis,
        "artifacts": {
            "merged_csv": str(MERGED_CSV),
            "plot_png": str(ANALYSIS_PNG),
        },
    }
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    jumps = diagnosis["jumps_static_to_motion"]
    print("=" * 72)
    print("R_bn forward axis vs GPS heading (verificacion matematica)")
    print("=" * 72)
    print(f"  Static heading error:   {diagnosis['static_0_2s'].get('heading_error_deg_mean', float('nan')):.2f} deg")
    print(f"  Motion heading error:   {diagnosis['motion_2_10s'].get('heading_error_deg_mean', float('nan')):.2f} deg")
    print(f"  Salto heading error:    {jumps.get('heading_error_deg', float('nan')):.2f} deg")
    print(f"  Salto gravity angle:    {jumps.get('gravity_angle_deg', float('nan')):.2f} deg")
    print(f"  Salto a_lin_h:          {jumps.get('a_lin_h', float('nan')):.4f} m/s2")
    cruise = diagnosis.get("cruise_straight", {})
    if cruise.get("samples", 0):
        print(f"  Crucero ({CRUISE_T0}-{CRUISE_T1}s) heading err: {cruise.get('heading_error_deg_mean', float('nan')):.2f} deg")
        print(f"  Crucero GPS bearing:    {cruise.get('gps_bearing_deg_circular_mean', float('nan')):.1f} deg")
        print(f"  Crucero forward heading:  {cruise.get('forward_heading_deg_circular_mean', float('nan')):.1f} deg")
    print(f"  Mecanismo:              {diagnosis.get('likely_mechanism')}")
    print(f"  Informe: {REPORT_JSON}")
    print(f"  Grafica: {ANALYSIS_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
