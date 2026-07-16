#!/usr/bin/env python3
"""Auditoria de cadena de referencias con ancla estatica.

Sensor -> R_mount -> Body -> Android -> NED -> EKF

Para cada enlace: ¿transformacion constante o dependiente del regimen dinamico?
Usa 0-2 s como ancla (tres mundos coinciden ~0.05 deg).
"""

from __future__ import annotations

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
DEFAULT_MOUNT = REPO_ROOT / "calibration" / "imu_mount.json"
CHAIN_CSV = BENCH_DIR / "propagation_chain_audit.csv"
MERGED_CSV = BENCH_DIR / "reference_chain_merged.csv"
REPORT_JSON = BENCH_DIR / "reference_chain_audit.json"
ANALYSIS_PNG = BENCH_DIR / "reference_chain_analysis.png"

STATIC_ANCHOR_END_S = 2.0
MOTION_END_S = 10.0
CRUISE_T0 = 11.4
CRUISE_T1 = 25.4
REGIME_JUMP_DEG = 1.0
REGIME_JUMP_MPS2 = 0.05

from analyze_real_run import (  # noqa: E402
    discover_t0_ns,
    estimate_mount_offset_deg,
    interpolate_series,
    load_orientation,
    parse_float,
    resolve_orientation_path,
)
from attitude_kinematics import (  # noqa: E402
    angle_between_deg,
    dcm_delta_angle_deg,
    euler321_to_dcm_bn,
    g_body_from_dcm,
    load_mount_matrix,
    normalize_vec,
)


@dataclass
class ChainTick:
    timestamp_s: float
    mount_residual_mps2: float
    ekf_android_tilt_deg: float
    ekf_android_drift_from_anchor_deg: float
    ekf_meas_tilt_deg: float
    ekf_meas_drift_from_anchor_deg: float
    android_meas_tilt_deg: float
    android_gravity_accel_sensor_deg: float
    android_gravity_ekf_body_deg: float
    convention_drift_deg: float
    a_lin_h: float
    gps_speed_mps: float


def load_sensor_logger_vectors(path: Path, t0_ns: int | None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times: list[float] = []
    vectors: list[np.ndarray] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV sin cabecera: {path}")
        lower = {name.lower(): name for name in reader.fieldnames}
        time_col = lower.get("time")
        seconds_col = lower.get("seconds_elapsed")
        z_col = lower.get("z")
        y_col = lower.get("y")
        x_col = lower.get("x")
        if not all([z_col, y_col, x_col]):
            raise ValueError(f"{path.name}: se requieren columnas z,y,x")

        for row in reader:
            x_val = parse_float(row.get(x_col))
            y_val = parse_float(row.get(y_col))
            z_val = parse_float(row.get(z_col))
            if None in (x_val, y_val, z_val):
                continue
            timestamp_s: float | None = None
            if time_col is not None:
                time_ns = parse_float(row.get(time_col))
                if time_ns is not None:
                    timestamp_s = (time_ns - t0_ns) * 1e-9 if t0_ns is not None else time_ns * 1e-9
            if timestamp_s is None and seconds_col is not None:
                timestamp_s = parse_float(row.get(seconds_col))
            if timestamp_s is None:
                continue
            times.append(timestamp_s)
            vectors.append(np.array([x_val, y_val, z_val], dtype=float))

    if not times:
        raise ValueError(f"Sin muestras validas en {path}")
    order = np.argsort(times)
    t_arr = np.array(times, dtype=float)[order]
    v_arr = np.stack([vectors[i] for i in order], axis=0)
    return t_arr, v_arr[:, 0], v_arr[:, 1], v_arr[:, 2]


def load_chain(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            if not raw.get("timestamp_s"):
                continue
            row: dict[str, float] = {"timestamp_s": float(raw["timestamp_s"])}
            for key, val in raw.items():
                if key == "timestamp_s" or val in (None, ""):
                    continue
                try:
                    row[key] = float(val)
                except ValueError:
                    pass
            rows.append(row)
    rows.sort(key=lambda item: item["timestamp_s"])
    return rows


def vec3(row: dict[str, float], prefix: str) -> np.ndarray:
    return np.array([row.get(f"{prefix}_x", 0.0), row.get(f"{prefix}_y", 0.0), row.get(f"{prefix}_z", 0.0)])


def window_mean(values: np.ndarray, t0: float, t1: float, times: np.ndarray) -> float:
    mask = (times >= t0) & (times <= t1)
    if not np.any(mask):
        return float("nan")
    return float(np.mean(values[mask]))


def summarize_link(
    name: str,
    description: str,
    expected: str,
    metric_deg_or_mps2: np.ndarray,
    times: np.ndarray,
    unit: str,
) -> dict:
    static_mean = window_mean(metric_deg_or_mps2, 0.0, STATIC_ANCHOR_END_S, times)
    motion_mean = window_mean(metric_deg_or_mps2, STATIC_ANCHOR_END_S, MOTION_END_S, times)
    cruise_mean = window_mean(metric_deg_or_mps2, CRUISE_T0, CRUISE_T1, times)
    jump = motion_mean - static_mean if not (math.isnan(static_mean) or math.isnan(motion_mean)) else float("nan")

    threshold = REGIME_JUMP_DEG if unit == "deg" else REGIME_JUMP_MPS2
    regime_dependent = bool(not math.isnan(jump) and abs(jump) >= threshold)

    if expected == "constant_geometric":
        verdict = "constant_geometric" if not regime_dependent else "not_constant_check_model"
    elif expected == "estimator_pair":
        verdict = "estimators_agree" if not regime_dependent else "estimators_diverge_in_dynamics"
    else:
        verdict = "regime_sensitive" if regime_dependent else "stable"

    return {
        "link": name,
        "description": description,
        "expected": expected,
        "unit": unit,
        "static_anchor_mean": static_mean,
        "motion_2_10_mean": motion_mean,
        "cruise_11_25_mean": cruise_mean,
        "jump_static_to_motion": jump,
        "regime_dependent": regime_dependent,
        "verdict": verdict,
    }


def build_chain_ticks(
    chain_rows: list[dict[str, float]],
    orientation_path: Path,
    input_dir: Path,
    mount_path: Path,
) -> list[ChainTick]:
    t0_ns = discover_t0_ns(input_dir if input_dir.is_dir() else None)
    r_mount = load_mount_matrix(mount_path)
    orient = load_orientation(orientation_path, t0_ns)

    gravity_path = input_dir / "Gravity.csv"
    total_accel_path = input_dir / "TotalAcceleration.csv"
    g_times, g_x, g_y, g_z = load_sensor_logger_vectors(gravity_path, t0_ns)
    a_times, a_x, a_y, a_z = load_sensor_logger_vectors(total_accel_path, t0_ns)

    times = np.array([r["timestamp_s"] for r in chain_rows], dtype=float)
    o_times = np.array([s.timestamp_s for s in orient], dtype=float)
    roll_o = np.array([s.roll_deg for s in orient], dtype=float)
    pitch_o = np.array([s.pitch_deg for s in orient], dtype=float)
    yaw_o = np.array([s.yaw_deg for s in orient], dtype=float)

    roll_ref = interpolate_series(times, o_times, roll_o)
    pitch_ref = interpolate_series(times, o_times, pitch_o)
    yaw_ref = interpolate_series(times, o_times, yaw_o)

    roll_ekf = np.array([r.get("roll_deg", 0.0) for r in chain_rows], dtype=float)
    pitch_ekf = np.array([r.get("pitch_deg", 0.0) for r in chain_rows], dtype=float)
    yaw_ekf = np.array([r.get("yaw_deg", 0.0) for r in chain_rows], dtype=float)

    roll_off, pitch_off, _yaw_off = estimate_mount_offset_deg(
        roll_ref, pitch_ref, yaw_ref, roll_ekf, pitch_ekf, yaw_ekf, times, static_end_s=STATIC_ANCHOR_END_S
    )

    g_grav_sensor = np.stack(
        [
            interpolate_series(times, g_times, g_x),
            interpolate_series(times, g_times, g_y),
            interpolate_series(times, g_times, g_z),
        ],
        axis=1,
    )
    g_accel_sensor = np.stack(
        [
            interpolate_series(times, a_times, a_x),
            interpolate_series(times, a_times, a_y),
            interpolate_series(times, a_times, a_z),
        ],
        axis=1,
    )

    static_mask = times <= STATIC_ANCHOR_END_S
    if not np.any(static_mask):
        static_mask = np.ones_like(times, dtype=bool)

    dcm_ekf_static = euler321_to_dcm_bn(
        math.radians(float(np.median(roll_ekf[static_mask]))),
        math.radians(float(np.median(pitch_ekf[static_mask]))),
        math.radians(float(np.median(yaw_ekf[static_mask]))),
    )
    dcm_android_static = euler321_to_dcm_bn(
        math.radians(float(np.median(roll_ref[static_mask] - roll_off))),
        math.radians(float(np.median(pitch_ref[static_mask] - pitch_off))),
        math.radians(float(np.median(yaw_ref[static_mask]))),
    )
    convention_anchor = dcm_ekf_static.T @ dcm_android_static

    ticks: list[ChainTick] = []
    for idx, row in enumerate(chain_rows):
        a_raw = vec3(row, "a_raw")
        a_body = vec3(row, "a_body")
        mount_residual = float(np.linalg.norm(r_mount @ a_raw - a_body))

        g_pred = vec3(row, "g_body_pred")
        if np.linalg.norm(g_pred) < 1e-6:
            g_pred = g_body_from_dcm(
                euler321_to_dcm_bn(
                    math.radians(roll_ekf[idx]),
                    math.radians(pitch_ekf[idx]),
                    math.radians(yaw_ekf[idx]),
                )
            )
        g_meas = vec3(row, "g_body_meas")
        roll_r = math.radians(roll_ref[idx] - roll_off)
        pitch_r = math.radians(pitch_ref[idx] - pitch_off)
        yaw_r = math.radians(yaw_ref[idx])
        g_ref = g_body_from_dcm(euler321_to_dcm_bn(roll_r, pitch_r, yaw_r))

        dcm_ekf = euler321_to_dcm_bn(
            math.radians(roll_ekf[idx]), math.radians(pitch_ekf[idx]), math.radians(yaw_ekf[idx])
        )
        dcm_android = euler321_to_dcm_bn(roll_r, pitch_r, yaw_r)
        convention_now = dcm_ekf.T @ dcm_android

        g_android_gravity_body = r_mount @ g_grav_sensor[idx]

        ekf_android = angle_between_deg(g_pred, g_ref)
        ekf_meas = angle_between_deg(g_pred, g_meas)
        android_meas = angle_between_deg(g_ref, g_meas)
        android_gravity_accel = angle_between_deg(g_grav_sensor[idx], g_accel_sensor[idx])
        android_gravity_ekf = angle_between_deg(g_android_gravity_body, g_pred)

        ticks.append(
            ChainTick(
                timestamp_s=row["timestamp_s"],
                mount_residual_mps2=mount_residual,
                ekf_android_tilt_deg=ekf_android,
                ekf_android_drift_from_anchor_deg=dcm_delta_angle_deg(convention_anchor, convention_now),
                ekf_meas_tilt_deg=ekf_meas,
                ekf_meas_drift_from_anchor_deg=ekf_meas,
                android_meas_tilt_deg=android_meas,
                android_gravity_accel_sensor_deg=android_gravity_accel,
                android_gravity_ekf_body_deg=android_gravity_ekf,
                convention_drift_deg=dcm_delta_angle_deg(convention_anchor, convention_now),
                a_lin_h=row.get("a_lin_h", 0.0),
                gps_speed_mps=row.get("gps_speed_mps", 0.0),
            )
        )

    static_idx = [i for i, t in enumerate(ticks) if t.timestamp_s <= STATIC_ANCHOR_END_S]
    if static_idx:
        anchor_ekf_android = float(np.median([ticks[i].ekf_android_tilt_deg for i in static_idx]))
        anchor_ekf_meas = float(np.median([ticks[i].ekf_meas_tilt_deg for i in static_idx]))
        anchor_android_meas = float(np.median([ticks[i].android_meas_tilt_deg for i in static_idx]))
        anchor_grav_accel = float(np.median([ticks[i].android_gravity_accel_sensor_deg for i in static_idx]))
    else:
        anchor_ekf_android = anchor_ekf_meas = anchor_android_meas = anchor_grav_accel = 0.0

    adjusted: list[ChainTick] = []
    for tick in ticks:
        adjusted.append(
            ChainTick(
                timestamp_s=tick.timestamp_s,
                mount_residual_mps2=tick.mount_residual_mps2,
                ekf_android_tilt_deg=tick.ekf_android_tilt_deg,
                ekf_android_drift_from_anchor_deg=tick.ekf_android_tilt_deg - anchor_ekf_android,
                ekf_meas_tilt_deg=tick.ekf_meas_tilt_deg,
                ekf_meas_drift_from_anchor_deg=tick.ekf_meas_tilt_deg - anchor_ekf_meas,
                android_meas_tilt_deg=tick.android_meas_tilt_deg,
                android_gravity_accel_sensor_deg=tick.android_gravity_accel_sensor_deg,
                android_gravity_ekf_body_deg=tick.android_gravity_ekf_body_deg,
                convention_drift_deg=tick.convention_drift_deg,
                a_lin_h=tick.a_lin_h,
                gps_speed_mps=tick.gps_speed_mps,
            )
        )
    return adjusted


def diagnose_chain(ticks: list[ChainTick]) -> dict:
    times = np.array([t.timestamp_s for t in ticks], dtype=float)

    links = [
        summarize_link(
            "L1_sensor_to_body_R_mount",
            "v_body = R_mount * v_sensor (calibracion estatica)",
            "constant_geometric",
            np.array([t.mount_residual_mps2 for t in ticks]),
            times,
            "mps2",
        ),
        summarize_link(
            "L2_body_EKF_vs_body_Android_tilt",
            "Inclinacion g_body: EKF vs Orientation (offset estatico)",
            "estimator_pair",
            np.array([t.ekf_android_tilt_deg for t in ticks]),
            times,
            "deg",
        ),
        summarize_link(
            "L2b_ekf_android_drift_from_static_anchor",
            "Desviacion relativa EKF-Android respecto ancla 0-2 s",
            "estimator_pair",
            np.array([t.ekf_android_drift_from_anchor_deg for t in ticks]),
            times,
            "deg",
        ),
        summarize_link(
            "L3_body_EKF_vs_accel_tilt",
            "Inclinacion g_body: EKF vs acelerometro normalizado",
            "estimator_pair",
            np.array([t.ekf_meas_tilt_deg for t in ticks]),
            times,
            "deg",
        ),
        summarize_link(
            "L4_android_vs_accel_tilt",
            "Inclinacion g_body: Orientation vs acelerometro",
            "estimator_pair",
            np.array([t.android_meas_tilt_deg for t in ticks]),
            times,
            "deg",
        ),
        summarize_link(
            "L5_android_gravity_vs_total_accel_sensor",
            "Gravity.csv vs TotalAcceleration.csv (marco sensor; fusion Android vs IMU total)",
            "estimator_pair",
            np.array([t.android_gravity_accel_sensor_deg for t in ticks]),
            times,
            "deg",
        ),
        summarize_link(
            "L6_android_gravity_vs_ekf_body",
            "R_mount*Gravity.csv vs g_body EKF",
            "estimator_pair",
            np.array([t.android_gravity_ekf_body_deg for t in ticks]),
            times,
            "deg",
        ),
        summarize_link(
            "L7_convention_bridge_drift",
            "Drift del puente R_ekf^T R_android respecto ancla estatica",
            "constant_geometric",
            np.array([t.convention_drift_deg for t in ticks]),
            times,
            "deg",
        ),
    ]

    regime_links = [link for link in links if link["regime_dependent"]]
    constant_links = [link for link in links if not link["regime_dependent"]]

    android_fusion = links[4]
    ekf_android = links[1]

    hypothesis = "inconclusive"
    l5_static = android_fusion.get("static_anchor_mean", float("nan"))
    l5_jump = android_fusion.get("jump_static_to_motion", float("nan"))
    l2_jump = ekf_android.get("jump_static_to_motion", float("nan"))
    l6_jump = links[5].get("jump_static_to_motion", float("nan"))

    if l5_static < 1.0 and l5_jump >= 1.0 and l2_jump >= 2.0:
        hypothesis = "android_separates_gravity_from_total_accel_during_dynamics"
    elif l2_jump >= 2.0 and l6_jump >= 1.5:
        hypothesis = "both_estimators_diverge_from_raw_accel_differently"
    elif l2_jump >= 2.0:
        hypothesis = "ekf_and_android_orientation_diverge_in_dynamics"

    return {
        "static_anchor_s": [0.0, STATIC_ANCHOR_END_S],
        "interpretation_notes": [
            "Ancla estatica: Android, acelerometro y EKF coinciden (~0.05 deg).",
            "No afirmar 'EKF equivocado'; medir divergencia entre estimadores.",
            "Si L1 es constante, R_mount es geometrico y no explica el salto.",
            "Si L5 crece en dinamica, Android separa gravedad de aceleracion especifica.",
            "L7 crece si los estimadores divergen; en estatico L7~0 descarta error FLU/NED global.",
        ],
        "links": links,
        "regime_dependent_links": [link["link"] for link in regime_links],
        "constant_links": [link["link"] for link in constant_links],
        "android_fusion_hypothesis": hypothesis,
        "solid_facts_h0_h9d": {
            "static_triad_agreement_deg": links[1]["static_anchor_mean"],
            "mount_residual_static_mps2": links[0]["static_anchor_mean"],
            "problem_onset": "acceleration_regime_not_time_not_turning",
            "heading_eliminated": True,
        },
    }


def write_merged(ticks: list[ChainTick], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "timestamp_s",
        "mount_residual_mps2",
        "ekf_android_tilt_deg",
        "ekf_android_drift_from_anchor_deg",
        "ekf_meas_tilt_deg",
        "android_meas_tilt_deg",
        "android_gravity_accel_sensor_deg",
        "android_gravity_ekf_body_deg",
        "convention_drift_deg",
        "a_lin_h",
        "gps_speed_mps",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for t in ticks:
            writer.writerow({field: getattr(t, field) for field in fields})


def plot_chain(ticks: list[ChainTick], path: Path) -> None:
    post = [t for t in ticks if t.timestamp_s >= 1.5]
    times = np.array([t.timestamp_s for t in post], dtype=float)

    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    fig.suptitle("Reference chain audit (drift from static anchor)", fontsize=14)

    axes[0].plot(times, [t.mount_residual_mps2 for t in post], label="L1 R_mount residual [m/s2]", linewidth=0.8)
    axes[0].set_ylabel("m/s2")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, [t.ekf_android_drift_from_anchor_deg for t in post], label="L2 EKF-Android drift", linewidth=0.8)
    axes[1].plot(times, [t.ekf_meas_tilt_deg for t in post], label="L3 EKF-accel angle", linewidth=0.8, alpha=0.7)
    axes[1].plot(times, [t.android_gravity_ekf_body_deg for t in post], label="L6 Android grav-EKF", linewidth=0.8, alpha=0.7)
    axes[1].axvline(STATIC_ANCHOR_END_S, color="#7f8c8d", linestyle=":")
    axes[1].set_ylabel("deg")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(
        times,
        [t.android_gravity_accel_sensor_deg for t in post],
        label="L5 Android Gravity vs Accel (sensor)",
        linewidth=0.8,
        color="#e67e22",
    )
    axes[2].plot(times, [t.android_meas_tilt_deg for t in post], label="L4 Orient vs accel", linewidth=0.8, alpha=0.7)
    axes[2].set_ylabel("deg")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(times, [t.a_lin_h for t in post], label="a_lin_h", linewidth=0.8, color="#8e44ad")
    axes[3].plot(times, [t.gps_speed_mps for t in post], label="GPS speed", linewidth=0.8, color="#27ae60", alpha=0.7)
    axes[3].set_ylabel("m/s2 / m/s")
    axes[3].set_xlabel("Tiempo [s]")
    axes[3].legend(fontsize=8)
    axes[3].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def run_reference_chain_audit(
    chain_csv: Path = CHAIN_CSV,
    input_dir: Path = DEFAULT_INPUT_DIR,
    mount_path: Path = DEFAULT_MOUNT,
    orientation_path: Path | None = None,
) -> dict:
    orient = resolve_orientation_path(orientation_path)
    if orient is None:
        raise FileNotFoundError("Orientation.csv no encontrado")
    if not chain_csv.is_file():
        raise FileNotFoundError(f"Falta {chain_csv}")

    chain = load_chain(chain_csv)
    ticks = build_chain_ticks(chain, orient, input_dir, mount_path)
    diagnosis = diagnose_chain(ticks)
    write_merged(ticks, MERGED_CSV)
    plot_chain(ticks, ANALYSIS_PNG)

    report = {
        "experiment": "reference_chain_audit",
        "purpose": "Validar cadena Sensor->R_mount->Body->Android->NED->EKF con ancla estatica",
        "chain": [
            "Sensor (Accelerometer/Gravity CSV)",
            "R_mount (calibracion)",
            "Body (FRD/NED EKF)",
            "Android Orientation (fusion AHRS)",
            "NED",
            "EKF R_bn",
        ],
        "diagnosis": diagnosis,
        "artifacts": {
            "merged_csv": str(MERGED_CSV),
            "plot_png": str(ANALYSIS_PNG),
        },
    }
    REPORT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    return report


def main() -> int:
    try:
        report = run_reference_chain_audit()
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    diagnosis = report["diagnosis"]
    print("=" * 72)
    print("Reference chain audit (static anchor 0-2 s)")
    print("=" * 72)
    for link in diagnosis["links"]:
        print(
            f"  {link['link']}: static={link['static_anchor_mean']:.3f} "
            f"motion={link['motion_2_10_mean']:.3f} jump={link['jump_static_to_motion']:.3f} "
            f"-> {link['verdict']}"
        )
    print(f"  Regime-dependent: {', '.join(diagnosis['regime_dependent_links']) or '(none)'}")
    print(f"  Android fusion hypothesis: {diagnosis['android_fusion_hypothesis']}")
    print(f"  Informe: {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
