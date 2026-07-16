#!/usr/bin/env python3
"""Auditoría de la cadena de datos IMU (sin tocar el EKF).

Verifica, con vectores sintéticos y muestras reales del replay:
  sensor (Sensor Logger / WT61C) -> parse_mobile_log -> R_mount -> predict()

Cadena de marcos documentada (ver FRAME_CHAIN abajo).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY = REPO_ROOT / "docs" / "benchmarks" / "real_run_replay.csv"
DEFAULT_ORIENTATION = REPO_ROOT / "data" / "real_run" / "Orientation.csv"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
SEARCH_DIRS = (
    REPO_ROOT / "data" / "real_run",
    REPO_ROOT,
    REPO_ROOT / "docs",
)
GRAVITY = 9.80665
STATIC_END_S = 30.0
TOL_EXACT = 1e-6
TOL_PHYS = 0.15  # m/s² o rad/s para comparaciones físicas

# Ángulos del análisis estático (rad)
MOUNT_ROLL_RAD = math.radians(-45.18)
MOUNT_PITCH_RAD = math.radians(-51.52)
MOUNT_YAW_RAD = math.radians(110.40)

FRAME_CHAIN = """
Cadena de marcos (definición explícita)
======================================

  WT61C / Sensor Logger (Android)
       |
       |  Trama UART WT61C: accel[0..2] = (x, y, z) del módulo WitMotion
       |  Sensor Logger CSV: columnas z, y, x -> parse_mobile_log lee x,y,z
       |  replay CSV: accel_x, accel_y, accel_z (m/s², marco del dispositivo)
       v
  Frame SENSOR (S) — ejes del móvil / IMU montado en soporte
       |
       |  real_run_replay.cpp: v_body = R_mount^T * v_sensor  (convención actual)
       |  Euler 3-2-1: R_mount = Rz(yaw) * Ry(pitch) * Rx(roll)
       v
  Frame VEHÍCULO / BODY (B) — esperado por ins_ekf (q_att: body -> NED)
       |
       |  predict(): a_corr = a_body - bias
       |            w_corr = w_body - bias
       |            integra actitud con w_corr
       |            a_ned = C_bn * a_corr
       |            a_ned -= [0, 0, +g]   <-- gravedad se resta EN NED, DESPUÉS de rotar
       v
  Frame NED (N)

Orden de gravedad en ins_ekf.cpp::predict (CONFIRMADO):
  1. Restar bias en body
  2. Integrar giroscopio (actitud)
  3. Rotar aceleración body -> NED (C_bn)
  4. Restar gravedad en NED: a_n[2] -= 9.80665

NO es: restar gravedad en body y luego rotar.
"""

Vec3 = tuple[float, float, float]
Matrix3 = list[list[float]]


@dataclass(frozen=True)
class OrientationSample:
    timestamp_s: float
    qw: float
    qx: float
    qy: float
    qz: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class MountVariant:
    name: str
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    use_transpose: bool


MOUNT_VARIANTS = (
    MountVariant("R_mount directa", MOUNT_ROLL_RAD, MOUNT_PITCH_RAD, MOUNT_YAW_RAD, False),
    MountVariant(
        "angulos invertidos",
        -MOUNT_ROLL_RAD,
        -MOUNT_PITCH_RAD,
        -MOUNT_YAW_RAD,
        False,
    ),
    MountVariant("R_mount^T", MOUNT_ROLL_RAD, MOUNT_PITCH_RAD, MOUNT_YAW_RAD, True),
)


def vec3_norm(vector: Vec3) -> float:
    return math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)


def vec3_normalize(vector: Vec3) -> Vec3:
    norm = vec3_norm(vector)
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def numpy_to_matrix3(matrix: np.ndarray) -> Matrix3:
    return matrix.tolist()


def matrix3_to_numpy(matrix: Matrix3) -> np.ndarray:
    return np.array(matrix, dtype=float)


def dcm_to_euler321_deg(dcm: Matrix3) -> Vec3:
    """Euler 3-2-1 coherente con quat_to_euler321 en ins_ekf.cpp."""
    m = matrix3_to_numpy(dcm)
    pitch = math.asin(max(-1.0, min(1.0, -float(m[2, 0]))))
    roll = math.atan2(float(m[2, 1]), float(m[2, 2]))
    yaw = math.atan2(float(m[1, 0]), float(m[0, 0]))
    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


def rotation_angle_deg(a: Matrix3, b: Matrix3) -> float:
    """Angulo entre dos rotaciones: acos((trace(R_rel)-1)/2)."""
    rel = matrix3_to_numpy(a) @ matrix3_to_numpy(b).T
    trace = float(np.trace(rel))
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return math.degrees(math.acos(cos_angle))


def rodrigues_align(unit_source: Vec3, unit_target: Vec3) -> Matrix3:
    """Rotacion minima que alinea unit_source -> unit_target."""
    u = np.array(unit_source, dtype=float)
    v = np.array(unit_target, dtype=float)
    cross = np.cross(u, v)
    dot = float(np.dot(u, v))
    norm_cross = float(np.linalg.norm(cross))
    if norm_cross < 1e-12:
        if dot > 0.0:
            return [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        # Antiparalelo: rotar 180 deg alrededor de un eje perpendicular.
        axis = np.array([1.0, 0.0, 0.0])
        if abs(u[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        axis = axis - u * float(np.dot(axis, u))
        axis /= np.linalg.norm(axis)
        wx = np.array(
            [
                [0.0, -axis[2], axis[1]],
                [axis[2], 0.0, -axis[0]],
                [-axis[1], axis[0], 0.0],
            ]
        )
        rot = np.eye(3) + 2.0 * (wx @ wx)
        return numpy_to_matrix3(rot)

    wx = np.array(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ]
    )
    rot = np.eye(3) + wx + wx @ wx * ((1.0 - dot) / (norm_cross * norm_cross))
    return numpy_to_matrix3(rot)


def kabsch_rotation(
    source_points: np.ndarray,
    target_points: np.ndarray,
    weights: np.ndarray | None = None,
) -> Matrix3:
    """Wahba/Kabsch: R tal que R @ source_i ~ target_i (minimos cuadrados)."""
    if source_points.shape != target_points.shape or source_points.shape[1] != 3:
        raise ValueError("source_points y target_points deben ser Nx3")

    if weights is None:
        weights = np.ones(source_points.shape[0], dtype=float)
    weights = weights / np.sum(weights)

    source_centroid = np.average(source_points, axis=0, weights=weights)
    target_centroid = np.average(target_points, axis=0, weights=weights)
    source_centered = source_points - source_centroid
    target_centered = target_points - target_centroid

    h = np.zeros((3, 3), dtype=float)
    for idx in range(source_points.shape[0]):
        h += weights[idx] * np.outer(source_centered[idx], target_centered[idx])

    u, _, vt = np.linalg.svd(h)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0.0:
        vt[-1, :] *= -1.0
        rot = vt.T @ u.T
    return numpy_to_matrix3(rot)


def quat_to_dcm_bn(qw: float, qx: float, qy: float, qz: float) -> Matrix3:
    """Replica quat_to_dcm_bn de ins_ekf.cpp (body->NED/world)."""
    return [
        [
            qw * qw + qx * qx - qy * qy - qz * qz,
            2.0 * ((qx * qy) - (qw * qz)),
            2.0 * ((qx * qz) + (qw * qy)),
        ],
        [
            2.0 * ((qx * qy) + (qw * qz)),
            qw * qw - qx * qx + qy * qy - qz * qz,
            2.0 * ((qy * qz) - (qw * qx)),
        ],
        [
            2.0 * ((qx * qz) - (qw * qy)),
            2.0 * ((qy * qz) + (qw * qx)),
            qw * qw - qx * qx - qy * qy + qz * qz,
        ],
    ]


def apply_matrix(matrix: Matrix3, vector: Vec3) -> Vec3:
    return mat3_vec3_mul(matrix, vector)


def gravity_alignment_error(matrix: Matrix3, g_sensor: Vec3, g_body: Vec3) -> float:
    aligned = apply_matrix(matrix, g_sensor)
    return vec3_norm(vec3_sub(aligned, g_body))


def discover_orientation_path(explicit: Path | None) -> Path | None:
    if explicit is not None:
        return explicit if explicit.is_file() else None
    for directory in SEARCH_DIRS:
        candidate = directory / "Orientation.csv"
        if candidate.is_file():
            return candidate
    return None


def discover_t0_ns() -> int | None:
    try:
        from parse_mobile_log import (
            ACCEL_FILE,
            GYRO_FILE,
            LOCATION_FILE,
            discover_input_dir,
            load_location_csv,
            load_vec3_csv,
        )

        input_dir = discover_input_dir(None)
        accel = load_vec3_csv(input_dir / ACCEL_FILE)
        gyro = load_vec3_csv(input_dir / GYRO_FILE)
        locations = load_location_csv(input_dir / LOCATION_FILE)
        return min(accel[0].time_ns, gyro[0].time_ns, locations[0].time_ns)
    except (FileNotFoundError, ValueError):
        return None


def load_orientation_samples(path: Path, t0_ns: int | None) -> list[OrientationSample]:
    samples: list[OrientationSample] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            return samples

        lower = {name.lower(): name for name in reader.fieldnames}
        required = ("time", "qw", "qx", "qy", "qz", "roll", "pitch", "yaw")
        if not all(key in lower for key in required):
            return samples

        for row in reader:
            time_ns = float(row[lower["time"]])
            qw = float(row[lower["qw"]])
            qx = float(row[lower["qx"]])
            qy = float(row[lower["qy"]])
            qz = float(row[lower["qz"]])
            roll = float(row[lower["roll"]])
            pitch = float(row[lower["pitch"]])
            yaw = float(row[lower["yaw"]])

            timestamp_s = (time_ns - t0_ns) * 1e-9 if t0_ns is not None else time_ns * 1e-9
            if max(abs(roll), abs(pitch), abs(yaw)) <= (2.0 * math.pi + 0.5):
                roll_deg = math.degrees(roll)
                pitch_deg = math.degrees(pitch)
                yaw_deg = math.degrees(yaw)
            else:
                roll_deg, pitch_deg, yaw_deg = roll, pitch, yaw

            samples.append(
                OrientationSample(
                    timestamp_s=timestamp_s,
                    qw=qw,
                    qx=qx,
                    qy=qy,
                    qz=qz,
                    roll_deg=roll_deg,
                    pitch_deg=pitch_deg,
                    yaw_deg=yaw_deg,
                )
            )
    samples.sort(key=lambda sample: sample.timestamp_s)
    return samples


def median_quaternion(samples: Sequence[OrientationSample]) -> tuple[float, float, float, float]:
    if not samples:
        return (1.0, 0.0, 0.0, 0.0)
    qw = statistics.median(sample.qw for sample in samples)
    qx = statistics.median(sample.qx for sample in samples)
    qy = statistics.median(sample.qy for sample in samples)
    qz = statistics.median(sample.qz for sample in samples)
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (qw / norm, qx / norm, qy / norm, qz / norm)


def build_euler321(roll_rad: float, pitch_rad: float, yaw_rad: float) -> Matrix3:
    """Replica de build_euler321_rotation_matrix en real_run_replay.cpp."""
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def build_mount_matrix(variant: MountVariant) -> Matrix3:
    matrix = build_euler321(variant.roll_rad, variant.pitch_rad, variant.yaw_rad)
    if variant.use_transpose:
        return mat3_transpose(matrix)
    return matrix


def mat3_transpose(matrix: Matrix3) -> Matrix3:
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


def mat3_vec3_mul(matrix: Matrix3, vector: Vec3) -> Vec3:
    return tuple(
        matrix[row][0] * vector[0] + matrix[row][1] * vector[1] + matrix[row][2] * vector[2]
        for row in range(3)
    )


def vec3_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def apply_mount(vector: Vec3, variant: MountVariant) -> Vec3:
    r_mount = build_euler321(variant.roll_rad, variant.pitch_rad, variant.yaw_rad)
    if variant.use_transpose:
        r_mount = mat3_transpose(r_mount)
    return mat3_vec3_mul(r_mount, vector)


def simulate_ekf_specific_ned(
    accel_body: Vec3,
    dcm_bn: list[list[float]] | None = None,
) -> Vec3:
    """Réplica del orden de gravedad en ins_ekf::predict (sin bias, dt)."""
    if dcm_bn is None:
        dcm_bn = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    a_ned = mat3_vec3_mul(dcm_bn, accel_body)
    return (a_ned[0], a_ned[1], a_ned[2] - GRAVITY)


def load_imu_rows(path: Path, max_rows: int | None = None) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("type") != "IMU":
                continue
            rows.append(
                {
                    "t": float(row["timestamp_s"]),
                    "accel": (
                        float(row["accel_x"]),
                        float(row["accel_y"]),
                        float(row["accel_z"]),
                    ),
                    "gyro": (
                        float(row["gyro_x"]),
                        float(row["gyro_y"]),
                        float(row["gyro_z"]),
                    ),
                }
            )
            if max_rows is not None and len(rows) >= max_rows:
                break
    return rows


def audit_dt(rows: Sequence[dict[str, float | str]]) -> dict[str, float]:
    times = [float(row["t"]) for row in rows]
    dts = [times[i + 1] - times[i] for i in range(len(times) - 1) if times[i + 1] > times[i]]
    if not dts:
        return {}
    return {
        "count": float(len(dts)),
        "mean": statistics.mean(dts),
        "median": statistics.median(dts),
        "min": min(dts),
        "max": max(dts),
        "stdev": statistics.pstdev(dts) if len(dts) > 1 else 0.0,
        "p01": sorted(dts)[max(0, int(0.01 * len(dts)) - 1)],
        "p99": sorted(dts)[min(len(dts) - 1, int(0.99 * len(dts)))],
    }


def replay_dt_clamped(dt_raw: float) -> float:
    """Réplica de compute_dt_s en real_run_replay.cpp."""
    default_dt = 0.01
    min_dt, max_dt = 0.001, 0.05
    if dt_raw <= 0.0:
        return default_dt
    if dt_raw < min_dt:
        return min_dt
    if dt_raw > max_dt:
        return max_dt
    return dt_raw


def median_vec3(vectors: Sequence[Vec3]) -> Vec3:
    if not vectors:
        return (0.0, 0.0, 0.0)
    return (
        statistics.median(v[0] for v in vectors),
        statistics.median(v[1] for v in vectors),
        statistics.median(v[2] for v in vectors),
    )


def print_section(title: str) -> None:
    print()
    print("=" * 72)
    print(title)
    print("=" * 72)


def audit_synthetic_vectors() -> None:
    print_section("1. Vectores sintéticos conocidos")

    g_sensor = (0.0, 0.0, -GRAVITY)
    omega_z = (0.0, 0.0, 1.0)

    print(f"Entrada gravedad sensor:  {g_sensor}")
    print(f"Entrada giro puro Z:      {omega_z} rad/s")
    print()

    for variant in MOUNT_VARIANTS:
        g_out = apply_mount(g_sensor, variant)
        w_out = apply_mount(omega_z, variant)
        print(f"[{variant.name}]")
        print(f"  accel_out = ({g_out[0]:+.6f}, {g_out[1]:+.6f}, {g_out[2]:+.6f})  |mag|={vec3_norm(g_out):.5f}")
        print(f"  gyro_out  = ({w_out[0]:+.6f}, {w_out[1]:+.6f}, {w_out[2]:+.6f})  |mag|={vec3_norm(w_out):.5f}")

    print()
    print("Interpretación: ninguna variante produce [0,0,+g] exacto desde g=(0,0,-g)")
    print("en marco sensor arbitrario. La prueba válida es con el vector MEDIDO en estático,")
    print("no con un eje canónico asumido del móvil.")


def audit_gravity_order() -> None:
    print_section("4. Orden de gravedad en predict()")

    body_level = (0.0, 0.0, GRAVITY)  # body alineado con NED, vehículo parado
    correct = simulate_ekf_specific_ned(body_level)
    wrong_body = (body_level[0], body_level[1], body_level[2] - GRAVITY)
    wrong = mat3_vec3_mul(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        wrong_body,
    )
    wrong_specific = (wrong[0], wrong[1], wrong[2] - GRAVITY)

    print(f"Body parado (alineado NED): a_body = (0, 0, +{GRAVITY})")
    print(f"  Orden EKF (rotar -> restar g en NED): a_specific_ned = {correct}")
    ok = vec3_norm(correct) < TOL_EXACT
    print(f"  -> aceleracion especifica esperada ~ 0: {'PASS' if ok else 'FAIL'}")
    print(f"  Orden INCORRECTO (restar g en body -> rotar -> restar g): {wrong_specific}")
    bad = vec3_norm(wrong_specific) > 1.0
    print(f"  -> produciria deriva falsa: {'CONFIRMADO (mal)' if bad else 'inesperado'}")


def audit_dt_stats(rows: Sequence[dict[str, float | str]]) -> None:
    print_section("3. Auditoría de dt (replay CSV)")

    stats = audit_dt(rows)
    if not stats:
        print("Sin filas IMU.")
        return

    mean_hz = 1.0 / stats["mean"] if stats["mean"] > 0 else 0.0
    median_hz = 1.0 / stats["median"] if stats["median"] > 0 else 0.0

    print(f"Muestras IMU:     {len(rows)}")
    print(f"dt medio:         {stats['mean']*1000:.4f} ms  (~{mean_hz:.1f} Hz)")
    print(f"dt mediana:       {stats['median']*1000:.4f} ms  (~{median_hz:.1f} Hz)")
    print(f"dt min / max:     {stats['min']*1000:.4f} / {stats['max']*1000:.4f} ms")
    print(f"dt p01 / p99:     {stats['p01']*1000:.4f} / {stats['p99']*1000:.4f} ms")
    print(f"dt desv. estandar:{stats['stdev']*1000:.4f} ms")
    print()
    print("Limites replay (compute_dt_s): clamp [1, 50] ms; default 10 ms si dt<=0")

    clamped_count = 0
    times = [float(r["t"]) for r in rows]
    for i in range(1, len(times)):
        raw = times[i] - times[i - 1]
        if raw > 0 and replay_dt_clamped(raw) != raw:
            clamped_count += 1
    print(f"Filas con dt clampeado: {clamped_count} / {max(0, len(times)-1)}")

    factor10_risk = stats["max"] / stats["min"] if stats["min"] > 0 else float("inf")
    print(f"Ratio max/min dt: {factor10_risk:.2f}x", end="")
    if factor10_risk > 10.0:
        print("  *** ALERTA: posible jitter temporal significativo")
    else:
        print("  (sin factor-10 sospechoso)")


def audit_real_samples(rows: Sequence[dict[str, float | str]], sample_count: int) -> None:
    print_section("5. Trazado de muestras reales (fase estática)")

    static_rows = [r for r in rows if float(r["t"]) <= STATIC_END_S]
    if not static_rows:
        static_rows = rows[:sample_count]
    else:
        static_rows = static_rows[:sample_count]

    g_median = median_vec3([row["accel"] for row in static_rows])  # type: ignore[arg-type]
    w_median = median_vec3([row["gyro"] for row in static_rows])  # type: ignore[arg-type]

    print(f"Muestras trazadas: {len(static_rows)} (t <= {STATIC_END_S}s)")
    print(f"Mediana accel cruda (sensor): ({g_median[0]:+.3f}, {g_median[1]:+.3f}, {g_median[2]:+.3f}) m/s²")
    print(f"|g| mediana cruda: {vec3_norm(g_median):.4f} m/s2  (esperado ~ {GRAVITY})")
    print(f"Mediana gyro cruda:           ({w_median[0]:+.6f}, {w_median[1]:+.6f}, {w_median[2]:+.6f}) rad/s")
    print()

    body_targets = (
        ((0.0, 0.0, GRAVITY), "body Z+ = gravedad (conv. EKF NED-down)"),
        ((0.0, 0.0, -GRAVITY), "body Z- = gravedad"),
        ((GRAVITY, 0.0, 0.0), "body X+ = gravedad"),
    )

    print("--- Comparación de variantes de montaje vs mediana estática ---")
    for variant in MOUNT_VARIANTS:
        g_body = apply_mount(g_median, variant)
        err_targets = []
        for target, label in body_targets:
            err = vec3_norm(vec3_sub(g_body, target))
            err_targets.append((err, label))
        best_err, best_label = min(err_targets, key=lambda item: item[0])
        print(f"[{variant.name}]")
        print(f"  accel_body = ({g_body[0]:+.4f}, {g_body[1]:+.4f}, {g_body[2]:+.4f})  |mag|={vec3_norm(g_body):.4f}")
        print(f"  mejor alineacion: {best_label}  error={best_err:.4f} m/s2")

    print()
    print("--- Trazado fila a fila (primeras muestras) ---")
    header = (
        f"{'t[s]':>8}  {'ax':>8} {'ay':>8} {'az':>8}  "
        f"{'bx':>8} {'by':>8} {'bz':>8}  "
        f"{'|a|':>6} {'|b|':>6}  {'a_n0':>8} {'a_n1':>8} {'a_n2':>8}"
    )
    print(header)
    print("-" * len(header))

    current_variant = MOUNT_VARIANTS[2]  # R_mount^T (convención activa en replay)
    for row in static_rows[:min(15, len(static_rows))]:
        raw: Vec3 = row["accel"]  # type: ignore[assignment]
        body = apply_mount(raw, current_variant)
        spec_ned = simulate_ekf_specific_ned(body)
        print(
            f"{float(row['t']):8.3f}  "
            f"{raw[0]:8.3f} {raw[1]:8.3f} {raw[2]:8.3f}  "
            f"{body[0]:8.3f} {body[1]:8.3f} {body[2]:8.3f}  "
            f"{vec3_norm(raw):6.3f} {vec3_norm(body):6.3f}  "
            f"{spec_ned[0]:8.3f} {spec_ned[1]:8.3f} {spec_ned[2]:8.3f}"
        )

    print()
    print("a_n* = aceleración específica en NED tras rotar body->NED y restar gravedad")
    print("(con C_bn=I, solo valido si body~NED; en estatico deberia tender a ~0)")


def export_imu_mount_calibration(
    output_path: Path,
    g_median: Vec3,
    matrix: Matrix3,
    body_target: str,
    alignment_error_mps2: float,
    static_samples: int,
    replay_path: Path,
) -> Path:
    euler = dcm_to_euler321_deg(matrix)
    payload = {
        "schema_version": 1,
        "source": "audit_imu_chain.py",
        "method": "gravity_alignment_rodrigues",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "replay_input": str(replay_path.relative_to(REPO_ROOT)),
        "sensor": "Sensor Logger / Android accelerometer+gyroscope",
        "vehicle": "real_run",
        "static_phase_end_s": STATIC_END_S,
        "static_samples": static_samples,
        "gravity_mps2": GRAVITY,
        "body_target": body_target,
        "sensor_median_accel_mps2": list(g_median),
        "alignment_error_mps2": alignment_error_mps2,
        "euler321_deg": {
            "roll": euler[0],
            "pitch": euler[1],
            "yaw": euler[2],
        },
        "apply_mode": "matrix",
        "rotation_matrix": matrix,
        "notes": (
            "Matriz sensor->body usada directamente en real_run_replay "
            "(v_body = rotation_matrix * v_sensor). Regenerar al cambiar soporte/movil."
        ),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    return output_path


def audit_wahba_kabsch(
    rows: Sequence[dict[str, float | str]],
    orientation_path: Path | None,
    export_calibration_path: Path | None = None,
) -> Path | None:
    print_section("6. Wahba/Kabsch — R_mount desde datos estaticos")

    static_rows = [r for r in rows if float(r["t"]) <= STATIC_END_S]
    if len(static_rows) < 10:
        print("Pocas muestras estaticas; se omite Kabsch.")
        return None

    accels = [row["accel"] for row in static_rows]  # type: ignore[misc]
    g_median = median_vec3(accels)
    g_norm = vec3_normalize(g_median)

    body_targets = {
        "body Z+ (EKF down)": (0.0, 0.0, GRAVITY),
        "body Z-": (0.0, 0.0, -GRAVITY),
        "body X+": (GRAVITY, 0.0, 0.0),
        "body Y+": (0.0, GRAVITY, 0.0),
    }

    print(f"Muestras estaticas: {len(static_rows)}")
    print(f"g_sensor mediana: ({g_median[0]:+.4f}, {g_median[1]:+.4f}, {g_median[2]:+.4f}) m/s2")
    print()

    # Rodrigues: una sola direccion (mediana).
    print("--- Rodrigues (vector mediano -> eje body) ---")
    rodrigues_results: list[tuple[str, Matrix3, float]] = []
    for label, target in body_targets.items():
        unit_target = vec3_normalize(target)
        matrix = rodrigues_align(g_norm, unit_target)
        aligned = apply_matrix(matrix, g_median)
        error = vec3_norm(vec3_sub(aligned, target))
        euler = dcm_to_euler321_deg(matrix)
        rodrigues_results.append((label, matrix, error))
        print(f"  {label:22s} error={error:6.4f} m/s2  RPY=({euler[0]:+7.2f}, {euler[1]:+7.2f}, {euler[2]:+7.2f}) deg")

    best_rodrigues = min(rodrigues_results, key=lambda item: item[2])
    print(f"  Mejor Rodrigues: {best_rodrigues[0]}  error={best_rodrigues[2]:.4f} m/s2")

    # Kabsch: todas las muestras estaticas -> mismo eje body.
    print()
    print("--- Kabsch/Wahba (N muestras estaticas) ---")
    source = np.array(accels, dtype=float)
    kabsch_results: list[tuple[str, Matrix3, float, float]] = []
    for label, target in body_targets.items():
        target_arr = np.tile(np.array(target, dtype=float), (source.shape[0], 1))
        matrix = kabsch_rotation(source, target_arr)
        residuals = [
            vec3_norm(vec3_sub(apply_matrix(matrix, tuple(row)), target))
            for row in source
        ]
        rmse = float(math.sqrt(sum(r * r for r in residuals) / len(residuals)))
        median_err = float(statistics.median(residuals))
        euler = dcm_to_euler321_deg(matrix)
        kabsch_results.append((label, matrix, rmse, median_err))
        print(
            f"  {label:22s} RMSE={rmse:6.4f}  mediana={median_err:6.4f} m/s2  "
            f"RPY=({euler[0]:+7.2f}, {euler[1]:+7.2f}, {euler[2]:+7.2f}) deg"
        )

    best_kabsch = min(kabsch_results, key=lambda item: item[2])
    print(f"  Mejor Kabsch: {best_kabsch[0]}  RMSE={best_kabsch[2]:.4f} m/s2")

    # Comparar con variantes Euler del replay.
    print()
    print("--- Comparacion vs Euler fijo (replay actual) ---")
    print(f"{'Variante':22s} {'err mediana':>12s} {'ang vs Kabsch':>14s}")
    for variant in MOUNT_VARIANTS:
        matrix = build_mount_matrix(variant)
        err = gravity_alignment_error(matrix, g_median, body_targets["body Z+ (EKF down)"])
        angle = rotation_angle_deg(matrix, best_kabsch[1])
        print(f"{variant.name:22s} {err:12.4f} {angle:13.2f} deg")

    angle_kabsch_euler = rotation_angle_deg(best_kabsch[1], build_mount_matrix(MOUNT_VARIANTS[2]))
    print(f"Angulo Kabsch vs R_mount^T activa: {angle_kabsch_euler:.2f} deg")

    # Validacion cruzada con Orientation.csv (Android).
    print()
    print("--- Validacion cruzada Orientation.csv ---")
    if orientation_path is None or not orientation_path.is_file():
        print("Orientation.csv no encontrado; se omite.")
        return

    t0_ns = discover_t0_ns()
    orientations = load_orientation_samples(orientation_path, t0_ns)
    static_orient = [s for s in orientations if s.timestamp_s <= STATIC_END_S]
    if not static_orient:
        static_orient = orientations[:200]

    qw, qx, qy, qz = median_quaternion(static_orient)
    dcm_sn = quat_to_dcm_bn(qw, qx, qy, qz)

    # Android: v_world = R(q) * v_device  =>  g_device = R^T * g_world
    g_world_candidates = {
        "ENU Z+": (0.0, 0.0, GRAVITY),
        "ENU Z-": (0.0, 0.0, -GRAVITY),
    }
    print(f"Quaternion mediano (estatico): qw={qw:+.4f} qx={qx:+.4f} qy={qy:+.4f} qz={qz:+.4f}")
    print(f"Orientacion mediana (deg): roll={statistics.median(s.roll_deg for s in static_orient):+.2f}  "
          f"pitch={statistics.median(s.pitch_deg for s in static_orient):+.2f}  "
          f"yaw={statistics.median(s.yaw_deg for s in static_orient):+.2f}")

    best_orient_err = float("inf")
    best_orient_label = ""
    for world_label, g_world in g_world_candidates.items():
        g_expected = apply_matrix(mat3_transpose(dcm_sn), g_world)
        err = vec3_norm(vec3_sub(g_expected, g_median))
        print(f"  g_esperado ({world_label}): ({g_expected[0]:+.3f}, {g_expected[1]:+.3f}, {g_expected[2]:+.3f})  "
              f"error vs accel medida={err:.4f} m/s2")
        if err < best_orient_err:
            best_orient_err = err
            best_orient_label = world_label

    print(f"  Mejor hipotesis Android: {best_orient_label}  error={best_orient_err:.4f} m/s2")
    if best_orient_err < 0.15:
        print("  -> Orientation.csv es COHERENTE con accel medida (parser/ejes OK en estatico).")
    elif best_orient_err < best_kabsch[2]:
        print("  -> Orientation.csv predice mejor que Kabsch multi-muestra (vibracion en estatico).")
    else:
        print("  -> Revisar convencion quaternion Android vs body EKF.")

    # Probar permutaciones de ejes del parser (x,y,z reorder).
    print()
    print("--- Permutaciones de ejes (parse_mobile_log) ---")
    permutations = {
        "xyz (actual)": (0, 1, 2),
        "xzy": (0, 2, 1),
        "yxz": (1, 0, 2),
        "yzx": (1, 2, 0),
        "zxy": (2, 0, 1),
        "zyx": (2, 1, 0),
    }
    signs = (
        (+1, +1, +1),
        (+1, +1, -1),
        (+1, -1, +1),
        (+1, -1, -1),
        (-1, +1, +1),
        (-1, +1, -1),
        (-1, -1, +1),
        (-1, -1, -1),
    )
    best_perm_err = float("inf")
    best_perm_name = ""
    target = body_targets["body Z+ (EKF down)"]
    for perm_name, perm in permutations.items():
        for sign in signs:
            remapped = tuple(g_median[perm[i]] * sign[i] for i in range(3))
            matrix = rodrigues_align(vec3_normalize(remapped), vec3_normalize(target))
            err = gravity_alignment_error(matrix, remapped, target)
            if err < best_perm_err:
                best_perm_err = err
                best_perm_name = f"{perm_name} sign{sign}"
    print(f"  Mejor permutacion+signo -> body Z+: {best_perm_name}  error={best_perm_err:.4f} m/s2")
    if best_perm_name.startswith("xyz (actual)") and best_perm_err < 0.15:
        print("  -> Mapeo xyz actual en parse_mobile_log es coherente (no requiere permutacion).")
    elif best_perm_err < 0.05:
        print("  *** CANDIDATO: corregir mapeo de ejes en parse_mobile_log")
    elif best_perm_err < best_rodrigues[2]:
        print("  -> Permutacion de ejes mejora respecto a Rodrigues sin permutar.")
    else:
        print("  -> Permutaciones no resuelven el problema por si solas.")

    if export_calibration_path is not None:
        exported = export_imu_mount_calibration(
            export_calibration_path,
            g_median,
            best_rodrigues[1],
            best_rodrigues[0],
            best_rodrigues[2],
            len(static_rows),
            DEFAULT_REPLAY,
        )
        print()
        print(f"Calibracion exportada: {exported}")
        return exported
    return None


def audit_summary(rows: Sequence[dict[str, float | str]]) -> None:
    print_section("Resumen de hipótesis descartadas / siguientes pasos")

    static = [r for r in rows if float(r["t"]) <= STATIC_END_S]
    g_med = median_vec3([r["accel"] for r in static])  # type: ignore[arg-type]

    errors = []
    for variant in MOUNT_VARIANTS:
        g_body = apply_mount(g_med, variant)
        err = min(
            vec3_norm(vec3_sub(g_body, (0.0, 0.0, GRAVITY))),
            vec3_norm(vec3_sub(g_body, (0.0, 0.0, -GRAVITY))),
        )
        errors.append((variant.name, err, vec3_norm(g_body)))

    errors.sort(key=lambda item: item[1])
    print("Error minimo vs gravedad pura en un eje body (fase estatica):")
    for name, err, mag in errors:
        print(f"  {name:20s}  error={err:7.3f} m/s2  |g|={mag:.3f}")

    print()
    print("Conclusion experimental:")
    print("  - Las tres convenciones de R_mount producen errores del mismo orden (~m/s2).")
    print("  - Ninguna alinea la gravedad medida a un eje body canonico con precision.")
    print("  - El problema probable NO es un signo Euler aislado, sino:")
    print("      * definición de ejes Sensor Logger (z,y,x) vs body EKF")
    print("      * unidades / bias / escala")
    print("      * o sincronización IMU-GPS")
    print()
    print("Siguiente paso recomendado (sin tocar el EKF):")
    print("  1. Aplicar R_mount de Kabsch (mejor eje body) en replay si error < 0.15 m/s2")
    print("  2. Corregir permutacion de ejes en parse_mobile_log si la seccion 6 lo indica")
    print("  3. Validar coherencia Orientation.csv vs accel antes de re-tocar el filtro")


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Auditoría cadena IMU (datos, no filtro)")
    parser.add_argument(
        "--replay",
        type=Path,
        default=DEFAULT_REPLAY,
        help=f"CSV replay parseado (por defecto: {DEFAULT_REPLAY})",
    )
    parser.add_argument(
        "--export-calibration",
        type=Path,
        nargs="?",
        const=DEFAULT_CALIBRATION,
        default=None,
        help=f"Exporta calibration/imu_mount.json (por defecto: {DEFAULT_CALIBRATION})",
    )
    parser.add_argument(
        "--orientation",
        type=Path,
        default=None,
        help="Orientation.csv de Sensor Logger (busca en data/real_run por defecto)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=20,
        help="Muestras reales a trazar en fase estática",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    print(FRAME_CHAIN)

    if not args.replay.is_file():
        print(f"ERROR: no existe {args.replay}", file=sys.stderr)
        return 1

    rows = load_imu_rows(args.replay)
    if not rows:
        print(f"ERROR: sin filas IMU en {args.replay}", file=sys.stderr)
        return 1

    audit_synthetic_vectors()
    audit_gravity_order()
    audit_dt_stats(rows)
    audit_real_samples(rows, args.samples)
    orientation_path = discover_orientation_path(args.orientation)
    audit_wahba_kabsch(rows, orientation_path, args.export_calibration)
    audit_summary(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
