"""Replica matematica de ins_ekf.cpp (euler321, quat, dcm, body<->ned)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

GRAVITY_MPS2 = 9.80665


def euler321_to_quat(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    q = np.array(
        [
            (cr * cp * cy) + (sr * sp * sy),
            (sr * cp * cy) - (cr * sp * sy),
            (cr * sp * cy) + (sr * cp * sy),
            (cr * cp * sy) - (sr * sp * cy),
        ],
        dtype=float,
    )
    q /= np.linalg.norm(q)
    return q


def quat_to_dcm_bn(q: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = q
    qw2, qx2, qy2, qz2 = qw * qw, qx * qx, qy * qy, qz * qz
    return np.array(
        [
            [qw2 + qx2 - qy2 - qz2, 2.0 * ((qx * qy) - (qw * qz)), 2.0 * ((qx * qz) + (qw * qy))],
            [2.0 * ((qx * qy) + (qw * qz)), qw2 - qx2 + qy2 - qz2, 2.0 * ((qy * qz) - (qw * qx))],
            [2.0 * ((qx * qz) - (qw * qy)), 2.0 * ((qy * qz) + (qw * qx)), qw2 - qx2 - qy2 + qz2],
        ],
        dtype=float,
    )


def euler321_to_dcm_bn(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    return quat_to_dcm_bn(euler321_to_quat(roll_rad, pitch_rad, yaw_rad))


def body_to_ned(dcm_bn: np.ndarray, body: np.ndarray) -> np.ndarray:
    return dcm_bn @ body


def ned_to_body(dcm_bn: np.ndarray, ned: np.ndarray) -> np.ndarray:
    return dcm_bn.T @ ned


def quat_integrate_first_order(q: np.ndarray, omega_radps: np.ndarray, dt_s: float) -> np.ndarray:
    """Replica ins_ekf.cpp quat_integrate_first_order (Hamilton, q_dot = 0.5 q ⊗ omega)."""
    half_dt = 0.5 * dt_s
    qw, qx, qy, qz = q
    wx, wy, wz = omega_radps
    q_out = q.copy()
    q_out[0] += half_dt * ((-qx * wx) - (qy * wy) - (qz * wz))
    q_out[1] += half_dt * ((qw * wx) + (qy * wz) - (qz * wy))
    q_out[2] += half_dt * ((qw * wy) - (qx * wz) + (qz * wx))
    q_out[3] += half_dt * ((qw * wz) + (qx * wy) - (qy * wx))
    q_out /= np.linalg.norm(q_out)
    return q_out


def angle_between_deg(a: np.ndarray, b: np.ndarray) -> float:
    a_u = a / (np.linalg.norm(a) + 1e-12)
    b_u = b / (np.linalg.norm(b) + 1e-12)
    dot = float(np.clip(np.dot(a_u, b_u), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def dcm_delta_angle_deg(dcm_anchor: np.ndarray, dcm_current: np.ndarray) -> float:
    """Angulo de rotacion entre dos DCM (body->ned)."""
    rel = dcm_anchor.T @ dcm_current
    trace = float(np.trace(rel))
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return math.degrees(math.acos(cos_angle))


def load_mount_matrix(path: Path) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    matrix = payload.get("rotation_matrix")
    if not matrix:
        raise ValueError(f"rotation_matrix ausente en {path}")
    return np.array(matrix, dtype=float)


def normalize_vec(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v.copy()
    return v / n


def g_body_from_dcm(dcm_bn: np.ndarray) -> np.ndarray:
    g_ned = np.array([0.0, 0.0, GRAVITY_MPS2], dtype=float)
    return ned_to_body(dcm_bn, g_ned)

