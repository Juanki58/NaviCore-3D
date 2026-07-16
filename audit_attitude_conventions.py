#!/usr/bin/env python3
"""Auditoria formal de convenciones y cadena de referencias.

Parte A (sintetica): quat_integrate, quat_to_dcm_bn, body_to_ned.
Parte B (empirica): Sensor -> R_mount -> Body -> Android -> NED -> EKF
  con ancla estatica 0-2 s (run_reference_chain_audit).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
REPORT_JSON = REPO_ROOT / "docs" / "benchmarks" / "attitude_convention_audit.json"
REFERENCE_REPORT_JSON = REPO_ROOT / "docs" / "benchmarks" / "reference_chain_audit.json"

from attitude_kinematics import (  # noqa: E402
    GRAVITY_MPS2,
    body_to_ned,
    euler321_to_dcm_bn,
    euler321_to_quat,
    g_body_from_dcm,
    ned_to_body,
    quat_integrate_first_order,
    quat_to_dcm_bn,
)


def test_dcm_orthonormal() -> dict:
    dcm = euler321_to_dcm_bn(math.radians(10), math.radians(-5), math.radians(30))
    identity = dcm @ dcm.T
    err = float(np.max(np.abs(identity - np.eye(3))))
    det = float(np.linalg.det(dcm))
    return {"orthonormal_max_err": err, "det": det, "pass": err < 1e-5 and abs(det - 1.0) < 1e-5}


def test_body_ned_inverse() -> dict:
    dcm = euler321_to_dcm_bn(0.1, -0.2, 0.5)
    v = np.array([1.2, -0.3, 9.8])
    roundtrip = ned_to_body(dcm, body_to_ned(dcm, v))
    err = float(np.linalg.norm(roundtrip - v))
    return {"roundtrip_err": err, "pass": err < 1e-5}


def test_gravity_norm() -> dict:
    dcm = euler321_to_dcm_bn(0.05, 0.02, 1.0)
    g_b = g_body_from_dcm(dcm)
    return {"g_body_norm": float(np.linalg.norm(g_b)), "pass": abs(float(np.linalg.norm(g_b)) - GRAVITY_MPS2) < 1e-4}


def test_quat_dcm_consistency() -> dict:
    q = euler321_to_quat(0.1, -0.2, 0.5)
    dcm_q = quat_to_dcm_bn(q)
    dcm_e = euler321_to_dcm_bn(0.1, -0.2, 0.5)
    err = float(np.max(np.abs(dcm_q - dcm_e)))
    return {"max_abs_diff": err, "pass": err < 1e-5}


def test_integrate_small_yaw() -> dict:
    q0 = euler321_to_quat(0.0, 0.0, 0.0)
    omega = np.array([0.0, 0.0, math.radians(1.0)], dtype=float)
    dt = 1.0
    q1 = quat_integrate_first_order(q0, omega, dt)
    dcm = quat_to_dcm_bn(q1)
    yaw = math.atan2(dcm[1, 0], dcm[0, 0])
    expected = math.radians(1.0)
    err = abs(yaw - expected)
    return {"yaw_after_1s_deg": math.degrees(yaw), "expected_deg": 1.0, "err_deg": math.degrees(err), "pass": err < math.radians(0.05)}


def test_ned_convention_doc() -> dict:
    return {
        "quaternion_hamilton": True,
        "quat_integrate": "q_dot = 0.5 * q (x) omega  (perturbacion derecha, ver fill_nhc comment)",
        "dcm_bn": "ned[i] = sum_j dcm_bn[i][j] * body[j]  =>  v_ned = R_bn * v_body",
        "ned_to_body": "v_body = R_bn^T * v_ned",
        "gravity_ned": "[0, 0, +g]  (Down positivo en NED)",
        "euler_sequence": "321 (roll, pitch, yaw)",
        "nhc_jacobian_note": "Perturbacion derecha q'=q*dq; H usa -[v_body]x",
    }


def run_synthetic_audit() -> dict:
    tests = {
        "dcm_orthonormal": test_dcm_orthonormal(),
        "body_ned_inverse": test_body_ned_inverse(),
        "gravity_norm_preserved": test_gravity_norm(),
        "quat_dcm_consistency": test_quat_dcm_consistency(),
        "integrate_small_yaw": test_integrate_small_yaw(),
    }
    convention = test_ned_convention_doc()
    all_pass = all(item.get("pass", False) for item in tests.values())
    return {
        "convention_documentation": convention,
        "synthetic_tests": tests,
        "all_pass": all_pass,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Auditoria de convenciones de actitud")
    parser.add_argument("--synthetic-only", action="store_true", help="Solo tests sinteticos")
    parser.add_argument("--empirical-only", action="store_true", help="Solo cadena de referencias")
    args = parser.parse_args()

    synthetic = None if args.empirical_only else run_synthetic_audit()
    reference = None
    if not args.synthetic_only:
        from audit_reference_chain import run_reference_chain_audit  # noqa: WPS433

        reference = run_reference_chain_audit()

    report = {
        "experiment": "attitude_convention_audit",
        "purpose": "Coherencia interna EKF + cadena de referencias con ancla estatica",
        "reformulated_question": (
            "Por que R_bn desarrolla ~4 deg error de inclinacion al entrar en regimen dinamico "
            "mientras heading horizontal sigue coherente?"
        ),
        "wording": "Medir divergencia entre estimadores; no afirmar 'EKF equivocado' sin ground truth.",
        "synthetic": synthetic,
        "reference_chain": reference,
    }

    REPORT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    print("=" * 72)
    print("Attitude convention audit")
    print("=" * 72)
    if synthetic is not None:
        for name, result in synthetic["synthetic_tests"].items():
            print(f"  [synthetic] {name}: {'PASS' if result.get('pass') else 'FAIL'}")
        print(f"  Synthetic all pass: {synthetic['all_pass']}")
    if reference is not None:
        diagnosis = reference["diagnosis"]
        for link in diagnosis["links"]:
            print(
                f"  [chain] {link['link']}: jump={link['jump_static_to_motion']:.3f} "
                f"regime_dep={link['regime_dependent']} -> {link['verdict']}"
            )
        print(f"  Android fusion hypothesis: {diagnosis['android_fusion_hypothesis']}")
        print(f"  Reference report: {REFERENCE_REPORT_JSON}")
    print(f"  Combined report: {REPORT_JSON}")

    if synthetic is not None and not synthetic["all_pass"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
