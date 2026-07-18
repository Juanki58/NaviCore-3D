#!/usr/bin/env python3
"""GAP-2 paso 3: descomposicion fuerza especifica vs actitud (A/B/C).

Separa dos mecanismos durante aceleracion longitudinal:

  H1 — Fisica correcta: f = g + a_vehiculo en body; R_bn·f tiene componente
       horizontal real (no es error de actitud).

  H2 — Error de actitud: R_bn inclina g_pred; leak persiste aunque a_corr
       sea solo gravedad vertical en body.

Pruebas (misma R_bn por tick):
  A) a_nav = R_bn @ a_corr                         [actual predict()]
  B) a_nav = R_bn @ [0, 0, a_corr.z]               [sin horizontal body]
  C) a_nav = R_bn @ (||a_corr|| * [0, 0, 1])       [modulo conservado, sin tilt meas]
  D) a_nav = R_bn @ g_body_pred                    [control: gravedad coherente con R_bn -> leak 0]

Instrumentacion por tick:
  a_body      = a_corr
  g_body_pred = R_bn^T @ g_ned
  f_residual  = a_corr - g_body_pred  (fuerza especifica no-gravedad segun actitud EKF)

Criterio:
  Si |a_lin,h|_A >> |a_lin,h|_B ~ 0  -> leak dominado por componente horizontal en f (H1).
  Si |a_lin,h|_B ~ |a_lin,h|_A         -> actitud proyecta gravedad en horizontal (H2).
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

CHAIN_CSV = BENCH_DIR / "propagation_chain_audit.csv"
REPORT_JSON = BENCH_DIR / "gap2_specific_force_decomposition_report.json"
MERGED_CSV = BENCH_DIR / "gap2_specific_force_decomposition.csv"
ANALYSIS_PNG = BENCH_DIR / "gap2_specific_force_decomposition_analysis.png"

GRAVITY = 9.80665
G_NED = np.array([0.0, 0.0, GRAVITY], dtype=float)
STATIC_END_S = 2.0
MOTION_T0 = 2.0
MOTION_T1 = 10.0
LEAK_EXPLAIN_H1_PCT = 85.0
ATTITUDE_LEAK_RATIO_H2 = 0.5


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


def vec3(row: dict, prefix: str) -> np.ndarray:
    return np.array(
        [
            float(row.get(f"{prefix}_x", 0) or 0),
            float(row.get(f"{prefix}_y", 0) or 0),
            float(row.get(f"{prefix}_z", 0) or 0),
        ],
        dtype=float,
    )


def body_to_ned(dcm_bn: np.ndarray, body: np.ndarray) -> np.ndarray:
    return dcm_bn @ body


def a_lin_horizontal(dcm_bn: np.ndarray, a_body: np.ndarray) -> tuple[float, float, np.ndarray, np.ndarray]:
    a_nav = body_to_ned(dcm_bn, a_body)
    a_lin = a_nav - G_NED
    a_lin_h = float(math.hypot(a_lin[0], a_lin[1]))
    a_nav_h = float(math.hypot(a_nav[0], a_nav[1]))
    return a_lin_h, a_nav_h, a_nav, a_lin


@dataclass
class TickSample:
    timestamp_s: float
    a_corr: np.ndarray
    g_body_pred: np.ndarray
    f_residual: np.ndarray
    a_lin_h_a: float
    a_lin_h_b: float
    a_lin_h_c: float
    a_lin_h_d: float
    a_lin_h_replay: float
    a_nav_h_a: float
    leak_frac_ab: float
    leak_frac_ac: float
    a_corr_h: float
    a_corr_x: float
    a_corr_y: float
    f_residual_h: float
    f_residual_x: float
    f_residual_y: float
    f_residual_z: float
    gravity_align_deg: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    dcm_replay_err_h: float


def build_samples(chain: list[dict]) -> list[TickSample]:
    samples: list[TickSample] = []
    for raw in chain:
        t = float(raw["timestamp_s"])
        a_corr = vec3(raw, "a_corr")
        g_pred = vec3(raw, "g_body_pred")
        a_nav_replay = vec3(raw, "a_nav_corr")
        roll = math.radians(float(raw.get("roll_deg") or 0.0))
        pitch = math.radians(float(raw.get("pitch_deg") or 0.0))
        yaw = math.radians(float(raw.get("yaw_deg") or 0.0))
        grav_align = float(raw.get("gravity_angle_deg") or 0.0)

        dcm = euler321_to_dcm_bn(roll, pitch, yaw)
        a_nav_py = body_to_ned(dcm, a_corr)
        dcm_err_h = float(math.hypot(a_nav_py[0] - a_nav_replay[0], a_nav_py[1] - a_nav_replay[1]))

        a_test_b = np.array([0.0, 0.0, a_corr[2]], dtype=float)
        a_norm = float(np.linalg.norm(a_corr))
        a_test_c = np.array([0.0, 0.0, a_norm], dtype=float)

        a_lin_h_a, _, _, _ = a_lin_horizontal(dcm, a_corr)
        a_lin_h_b, _, _, _ = a_lin_horizontal(dcm, a_test_b)
        a_lin_h_c, a_nav_h_a, _, _ = a_lin_horizontal(dcm, a_test_c)
        a_lin_h_d, _, _, _ = a_lin_horizontal(dcm, g_pred)
        a_lin_h_replay = float(raw.get("a_lin_h") or 0.0)

        f_res = a_corr - g_pred
        a_corr_h = float(math.hypot(a_corr[0], a_corr[1]))
        f_res_h = float(math.hypot(f_res[0], f_res[1]))

        denom_a = max(a_lin_h_a, 1e-6)
        leak_ab = max(0.0, (a_lin_h_a - a_lin_h_b) / denom_a)
        leak_ac = max(0.0, (a_lin_h_a - a_lin_h_c) / denom_a)

        samples.append(
            TickSample(
                timestamp_s=t,
                a_corr=a_corr,
                g_body_pred=g_pred,
                f_residual=f_res,
                a_lin_h_a=a_lin_h_a,
                a_lin_h_b=a_lin_h_b,
                a_lin_h_c=a_lin_h_c,
                a_lin_h_d=a_lin_h_d,
                a_lin_h_replay=a_lin_h_replay,
                a_nav_h_a=a_nav_h_a,
                leak_frac_ab=leak_ab,
                leak_frac_ac=leak_ac,
                a_corr_h=a_corr_h,
                a_corr_x=float(a_corr[0]),
                a_corr_y=float(a_corr[1]),
                f_residual_h=f_res_h,
                f_residual_x=float(f_res[0]),
                f_residual_y=float(f_res[1]),
                f_residual_z=float(f_res[2]),
                gravity_align_deg=grav_align,
                roll_deg=float(raw.get("roll_deg") or 0.0),
                pitch_deg=float(raw.get("pitch_deg") or 0.0),
                yaw_deg=float(raw.get("yaw_deg") or 0.0),
                dcm_replay_err_h=dcm_err_h,
            )
        )
    return samples


def window_stats(samples: list[TickSample], t0: float, t1: float) -> dict[str, float]:
    w = [s for s in samples if t0 <= s.timestamp_s <= t1]
    if not w:
        return {}

    def stat(name: str) -> dict[str, float]:
        vals = np.array([getattr(s, name) for s in w], dtype=float)
        return {
            f"{name}_mean": float(np.mean(vals)),
            f"{name}_rms": float(np.sqrt(np.mean(vals * vals))),
            f"{name}_p95": float(np.percentile(vals, 95)),
            f"{name}_max": float(np.max(vals)),
        }

    out: dict[str, float] = {}
    for attr in [
        "a_lin_h_a", "a_lin_h_b", "a_lin_h_c", "a_lin_h_d", "a_lin_h_replay",
        "a_corr_h", "f_residual_h", "f_residual_x", "f_residual_y", "f_residual_z",
        "leak_frac_ab", "leak_frac_ac", "gravity_align_deg", "dcm_replay_err_h",
    ]:
        out.update(stat(attr))
    return out


def corr(samples: list[TickSample], t0: float, t1: float, a: str, b: str) -> float:
    w = [s for s in samples if t0 <= s.timestamp_s <= t1]
    x = np.array([getattr(s, a) for s in w], dtype=float)
    y = np.array([getattr(s, b) for s in w], dtype=float)
    if len(x) < 3 or np.std(x) < 1e-12 or np.std(y) < 1e-12:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def diagnose(static: dict[str, float], motion: dict[str, float], motion_corr: dict[str, float]) -> dict:
    a_mean = motion.get("a_lin_h_a_mean", float("nan"))
    b_mean = motion.get("a_lin_h_b_mean", float("nan"))
    c_mean = motion.get("a_lin_h_c_mean", float("nan"))
    d_mean = motion.get("a_lin_h_d_mean", float("nan"))
    d_max = motion.get("a_lin_h_d_max", float("nan"))
    leak_ab = motion.get("leak_frac_ab_mean", float("nan"))
    leak_ac = motion.get("leak_frac_ac_mean", float("nan"))

    b_ratio = b_mean / max(a_mean, 1e-6)
    c_ratio = c_mean / max(a_mean, 1e-6)
    sf_contrib = a_mean - b_mean
    b_minus_d = b_mean - d_mean
    attitude_self_consistent = d_max < 1e-4
    residual_b_arcmin_deg = math.degrees(math.asin(min(b_mean / GRAVITY, 1.0)))

    if attitude_self_consistent and sf_contrib / max(a_mean, 1e-6) >= 0.55:
        verdict = "H1_SPECIFIC_FORCE_DOMINANT"
        summary = (
            "Prueba D: coherencia algebraica quat/DCM/body_to_ned (R_bn·g_body_pred=g_ned). "
            "NO demuestra actitud fisica perfecta. "
            f"Leak principal ({sf_contrib:.2f} m/s2, {100*sf_contrib/max(a_mean,1e-6):.0f}% de A) "
            "procede de f_horizontal en body. "
            f"Residuo B ({b_mean:.2f} m/s2, ~{residual_b_arcmin_deg:.1f} deg equiv.) "
            "merece analisis aparte: pitch/roll real, ruido, o [0,0,az] != g_body_pred."
        )
    elif not attitude_self_consistent:
        verdict = "H2_ATTITUDE_PROJECTION_DOMINANT"
        summary = (
            "R_bn no es auto-coherente con g_body_pred (prueba D falla). "
            "Revisar quat_integrate / convencion DCM."
        )
    elif b_ratio >= ATTITUDE_LEAK_RATIO_H2 or c_ratio >= ATTITUDE_LEAK_RATIO_H2:
        verdict = "H2_ATTITUDE_PROJECTION_DOMINANT"
        summary = (
            "Incluso con a_corr vertical en body (B/C), persiste leak horizontal significativo "
            "y D no confirma coherencia. Revisar actitud."
        )
    else:
        verdict = "MIXED_MECHANISM"
        summary = (
            "Contribucion mixta entre fuerza especifica horizontal y proyeccion vertical "
            "bajo pitch/roll. R_bn parece coherente (D~0)."
        )

    f_static_h = static.get("f_residual_h_mean", float("nan"))

    return {
        "verdict": verdict,
        "summary": summary,
        "abc_comparison": {
            "motion_a_lin_h_A_mean": a_mean,
            "motion_a_lin_h_B_mean": b_mean,
            "motion_a_lin_h_C_mean": c_mean,
            "motion_a_lin_h_D_mean": d_mean,
            "motion_a_lin_h_D_max": d_max,
            "B_over_A_ratio": b_ratio,
            "C_over_A_ratio": c_ratio,
            "leak_explained_by_B_fraction_mean": leak_ab,
            "leak_explained_by_C_fraction_mean": leak_ac,
            "specific_force_contribution_mean": sf_contrib,
            "attitude_vertical_only_leak_mean": b_mean,
            "B_minus_D_mean": b_minus_d,
            "residual_B_equiv_tilt_deg": residual_b_arcmin_deg,
            "test_D_interpretation": (
                "Identidad algebraica: valida coherencia interna quat/DCM/proyeccion. "
                "NO valida que R_bn represente orientacion fisica correcta."
            ),
            "attitude_algebraic_self_consistent": attitude_self_consistent,
            "interpretation": {
                "A_ne_B": abs(a_mean - b_mean) > 0.1,
                "B_still_leaks": b_mean > 0.15,
                "C_vs_B": "modulo conservado" if abs(c_mean - b_mean) > 0.05 else "modulo irrelevante",
            },
        },
        "residual_instrumentation": {
            "static_f_residual_h_mean": f_static_h,
            "static_f_residual_h_p95": static.get("f_residual_h_p95", float("nan")),
            "motion_f_residual_x_mean": motion.get("f_residual_x_mean", float("nan")),
            "motion_f_residual_y_mean": motion.get("f_residual_y_mean", float("nan")),
            "motion_f_residual_z_mean": motion.get("f_residual_z_mean", float("nan")),
            "motion_f_residual_h_mean": motion.get("f_residual_h_mean", float("nan")),
            "motion_a_corr_h_mean": motion.get("a_corr_h_mean", float("nan")),
            "corr_f_residual_x_vs_a_corr_x": motion_corr.get("f_residual_x_vs_a_corr_x", float("nan")),
            "corr_f_residual_h_vs_a_corr_h": motion_corr.get("f_residual_h_vs_a_corr_h", float("nan")),
            "corr_f_residual_y_vs_a_corr_y": motion_corr.get("f_residual_y_vs_a_corr_y", float("nan")),
            "expected_at_rest": "f_residual ~ 0",
            "expected_longitudinal_accel": "f_residual_x ~ componente longitudinal en body FRD (+X forward)",
        },
        "dcm_validation": {
            "motion_dcm_replay_err_h_mean": motion.get("dcm_replay_err_h_mean", float("nan")),
            "motion_dcm_replay_err_h_max": motion.get("dcm_replay_err_h_max", float("nan")),
        },
    }


def write_csv(samples: list[TickSample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "timestamp_s",
        "a_corr_x", "a_corr_y", "a_corr_z", "a_corr_h",
        "g_body_pred_x", "g_body_pred_y", "g_body_pred_z",
        "f_residual_x", "f_residual_y", "f_residual_z", "f_residual_h",
        "a_lin_h_A", "a_lin_h_B", "a_lin_h_C", "a_lin_h_D", "a_lin_h_replay",
        "leak_frac_AB", "leak_frac_AC",
        "gravity_align_deg", "roll_deg", "pitch_deg", "yaw_deg",
        "dcm_replay_err_h",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for s in samples:
            w.writerow(
                {
                    "timestamp_s": s.timestamp_s,
                    "a_corr_x": s.a_corr[0],
                    "a_corr_y": s.a_corr[1],
                    "a_corr_z": s.a_corr[2],
                    "a_corr_h": s.a_corr_h,
                    "g_body_pred_x": s.g_body_pred[0],
                    "g_body_pred_y": s.g_body_pred[1],
                    "g_body_pred_z": s.g_body_pred[2],
                    "f_residual_x": s.f_residual_x,
                    "f_residual_y": s.f_residual_y,
                    "f_residual_z": s.f_residual_z,
                    "f_residual_h": s.f_residual_h,
                    "a_lin_h_A": s.a_lin_h_a,
                    "a_lin_h_B": s.a_lin_h_b,
                    "a_lin_h_C": s.a_lin_h_c,
                    "a_lin_h_D": s.a_lin_h_d,
                    "a_lin_h_replay": s.a_lin_h_replay,
                    "leak_frac_AB": s.leak_frac_ab,
                    "leak_frac_AC": s.leak_frac_ac,
                    "gravity_align_deg": s.gravity_align_deg,
                    "roll_deg": s.roll_deg,
                    "pitch_deg": s.pitch_deg,
                    "yaw_deg": s.yaw_deg,
                    "dcm_replay_err_h": s.dcm_replay_err_h,
                }
            )


def plot(samples: list[TickSample], path: Path) -> None:
    post = [s for s in samples if s.timestamp_s >= 1.5]
    t = np.array([s.timestamp_s for s in post])

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    fig.suptitle("GAP-2: descomposicion fuerza especifica (A/B/C) + f_residual", fontsize=13)

    axes[0].plot(t, [s.a_lin_h_a for s in post], label="A R_bn·a_corr", linewidth=0.8)
    axes[0].plot(t, [s.a_lin_h_b for s in post], label="B R_bn·[0,0,az]", linewidth=0.8)
    axes[0].plot(t, [s.a_lin_h_c for s in post], label="C R_bn·||a||·e_z", linewidth=0.8, alpha=0.8)
    axes[0].plot(t, [s.a_lin_h_replay for s in post], label="replay a_lin,h", linewidth=0.6, linestyle="--", alpha=0.7)
    axes[0].axvspan(MOTION_T0, MOTION_T1, color="#f9e79f", alpha=0.3)
    axes[0].set_ylabel("|a_lin,h| [m/s2]")
    axes[0].legend(fontsize=7)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(t, [s.f_residual_x for s in post], label="f_res X (long)", linewidth=0.8)
    axes[1].plot(t, [s.f_residual_y for s in post], label="f_res Y (lat)", linewidth=0.8, alpha=0.8)
    axes[1].plot(t, [s.f_residual_z for s in post], label="f_res Z", linewidth=0.8, alpha=0.7)
    axes[1].axvspan(MOTION_T0, MOTION_T1, color="#f9e79f", alpha=0.3)
    axes[1].set_ylabel("f_residual [m/s2]")
    axes[1].legend(fontsize=7)
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(t, [s.a_corr_h for s in post], label="|a_corr|_h body", linewidth=0.8)
    axes[2].plot(t, [s.f_residual_h for s in post], label="|f_residual|_h body", linewidth=0.8)
    axes[2].plot(t, [s.leak_frac_ab * 100.0 for s in post], label="leak explicado B (%)", linewidth=0.8, alpha=0.7)
    axes[2].axvspan(MOTION_T0, MOTION_T1, color="#f9e79f", alpha=0.3)
    axes[2].set_ylabel("body / %")
    axes[2].legend(fontsize=7)
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(t, [s.a_corr[0] for s in post], label="a_corr X", linewidth=0.8)
    axes[3].plot(t, [s.gravity_align_deg for s in post], label="gravity_align", linewidth=0.8, alpha=0.7)
    axes[3].axvspan(MOTION_T0, MOTION_T1, color="#f9e79f", alpha=0.3)
    axes[3].set_xlabel("t [s]")
    axes[3].set_ylabel("m/s2 / deg")
    axes[3].legend(fontsize=7)
    axes[3].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def load_chain(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open(newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            if raw.get("timestamp_s"):
                rows.append(raw)
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-2 specific force decomposition A/B/C")
    parser.add_argument("--chain-csv", type=Path, default=CHAIN_CSV)
    args = parser.parse_args()

    if not args.chain_csv.is_file():
        print(f"ERROR: falta {args.chain_csv}", file=sys.stderr)
        return 1

    samples = build_samples(load_chain(args.chain_csv))
    static = window_stats(samples, 0.0, STATIC_END_S)
    motion = window_stats(samples, MOTION_T0, MOTION_T1)
    motion_corr = {
        "f_residual_x_vs_a_corr_x": corr(samples, MOTION_T0, MOTION_T1, "f_residual_x", "a_corr_x"),
        "f_residual_h_vs_a_corr_h": corr(samples, MOTION_T0, MOTION_T1, "f_residual_h", "a_corr_h"),
        "f_residual_y_vs_a_corr_y": corr(samples, MOTION_T0, MOTION_T1, "f_residual_y", "a_corr_y"),
        "a_lin_h_A_vs_replay": corr(samples, MOTION_T0, MOTION_T1, "a_lin_h_a", "a_lin_h_replay"),
    }
    diagnosis = diagnose(static, motion, motion_corr)

    write_csv(samples, MERGED_CSV)
    plot(samples, ANALYSIS_PNG)

    report = {
        "experiment": "gap2_specific_force_decomposition",
        "tests": {
            "A": "a_nav = R_bn @ a_corr",
            "B": "a_nav = R_bn @ [0, 0, a_corr.z]",
            "C": "a_nav = R_bn @ (||a_corr|| * [0,0,1])",
            "D": "a_nav = R_bn @ g_body_pred (sanity: debe dar |a_lin,h|~0)",
        },
        "instrumentation": "f_residual = a_corr - g_body_pred",
        "windows": {"static_s": [0.0, STATIC_END_S], "motion_s": [MOTION_T0, MOTION_T1]},
        "static_stats": static,
        "motion_stats": motion,
        "motion_correlations": motion_corr,
        "diagnosis": diagnosis,
        "artifacts": {"merged_csv": str(MERGED_CSV), "plot_png": str(ANALYSIS_PNG)},
    }
    with REPORT_JSON.open("w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write("\n")

    d = diagnosis
    abc = d["abc_comparison"]
    res = d["residual_instrumentation"]
    print("=" * 72)
    print("GAP-2 descomposicion fuerza especifica — pruebas A/B/C")
    print("=" * 72)
    print(f"  DCM vs replay err_h (motion mean): {motion.get('dcm_replay_err_h_mean', float('nan')):.6f} m/s2")
    print(f"  corr(a_lin_h_A, replay):           {motion_corr.get('a_lin_h_A_vs_replay', float('nan')):.4f}")
    print()
    print("  Estatico f_residual_h mean/p95:")
    print(f"    {static.get('f_residual_h_mean', float('nan')):.4f} / {static.get('f_residual_h_p95', float('nan')):.4f} m/s2")
    print()
    print("  Motion 2-10 s |a_lin,h| mean:")
    print(f"    A (actual):  {abc['motion_a_lin_h_A_mean']:.3f} m/s2")
    print(f"    B (solo Z):  {abc['motion_a_lin_h_B_mean']:.3f} m/s2  (ratio B/A={abc['B_over_A_ratio']:.3f})")
    print(f"    C (||a|| ez): {abc['motion_a_lin_h_C_mean']:.3f} m/s2  (ratio C/A={abc['C_over_A_ratio']:.3f})")
    print(f"    D (g_pred):   {abc.get('motion_a_lin_h_D_mean', float('nan')):.6f} m/s2  (max={abc.get('motion_a_lin_h_D_max', float('nan')):.6f})")
    print(f"    leak explicado por B: {abc['leak_explained_by_B_fraction_mean']*100:.1f}%")
    print()
    print("  f_residual motion mean [X,Y,Z,H]:")
    print(f"    [{res['motion_f_residual_x_mean']:.3f}, {res['motion_f_residual_y_mean']:.3f}, "
          f"{res['motion_f_residual_z_mean']:.3f}, {res['motion_f_residual_h_mean']:.3f}] m/s2")
    print(f"  corr(f_res_x, a_corr_x)={motion_corr.get('f_residual_x_vs_a_corr_x', float('nan')):.3f}")
    print(f"  corr(f_res_h, a_corr_h)={motion_corr.get('f_residual_h_vs_a_corr_h', float('nan')):.3f}")
    print()
    print(f"  VEREDICTO: {d['verdict']}")
    print(f"  {d['summary']}")
    print(f"  Informe: {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
