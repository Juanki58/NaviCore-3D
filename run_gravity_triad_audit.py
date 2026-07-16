#!/usr/bin/env python3
"""Auditoria triada de gravedad en body frame: pred (EKF) vs ref (Orient) vs meas (accel).

En cada tick:
  g_body_pred = R_bn_EKF^T * g_ned
  g_body_ref  = R_bn_Orient^T * g_ned   (offset montaje estatico roll/pitch)
  g_body_meas = normalize(a_corr)

Angulos: pred<->ref, pred<->meas, ref<->meas

Separa: EKF vs Android vs acelerometro sin debatir contaminacion dinamica.
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
MERGED_CSV = BENCH_DIR / "gravity_triad_merged.csv"
REPORT_JSON = BENCH_DIR / "gravity_triad_report.json"
ANALYSIS_PNG = BENCH_DIR / "gravity_triad_analysis.png"

STATIC_END_S = 2.0
MOTION_END_S = 10.0
CRUISE_T0 = 11.4
CRUISE_T1 = 25.4
STATIC_OFFSET_END_S = 2.0

from analyze_real_run import (  # noqa: E402
    discover_t0_ns,
    estimate_mount_offset_deg,
    interpolate_series,
    load_orientation,
    resolve_orientation_path,
)
from attitude_kinematics import (  # noqa: E402
    angle_between_deg,
    euler321_to_dcm_bn,
    g_body_from_dcm,
)


@dataclass
class TriadSample:
    timestamp_s: float
    g_pred: np.ndarray
    g_ref: np.ndarray
    g_meas: np.ndarray
    angle_pred_ref_deg: float
    angle_pred_meas_deg: float
    angle_ref_meas_deg: float
    a_lin_h: float
    gps_speed_mps: float


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


def vec_from_row(row: dict[str, float], prefix: str) -> np.ndarray:
    return np.array(
        [row.get(f"{prefix}_x", 0.0), row.get(f"{prefix}_y", 0.0), row.get(f"{prefix}_z", 0.0)],
        dtype=float,
    )


def build_triad_samples(
    chain_rows: list[dict[str, float]],
    orientation_path: Path,
    input_dir: Path,
) -> list[TriadSample]:
    t0_ns = discover_t0_ns(input_dir if input_dir.is_dir() else None)
    orient = load_orientation(orientation_path, t0_ns)
    times = np.array([r["timestamp_s"] for r in chain_rows], dtype=float)
    roll_ekf = np.array([r.get("roll_deg", 0.0) for r in chain_rows], dtype=float)
    pitch_ekf = np.array([r.get("pitch_deg", 0.0) for r in chain_rows], dtype=float)
    yaw_ekf = np.array([r.get("yaw_deg", 0.0) for r in chain_rows], dtype=float)

    o_times = np.array([s.timestamp_s for s in orient], dtype=float)
    roll_o = np.array([s.roll_deg for s in orient], dtype=float)
    pitch_o = np.array([s.pitch_deg for s in orient], dtype=float)
    yaw_o = np.array([s.yaw_deg for s in orient], dtype=float)

    roll_ref = interpolate_series(times, o_times, roll_o)
    pitch_ref = interpolate_series(times, o_times, pitch_o)
    yaw_ref = interpolate_series(times, o_times, yaw_o)

    static_mask = times <= STATIC_OFFSET_END_S
    if not np.any(static_mask):
        static_mask = np.ones_like(times, dtype=bool)

    roll_off, pitch_off, _yaw_off = estimate_mount_offset_deg(
        roll_ref,
        pitch_ref,
        yaw_ref,
        roll_ekf,
        pitch_ekf,
        yaw_ekf,
        times,
        static_end_s=float(np.max(times[static_mask])),
    )

    samples: list[TriadSample] = []
    for idx, row in enumerate(chain_rows):
        g_pred = vec_from_row(row, "g_body_pred")
        g_meas = vec_from_row(row, "g_body_meas")
        if np.linalg.norm(g_pred) < 1e-6:
            g_pred = g_body_from_dcm(
                euler321_to_dcm_bn(
                    math.radians(roll_ekf[idx]),
                    math.radians(pitch_ekf[idx]),
                    math.radians(yaw_ekf[idx]),
                )
            )

        roll_r = math.radians(roll_ref[idx] - roll_off)
        pitch_r = math.radians(pitch_ref[idx] - pitch_off)
        yaw_r = math.radians(yaw_ref[idx])
        g_ref = g_body_from_dcm(euler321_to_dcm_bn(roll_r, pitch_r, yaw_r))

        samples.append(
            TriadSample(
                timestamp_s=row["timestamp_s"],
                g_pred=g_pred,
                g_ref=g_ref,
                g_meas=g_meas,
                angle_pred_ref_deg=angle_between_deg(g_pred, g_ref),
                angle_pred_meas_deg=angle_between_deg(g_pred, g_meas),
                angle_ref_meas_deg=angle_between_deg(g_ref, g_meas),
                a_lin_h=row.get("a_lin_h", 0.0),
                gps_speed_mps=row.get("gps_speed_mps", 0.0),
            )
        )
    return samples


def window(samples: list[TriadSample], t0: float, t1: float) -> list[TriadSample]:
    return [s for s in samples if t0 <= s.timestamp_s <= t1]


def summarize(samples: list[TriadSample], label: str) -> dict:
    if not samples:
        return {"label": label, "samples": 0}
    pred_ref = np.array([s.angle_pred_ref_deg for s in samples], dtype=float)
    pred_meas = np.array([s.angle_pred_meas_deg for s in samples], dtype=float)
    ref_meas = np.array([s.angle_ref_meas_deg for s in samples], dtype=float)
    alin = np.array([s.a_lin_h for s in samples], dtype=float)
    out = {
        "label": label,
        "samples": len(samples),
        "angle_pred_ref_deg_mean": float(np.mean(pred_ref)),
        "angle_pred_ref_deg_median": float(np.median(pred_ref)),
        "angle_pred_meas_deg_mean": float(np.mean(pred_meas)),
        "angle_ref_meas_deg_mean": float(np.mean(ref_meas)),
        "a_lin_h_mean": float(np.mean(alin)),
    }
    if len(samples) > 2:
        out["corr_pred_ref_vs_a_lin_h"] = float(np.corrcoef(pred_ref, alin)[0, 1])
        out["corr_pred_meas_vs_a_lin_h"] = float(np.corrcoef(pred_meas, alin)[0, 1])
        out["corr_pred_ref_vs_pred_meas"] = float(np.corrcoef(pred_ref, pred_meas)[0, 1])
    return out


def diagnose(samples: list[TriadSample]) -> dict:
    post = [s for s in samples if s.timestamp_s >= 1.5]
    static = window(post, 0.0, STATIC_END_S)
    motion = window(post, STATIC_END_S, MOTION_END_S)
    cruise = window(post, CRUISE_T0, CRUISE_T1)

    static_sum = summarize(static, "static_0_2s")
    motion_sum = summarize(motion, "motion_2_10s")
    cruise_sum = summarize(cruise, "cruise_11_25s")

    def jump(key: str) -> float:
        return motion_sum.get(key, 0.0) - static_sum.get(key, 0.0)

    jumps = {
        "angle_pred_ref_deg": jump("angle_pred_ref_deg_mean"),
        "angle_pred_meas_deg": jump("angle_pred_meas_deg_mean"),
        "angle_ref_meas_deg": jump("angle_ref_meas_deg_mean"),
        "a_lin_h": jump("a_lin_h_mean"),
    }

    pr_jump = jumps["angle_pred_ref_deg"]
    pm_jump = jumps["angle_pred_meas_deg"]
    rm_jump = jumps["angle_ref_meas_deg"]
    static_pr = static_sum.get("angle_pred_ref_deg_mean", 0.0)

    mechanism = "inconclusive"
    if static_pr < 0.5 and pr_jump >= 2.0:
        mechanism = "ekf_tilt_diverges_from_orientation_in_dynamic_regime"
    elif pm_jump >= 2.0 and pr_jump < 1.5:
        mechanism = "accel_contamination_pred_still_matches_ref"

    hourly: list[dict] = []
    for t0 in range(2, 11):
        t1 = t0 + 1
        w = window(post, float(t0), float(t1))
        if w:
            hourly.append(summarize(w, f"t_{t0}_{t1}s"))

    return {
        "static_0_2s": static_sum,
        "motion_2_10s": motion_sum,
        "cruise_11_25s": cruise_sum,
        "jumps_static_to_motion": jumps,
        "hourly_motion_profile": hourly,
        "likely_mechanism": mechanism,
        "notes": {
            "static_triad_agreement_deg": {
                "pred_ref": static_sum.get("angle_pred_ref_deg_mean"),
                "pred_meas": static_sum.get("angle_pred_meas_deg_mean"),
                "ref_meas": static_sum.get("angle_ref_meas_deg_mean"),
            },
            "ref_meas_jump_expected_in_dynamics": rm_jump >= 2.0,
            "cannot_be_heading_only": (
                "pred<->ref usa solo inclinacion (yaw no afecta g_body); "
                "salto con pred_ref bajo en estatico descarta offset de marco estatico."
            ),
        },
        "interpretation": (
            "Estatico: pred, ref y meas coinciden (~0.05-0.13 deg). "
            "En dinamica fuerte pred<->ref crece (EKF != Android). "
            "ref<->meas tambien crece (accel != gravedad); eso no implica que EKF este bien."
        ),
    }


def write_merged(samples: list[TriadSample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "timestamp_s",
                "angle_pred_ref_deg",
                "angle_pred_meas_deg",
                "angle_ref_meas_deg",
                "a_lin_h",
                "gps_speed_mps",
                "g_pred_x",
                "g_pred_y",
                "g_pred_z",
                "g_ref_x",
                "g_ref_y",
                "g_ref_z",
                "g_meas_x",
                "g_meas_y",
                "g_meas_z",
            ],
        )
        writer.writeheader()
        for s in samples:
            writer.writerow(
                {
                    "timestamp_s": s.timestamp_s,
                    "angle_pred_ref_deg": s.angle_pred_ref_deg,
                    "angle_pred_meas_deg": s.angle_pred_meas_deg,
                    "angle_ref_meas_deg": s.angle_ref_meas_deg,
                    "a_lin_h": s.a_lin_h,
                    "gps_speed_mps": s.gps_speed_mps,
                    "g_pred_x": s.g_pred[0],
                    "g_pred_y": s.g_pred[1],
                    "g_pred_z": s.g_pred[2],
                    "g_ref_x": s.g_ref[0],
                    "g_ref_y": s.g_ref[1],
                    "g_ref_z": s.g_ref[2],
                    "g_meas_x": s.g_meas[0],
                    "g_meas_y": s.g_meas[1],
                    "g_meas_z": s.g_meas[2],
                }
            )


def plot_analysis(samples: list[TriadSample], path: Path) -> None:
    post = [s for s in samples if s.timestamp_s >= 1.5]
    times = np.array([s.timestamp_s for s in post], dtype=float)
    pr = np.array([s.angle_pred_ref_deg for s in post], dtype=float)
    pm = np.array([s.angle_pred_meas_deg for s in post], dtype=float)
    rm = np.array([s.angle_ref_meas_deg for s in post], dtype=float)
    alin = np.array([s.a_lin_h for s in post], dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Gravity triad: g_body pred / ref / meas", fontsize=14)

    axes[0].plot(times, pr, label="pred<->ref (EKF vs Orient)", linewidth=0.8)
    axes[0].plot(times, pm, label="pred<->meas", linewidth=0.8, alpha=0.85)
    axes[0].plot(times, rm, label="ref<->meas (Orient vs accel)", linewidth=0.8, alpha=0.85)
    axes[0].axvline(STATIC_END_S, color="#7f8c8d", linestyle=":")
    axes[0].set_ylabel("[deg]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, alin, color="#8e44ad", linewidth=0.8)
    axes[1].set_ylabel("a_lin_h [m/s2]")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(times, np.array([s.gps_speed_mps for s in post]), color="#27ae60", linewidth=0.8)
    axes[2].set_ylabel("GPS speed")
    axes[2].set_xlabel("Tiempo [s]")
    axes[2].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Gravity triad audit")
    parser.add_argument("--chain-csv", type=Path, default=CHAIN_CSV)
    parser.add_argument("--orientation", type=Path, default=None)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    args = parser.parse_args()

    if not args.chain_csv.is_file():
        print(f"ERROR: falta {args.chain_csv}", file=sys.stderr)
        return 1
    orientation_path = resolve_orientation_path(args.orientation)
    if orientation_path is None:
        print("ERROR: no se encontro Orientation.csv", file=sys.stderr)
        return 1

    chain = load_chain(args.chain_csv)
    samples = build_triad_samples(chain, orientation_path, args.input_dir)
    diagnosis = diagnose(samples)
    write_merged(samples, MERGED_CSV)
    plot_analysis(samples, ANALYSIS_PNG)

    report = {
        "experiment": "gravity_triad_audit",
        "question": (
            "Por que R_bn desarrolla ~4 deg error de inclinacion al entrar en dinamica "
            "mientras heading horizontal sigue coherente?"
        ),
        "triad": "g_body_pred (EKF), g_body_ref (Orientation), g_body_meas (accel norm)",
        "angles": ["pred_ref", "pred_meas", "ref_meas"],
        "diagnosis": diagnosis,
        "artifacts": {"merged_csv": str(MERGED_CSV), "plot_png": str(ANALYSIS_PNG)},
    }
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    jumps = diagnosis["jumps_static_to_motion"]
    print("=" * 72)
    print("Gravity triad audit (pred / ref / meas)")
    print("=" * 72)
    print(f"  Static pred<->ref:  {diagnosis['static_0_2s'].get('angle_pred_ref_deg_mean', float('nan')):.3f} deg")
    print(f"  Motion pred<->ref:  {diagnosis['motion_2_10s'].get('angle_pred_ref_deg_mean', float('nan')):.3f} deg")
    print(f"  Salto pred<->ref:   {jumps.get('angle_pred_ref_deg', float('nan')):.3f} deg")
    print(f"  Salto pred<->meas:  {jumps.get('angle_pred_meas_deg', float('nan')):.3f} deg")
    print(f"  Salto ref<->meas:   {jumps.get('angle_ref_meas_deg', float('nan')):.3f} deg")
    print(f"  Mecanismo:          {diagnosis.get('likely_mechanism')}")
    print(f"  Informe: {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
