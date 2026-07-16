#!/usr/bin/env python3
"""Auditoria final: cadena sensor -> mount -> body -> bias -> R_bn -> nav -> a_lin.

Objetivo: decidir entre dos mecanismos fisicos:
  (1) R_bn deja de representar la orientacion real en dinamica
  (2) transformacion sensor/body/nav proyecta mal la aceleracion longitudinal
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
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"

CHAIN_CSV = BENCH_DIR / "propagation_chain_audit.csv"
H9C_MERGED = BENCH_DIR / "h9c_orientation_merged.csv"
REPORT_JSON = BENCH_DIR / "propagation_chain_audit_report.json"
ANALYSIS_PNG = BENCH_DIR / "propagation_chain_audit_analysis.png"

PREDICT_ONLY_END_S = 60.0
STATIC_END_S = 2.0
MOTION_END_S = 10.0

from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


STAGE_PREFIXES = (
    "a_raw",
    "a_body",
    "bias",
    "a_corr",
    "a_nav_body",
    "a_nav_corr",
    "r_bn_bias",
    "nav_body_minus_corr",
    "a_lin",
)


@dataclass
class ChainSample:
    timestamp_s: float
    stages: dict[str, np.ndarray]
    stage_norms: dict[str, float]
    stage_h: dict[str, float]
    gravity_angle_deg: float
    proj_body: np.ndarray
    proj_ned: np.ndarray
    roll_deg: float
    pitch_deg: float
    gps_speed_mps: float
    h9a_applied: bool


def parse_vec(row: dict, prefix: str) -> np.ndarray:
    return np.array(
        [float(row.get(f"{prefix}_x", 0.0) or 0.0),
         float(row.get(f"{prefix}_y", 0.0) or 0.0),
         float(row.get(f"{prefix}_z", 0.0) or 0.0)],
        dtype=float,
    )


def load_chain_csv(path: Path) -> list[ChainSample]:
    rows: list[ChainSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t_text = raw.get("timestamp_s")
            if not t_text:
                continue
            stages = {p: parse_vec(raw, p) for p in STAGE_PREFIXES}
            stage_norms = {p: float(raw.get(f"{p}_norm", 0.0) or 0.0) for p in STAGE_PREFIXES}
            stage_h = {}
            for p in STAGE_PREFIXES:
                key = f"{p}_h"
                if key in raw and raw[key]:
                    stage_h[p] = float(raw[key])
            rows.append(
                ChainSample(
                    timestamp_s=float(t_text),
                    stages=stages,
                    stage_norms=stage_norms,
                    stage_h=stage_h,
                    gravity_angle_deg=float(raw.get("gravity_angle_deg") or 0.0),
                    proj_body=np.array(
                        [
                            float(raw.get("proj_body_long_mps2") or 0.0),
                            float(raw.get("proj_body_lat_mps2") or 0.0),
                            float(raw.get("proj_body_vert_mps2") or 0.0),
                        ],
                        dtype=float,
                    ),
                    proj_ned=parse_vec(raw, "a_lin"),
                    roll_deg=float(raw.get("roll_deg") or 0.0),
                    pitch_deg=float(raw.get("pitch_deg") or 0.0),
                    gps_speed_mps=float(raw.get("gps_speed_mps") or 0.0),
                    h9a_applied=bool(int(float(raw.get("h9a_applied") or 0))),
                )
            )
    if not rows:
        raise ValueError(f"CSV vacio: {path}")
    rows.sort(key=lambda sample: sample.timestamp_s)
    return rows


def run_chain_replay(replay_csv: Path, replay_exe: Path, calibration: Path) -> None:
    cmd = [
        str(replay_exe),
        "--input", str(replay_csv),
        "--output", str(BENCH_DIR / "propagation_chain_replay_output.csv"),
        "--mount-mode", "calibration",
        "--mount-calibration", str(calibration),
        "--yaw-init", "zero",
        "--predict-only", "--predict-only-end-s", str(PREDICT_ONLY_END_S),
        "--h9a-gravity-tilt-init",
        "--propagation-chain-audit-csv", str(CHAIN_CSV),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def post_init(samples: list[ChainSample]) -> list[ChainSample]:
    post = [s for s in samples if s.h9a_applied]
    return post if post else samples


def window(samples: list[ChainSample], t0: float, t1: float) -> list[ChainSample]:
    return [s for s in samples if t0 <= s.timestamp_s <= t1]


def summarize(samples: list[ChainSample], label: str) -> dict:
    if not samples:
        return {"label": label, "samples": 0}

    def mean_h(key: str) -> float:
        vals = [s.stage_h[key] for s in samples if key in s.stage_h]
        return float(np.mean(vals)) if vals else float("nan")

    out = {
        "label": label,
        "samples": len(samples),
        "gravity_angle_deg_mean": float(np.mean([s.gravity_angle_deg for s in samples])),
        "a_lin_h_mean": mean_h("a_lin"),
        "a_nav_body_h_mean": mean_h("a_nav_body"),
        "a_nav_corr_h_mean": mean_h("a_nav_corr"),
        "r_bn_bias_h_mean": mean_h("r_bn_bias"),
        "nav_body_minus_corr_h_mean": mean_h("nav_body_minus_corr"),
        "proj_body_long_mean": float(np.mean([s.proj_body[0] for s in samples])),
        "proj_body_lat_mean": float(np.mean([s.proj_body[1] for s in samples])),
    }
    if len(samples) > 2:
        alin = np.array([s.stage_h.get("a_lin", 0.0) for s in samples], dtype=float)
        ang = np.array([s.gravity_angle_deg for s in samples], dtype=float)
        navb = np.array([s.stage_h.get("a_nav_body", 0.0) for s in samples], dtype=float)
        navc = np.array([s.stage_h.get("a_nav_corr", 0.0) for s in samples], dtype=float)
        rb = np.array([s.stage_h.get("r_bn_bias", 0.0) for s in samples], dtype=float)
        out["corr_gravity_angle_vs_a_lin_h"] = float(np.corrcoef(ang, alin)[0, 1])
        out["corr_a_nav_body_h_vs_a_lin_h"] = float(np.corrcoef(navb, alin)[0, 1])
        out["corr_a_nav_corr_h_vs_a_lin_h"] = float(np.corrcoef(navc, alin)[0, 1])
        out["corr_r_bn_bias_h_vs_a_lin_h"] = float(np.corrcoef(rb, alin)[0, 1])
    return out


def load_h9c_delta_tilt(path: Path) -> dict[float, float]:
    out: dict[float, float] = {}
    if not path.is_file():
        return out
    with path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            if not raw.get("timestamp_s"):
                continue
            out[float(raw["timestamp_s"])] = float(raw.get("delta_tilt_mag_deg") or 0.0)
    return out


def diagnose(samples: list[ChainSample], h9c_delta: dict[float, float]) -> dict:
    post = post_init(samples)
    static = window(post, 0.0, STATIC_END_S)
    motion = window(post, STATIC_END_S, MOTION_END_S)
    static_sum = summarize(static, "static_0_2s")
    motion_sum = summarize(motion, "motion_2_10s")

    def jump(key: str) -> float:
        return motion_sum.get(key, 0.0) - static_sum.get(key, 0.0)

    jumps = {
        "gravity_angle_deg": jump("gravity_angle_deg_mean"),
        "a_lin_h": jump("a_lin_h_mean"),
        "a_nav_body_h": jump("a_nav_body_h_mean"),
        "a_nav_corr_h": jump("a_nav_corr_h_mean"),
        "r_bn_bias_h": jump("r_bn_bias_h_mean"),
        "nav_body_minus_corr_h": jump("nav_body_minus_corr_h_mean"),
        "proj_body_long": jump("proj_body_long_mean"),
    }

    # Sanity: nav_body_minus_corr should track r_bn_bias_h
    rb_static = static_sum.get("r_bn_bias_h_mean", float("nan"))
    rb_motion = motion_sum.get("r_bn_bias_h_mean", float("nan"))
    nbc_static = static_sum.get("nav_body_minus_corr_h_mean", float("nan"))
    nbc_motion = motion_sum.get("nav_body_minus_corr_h_mean", float("nan"))

    bias_frame_consistent = (
        math.isfinite(rb_static)
        and math.isfinite(nbc_static)
        and abs(rb_static - nbc_static) < 0.02
        and abs(rb_motion - nbc_motion) < 0.05
    )

    # Can bias explain the a_lin jump?
    alin_jump = jumps["a_lin_h"]
    nav_corr_jump = jumps["a_nav_corr_h"]
    nav_body_jump = jumps["a_nav_body_h"]
    rbias_jump = jumps["r_bn_bias_h"]
    grav_jump = jumps["gravity_angle_deg"]

    bias_explains_jump = abs(rbias_jump) >= 0.5 * abs(alin_jump) if abs(alin_jump) > 0.05 else False

    mechanism = "inconclusive"
    if grav_jump >= 2.0 and motion_sum.get("corr_gravity_angle_vs_a_lin_h", 0.0) > 0.9:
        mechanism = "mechanism_1_attitude_r_bn_misrepresents_orientation"
    elif (
        abs(nav_corr_jump - nav_body_jump) < 0.1 * max(abs(nav_corr_jump), 0.01)
        and not bias_explains_jump
    ):
        mechanism = "mechanism_2_mount_or_body_nav_projection"
    elif bias_explains_jump and bias_frame_consistent:
        mechanism = "bias_projection_amplifies_with_attitude_change"
    elif grav_jump >= 2.0:
        mechanism = "mechanism_1_attitude_dominant_with_projection_coupling"

    h9c_corr = float("nan")
    if h9c_delta and motion:
        pairs = []
        for s in motion:
            keys = np.array(sorted(h9c_delta.keys()), dtype=float)
            idx = int(np.argmin(np.abs(keys - s.timestamp_s)))
            if abs(keys[idx] - s.timestamp_s) <= 0.05:
                pairs.append((s.gravity_angle_deg, h9c_delta[float(keys[idx])]))
        if len(pairs) > 10:
            a = np.array([p[0] for p in pairs], dtype=float)
            b = np.array([p[1] for p in pairs], dtype=float)
            h9c_corr = float(np.corrcoef(a, b)[0, 1])

    regime = "step_transition" if grav_jump >= 2.0 and alin_jump >= 0.4 else "gradual_or_flat"

    return {
        "static_0_2s": static_sum,
        "motion_2_10s": motion_sum,
        "jumps_static_to_motion": jumps,
        "bias_frame_consistent": bias_frame_consistent,
        "bias_explains_a_lin_jump": bias_explains_jump,
        "regime_pattern": regime,
        "corr_h9c_gravity_angle_vs_orient_delta_tilt_motion": h9c_corr,
        "likely_mechanism": mechanism,
        "decision": {
            "mechanism_1_attitude": mechanism.startswith("mechanism_1"),
            "mechanism_2_mount_projection": mechanism == "mechanism_2_mount_or_body_nav_projection",
            "bias_projection_artifact": mechanism == "bias_projection_amplifies_with_attitude_change",
        },
    }


def plot_analysis(samples: list[ChainSample], path: Path) -> None:
    post = post_init(samples)
    times = np.array([s.timestamp_s for s in post], dtype=float)
    grav = np.array([s.gravity_angle_deg for s in post], dtype=float)
    alin = np.array([s.stage_h.get("a_lin", 0.0) for s in post], dtype=float)
    navb = np.array([s.stage_h.get("a_nav_body", 0.0) for s in post], dtype=float)
    navc = np.array([s.stage_h.get("a_nav_corr", 0.0) for s in post], dtype=float)
    rb = np.array([s.stage_h.get("r_bn_bias", 0.0) for s in post], dtype=float)

    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
    fig.suptitle("Propagation chain audit (sensor -> mount -> body -> nav -> a_lin)", fontsize=14)

    axes[0].plot(times, grav, color="#c0392b", linewidth=0.8, label="gravity angle")
    axes[0].axvline(STATIC_END_S, color="#7f8c8d", linestyle=":", label="2 s")
    axes[0].set_ylabel("[deg]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, alin, label="a_lin_h", linewidth=0.8)
    axes[1].plot(times, navb, label="a_nav_body_h", linewidth=0.8, alpha=0.85)
    axes[1].plot(times, navc, label="a_nav_corr_h", linewidth=0.8, alpha=0.85)
    axes[1].plot(times, rb, label="R_bn*bias_h", linewidth=0.7, alpha=0.7)
    axes[1].set_ylabel("[m/s2]")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.25)

    norms = np.array([[s.stage_norms.get(p, np.nan) for p in STAGE_PREFIXES] for s in post])
    for idx, p in enumerate(STAGE_PREFIXES):
        axes[2].plot(times, norms[:, idx], linewidth=0.6, alpha=0.8, label=p)
    axes[2].set_ylabel("stage norm")
    axes[2].legend(fontsize=6, ncol=3)
    axes[2].grid(True, alpha=0.25)

    proj_long = np.array([s.proj_body[0] for s in post], dtype=float)
    axes[3].plot(times, proj_long, color="#2980b9", linewidth=0.8, label="proj body long")
    axes[3].set_ylabel("[m/s2]")
    axes[3].set_xlabel("Tiempo [s]")
    axes[3].legend(fontsize=8)
    axes[3].grid(True, alpha=0.25)

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Propagation chain audit")
    parser.add_argument("--skip-replay", action="store_true")
    args = parser.parse_args()

    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    replay_csv = resolve_replay_path(None)
    ensure_calibration(DEFAULT_CALIBRATION)

    if not args.skip_replay:
        run_chain_replay(replay_csv, DEFAULT_REPLAY_EXE, DEFAULT_CALIBRATION)

    if not CHAIN_CSV.is_file():
        print("ERROR: falta propagation chain CSV", file=sys.stderr)
        return 1

    samples = load_chain_csv(CHAIN_CSV)
    h9c_delta = load_h9c_delta_tilt(H9C_MERGED)
    diagnosis = diagnose(samples, h9c_delta)
    plot_analysis(samples, ANALYSIS_PNG)

    report = {
        "experiment": "propagation_chain_audit",
        "chain": "a_raw -> R_mount -> a_body -> bias -> a_corr -> R_bn -> a_nav -> -g -> a_lin",
        "diagnosis": diagnosis,
        "interpretation": diagnosis["likely_mechanism"],
        "artifacts": {"csv": str(CHAIN_CSV), "plot_png": str(ANALYSIS_PNG)},
    }
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    jumps = diagnosis["jumps_static_to_motion"]
    print("=" * 72)
    print("Propagation chain audit (final)")
    print("=" * 72)
    print(f"  Regimen:              {diagnosis['regime_pattern']}")
    print(f"  Salto gravity_angle:  {jumps.get('gravity_angle_deg', float('nan')):.2f} deg")
    print(f"  Salto a_lin_h:        {jumps.get('a_lin_h', float('nan')):.4f} m/s2")
    print(f"  Salto a_nav_body_h:   {jumps.get('a_nav_body_h', float('nan')):.4f} m/s2")
    print(f"  Salto a_nav_corr_h:   {jumps.get('a_nav_corr_h', float('nan')):.4f} m/s2")
    print(f"  Salto R_bn*bias_h:    {jumps.get('r_bn_bias_h', float('nan')):.4f} m/s2")
    print(f"  Bias frame OK:        {diagnosis['bias_frame_consistent']}")
    print(f"  Bias explica salto:   {diagnosis['bias_explains_a_lin_jump']}")
    print(f"  Mecanismo:            {diagnosis['likely_mechanism']}")
    print(f"  Informe: {REPORT_JSON}")
    print(f"  Grafica: {ANALYSIS_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
