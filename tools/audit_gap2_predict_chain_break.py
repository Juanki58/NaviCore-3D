#!/usr/bin/env python3
"""GAP-2 paso 2: auditoria dirigida predict() — cadena de cuatro transformaciones.

Cadena replay + predict():
  f_sensor --[R_mount]--> f_body --[-bias]--> a_corr
       --[quat_integrate]--> R_bn --[R_bn]--> f_nav --[-g_NED]--> a_lin

Objetivo: identificar la PRIMERA etapa que rompe gravity_align al entrar en dinamica.

Metricas por etapa:
  E0 mount:     |a_body|_h_body = sqrt(ax^2+ay^2)  (FRD body)
  E1 bias:      |delta a| (negligible)
  E2 meas tilt: angle(a_corr, g_body_ref=[0,0,g])
  E3 R_bn pred: angle(g_body_pred, g_body_ref)
  E4 gravity_align = angle(a_corr_unit, g_pred_unit)  [definicion replay]
  E5 R_bn proj:   |a_nav_corr|_h
  E6 -g:        |a_lin|_h  (identico a E5 pre-g, H9d)

Detecta primer tick donde gravity_align > umbral y que etapa ya divergia.
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
H9B_CSV = BENCH_DIR / "h9b_attitude_propagation.csv"
REPORT_JSON = BENCH_DIR / "gap2_predict_chain_break_report.json"
MERGED_CSV = BENCH_DIR / "gap2_predict_chain_break_merged.csv"
ANALYSIS_PNG = BENCH_DIR / "gap2_predict_chain_break_analysis.png"

GRAVITY = 9.80665
G_BODY_REF = np.array([0.0, 0.0, GRAVITY], dtype=float)
STATIC_END_S = 2.0
MOTION_T0 = 2.0
MOTION_T1 = 10.0
GRAVITY_ALIGN_ONSET_DEG = 1.0


@dataclass
class StageSample:
    timestamp_s: float
    # stages
    a_body_h: float
    a_corr_h_body: float
    bias_h_body: float
    meas_tilt_from_g_deg: float
    pred_tilt_from_g_deg: float
    gravity_align_deg: float
    a_nav_pre_h: float
    a_lin_h: float
    g_sin_gravity_align: float
    # attitude
    roll_deg: float
    pitch_deg: float
    delta_theta_int_mag_deg: float
    delta_theta_gravity_step_mag_deg: float
    delta_pitch_step_deg: float


def vec3(row: dict, prefix: str) -> np.ndarray:
    return np.array(
        [float(row.get(f"{prefix}_x", 0) or 0), float(row.get(f"{prefix}_y", 0) or 0), float(row.get(f"{prefix}_z", 0) or 0)],
        dtype=float,
    )


def angle_deg(u: np.ndarray, v: np.ndarray) -> float:
    nu = np.linalg.norm(u)
    nv = np.linalg.norm(v)
    if nu < 1e-9 or nv < 1e-9:
        return 0.0
    c = float(np.clip(np.dot(u, v) / (nu * nv), -1.0, 1.0))
    return math.degrees(math.acos(c))


def load_chain(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open(newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            if raw.get("timestamp_s"):
                rows.append(raw)
    return rows


def load_h9b_by_time(path: Path) -> dict[float, dict]:
    out: dict[float, dict] = {}
    if not path.is_file():
        return out
    with path.open(newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            if raw.get("timestamp_s"):
                out[round(float(raw["timestamp_s"]), 6)] = raw
    return out


def build_samples(chain: list[dict], h9b: dict[float, dict]) -> list[StageSample]:
    samples: list[StageSample] = []
    prev_pitch: float | None = None
    for raw in chain:
        t = float(raw["timestamp_s"])
        a_body = vec3(raw, "a_body")
        a_corr = vec3(raw, "a_corr")
        bias = vec3(raw, "bias")
        g_pred = vec3(raw, "g_body_pred")
        grav_align = float(raw.get("gravity_angle_deg") or 0.0)
        roll = float(raw.get("roll_deg") or 0.0)
        pitch = float(raw.get("pitch_deg") or 0.0)

        h9b_row = h9b.get(round(t, 6), {})
        dtheta = float(h9b_row.get("delta_theta_int_mag_deg") or 0.0)
        dgrav_step = float(h9b_row.get("delta_theta_gravity_step_mag_deg") or 0.0)
        dp = 0.0 if prev_pitch is None else pitch - prev_pitch
        prev_pitch = pitch

        samples.append(
            StageSample(
                timestamp_s=t,
                a_body_h=float(math.hypot(a_body[0], a_body[1])),
                a_corr_h_body=float(math.hypot(a_corr[0], a_corr[1])),
                bias_h_body=float(math.hypot(bias[0], bias[1])),
                meas_tilt_from_g_deg=angle_deg(a_corr, G_BODY_REF),
                pred_tilt_from_g_deg=angle_deg(g_pred, G_BODY_REF),
                gravity_align_deg=grav_align,
                a_nav_pre_h=abs(float(raw.get("a_nav_corr_h") or raw.get("a_nav_body_h") or 0.0)),
                a_lin_h=abs(float(raw.get("a_lin_h") or 0.0)),
                g_sin_gravity_align=GRAVITY * math.sin(math.radians(grav_align)),
                roll_deg=roll,
                pitch_deg=pitch,
                delta_theta_int_mag_deg=dtheta,
                delta_theta_gravity_step_mag_deg=dgrav_step,
                delta_pitch_step_deg=dp,
            )
        )
    return samples


def static_baseline(samples: list[StageSample], t1: float = STATIC_END_S) -> dict[str, float]:
    static = [s for s in samples if s.timestamp_s <= t1]
    if not static:
        return {}
    keys = [
        "a_body_h", "a_corr_h_body", "bias_h_body", "meas_tilt_from_g_deg",
        "pred_tilt_from_g_deg", "gravity_align_deg", "a_nav_pre_h", "a_lin_h",
    ]
    out: dict[str, float] = {}
    for k in keys:
        vals = np.array([getattr(s, k) for s in static], dtype=float)
        out[f"{k}_mean"] = float(np.mean(vals))
        out[f"{k}_p95"] = float(np.percentile(vals, 95))
        out[f"{k}_max"] = float(np.max(vals))
    return out


def find_first_exceed(samples: list[StageSample], attr: str, threshold: float, t_min: float) -> dict | None:
    for s in samples:
        if s.timestamp_s < t_min:
            continue
        if getattr(s, attr) > threshold:
            return {"t_s": s.timestamp_s, "value": getattr(s, attr), "metric": attr, "threshold": threshold}
    return None


def analyze_onset(samples: list[StageSample], baseline: dict) -> dict:
    motion = [s for s in samples if MOTION_T0 <= s.timestamp_s <= MOTION_T1]
    first_grav = find_first_exceed(samples, "gravity_align_deg", GRAVITY_ALIGN_ONSET_DEG, STATIC_END_S)
    first_alin = find_first_exceed(samples, "a_lin_h", 0.1, STATIC_END_S)
    first_nav = find_first_exceed(samples, "a_nav_pre_h", 0.1, STATIC_END_S)

    def stage_threshold(attr: str) -> float:
        p95 = baseline.get(f"{attr}_p95", 0.0)
        if "tilt" in attr or "align" in attr:
            return max(p95 * 3.0, GRAVITY_ALIGN_ONSET_DEG)
        return max(p95 * 3.0, 0.05)

    stage_order_all = [
        ("E0_mount_body_h", "a_body_h"),
        ("E1_bias_body_h", "bias_h_body"),
        ("E2_meas_tilt_from_g", "meas_tilt_from_g_deg"),
        ("E3_pred_tilt_from_g", "pred_tilt_from_g_deg"),
        ("E4_gravity_align", "gravity_align_deg"),
        ("E5_nav_pre_h", "a_nav_pre_h"),
    ]
    gravity_stage_order = [
        ("E2_meas_tilt_from_g", "meas_tilt_from_g_deg"),
        ("E3_pred_tilt_from_g", "pred_tilt_from_g_deg"),
        ("E4_gravity_align", "gravity_align_deg"),
    ]

    stage_breaks: list[dict] = []
    for name, attr in stage_order_all:
        thr = stage_threshold(attr)
        hit = find_first_exceed(samples, attr, thr, STATIC_END_S)
        if hit:
            stage_breaks.append({"stage": name, **hit, "static_p95": baseline.get(f"{attr}_p95", 0.0)})

    first_stage_break: dict | None = None
    gravity_breaks = [b for b in stage_breaks if b["stage"] in {s for s, _ in gravity_stage_order}]
    if gravity_breaks:
        first_stage_break = min(gravity_breaks, key=lambda b: b["t_s"])
    elif stage_breaks:
        first_stage_break = stage_breaks[0]

    # correlaciones en motion
    def corr(a: str, b: str) -> float:
        x = np.array([getattr(s, a) for s in motion], dtype=float)
        y = np.array([getattr(s, b) for s in motion], dtype=float)
        if len(x) < 3 or np.std(x) < 1e-12:
            return float("nan")
        return float(np.corrcoef(x, y)[0, 1])

    return {
        "first_gravity_align_1deg": first_grav,
        "first_a_lin_h_0p1": first_alin,
        "first_a_nav_pre_h_0p1": first_nav,
        "first_stage_break_vs_static_p95": first_stage_break,
        "all_stage_breaks_vs_static_p95": stage_breaks,
        "first_mount_body_h_break": next((b for b in stage_breaks if b["stage"] == "E0_mount_body_h"), None),
        "motion_correlations": {
            "gravity_align_vs_meas_tilt": corr("gravity_align_deg", "meas_tilt_from_g_deg"),
            "gravity_align_vs_pred_tilt": corr("gravity_align_deg", "pred_tilt_from_g_deg"),
            "gravity_align_vs_delta_theta_gravity_step": corr("gravity_align_deg", "delta_theta_gravity_step_mag_deg"),
            "gravity_align_vs_delta_theta_int": corr("gravity_align_deg", "delta_theta_int_mag_deg"),
            "a_lin_h_vs_gravity_align": corr("a_lin_h", "gravity_align_deg"),
            "meas_tilt_vs_a_body_h": corr("meas_tilt_from_g_deg", "a_body_h"),
        },
        "motion_means": {
            "meas_tilt_from_g_deg": float(np.mean([s.meas_tilt_from_g_deg for s in motion])) if motion else float("nan"),
            "pred_tilt_from_g_deg": float(np.mean([s.pred_tilt_from_g_deg for s in motion])) if motion else float("nan"),
            "gravity_align_deg": float(np.mean([s.gravity_align_deg for s in motion])) if motion else float("nan"),
            "a_body_h": float(np.mean([s.a_body_h for s in motion])) if motion else float("nan"),
        },
    }


def diagnose(onset: dict, baseline: dict) -> dict:
    break_info = onset.get("first_stage_break_vs_static_p95") or {}
    stage = break_info.get("stage", "unknown")
    first_grav = onset.get("first_gravity_align_1deg") or {}
    t_grav = first_grav.get("t_s")

    if stage.startswith("E0") or stage.startswith("E1"):
        verdict = "BREAK_AT_MOUNT_OR_BIAS"
        root = "Revisar R_mount o bias antes de R_bn."
    elif stage.startswith("E2") or stage.startswith("E4"):
        verdict = "BREAK_AT_SPECIFIC_FORCE_CONTAMINATION"
        root = (
            "a_corr (fuerza especifica medida) se inclina respecto a g_body_ref "
            "mientras pred_tilt (R_bn) permanece bajo: contaminacion horizontal en "
            "body_to_ned(a_corr) en ins_ekf.cpp ~723, no mount ni -g."
        )
    elif stage.startswith("E3") or stage.startswith("E4"):
        verdict = "BREAK_AT_R_bn_ATTITUDE"
        root = (
            "Primera ruptura en pred_tilt/gravity_align: quat_integrate (R_bn) "
            "diverge de a_corr como referencia gravedad. Abrir ins_ekf.cpp "
            "lineas 701-723 (quat_integrate + body_to_ned), no mount."
        )
    elif stage.startswith("E5"):
        verdict = "BREAK_AT_PROJECTION"
        root = "Revisar body_to_ned(dcm_bn, a_corr) — improbable si gravity_align ya roto."
    else:
        verdict = "INCONCLUSIVE"
        root = "Revisar datos."

    return {
        "verdict": verdict,
        "first_break_stage": stage,
        "first_break_t_s": break_info.get("t_s"),
        "first_gravity_align_1deg_t_s": t_grav,
        "recommended_predict_audit_lines": "ins_ekf.cpp ~669-732 (quat_integrate, quat_to_dcm_bn, body_to_ned, -kGravityNed)",
        "recommended_root_cause_focus": root,
        "static_baseline": baseline,
        "onset_analysis": onset,
        "chain_verdict_summary": (
            "Mount/bias estables en estatico. gravity_align = angle(g_pred, a_corr_unit); "
            "crece cuando R_bn (post quat_integrate) deja de coincidir con direccion de a_corr. "
            "Resta -g no anade leak (H9d). Mecanismo: R_bn + interpretacion gravedad en dinamica."
        ),
    }


def write_merged(samples: list[StageSample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "timestamp_s", "a_body_h", "a_corr_h_body", "bias_h_body",
                "meas_tilt_from_g_deg", "pred_tilt_from_g_deg", "gravity_align_deg",
                "a_nav_pre_h", "a_lin_h", "g_sin_gravity_align",
                "roll_deg", "pitch_deg",
                "delta_theta_int_mag_deg", "delta_theta_gravity_step_mag_deg",
            ],
        )
        w.writeheader()
        for s in samples:
            w.writerow({k: getattr(s, k) for k in w.fieldnames})


def plot(samples: list[StageSample], onset: dict, path: Path) -> None:
    post = [s for s in samples if s.timestamp_s >= 1.5]
    t = np.array([s.timestamp_s for s in post])
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    fig.suptitle("GAP-2: ruptura por etapa en cadena predict()", fontsize=13)

    axes[0].plot(t, [s.meas_tilt_from_g_deg for s in post], label="E2 meas tilt", linewidth=0.8)
    axes[0].plot(t, [s.pred_tilt_from_g_deg for s in post], label="E3 pred tilt", linewidth=0.8)
    axes[0].plot(t, [s.gravity_align_deg for s in post], label="E4 gravity_align", linewidth=0.8)
    axes[0].axhline(GRAVITY_ALIGN_ONSET_DEG, color="#e74c3c", linestyle=":", linewidth=0.7)
    axes[0].axvspan(MOTION_T0, MOTION_T1, color="#f9e79f", alpha=0.3)
    if onset.get("first_gravity_align_1deg"):
        tg = onset["first_gravity_align_1deg"]["t_s"]
        axes[0].axvline(tg, color="#c0392b", linestyle="--", linewidth=0.8, label=f"1 deg @ {tg:.1f}s")
    axes[0].set_ylabel("[deg]")
    axes[0].legend(fontsize=7)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(t, [s.a_body_h for s in post], label="E0 |a_body|_h", linewidth=0.8)
    axes[1].plot(t, [s.a_lin_h for s in post], label="E6 |a_lin|_h", linewidth=0.8)
    axes[1].plot(t, [s.g_sin_gravity_align for s in post], label="g*sin(grav_align)", linewidth=0.8, alpha=0.7)
    axes[1].set_ylabel("[m/s2]")
    axes[1].legend(fontsize=7)
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(t, [s.delta_theta_gravity_step_mag_deg for s in post], label="dtheta gravity step", linewidth=0.8)
    axes[2].plot(t, [s.delta_theta_int_mag_deg for s in post], label="dtheta gyro int", linewidth=0.8, alpha=0.7)
    axes[2].set_ylabel("[deg/tick]")
    axes[2].legend(fontsize=7)
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(t, [s.pitch_deg for s in post], label="pitch", linewidth=0.8)
    axes[3].plot(t, [s.roll_deg for s in post], label="roll", linewidth=0.8, alpha=0.7)
    axes[3].set_xlabel("t [s]")
    axes[3].set_ylabel("[deg]")
    axes[3].legend(fontsize=7)
    axes[3].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-2 predict chain break audit")
    parser.add_argument("--chain-csv", type=Path, default=CHAIN_CSV)
    parser.add_argument("--h9b-csv", type=Path, default=H9B_CSV)
    args = parser.parse_args()

    if not args.chain_csv.is_file():
        print(f"ERROR: falta {args.chain_csv}", file=sys.stderr)
        return 1

    chain = load_chain(args.chain_csv)
    h9b = load_h9b_by_time(args.h9b_csv)
    samples = build_samples(chain, h9b)
    baseline = static_baseline(samples)
    onset = analyze_onset(samples, baseline)
    diagnosis = diagnose(onset, baseline)

    write_merged(samples, MERGED_CSV)
    plot(samples, onset, ANALYSIS_PNG)

    report = {
        "experiment": "gap2_predict_chain_break",
        "predict_chain": "mount -> body -> bias -> quat_integrate/R_bn -> NED -> -g",
        "diagnosis": diagnosis,
        "artifacts": {"merged_csv": str(MERGED_CSV), "plot_png": str(ANALYSIS_PNG)},
    }
    with REPORT_JSON.open("w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write("\n")

    b = baseline
    o = onset
    d = diagnosis
    print("=" * 72)
    print("GAP-2 auditoria dirigida predict() — ruptura por etapa")
    print("=" * 72)
    print("  Estatico (0-2 s) p95:")
    print(f"    |a_body|_h:        {b.get('a_body_h_p95', float('nan')):.4f} m/s2")
    print(f"    meas_tilt:         {b.get('meas_tilt_from_g_deg_p95', float('nan')):.3f} deg")
    print(f"    pred_tilt:         {b.get('pred_tilt_from_g_deg_p95', float('nan')):.3f} deg")
    print(f"    gravity_align:     {b.get('gravity_align_deg_p95', float('nan')):.3f} deg")
    print()
    fs = o.get("first_stage_break_vs_static_p95") or {}
    print(f"  Primera ruptura (3x p95 estatico): {fs.get('stage')} @ t={fs.get('t_s')} s  value={fs.get('value')}")
    fg = o.get("first_gravity_align_1deg") or {}
    print(f"  gravity_align > 1 deg:             t={fg.get('t_s')} s")
    fa = o.get("first_a_lin_h_0p1") or {}
    print(f"  |a_lin,h| > 0.1 m/s2:              t={fa.get('t_s')} s")
    print()
    mm = o.get("motion_means", {})
    print(f"  Motion 2-10 s medias:")
    print(f"    meas_tilt={mm.get('meas_tilt_from_g_deg', float('nan')):.2f} deg  pred_tilt={mm.get('pred_tilt_from_g_deg', float('nan')):.2f} deg")
    print(f"    gravity_align={mm.get('gravity_align_deg', float('nan')):.2f} deg  |a_body|_h={mm.get('a_body_h', float('nan')):.3f} m/s2")
    print()
    c = o.get("motion_correlations", {})
    print(f"  corr(gravity_align, meas_tilt)={c.get('gravity_align_vs_meas_tilt', float('nan')):.3f}")
    print(f"  corr(gravity_align, pred_tilt)={c.get('gravity_align_vs_pred_tilt', float('nan')):.3f}")
    print(f"  corr(gravity_align, dtheta_grav_step)={c.get('gravity_align_vs_delta_theta_gravity_step', float('nan')):.3f}")
    print()
    print(f"  VEREDICTO: {d['verdict']}")
    print(f"  {d['recommended_root_cause_focus']}")
    print(f"  predict(): {d['recommended_predict_audit_lines']}")
    print(f"  Informe: {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
