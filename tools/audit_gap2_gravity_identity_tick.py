#!/usr/bin/env python3
"""GAP-2 paso 1: identidad fisica tick a tick.

Cadena EKF (H9d):
  f_body -> R_bn -> f_nav -> (-g) -> a_lin

Identidad bajo prueba (muestra a muestra, no solo media):
  |a_lin,h| ~= GRAVITY * sin(delta_tilt)

delta_tilt candidatos:
  A) gravity_align_deg     — angulo g_pred vs g_meas (interno EKF, cadena)
  B) delta_tilt_mag_deg    — |EKF - Orientation| (H9c)
  C) delta_pitch_deg       — componente pitch EKF vs ref

Criterio H0: NRMSE < 15% y >90% muestras con error relativo <15% (ventana 2-10 s).

Si A pasa tick a tick -> mecanismo causal validado (tilt -> proy. g -> a_lin,h).
Si no -> buscar contaminacion en f_body o definicion body antes de abrir predict().
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

H9C_MERGED = BENCH_DIR / "h9c_orientation_merged.csv"
CHAIN_CSV = BENCH_DIR / "propagation_chain_audit.csv"
REPORT_JSON = BENCH_DIR / "gap2_gravity_identity_tick_report.json"
TICK_CSV = BENCH_DIR / "gap2_gravity_identity_tick.csv"
ANALYSIS_PNG = BENCH_DIR / "gap2_gravity_identity_tick_analysis.png"

GRAVITY = 9.80665
MOTION_T0 = 2.0
MOTION_T1 = 10.0
REL_ERR_PASS = 0.15
NRMSE_PASS_PCT = 15.0
PCT_SAMPLES_PASS = 90.0


@dataclass
class TickRow:
    timestamp_s: float
    a_lin_h: float
    a_nav_pre_h: float
    delta_tilt_a_deg: float
    delta_tilt_b_deg: float
    delta_tilt_c_deg: float
    pred_a: float
    pred_b: float
    pred_c: float
    resid_a: float
    resid_b: float
    resid_c: float
    rel_err_a_pct: float
    rel_err_b_pct: float
    rel_err_c_pct: float


def g_sin_deg(deg: float) -> float:
    return GRAVITY * math.sin(math.radians(deg))


def rel_err_pct(obs: float, pred: float) -> float:
    denom = max(abs(obs), 1e-6)
    return 100.0 * abs(obs - pred) / denom


def load_merged_sources(h9c_path: Path, chain_path: Path) -> list[TickRow]:
    chain_nav: dict[float, tuple[float, float]] = {}
    if chain_path.is_file():
        with chain_path.open(newline="", encoding="utf-8") as handle:
            for raw in csv.DictReader(handle):
                if not raw.get("timestamp_s"):
                    continue
                t = float(raw["timestamp_s"])
                nav_h = abs(float(raw.get("a_nav_body_h") or 0.0))
                grav = float(raw.get("gravity_angle_deg") or 0.0)
                chain_nav[round(t, 6)] = (nav_h, grav)

    rows: list[TickRow] = []
    with h9c_path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            if not raw.get("timestamp_s"):
                continue
            t = float(raw["timestamp_s"])
            key = round(t, 6)
            nav_h, grav_chain = chain_nav.get(key, (float("nan"), float("nan")))
            alin = float(raw["a_lin_h_mps2"])
            if math.isfinite(nav_h):
                a_nav = nav_h
            else:
                a_nav = alin

            grav = float(raw.get("gravity_alignment_error_deg") or 0.0)
            if math.isfinite(grav_chain) and grav_chain > grav:
                grav = grav_chain

            dt_mag = float(raw.get("delta_tilt_mag_deg") or 0.0)
            dp = float(raw.get("delta_pitch_deg") or 0.0)

            pa, pb, pc = g_sin_deg(grav), g_sin_deg(dt_mag), g_sin_deg(abs(dp))
            rows.append(
                TickRow(
                    timestamp_s=t,
                    a_lin_h=alin,
                    a_nav_pre_h=a_nav,
                    delta_tilt_a_deg=grav,
                    delta_tilt_b_deg=dt_mag,
                    delta_tilt_c_deg=abs(dp),
                    pred_a=pa,
                    pred_b=pb,
                    pred_c=pc,
                    resid_a=alin - pa,
                    resid_b=alin - pb,
                    resid_c=alin - pc,
                    rel_err_a_pct=rel_err_pct(alin, pa),
                    rel_err_b_pct=rel_err_pct(alin, pb),
                    rel_err_c_pct=rel_err_pct(alin, pc),
                )
            )
    rows.sort(key=lambda r: r.timestamp_s)
    return rows


def filter_window(rows: list[TickRow], t0: float, t1: float) -> list[TickRow]:
    return [r for r in rows if t0 <= r.timestamp_s <= t1]


def eval_identity(rows: list[TickRow], label: str, pred_attr: str, resid_attr: str, rel_attr: str) -> dict:
    if not rows:
        return {"label": label, "samples": 0}

    obs = np.array([r.a_lin_h for r in rows], dtype=float)
    pred = np.array([getattr(r, pred_attr) for r in rows], dtype=float)
    resid = np.array([getattr(r, resid_attr) for r in rows], dtype=float)
    rel = np.array([getattr(r, rel_attr) for r in rows], dtype=float)

    rms = float(np.sqrt(np.mean(resid**2)))
    max_abs = float(np.max(np.abs(resid)))
    mean_obs = float(np.mean(obs))
    mean_pred = float(np.mean(pred))
    nrmse = 100.0 * rms / max(mean_obs, 1e-9)
    corr = float(np.corrcoef(obs, pred)[0, 1]) if obs.size > 2 and np.std(obs) > 1e-12 else float("nan")
    ss_res = float(np.sum(resid**2))
    ss_tot = float(np.sum((obs - mean_obs) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")

    pct_within_15 = 100.0 * float(np.mean(rel <= 100.0 * REL_ERR_PASS))
    pct_within_10 = 100.0 * float(np.mean(rel <= 10.0))

    identity_holds = (
        nrmse <= NRMSE_PASS_PCT
        and pct_within_15 >= PCT_SAMPLES_PASS
        and (math.isnan(corr) or corr >= 0.95)
    )

    return {
        "label": label,
        "samples": len(rows),
        "observed_mean_mps2": mean_obs,
        "predicted_mean_mps2": mean_pred,
        "rms_error_mps2": rms,
        "max_abs_error_mps2": max_abs,
        "nrmse_pct": nrmse,
        "correlation": corr,
        "r_squared": r2,
        "median_rel_error_pct": float(np.median(rel)),
        "p95_rel_error_pct": float(np.percentile(rel, 95)),
        "max_rel_error_pct": float(np.max(rel)),
        "pct_samples_rel_err_le_10": pct_within_10,
        "pct_samples_rel_err_le_15": pct_within_15,
        "identity_holds_tick_by_tick": identity_holds,
        "g_sin_4deg_reference_mps2": g_sin_deg(4.0),
    }


def eval_alin_vs_nav_pre(rows: list[TickRow], label: str) -> dict:
    if not rows:
        return {"label": label, "samples": 0}
    alin = np.array([r.a_lin_h for r in rows], dtype=float)
    nav = np.array([r.a_nav_pre_h for r in rows], dtype=float)
    diff = alin - nav
    return {
        "label": label,
        "samples": len(rows),
        "max_abs_diff_mps2": float(np.max(np.abs(diff))),
        "rms_diff_mps2": float(np.sqrt(np.mean(diff**2))),
        "correlation": float(np.corrcoef(alin, nav)[0, 1]) if len(rows) > 2 else float("nan"),
        "identical_h9d": float(np.max(np.abs(diff))) < 1e-4,
    }


def diagnose(eval_a: dict, eval_b: dict, eval_c: dict, h9d: dict) -> dict:
    if eval_a.get("identity_holds_tick_by_tick"):
        verdict = "MECHANISM_VALIDATED_TICK_BY_TICK"
        mechanism = (
            "|a_lin,h| ~= g*sin(gravity_align) en cada tick (2-10 s). "
            "Mecanismo causal: delta_tilt -> proyeccion horizontal g -> a_lin,h -> v -> p."
        )
        next_step = (
            "Abrir predict() solo en cadena: f_body -> R_bn -> f_nav -> (-g); "
            "buscar primera transformacion que rompe gravity_align."
        )
    elif eval_a.get("correlation", 0) > 0.95 and eval_a.get("nrmse_pct", 999) < 25:
        verdict = "MECHANISM_MOSTLY_VALIDATED"
        mechanism = "Identidad casi cumplida con gravity_align; residual acotado."
        next_step = "Inspeccion dirigida R_bn y resta g en predict()."
    elif eval_b.get("correlation", 0) > 0.85:
        verdict = "MECHANISM_MEAN_ONLY_NOT_TICK"
        mechanism = (
            "Medias coherentes (4 deg, 0.7 m/s2) pero Orientation delta_tilt "
            "no satisface identidad tick a tick."
        )
        next_step = "Usar gravity_align interno; no Orientation como delta_tilt en identidad."
    else:
        verdict = "MECHANISM_REJECTED"
        mechanism = "Existe componente adicional mas alla de g*sin(delta_tilt)."
        next_step = "Auditar f_body/mount antes de R_bn."

    return {
        "verdict": verdict,
        "mechanism": mechanism,
        "next_step": next_step,
        "h9d_alin_equals_nav_pre": h9d,
        "candidate_A_gravity_align": eval_a,
        "candidate_B_orientation_delta_tilt": eval_b,
        "candidate_C_delta_pitch": eval_c,
    }


def write_tick_csv(rows: list[TickRow], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        w = csv.DictWriter(
            handle,
            fieldnames=[
                "timestamp_s", "a_lin_h", "a_nav_pre_h",
                "gravity_align_deg", "delta_tilt_orientation_deg", "delta_pitch_deg",
                "g_sin_gravity_align", "g_sin_orientation_tilt", "g_sin_delta_pitch",
                "residual_gravity_align", "residual_orientation", "residual_pitch",
                "rel_err_gravity_align_pct", "rel_err_orientation_pct", "rel_err_pitch_pct",
            ],
        )
        w.writeheader()
        for r in rows:
            w.writerow(
                {
                    "timestamp_s": r.timestamp_s,
                    "a_lin_h": r.a_lin_h,
                    "a_nav_pre_h": r.a_nav_pre_h,
                    "gravity_align_deg": r.delta_tilt_a_deg,
                    "delta_tilt_orientation_deg": r.delta_tilt_b_deg,
                    "delta_pitch_deg": r.delta_tilt_c_deg,
                    "g_sin_gravity_align": r.pred_a,
                    "g_sin_orientation_tilt": r.pred_b,
                    "g_sin_delta_pitch": r.pred_c,
                    "residual_gravity_align": r.resid_a,
                    "residual_orientation": r.resid_b,
                    "residual_pitch": r.resid_c,
                    "rel_err_gravity_align_pct": r.rel_err_a_pct,
                    "rel_err_orientation_pct": r.rel_err_b_pct,
                    "rel_err_pitch_pct": r.rel_err_c_pct,
                }
            )


def plot(rows: list[TickRow], eval_a: dict, eval_b: dict, path: Path) -> None:
    motion = filter_window(rows, MOTION_T0, MOTION_T1)
    if not motion:
        return

    t = np.array([r.timestamp_s for r in motion])
    obs = np.array([r.a_lin_h for r in motion])
    pa = np.array([r.pred_a for r in motion])
    pb = np.array([r.pred_b for r in motion])
    ra = np.array([r.rel_err_a_pct for r in motion])

    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    fig.suptitle("GAP-2 identidad tick a tick: |a_lin,h| vs g*sin(delta_tilt)", fontsize=13)

    ax = axes[0, 0]
    ax.plot(t, obs, label="|a_lin,h|", linewidth=0.8, color="#c0392b")
    ax.plot(t, pa, label="g*sin(gravity_align)", linewidth=0.8, color="#2980b9", alpha=0.85)
    ax.plot(t, pb, label="g*sin(Orient tilt)", linewidth=0.8, color="#27ae60", alpha=0.6)
    ax.set_ylabel("[m/s2]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.25)

    ax = axes[0, 1]
    ax.scatter(pa, obs, s=6, alpha=0.45, color="#2980b9")
    lim = max(float(np.max(obs)), float(np.max(pa)), 0.05) * 1.05
    ax.plot([0, lim], [0, lim], "r--", linewidth=0.8)
    ax.set_xlabel("g*sin(gravity_align)")
    ax.set_ylabel("|a_lin,h|")
    ax.set_title(f"r={eval_a.get('correlation', float('nan')):.4f}  NRMSE={eval_a.get('nrmse_pct', float('nan')):.1f}%")
    ax.grid(True, alpha=0.25)

    ax = axes[1, 0]
    ax.scatter(pb, obs, s=6, alpha=0.45, color="#27ae60")
    lim2 = max(float(np.max(obs)), float(np.max(pb)), 0.05) * 1.05
    ax.plot([0, lim2], [0, lim2], "r--", linewidth=0.8)
    ax.set_xlabel("g*sin(Orientation delta_tilt)")
    ax.set_ylabel("|a_lin,h|")
    ax.set_title(f"r={eval_b.get('correlation', float('nan')):.4f}  NRMSE={eval_b.get('nrmse_pct', float('nan')):.1f}%")
    ax.grid(True, alpha=0.25)

    ax = axes[1, 1]
    ax.plot(t, ra, color="#e67e22", linewidth=0.7)
    ax.axhline(100.0 * REL_ERR_PASS, color="#e74c3c", linestyle="--", linewidth=0.8, label="15%")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("error rel. [%]")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-2 gravity identity tick-by-tick")
    parser.add_argument("--h9c-merged", type=Path, default=H9C_MERGED)
    parser.add_argument("--chain-csv", type=Path, default=CHAIN_CSV)
    args = parser.parse_args()

    if not args.h9c_merged.is_file():
        print(f"ERROR: falta {args.h9c_merged}", file=sys.stderr)
        return 1

    rows = load_merged_sources(args.h9c_merged, args.chain_csv)
    motion = filter_window(rows, MOTION_T0, MOTION_T1)

    eval_a = eval_identity(motion, "A_gravity_align", "pred_a", "resid_a", "rel_err_a_pct")
    eval_b = eval_identity(motion, "B_orientation_delta_tilt", "pred_b", "resid_b", "rel_err_b_pct")
    eval_c = eval_identity(motion, "C_delta_pitch", "pred_c", "resid_c", "rel_err_c_pct")
    h9d = eval_alin_vs_nav_pre(motion, "motion_2_10s")

    diagnosis = diagnose(eval_a, eval_b, eval_c, h9d)

    write_tick_csv(rows, TICK_CSV)
    plot(rows, eval_a, eval_b, ANALYSIS_PNG)

    report = {
        "experiment": "gap2_gravity_identity_tick_by_tick",
        "identity": "|a_lin,h| ~= GRAVITY * sin(delta_tilt)",
        "window_s": [MOTION_T0, MOTION_T1],
        "pass_criteria": {
            "nrmse_pct_max": NRMSE_PASS_PCT,
            "pct_samples_rel_err_le_15_min": PCT_SAMPLES_PASS,
            "correlation_min": 0.95,
        },
        "evaluations": {
            "A_gravity_align_internal": eval_a,
            "B_orientation_delta_tilt": eval_b,
            "C_delta_pitch": eval_c,
            "h9d_a_lin_vs_a_nav_pre": h9d,
        },
        "diagnosis": diagnosis,
        "artifacts": {"tick_csv": str(TICK_CSV), "plot_png": str(ANALYSIS_PNG)},
    }
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    print("=" * 72)
    print("GAP-2 identidad fisica TICK A TICK  [2-10 s]")
    print("  |a_lin,h| ~= g*sin(delta_tilt)")
    print("=" * 72)
    print(f"  g*sin(4 deg) = {g_sin_deg(4.0):.4f} m/s2  |  |a_lin,h| media = {eval_a['observed_mean_mps2']:.4f} m/s2")
    print()
    for tag, ev in [("A gravity_align (interno EKF)", eval_a), ("B Orientation delta_tilt", eval_b), ("C delta_pitch", eval_c)]:
        print(f"  [{tag}]")
        print(f"    RMS={ev['rms_error_mps2']:.4f}  max={ev['max_abs_error_mps2']:.4f}  NRMSE={ev['nrmse_pct']:.1f}%")
        print(f"    r={ev['correlation']:.4f}  R2={ev['r_squared']:.4f}")
        print(f"    pct err<15%: {ev['pct_samples_rel_err_le_15']:.1f}%  p95 err={ev['p95_rel_error_pct']:.1f}%")
        print(f"    identidad tick a tick: {'SI' if ev['identity_holds_tick_by_tick'] else 'NO'}")
        print()
    print(f"  H9d: a_lin,h == |a_nav_pre|_h  max_diff={h9d['max_abs_diff_mps2']:.2e}  r={h9d['correlation']:.6f}")
    print()
    print(f"  VEREDICTO: {diagnosis['verdict']}")
    print(f"  {diagnosis['mechanism']}")
    print(f"  Siguiente: {diagnosis['next_step']}")
    print(f"  Informe: {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
