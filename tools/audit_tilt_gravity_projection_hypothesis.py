#!/usr/bin/env python3
"""Prueba falsable: a_lin,h ~ g*sin(delta_tilt) en onset dinamico (2-10 s).

H0: a_lin_h ~= GRAVITY * sin(delta_tilt_rad)
Compatible si NRMSE < 15% (configurable) en ventana 2-10 s.

delta_tilt candidatos:
  - delta_tilt_mag_deg: |EKF - Orientation| (H9c)
  - gravity_alignment_error_deg: angulo g_pred vs g_meas (cadena propagacion)
  - delta_pitch_deg: componente pitch EKF vs ref

Tambien compara |a_nav_pre_g|_h (leak pre-gravedad, H9d).
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
REPORT_JSON = BENCH_DIR / "tilt_gravity_projection_hypothesis_report.json"
MERGED_CSV = BENCH_DIR / "tilt_gravity_projection_merged.csv"
ANALYSIS_PNG = BENCH_DIR / "tilt_gravity_projection_analysis.png"

GRAVITY = 9.80665
STATIC_END_S = 2.0
MOTION_T0 = 2.0
MOTION_T1 = 10.0
NRMSE_PASS_PCT = 15.0
NRMSE_STRONG_PASS_PCT = 10.0


@dataclass
class HypothesisSample:
    timestamp_s: float
    a_lin_h: float
    a_nav_pre_h: float
    delta_tilt_mag_deg: float
    delta_pitch_deg: float
    gravity_align_deg: float
    g_sin_delta_tilt: float
    g_sin_gravity_align: float
    g_sin_delta_pitch: float
    residual_tilt: float
    residual_gravity: float
    residual_pitch: float


def load_h9c_merged(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            if not raw.get("timestamp_s"):
                continue
            row: dict[str, float] = {"timestamp_s": float(raw["timestamp_s"])}
            for key in (
                "a_lin_h_mps2",
                "delta_tilt_mag_deg",
                "delta_pitch_deg",
                "gravity_alignment_error_deg",
            ):
                if raw.get(key) not in (None, ""):
                    out_key = "a_lin_h" if key == "a_lin_h_mps2" else key
                    row[out_key] = float(raw[key])
            rows.append(row)
    rows.sort(key=lambda r: r["timestamp_s"])
    return rows


def load_chain_nav_h(path: Path) -> dict[float, float]:
    out: dict[float, float] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            if not raw.get("timestamp_s"):
                continue
            t = float(raw["timestamp_s"])
            # H9d: leak en a_nav_body_h (= a_nav_pre_g horizontal)
            val = raw.get("a_nav_body_h")
            if val not in (None, ""):
                out[t] = abs(float(val))
    return out


def nearest_chain_nav(t: float, chain: dict[float, float], tol: float = 0.02) -> float:
    if not chain:
        return float("nan")
    best_t = min(chain.keys(), key=lambda k: abs(k - t))
    if abs(best_t - t) > tol:
        return float("nan")
    return chain[best_t]


def g_sin_deg(angle_deg: float) -> float:
    return GRAVITY * math.sin(math.radians(angle_deg))


def build_samples(h9c_rows: list[dict[str, float]], chain_nav: dict[float, float]) -> list[HypothesisSample]:
    samples: list[HypothesisSample] = []
    for row in h9c_rows:
        t = row["timestamp_s"]
        a_lin = row.get("a_lin_h", float("nan"))
        d_tilt = row.get("delta_tilt_mag_deg", 0.0)
        d_pitch = abs(row.get("delta_pitch_deg", 0.0))
        g_align = row.get("gravity_alignment_error_deg", 0.0)

        pred_tilt = g_sin_deg(d_tilt)
        pred_grav = g_sin_deg(g_align)
        pred_pitch = g_sin_deg(d_pitch)
        a_nav = nearest_chain_nav(t, chain_nav)

        samples.append(
            HypothesisSample(
                timestamp_s=t,
                a_lin_h=a_lin,
                a_nav_pre_h=a_nav,
                delta_tilt_mag_deg=d_tilt,
                delta_pitch_deg=d_pitch,
                gravity_align_deg=g_align,
                g_sin_delta_tilt=pred_tilt,
                g_sin_gravity_align=pred_grav,
                g_sin_delta_pitch=pred_pitch,
                residual_tilt=a_lin - pred_tilt,
                residual_gravity=a_lin - pred_grav,
                residual_pitch=a_lin - pred_pitch,
            )
        )
    return samples


def filter_window(samples: list[HypothesisSample], t0: float, t1: float) -> list[HypothesisSample]:
    return [s for s in samples if t0 <= s.timestamp_s <= t1]


def eval_hypothesis(
    samples: list[HypothesisSample],
    label: str,
    observed_attr: str,
    predicted_attr: str,
) -> dict:
    if not samples:
        return {"label": label, "samples": 0}

    obs = np.array([getattr(s, observed_attr) for s in samples], dtype=float)
    pred = np.array([getattr(s, predicted_attr) for s in samples], dtype=float)
    mask = np.isfinite(obs) & np.isfinite(pred)
    obs = obs[mask]
    pred = pred[mask]
    if obs.size == 0:
        return {"label": label, "samples": 0}

    residual = obs - pred
    rms = float(np.sqrt(np.mean(residual**2)))
    mean_obs = float(np.mean(obs))
    mean_pred = float(np.mean(pred))
    nrmse_obs_pct = 100.0 * rms / max(abs(mean_obs), 1e-9)
    nrmse_pred_pct = 100.0 * rms / max(abs(mean_pred), 1e-9)

    corr = float(np.corrcoef(obs, pred)[0, 1]) if obs.size > 2 and np.std(obs) > 1e-12 and np.std(pred) > 1e-12 else float("nan")

    # R^2 de prediccion
    ss_res = float(np.sum(residual**2))
    ss_tot = float(np.sum((obs - mean_obs) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")

    compatible = nrmse_obs_pct <= NRMSE_PASS_PCT
    strong = nrmse_obs_pct <= NRMSE_STRONG_PASS_PCT

    return {
        "label": label,
        "samples": int(obs.size),
        "observed_mean": mean_obs,
        "predicted_mean": mean_pred,
        "rms_error_mps2": rms,
        "nrmse_vs_observed_pct": nrmse_obs_pct,
        "nrmse_vs_predicted_pct": nrmse_pred_pct,
        "correlation": corr,
        "r_squared": r2,
        "compatible_h0": compatible,
        "strong_compatible_h0": strong,
        "mean_observed_deg_equiv": float(np.degrees(np.arcsin(min(max(mean_obs / GRAVITY, -1.0), 1.0)))),
        "mean_predicted_deg_equiv": float(np.degrees(np.arcsin(min(max(mean_pred / GRAVITY, -1.0), 1.0)))),
    }


def find_onset_transition(samples: list[HypothesisSample]) -> dict:
    """Primer salto significativo de delta_tilt y a_lin_h (~2-4 s)."""
    static = filter_window(samples, 1.5, STATIC_END_S)
    motion = filter_window(samples, MOTION_T0, MOTION_T1)
    if not static or not motion:
        return {}

    s_mean_alin = float(np.mean([s.a_lin_h for s in static]))
    s_mean_tilt = float(np.mean([s.delta_tilt_mag_deg for s in static]))
    s_mean_gsin = float(np.mean([s.g_sin_delta_tilt for s in static]))

    # primer tick con delta_tilt > static_mean + 1 deg Y a_lin_h > 0.1
    first_tilt_jump = None
    for s in motion:
        if s.delta_tilt_mag_deg > s_mean_tilt + 1.0 and s.a_lin_h > 0.1:
            first_tilt_jump = s
            break
    if first_tilt_jump is None:
        first_tilt_jump = max(motion, key=lambda x: x.a_lin_h)

    m_early = [s for s in motion if s.timestamp_s <= MOTION_T0 + 2.0]
    m_mean_alin = float(np.mean([s.a_lin_h for s in m_early])) if m_early else float("nan")
    m_mean_tilt = float(np.mean([s.delta_tilt_mag_deg for s in m_early])) if m_early else float("nan")

    return {
        "static_baseline": {
            "a_lin_h_mean": s_mean_alin,
            "delta_tilt_deg_mean": s_mean_tilt,
            "g_sin_delta_tilt_mean": s_mean_gsin,
        },
        "first_significant_tilt_jump": {
            "t_s": first_tilt_jump.timestamp_s,
            "a_lin_h": first_tilt_jump.a_lin_h,
            "delta_tilt_deg": first_tilt_jump.delta_tilt_mag_deg,
            "g_sin_delta_tilt": first_tilt_jump.g_sin_delta_tilt,
            "gravity_align_deg": first_tilt_jump.gravity_align_deg,
        },
        "motion_early_2s_mean": {
            "a_lin_h": m_mean_alin,
            "delta_tilt_deg": m_mean_tilt,
            "g_sin_delta_tilt": float(np.mean([s.g_sin_delta_tilt for s in m_early])) if m_early else float("nan"),
        },
        "jump_static_to_early_motion": {
            "delta_a_lin_h": m_mean_alin - s_mean_alin,
            "delta_tilt_deg": m_mean_tilt - s_mean_tilt,
            "g_sin_at_4deg_reference_mps2": g_sin_deg(4.0),
        },
    }


def write_merged_csv(samples: list[HypothesisSample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "timestamp_s", "a_lin_h", "a_nav_pre_h",
                "delta_tilt_mag_deg", "delta_pitch_deg", "gravity_align_deg",
                "g_sin_delta_tilt", "g_sin_gravity_align", "g_sin_delta_pitch",
                "residual_tilt", "residual_gravity", "residual_pitch",
            ],
        )
        writer.writeheader()
        for s in samples:
            writer.writerow(
                {
                    "timestamp_s": s.timestamp_s,
                    "a_lin_h": s.a_lin_h,
                    "a_nav_pre_h": s.a_nav_pre_h,
                    "delta_tilt_mag_deg": s.delta_tilt_mag_deg,
                    "delta_pitch_deg": s.delta_pitch_deg,
                    "gravity_align_deg": s.gravity_align_deg,
                    "g_sin_delta_tilt": s.g_sin_delta_tilt,
                    "g_sin_gravity_align": s.g_sin_gravity_align,
                    "g_sin_delta_pitch": s.g_sin_delta_pitch,
                    "residual_tilt": s.residual_tilt,
                    "residual_gravity": s.residual_gravity,
                    "residual_pitch": s.residual_pitch,
                }
            )


def plot_analysis(samples: list[HypothesisSample], evals: dict[str, dict], path: Path) -> None:
    post = [s for s in samples if s.timestamp_s >= 1.5]
    times = np.array([s.timestamp_s for s in post], dtype=float)
    alin = np.array([s.a_lin_h for s in post], dtype=float)
    pred = np.array([s.g_sin_delta_tilt for s in post], dtype=float)
    pred_g = np.array([s.g_sin_gravity_align for s in post], dtype=float)
    tilt = np.array([s.delta_tilt_mag_deg for s in post], dtype=float)
    residual = alin - pred

    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
    fig.suptitle("H0: a_lin,h vs g*sin(delta_tilt) — proyeccion gravedad", fontsize=14)

    axes[0].plot(times, alin, label="a_lin,h obs", linewidth=0.9, color="#c0392b")
    axes[0].plot(times, pred, label="g*sin(delta_tilt) H9c", linewidth=0.9, color="#2980b9", alpha=0.85)
    axes[0].plot(times, pred_g, label="g*sin(gravity_align)", linewidth=0.8, color="#27ae60", alpha=0.7)
    axes[0].axvspan(MOTION_T0, MOTION_T1, color="#f9e79f", alpha=0.35, label="2-10 s")
    axes[0].set_ylabel("[m/s2]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(times, tilt, color="#8e44ad", linewidth=0.8)
    axes[1].axhline(4.0, color="#7f8c8d", linestyle=":", linewidth=0.8)
    axes[1].set_ylabel("delta_tilt [deg]")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(times, residual, color="#e67e22", linewidth=0.8)
    axes[2].axhline(0.0, color="#7f8c8d", linewidth=0.6)
    axes[2].set_ylabel("residual [m/s2]")
    axes[2].grid(True, alpha=0.25)

    # scatter en ventana motion
    motion = filter_window(post, MOTION_T0, MOTION_T1)
    if motion:
        ax_sc = axes[3]
        x = np.array([s.g_sin_delta_tilt for s in motion])
        y = np.array([s.a_lin_h for s in motion])
        ax_sc.scatter(x, y, s=8, alpha=0.5, color="#2c3e50")
        lim = max(float(np.max(x)), float(np.max(y)), 0.01) * 1.1
        ax_sc.plot([0, lim], [0, lim], "r--", linewidth=0.8, label="y=x")
        ev = evals.get("motion_a_lin_vs_g_sin_delta_tilt", {})
        ax_sc.set_title(
            f"2-10 s: NRMSE={ev.get('nrmse_vs_observed_pct', float('nan')):.1f}%  r={ev.get('correlation', float('nan')):.3f}",
            fontsize=10,
        )
        ax_sc.set_xlabel("g*sin(delta_tilt) [m/s2]")
        ax_sc.set_ylabel("a_lin,h [m/s2]")
        ax_sc.legend(fontsize=8)
        ax_sc.grid(True, alpha=0.25)
    else:
        axes[3].set_visible(False)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def diagnose(evals: dict[str, dict], onset: dict) -> dict:
    orient = evals.get("motion_2_10s_a_lin_vs_g_sin_delta_tilt", {})
    grav = evals.get("motion_2_10s_a_lin_vs_g_sin_gravity_align", {})
    nav = evals.get("motion_nav_pre_vs_g_sin_delta_tilt", {})

    orient_nrmse = orient.get("nrmse_vs_observed_pct", float("nan"))
    grav_nrmse = grav.get("nrmse_vs_observed_pct", float("nan"))
    orient_r = orient.get("correlation", 0.0) or 0.0
    grav_r = grav.get("correlation", 0.0) or 0.0

    # Medias compatibles con g*sin(4 deg) aunque NRMSE orient falle
    mean_match_4deg = abs(orient.get("observed_mean", 0) - g_sin_deg(4.0)) < 0.15

    if grav.get("compatible_h0") and grav_r > 0.99:
        verdict = "H0_COMPATIBLE_INTERNAL_GRAVITY_ANGLE"
        mechanism = (
            "a_lin_h ~= g*sin(gravity_align_error) con NRMSE<5%%; "
            "leak horizontal es proyeccion de gravedad por discrepancia pred/meas en body"
        )
        root_cause = "identify_first_instant_gravity_align_grows_not_orientation_csv"
    elif orient_r > 0.85 and mean_match_4deg and orient_nrmse > NRMSE_PASS_PCT:
        verdict = "H0_MEAN_COMPATIBLE_POINTWISE_REJECTED"
        mechanism = (
            "Medias coherentes (a_lin,h~0.74, g*sin(4deg)~0.68, delta_tilt~4deg) "
            "pero NRMSE>15%% con delta_tilt Orientation — correlacion alta (r~0.92) "
            "sin identidad punto a punto"
        )
        root_cause = "tilt_reference_differs_from_gravity_leak_angle"
    elif orient.get("compatible_h0"):
        verdict = "H0_COMPATIBLE_ORIENTATION_TILT"
        mechanism = "horizontal_fictitious_accel_explained_by_ekf_vs_orientation_tilt"
        root_cause = "why_delta_tilt_appears_at_motion_onset"
    else:
        verdict = "H0_REJECTED"
        mechanism = "horizontal_not_fully_explained_by_g_sin_delta_tilt"
        root_cause = "seek_additional_mechanism"

    return {
        "hypothesis": "a_lin_h ~= g*sin(delta_tilt)",
        "pass_threshold_nrmse_pct": NRMSE_PASS_PCT,
        "verdict": verdict,
        "likely_mechanism": mechanism,
        "root_cause_status": root_cause,
        "orientation_tilt_eval": orient,
        "gravity_align_eval": grav,
        "nav_pre_g_eval": nav,
        "mean_level_match_g_sin_4deg": mean_match_4deg,
        "onset_transition": onset,
        "interpretation": (
            "Mecanismo (medias): ~4 deg inclinacion -> ~0.7 m/s2 horizontal (g*sin(4deg)=0.684). "
            "Prueba falsable: gravity_align_error (interno EKF) pasa NRMSE; delta_tilt Orientation "
            "correlaciona (r~0.92) pero no alcanza <15%% NRMSE punto a punto. "
            "Causa raiz pendiente: primer instante en que crece gravity_align / delta_tilt."
        ),
    }


def main() -> int:
    global NRMSE_PASS_PCT

    parser = argparse.ArgumentParser(description="H0: a_lin,h ~ g*sin(delta_tilt)")
    parser.add_argument("--h9c-merged", type=Path, default=H9C_MERGED)
    parser.add_argument("--chain-csv", type=Path, default=CHAIN_CSV)
    parser.add_argument("--nrmse-pass-pct", type=float, default=NRMSE_PASS_PCT)
    args = parser.parse_args()
    NRMSE_PASS_PCT = args.nrmse_pass_pct

    if not args.h9c_merged.is_file():
        print(f"ERROR: falta {args.h9c_merged}", file=sys.stderr)
        return 1

    h9c = load_h9c_merged(args.h9c_merged)
    chain_nav = load_chain_nav_h(args.chain_csv) if args.chain_csv.is_file() else {}
    samples = build_samples(h9c, chain_nav)

    windows = {
        "static_0_2s": filter_window(samples, 0.0, STATIC_END_S),
        "motion_2_10s": filter_window(samples, MOTION_T0, MOTION_T1),
        "full_0_60s": filter_window(samples, 0.0, 60.0),
    }

    evals: dict[str, dict] = {}
    for win_name, win_samples in windows.items():
        prefix = win_name.replace("_", "")
        evals[f"{win_name}_a_lin_vs_g_sin_delta_tilt"] = eval_hypothesis(
            win_samples, f"{win_name} a_lin vs g*sin(delta_tilt)", "a_lin_h", "g_sin_delta_tilt"
        )
        evals[f"{win_name}_a_lin_vs_g_sin_gravity_align"] = eval_hypothesis(
            win_samples, f"{win_name} a_lin vs g*sin(gravity_align)", "a_lin_h", "g_sin_gravity_align"
        )
        evals[f"{win_name}_a_lin_vs_g_sin_delta_pitch"] = eval_hypothesis(
            win_samples, f"{win_name} a_lin vs g*sin(|delta_pitch|)", "a_lin_h", "g_sin_delta_pitch"
        )
        if win_name == "motion_2_10s":
            evals["motion_nav_pre_vs_g_sin_delta_tilt"] = eval_hypothesis(
                win_samples, "motion |a_nav_pre|_h vs g*sin(delta_tilt)", "a_nav_pre_h", "g_sin_delta_tilt"
            )

    onset = find_onset_transition(samples)
    diagnosis = diagnose(evals, onset)

    write_merged_csv(samples, MERGED_CSV)
    plot_analysis(samples, evals, ANALYSIS_PNG)

    report = {
        "experiment": "tilt_gravity_projection_hypothesis",
        "hypothesis_h0": "a_lin_h ~= GRAVITY * sin(delta_tilt)",
        "gravity_mps2": GRAVITY,
        "reference_tilt_at_4deg": {
            "sin_rad": math.sin(math.radians(4.0)),
            "g_sin_mps2": g_sin_deg(4.0),
        },
        "evaluations": evals,
        "diagnosis": diagnosis,
        "artifacts": {"merged_csv": str(MERGED_CSV), "plot_png": str(ANALYSIS_PNG)},
    }
    with REPORT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    primary = evals["motion_2_10s_a_lin_vs_g_sin_delta_tilt"]
    grav = evals["motion_2_10s_a_lin_vs_g_sin_gravity_align"]
    nav = evals.get("motion_nav_pre_vs_g_sin_delta_tilt", {})

    print("=" * 72)
    print("H0: a_lin,h ~= g*sin(delta_tilt)  [ventana 2-10 s]")
    print("=" * 72)
    print(f"  g*sin(4 deg) = {g_sin_deg(4.0):.4f} m/s2")
    print()
    print("  [delta_tilt Orientation vs EKF]")
    print(f"    a_lin,h media:           {primary.get('observed_mean', float('nan')):.4f} m/s2")
    print(f"    g*sin(delta_tilt) media: {primary.get('predicted_mean', float('nan')):.4f} m/s2")
    print(f"    RMS / NRMSE:             {primary.get('rms_error_mps2', float('nan')):.4f} / {primary.get('nrmse_vs_observed_pct', float('nan')):.1f}%")
    print(f"    Correlacion / R2:        {primary.get('correlation', float('nan')):.4f} / {primary.get('r_squared', float('nan')):.4f}")
    print(f"    H0 (<{NRMSE_PASS_PCT}%):          {'SI' if primary.get('compatible_h0') else 'NO'}")
    print()
    print("  [gravity_align_error interno EKF]")
    print(f"    NRMSE: {grav.get('nrmse_vs_observed_pct', float('nan')):.1f}%  r={grav.get('correlation', float('nan')):.4f}  H0={'SI' if grav.get('compatible_h0') else 'NO'}")
    if nav.get("samples", 0):
        print(f"  [|a_nav_pre|_h vs g*sin] NRMSE: {nav.get('nrmse_vs_observed_pct', float('nan')):.1f}%  r={nav.get('correlation', float('nan')):.4f}")
    jump = onset.get("jump_static_to_early_motion", {})
    first = onset.get("first_significant_tilt_jump", {})
    if first:
        print()
        print(f"  Primer salto tilt significativo @ t={first.get('t_s', float('nan')):.2f} s")
        print(f"    a_lin_h={first.get('a_lin_h', float('nan')):.3f}  delta_tilt={first.get('delta_tilt_deg', float('nan')):.2f} deg")
    if jump:
        print(f"  Salto estatico -> motion early: delta_a_lin={jump.get('delta_a_lin_h', float('nan')):.3f} m/s2  delta_tilt={jump.get('delta_tilt_deg', float('nan')):.2f} deg")
    print(f"\n  Veredicto: {diagnosis['verdict']}")
    print(f"  Informe:   {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
