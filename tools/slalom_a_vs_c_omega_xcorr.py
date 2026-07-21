#!/usr/bin/env python3
"""SLALOM A vs C: cross-correlation ‖ω‖(t) vs |d(Δdrift)/dt|(t+τ).

Sweeps lag τ ∈ [0, τ_max] to test delayed attitude→velocity→position propagation
(vs instantaneous argmax coincidence). Prefer CSV measured yaw_rate when alive;
else reconstruct from slalom truth kinematics. Also compare measured vs truth
when both exist.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
CELL_A = REPO / "docs/benchmarks/slalom_cellA_jcorrect_imuideal_s71_telemetry.csv"
CELL_C = REPO / "docs/benchmarks/slalom_cellC_jlegacy_imuideal_s71_telemetry.csv"
OUT_DIR = REPO / "docs/benchmarks/jacobian_imu_ab"
OUT_JSON = OUT_DIR / "slalom_a_vs_c_omega_xcorr.json"
OUT_MD = OUT_DIR / "slalom_a_vs_c_omega_xcorr.md"
OUT_PNG = OUT_DIR / "fig_slalom_omega_xcorr.png"

TC04_SPEED_KMH = 50.0
TC04_SPEED_MPS = TC04_SPEED_KMH / 3.6
TC04_MAX_LATERAL_ACCEL_MPS2 = 3.0
TC04_SLALOM_PERIOD_S = 4.0
K_TWO_PI = 2.0 * math.pi
K_SLALOM_OMEGA_RADPS = K_TWO_PI / TC04_SLALOM_PERIOD_S
K_YAW_AMPLITUDE_RAD = TC04_MAX_LATERAL_ACCEL_MPS2 / (
    TC04_SPEED_MPS * K_SLALOM_OMEGA_RADPS
)

TAU_MAX_S = 3.0
# Analysis window for series (need room for lag at end)
T_ANALYSIS = (0.0, 20.0)
# Secondary focus around first maneuvers
T_FOCUS = (0.0, 8.0)
LOCAL_PEAK_FRAC = 0.85  # local max if >= this * global max and separation


def load_telemetry(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["t_s"] = df["time_us"].astype(float) * 1e-6
    return df


def truth_omega(t_s: np.ndarray) -> np.ndarray:
    phase = K_SLALOM_OMEGA_RADPS * t_s
    return K_YAW_AMPLITUDE_RAD * K_SLALOM_OMEGA_RADPS * np.cos(phase)


def yaw_rate_status(yaw: np.ndarray) -> dict:
    y = np.asarray(yaw, dtype=float)
    finite = y[np.isfinite(y)]
    all_zero = bool(len(finite) == 0 or np.max(np.abs(finite)) < 1e-12)
    return {
        "n": int(len(y)),
        "all_zero": all_zero,
        "max_abs": float(np.max(np.abs(finite))) if len(finite) else 0.0,
        "rms": float(np.sqrt(np.mean(finite**2))) if len(finite) else 0.0,
    }


def pearson_r(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if len(a) < 3:
        return float("nan")
    sa = a.std()
    sb = b.std()
    if sa < 1e-15 or sb < 1e-15:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def cross_corr_vs_lag(
    t_s: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    tau_max_s: float,
) -> dict:
    """r(τ) = corr(x(t), y(t+τ)) for τ in [0, tau_max], same sampling grid."""
    t_s = np.asarray(t_s, dtype=float)
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    dt = float(np.median(np.diff(t_s)))
    if dt <= 0:
        raise ValueError("non-positive dt")
    n_lag = int(round(tau_max_s / dt))
    taus = []
    rs = []
    ns = []
    for k in range(n_lag + 1):
        # x[0 : N-k] with y[k : N]
        if k == 0:
            xa, ya = x, y
        else:
            xa, ya = x[:-k], y[k:]
        n = len(xa)
        # require overlap covering at least half of analysis
        if n < max(50, len(x) // 4):
            break
        r = pearson_r(xa, ya)
        taus.append(k * dt)
        rs.append(r)
        ns.append(n)
    taus_a = np.asarray(taus)
    rs_a = np.asarray(rs, dtype=float)
    # global max among finite
    finite = np.isfinite(rs_a)
    if not np.any(finite):
        return {
            "dt_s": dt,
            "tau_s": taus,
            "r": rs,
            "n_overlap": ns,
            "tau_peak_s": None,
            "r_peak": None,
            "r_at_0": None,
            "local_peaks": [],
            "clear_peak": False,
        }
    i_peak = int(np.nanargmax(rs_a))
    r_peak = float(rs_a[i_peak])
    tau_peak = float(taus_a[i_peak])
    r0 = float(rs_a[0]) if finite[0] else float("nan")

    # local peaks: strict local max, r >= LOCAL_PEAK_FRAC * r_peak, τ>0 preferred
    local = []
    for i in range(1, len(rs_a) - 1):
        if not finite[i]:
            continue
        if rs_a[i] >= rs_a[i - 1] and rs_a[i] >= rs_a[i + 1]:
            if rs_a[i] >= LOCAL_PEAK_FRAC * r_peak:
                local.append(
                    {
                        "tau_s": float(taus_a[i]),
                        "r": float(rs_a[i]),
                        "n_overlap": int(ns[i]),
                    }
                )
    # include τ=0 if it is the global max
    if i_peak == 0:
        local.insert(
            0,
            {"tau_s": 0.0, "r": r_peak, "n_overlap": int(ns[0]), "is_global": True},
        )
    elif not any(abs(p["tau_s"] - tau_peak) < 0.5 * dt for p in local):
        local.append(
            {
                "tau_s": tau_peak,
                "r": r_peak,
                "n_overlap": int(ns[i_peak]),
                "is_global": True,
            }
        )

    # "clear peak": max at τ>0 and r_peak meaningfully above r(0)
    clear = bool(
        tau_peak > 0.5 * dt
        and np.isfinite(r_peak)
        and (r_peak - r0) > 0.05
        and r_peak > 0.2
    )

    return {
        "dt_s": dt,
        "tau_s": [float(v) for v in taus_a],
        "r": [float(v) if np.isfinite(v) else None for v in rs_a],
        "n_overlap": ns,
        "tau_peak_s": tau_peak,
        "r_peak": r_peak,
        "r_at_0": r0,
        "delta_r_peak_minus_r0": float(r_peak - r0) if np.isfinite(r0) else None,
        "local_peaks": local,
        "clear_peak": clear,
    }


def window_mask(t_s: np.ndarray, t0: float, t1: float) -> np.ndarray:
    return (t_s >= t0) & (t_s <= t1)


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    a = load_telemetry(CELL_A)
    c = load_telemetry(CELL_C)
    # align on time (should be identical grids)
    n = min(len(a), len(c))
    a = a.iloc[:n].reset_index(drop=True)
    c = c.iloc[:n].reset_index(drop=True)
    t_s = a["t_s"].to_numpy()
    drift_a = a["drift_m"].to_numpy(dtype=float)
    drift_c = c["drift_m"].to_numpy(dtype=float)
    d_drift = drift_a - drift_c
    # |d(Δdrift)/dt| via gradient
    d_rate = np.abs(np.gradient(d_drift, t_s))

    yr_a = a["yaw_rate"].to_numpy(dtype=float) if "yaw_rate" in a.columns else np.zeros(n)
    yr_c = c["yaw_rate"].to_numpy(dtype=float) if "yaw_rate" in c.columns else np.zeros(n)
    st_a = yaw_rate_status(yr_a)
    st_c = yaw_rate_status(yr_c)
    w_truth = np.abs(truth_omega(t_s))

    if not st_a["all_zero"]:
        w_meas = np.abs(yr_a)
        omega_source = "csv_measured_A"
    else:
        w_meas = w_truth.copy()
        omega_source = "truth_kinematics_fallback"

    # measured vs truth agreement (when CSV alive)
    meas_vs_truth = None
    if not st_a["all_zero"]:
        meas_vs_truth = {
            "pearson_r": pearson_r(yr_a, truth_omega(t_s)),
            "max_abs_err": float(np.max(np.abs(yr_a - truth_omega(t_s)))),
            "rms_err": float(np.sqrt(np.mean((yr_a - truth_omega(t_s)) ** 2))),
            "note": (
                "ideal SLALOM: imu.gyro_z := truth.yaw_rate "
                "(make_ideal_slalom_imu); CSV now logs that measured ω"
            ),
        }

    results = {
        "cells": {"A": str(CELL_A.name), "C": str(CELL_C.name)},
        "yaw_rate_csv": {"A": st_a, "C": st_c},
        "omega_source_for_xcorr": omega_source,
        "measured_vs_truth": meas_vs_truth,
        "tau_max_s": TAU_MAX_S,
        "windows": {},
    }

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
    colors = {"analysis": "C0", "focus": "C1", "truth_check": "C2"}

    for win_name, (t0, t1) in (
        ("analysis_0_20s", T_ANALYSIS),
        ("focus_0_8s", T_FOCUS),
    ):
        m = window_mask(t_s, t0, t1)
        # leave room for lag inside window: truncate y series end conceptually via overlap
        t_w = t_s[m]
        x_w = w_meas[m]
        y_w = d_rate[m]
        xc = cross_corr_vs_lag(t_w, x_w, y_w, TAU_MAX_S)
        results["windows"][win_name] = {
            "t0": t0,
            "t1": t1,
            "n": int(m.sum()),
            "xcorr": {
                k: v
                for k, v in xc.items()
                if k not in ("tau_s", "r", "n_overlap")
            },
            "xcorr_curve": {
                "tau_s": xc["tau_s"],
                "r": xc["r"],
            },
        }
        ax = axes[0] if win_name.startswith("analysis") else axes[1]
        ax.plot(xc["tau_s"], xc["r"], color=colors["analysis" if "analysis" in win_name else "focus"], label=win_name)
        if xc["tau_peak_s"] is not None:
            ax.axvline(xc["tau_peak_s"], color="0.4", ls="--", lw=0.9)
            ax.scatter([xc["tau_peak_s"]], [xc["r_peak"]], zorder=3)
        ax.set_ylabel("Pearson r")
        ax.set_title(
            f"{win_name}: peak τ={xc['tau_peak_s']} s, r={xc['r_peak']} "
            f"(clear={xc['clear_peak']})"
        )
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)

    # optional: same xcorr with truth ω even if CSV alive (sanity)
    if omega_source != "truth_kinematics_fallback":
        m = window_mask(t_s, *T_FOCUS)
        xc_truth = cross_corr_vs_lag(t_s[m], w_truth[m], d_rate[m], TAU_MAX_S)
        results["windows"]["focus_0_8s_truth_omega"] = {
            "t0": T_FOCUS[0],
            "t1": T_FOCUS[1],
            "n": int(m.sum()),
            "xcorr": {
                k: v
                for k, v in xc_truth.items()
                if k not in ("tau_s", "r", "n_overlap")
            },
        }
        axes[1].plot(
            xc_truth["tau_s"],
            xc_truth["r"],
            color=colors["truth_check"],
            ls=":",
            label="focus truth_ω",
        )
        axes[1].legend(loc="best", fontsize=8)

    axes[1].set_xlabel("lag τ (s)  —  y = |d(Δdrift)/dt|(t+τ), x = ‖ω‖(t)")
    fig.suptitle(
        f"SLALOM A vs C xcorr  |  ω source: {omega_source}",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=140)
    plt.close(fig)

    # Instantaneous argmax lag (for continuity with prior note)
    m = window_mask(t_s, 1.0, 5.0)
    t_m = t_s[m]
    i_w = int(np.argmax(w_meas[m]))
    i_r = int(np.argmax(d_rate[m]))
    results["argmax_lag_1_5s"] = {
        "t_omega_s": float(t_m[i_w]),
        "t_divrate_s": float(t_m[i_r]),
        "lag_s": float(t_m[i_r] - t_m[i_w]),
        "note": "argmax coincidence (previous check); not the primary test",
    }

    # Verdict
    focus = results["windows"]["focus_0_8s"]["xcorr"]
    analysis = results["windows"]["analysis_0_20s"]["xcorr"]
    if focus["clear_peak"] or analysis["clear_peak"]:
        verdict = (
            "DELAYED_COUPLING_SUPPORTED — clear xcorr peak at τ>0; "
            "attitude→vel→pos chain remains viable; anchor K/P to reported lag(s)"
        )
    elif (
        focus["tau_peak_s"] is not None
        and focus["tau_peak_s"] > 0.05
        and focus["r_peak"] is not None
        and focus["r_peak"] > 0.15
    ):
        verdict = (
            "WEAK_DELAYED_SIGNAL — peak at τ>0 but not clear by thresholds; "
            "do not discard attitude family yet; inspect curve / local peaks"
        )
    else:
        verdict = (
            "ATTITUDE_FAMILY_WEAKENED — no clear xcorr peak for ‖ω‖ vs |dΔ/dt|; "
            "consider other suspects (e.g. accel bias)"
        )
    results["verdict"] = verdict

    with OUT_JSON.open("w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    # Markdown
    lines = [
        "# SLALOM A vs C — cross-correlation ‖ω‖ × |d(Δdrift)/dt|",
        "",
        f"**ω source:** `{omega_source}`  ",
        f"**τ sweep:** 0 … {TAU_MAX_S} s  ",
        f"**Figure:** `{OUT_PNG.name}`  ",
        "",
        "## Measured ω (CSV yaw_rate)",
        "",
        f"- A all-zero: **{st_a['all_zero']}** (max|yaw_rate|={st_a['max_abs']:.6g})",
        f"- C all-zero: **{st_c['all_zero']}** (max|yaw_rate|={st_c['max_abs']:.6g})",
        "",
    ]
    if meas_vs_truth:
        lines += [
            "### Measured vs truth kinematics",
            "",
            f"- pearson r: **{meas_vs_truth['pearson_r']:.6f}**",
            f"- max|err|: {meas_vs_truth['max_abs_err']:.3e}",
            f"- rms err: {meas_vs_truth['rms_err']:.3e}",
            f"- {meas_vs_truth['note']}",
            "",
        ]
    else:
        lines += [
            "CSV still dead → xcorr used truth reconstruction "
            "`A·ω·cos(ω·t)` (same as filter input under ideal IMU).",
            "",
        ]

    lines += [
        "## Cross-correlation peaks",
        "",
        "| Window | τ_peak (s) | r_peak | r(0) | Δr | clear_peak |",
        "|--------|------------|--------|------|-----|------------|",
    ]
    for win_name in ("analysis_0_20s", "focus_0_8s"):
        xc = results["windows"][win_name]["xcorr"]
        lines.append(
            f"| {win_name} | {xc['tau_peak_s']:.3f} | {xc['r_peak']:.4f} | "
            f"{xc['r_at_0']:.4f} | {xc['delta_r_peak_minus_r0']:.4f} | "
            f"**{xc['clear_peak']}** |"
        )
    lines += ["", "### Local peaks (focus 0–8 s)", ""]
    for p in results["windows"]["focus_0_8s"]["xcorr"]["local_peaks"]:
        lines.append(f"- τ={p['tau_s']:.3f} s, r={p['r']:.4f}")

    al = results["argmax_lag_1_5s"]
    lines += [
        "",
        "## Prior argmax lag (continuity)",
        "",
        f"- t_‖ω‖={al['t_omega_s']:.3f} s, t_|dΔ/dt|={al['t_divrate_s']:.3f} s, "
        f"lag=**{al['lag_s']:+.3f} s** (not the primary test)",
        "",
        "## Verdict",
        "",
        f"**{verdict}**",
        "",
        "Next: anchor K/P autopsy to τ_peak / local peaks above "
        "(and/or ~3.4 s if still a rate max), not to instantaneous argmax.",
        "",
    ]
    OUT_MD.write_text("\n".join(lines), encoding="utf-8")
    print(OUT_MD.read_text(encoding="utf-8"))
    print(f"Wrote {OUT_JSON}")
    print(f"Wrote {OUT_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
