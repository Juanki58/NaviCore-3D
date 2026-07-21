#!/usr/bin/env python3
"""SLALOM cell A vs C: omega coincidence + drift-delta burstiness (1.3-2.0 s).

Reads jcorrect (A) vs jlegacy (C) imuideal telemetry. Reconstructs truth yaw-rate
from slalom_scenario.cpp kinematics when CSV yaw_rate is dead (all zeros).
Reports burstiness B / top3_share (GAP-3 spirit) and omega vs |d(Delta drift)/dt|
coincidence metrics. No causal claims beyond coincidence.
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
OUT_JSON = OUT_DIR / "slalom_a_vs_c_omega_burstiness.json"
OUT_MD = OUT_DIR / "slalom_a_vs_c_omega_burstiness.md"
OUT_PNG = OUT_DIR / "fig_slalom_omega_vs_ddrift.png"

# From src/targets/generic_pc/slalom_benchmark.hpp + slalom_scenario.cpp
TC04_SPEED_KMH = 50.0
TC04_SPEED_MPS = TC04_SPEED_KMH / 3.6
TC04_MAX_LATERAL_ACCEL_MPS2 = 3.0
TC04_SLALOM_PERIOD_S = 4.0
K_TWO_PI = 2.0 * math.pi
K_SLALOM_OMEGA_RADPS = K_TWO_PI / TC04_SLALOM_PERIOD_S
K_YAW_AMPLITUDE_RAD = TC04_MAX_LATERAL_ACCEL_MPS2 / (
    TC04_SPEED_MPS * K_SLALOM_OMEGA_RADPS
)

# Burstiness thresholds (stated explicitly in outputs)
THRESH_TOP3_SHARE = 0.5
THRESH_B = 0.25

WIN_PRIMARY = (1.3, 2.0)
WIN_CONTEXT = (1.0, 5.0)
WIN_PLOT = (0.0, 5.0)


def load_telemetry(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["t_s"] = df["time_us"].astype(float) * 1e-6
    return df


def unwrap_yaw(yaw: np.ndarray) -> np.ndarray:
    """Unwrap yaw assuming radians (telemetry yaw is rad in this suite)."""
    return np.unwrap(np.asarray(yaw, dtype=float))


def finite_diff_yaw_rate(t_s: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    yaw_u = unwrap_yaw(yaw)
    w = np.zeros_like(yaw_u)
    dt = np.diff(t_s)
    dy = np.diff(yaw_u)
    # central-ish: forward at 0, backward at end, midpoints elsewhere via gradient
    with np.errstate(divide="ignore", invalid="ignore"):
        w = np.gradient(yaw_u, t_s)
    return w


def truth_omega(t_s: np.ndarray) -> np.ndarray:
    phase = K_SLALOM_OMEGA_RADPS * t_s
    return K_YAW_AMPLITUDE_RAD * K_SLALOM_OMEGA_RADPS * np.cos(phase)


def burstiness_metrics(x: np.ndarray, t_s: np.ndarray) -> dict:
    """B = max|dx|/sum|dx|; top3_share; dominate ticks (by end-of-interval time)."""
    if len(x) < 2:
        return {
            "n_intervals": 0,
            "B": None,
            "top3_share": None,
            "sum_abs_dx": 0.0,
            "max_abs_dx": None,
            "bursty": False,
            "dominating_ticks": [],
        }
    dx = np.diff(x)
    abs_dx = np.abs(dx)
    s = float(abs_dx.sum())
    if s <= 0.0:
        return {
            "n_intervals": int(len(dx)),
            "B": 0.0,
            "top3_share": 0.0,
            "sum_abs_dx": 0.0,
            "max_abs_dx": 0.0,
            "bursty": False,
            "dominating_ticks": [],
        }
    B = float(abs_dx.max() / s)
    order = np.argsort(-abs_dx)
    top3_idx = order[: min(3, len(order))]
    top3_share = float(abs_dx[top3_idx].sum() / s)
    # tick i in original series ends interval i-1 -> i; report end tick index in window frame
    dominating = []
    for rank, j in enumerate(top3_idx, start=1):
        dominating.append(
            {
                "rank": rank,
                "interval_i": int(j + 1),  # end index within series
                "t_end_s": float(t_s[j + 1]),
                "t_start_s": float(t_s[j]),
                "dx": float(dx[j]),
                "abs_dx": float(abs_dx[j]),
                "share": float(abs_dx[j] / s),
            }
        )
    bursty = bool(top3_share > THRESH_TOP3_SHARE or B > THRESH_B)
    return {
        "n_intervals": int(len(dx)),
        "B": B,
        "top3_share": top3_share,
        "sum_abs_dx": s,
        "max_abs_dx": float(abs_dx.max()),
        "bursty": bursty,
        "classification_rule": (
            f"bursty if top3_share > {THRESH_TOP3_SHARE} OR B > {THRESH_B}"
        ),
        "dominating_ticks": dominating,
    }


def window_mask(t_s: np.ndarray, t0: float, t1: float) -> np.ndarray:
    return (t_s >= t0) & (t_s <= t1)


def series_in_window(t_s: np.ndarray, x: np.ndarray, t0: float, t1: float):
    m = window_mask(t_s, t0, t1)
    return t_s[m], x[m]


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    a = load_telemetry(CELL_A)
    c = load_telemetry(CELL_C)
    if len(a) != len(c) or not np.allclose(a["time_us"].values, c["time_us"].values):
        raise SystemExit("A/C time grids differ")

    t_s = a["t_s"].to_numpy(dtype=float)
    drift_a = a["drift_m"].to_numpy(dtype=float)
    drift_c = c["drift_m"].to_numpy(dtype=float)
    delta = drift_a - drift_c  # signed
    abs_delta = np.abs(delta)
    abs_a = np.abs(drift_a)
    abs_c = np.abs(drift_c)

    yaw_rate_a_csv = a["yaw_rate"].to_numpy(dtype=float)
    yaw_rate_c_csv = c["yaw_rate"].to_numpy(dtype=float)
    yaw_rate_dead = bool(np.all(yaw_rate_a_csv == 0.0) and np.all(yaw_rate_c_csv == 0.0))

    omega_truth = truth_omega(t_s)
    omega_fd_a = finite_diff_yaw_rate(t_s, a["yaw"].to_numpy(dtype=float))
    omega_fd_c = finite_diff_yaw_rate(t_s, c["yaw"].to_numpy(dtype=float))

    # Per-tick derivatives of abs_delta / signed delta for coincidence
    d_abs_delta_dt = np.gradient(abs_delta, t_s)
    d_delta_dt = np.gradient(delta, t_s)
    abs_d_delta_dt = np.abs(d_delta_dt)

    results: dict = {
        "scenario": "SLALOM",
        "seed": 71,
        "comparison": "A (jcorrect+imuideal) vs C (jlegacy+imuideal)",
        "inputs": {
            "A": str(CELL_A.relative_to(REPO).as_posix()),
            "C": str(CELL_C.relative_to(REPO).as_posix()),
        },
        "kinematics": {
            "source": "src/scenarios/slalom_scenario.cpp + slalom_benchmark.hpp",
            "TC04_SPEED_MPS": TC04_SPEED_MPS,
            "TC04_MAX_LATERAL_ACCEL_MPS2": TC04_MAX_LATERAL_ACCEL_MPS2,
            "TC04_SLALOM_PERIOD_S": TC04_SLALOM_PERIOD_S,
            "kSlalomOmegaRadps": K_SLALOM_OMEGA_RADPS,
            "kYawAmplitudeRad": K_YAW_AMPLITUDE_RAD,
            "formula": (
                "phase = kSlalomOmegaRadps * t_s; "
                "yaw_rate = kYawAmplitudeRad * kSlalomOmegaRadps * cos(phase)"
            ),
        },
        "yaw_rate_csv": {
            "A_all_zero": bool(np.all(yaw_rate_a_csv == 0.0)),
            "C_all_zero": bool(np.all(yaw_rate_c_csv == 0.0)),
            "dead": yaw_rate_dead,
            "note": (
                "CSV yaw_rate all zeros for SLALOM; omega_truth reconstructed "
                "from scenario kinematics. Secondary: finite-diff of filter yaw."
            ),
        },
        "burstiness_thresholds": {
            "top3_share": THRESH_TOP3_SHARE,
            "B": THRESH_B,
            "rule": f"bursty if top3_share > {THRESH_TOP3_SHARE} OR B > {THRESH_B}",
        },
        "n_rows": int(len(t_s)),
        "duration_s": float(t_s[-1] - t_s[0]),
    }

    # Burstiness on windows
    burst_block: dict = {}
    for label, (t0, t1) in {
        "window_1p3_2p0": WIN_PRIMARY,
        "window_1p0_5p0": WIN_CONTEXT,
        "whole_run": (float(t_s[0]), float(t_s[-1])),
    }.items():
        tw, dw = series_in_window(t_s, delta, t0, t1)
        _, aw = series_in_window(t_s, abs_a, t0, t1)
        # also |d(drift_A-drift_C)| path: burstiness on signed delta (per-tick dx)
        # and on abs_a; plus whole-run note for |d(delta)| via abs(diff(delta))
        m_delta = burstiness_metrics(dw, tw)
        m_abs_a = burstiness_metrics(aw, tw)
        # B on |d(delta)| series meaning per-tick |dx| of signed delta — same as m_delta
        # Explicit whole-run comparison series name as requested
        burst_block[label] = {
            "t0": t0,
            "t1": t1,
            "x_signed_delta_drift_A_minus_C": m_delta,
            "x_abs_drift_A": m_abs_a,
        }
    results["burstiness"] = burst_block

    # Coincidence metrics
    m_primary = window_mask(t_s, *WIN_PRIMARY)
    m_1_5 = window_mask(t_s, *WIN_CONTEXT)
    m_0_5 = window_mask(t_s, *WIN_PLOT)

    # At time of max |d(delta)/dt| in 1.3-2s, what is |omega|?
    idx_local = np.argmax(abs_d_delta_dt[m_primary])
    idx_max_ddrift = np.where(m_primary)[0][idx_local]
    t_max_ddrift = float(t_s[idx_max_ddrift])

    # At time of max |omega| in first 5s (first turn peak)
    idx_local_w = np.argmax(np.abs(omega_truth[m_0_5]))
    idx_max_w = np.where(m_0_5)[0][idx_local_w]
    t_max_w = float(t_s[idx_max_w])

    # Lag in 1-5 s: argmax |omega| vs argmax |d(delta)/dt|
    idx_w_15 = np.where(m_1_5)[0][np.argmax(np.abs(omega_truth[m_1_5]))]
    idx_d_15 = np.where(m_1_5)[0][np.argmax(abs_d_delta_dt[m_1_5])]
    lag_s = float(t_s[idx_d_15] - t_s[idx_w_15])  # positive => divergence peak after omega peak

    # |omega| peaks are tied (cos extrema): t = 0, T/2, T, ... within 0-5 s
    abs_w05 = np.abs(omega_truth[m_0_5])
    wmax = float(abs_w05.max())
    tol = 1e-9 + 1e-6 * wmax
    peak_idxs_05 = np.where(m_0_5)[0][np.where(abs_w05 >= wmax - tol)[0]]
    # Deduplicate near-ties to exact kinematic extrema (keep samples at local max)
    tied_peaks = []
    for idx in peak_idxs_05:
        tied_peaks.append(
            {
                "t_s": float(t_s[idx]),
                "abs_omega_truth": float(abs(omega_truth[idx])),
                "omega_truth": float(omega_truth[idx]),
                "abs_delta_drift_m": float(abs_delta[idx]),
                "delta_drift_m": float(delta[idx]),
                "abs_d_delta_dt": float(abs_d_delta_dt[idx]),
                "d_delta_dt": float(d_delta_dt[idx]),
            }
        )
    # Prefer exact half-period samples: 0, 2, 4
    interior_peaks = [p for p in tied_peaks if p["t_s"] > 0.05]
    first_interior = interior_peaks[0] if interior_peaks else tied_peaks[0]

    coincidence = {
        "at_max_abs_d_delta_dt_in_1p3_2p0": {
            "t_s": t_max_ddrift,
            "abs_d_delta_dt": float(abs_d_delta_dt[idx_max_ddrift]),
            "d_delta_dt": float(d_delta_dt[idx_max_ddrift]),
            "delta_drift_m": float(delta[idx_max_ddrift]),
            "abs_delta_drift_m": float(abs_delta[idx_max_ddrift]),
            "abs_omega_truth": float(abs(omega_truth[idx_max_ddrift])),
            "abs_omega_fd_A": float(abs(omega_fd_a[idx_max_ddrift])),
            "abs_omega_fd_C": float(abs(omega_fd_c[idx_max_ddrift])),
        },
        "at_max_abs_omega_truth_in_0_5": {
            "t_s": t_max_w,
            "abs_omega_truth": float(abs(omega_truth[idx_max_w])),
            "omega_truth": float(omega_truth[idx_max_w]),
            "abs_delta_drift_m": float(abs_delta[idx_max_w]),
            "delta_drift_m": float(delta[idx_max_w]),
            "abs_d_delta_dt": float(abs_d_delta_dt[idx_max_w]),
            "d_delta_dt": float(d_delta_dt[idx_max_w]),
            "note": (
                "first argmax |omega| in 0-5 s is t=0 (cos phase); "
                "|omega| also peaks at T/2=2s, T=4s (tied amplitude)"
            ),
            "tied_peak_times_s": [float(p["t_s"]) for p in tied_peaks],
        },
        "at_first_interior_omega_peak_0_5": first_interior,
        "lag_1p0_5p0": {
            "t_argmax_abs_omega_s": float(t_s[idx_w_15]),
            "t_argmax_abs_d_delta_dt_s": float(t_s[idx_d_15]),
            "lag_s_divergence_minus_omega": lag_s,
            "definition": (
                "lag = t(argmax |d(delta)/dt|) - t(argmax |omega_truth|) in [1,5] s; "
                "positive means divergence-rate peak after omega peak"
            ),
        },
        "omega_at_divergence_rate_peak_note": (
            "Coincidence only: |omega| at max |d(delta)/dt| in 1.3-2 s; "
            "no causal attribution."
        ),
    }
    results["coincidence"] = coincidence

    # Plot
    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
    mplot = window_mask(t_s, *WIN_PLOT)
    tp = t_s[mplot]
    ax0.plot(tp, abs_delta[mplot], label="|drift_A - drift_C|", color="C0", lw=1.5)
    ax0.plot(tp, abs_a[mplot], label="|drift_A|", color="C1", lw=1.0, alpha=0.85)
    ax0.plot(tp, abs_c[mplot], label="|drift_C|", color="C2", lw=1.0, alpha=0.85)
    ax0.axvspan(WIN_PRIMARY[0], WIN_PRIMARY[1], color="orange", alpha=0.18, label="1.3-2.0 s")
    ax0.set_ylabel("drift (m)")
    ax0.legend(loc="upper left", fontsize=8)
    ax0.grid(True, alpha=0.3)
    ax0.set_title("SLALOM A vs C: drift divergence vs truth |ω| (seed 71)")

    ax1.plot(tp, np.abs(omega_truth[mplot]), label="|ω_truth| (kinematics)", color="C3", lw=1.5)
    ax1.plot(tp, np.abs(omega_fd_a[mplot]), label="|d(yaw_A)/dt|", color="C4", lw=1.0, alpha=0.8)
    ax1.plot(tp, np.abs(omega_fd_c[mplot]), label="|d(yaw_C)/dt|", color="C5", lw=1.0, alpha=0.8, ls="--")
    ax1.axvspan(WIN_PRIMARY[0], WIN_PRIMARY[1], color="orange", alpha=0.18)
    t_interior_w = float(first_interior["t_s"])
    ax1.axvline(
        t_interior_w,
        color="C3",
        ls=":",
        alpha=0.7,
        label=f"|w| peak (interior) @ {t_interior_w:.3f}s",
    )
    ax1.axvline(
        t_max_ddrift,
        color="C0",
        ls=":",
        alpha=0.7,
        label=f"argmax |d(delta)/dt| @ {t_max_ddrift:.3f}s (1.3-2)",
    )
    ax1.axvline(
        float(t_s[idx_d_15]),
        color="C0",
        ls="--",
        alpha=0.5,
        label=f"argmax |d(delta)/dt| @ {float(t_s[idx_d_15]):.3f}s (1-5)",
    )
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("|ω| (rad/s)")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(*WIN_PLOT)
    fig.savefig(OUT_PNG, dpi=140)
    plt.close(fig)

    # Markdown
    b13 = burst_block["window_1p3_2p0"]["x_signed_delta_drift_A_minus_C"]
    b13_a = burst_block["window_1p3_2p0"]["x_abs_drift_A"]
    b_whole = burst_block["whole_run"]["x_signed_delta_drift_A_minus_C"]
    b15 = burst_block["window_1p0_5p0"]["x_signed_delta_drift_A_minus_C"]

    def fmt_b(m: dict) -> str:
        if m["B"] is None:
            return "n/a"
        return (
            f"B={m['B']:.4f}, top3_share={m['top3_share']:.4f}, "
            f"bursty={m['bursty']}, max|dx|={m['max_abs_dx']:.6g}, "
            f"sum|dx|={m['sum_abs_dx']:.6g}"
        )

    dom_lines = []
    for d in b13["dominating_ticks"]:
        dom_lines.append(
            f"  - rank {d['rank']}: t∈[{d['t_start_s']:.3f},{d['t_end_s']:.3f}] s, "
            f"dx={d['dx']:+.6g}, share={d['share']:.3f}"
        )
    dom_txt = "\n".join(dom_lines) if dom_lines else "  - (none)"

    md = f"""# SLALOM A vs C — ω coincidence & drift-delta burstiness

**Scenario:** SLALOM seed 71 · A (`jcorrect`+`imuideal`) vs C (`jlegacy`+`imuideal`)  
**Artifacts:** `slalom_a_vs_c_omega_burstiness.json`, `fig_slalom_omega_vs_ddrift.png`  
**Scope:** coincidence / burstiness metrics only — no causal claim.

## CSV yaw_rate

- A all-zero: **{results['yaw_rate_csv']['A_all_zero']}**
- C all-zero: **{results['yaw_rate_csv']['C_all_zero']}**
- **Dead:** {yaw_rate_dead} → reconstructed `ω_truth` from `slalom_scenario.cpp`:
  - `kSlalomOmegaRadps = 2π / {TC04_SLALOM_PERIOD_S} = {K_SLALOM_OMEGA_RADPS:.8f}`
  - `kYawAmplitudeRad = {TC04_MAX_LATERAL_ACCEL_MPS2} / (v·ω) = {K_YAW_AMPLITUDE_RAD:.8f}`
  - `yaw_rate = A·ω·cos(ω·t)`
- Secondary: finite-difference of filter `yaw` for A and C.

## Burstiness thresholds

Rule: **bursty** if `top3_share > {THRESH_TOP3_SHARE}` OR `B > {THRESH_B}`  
(same spirit as GAP-3: `B = max|Δ| / Σ|Δ|` on per-tick deltas).

### Window 1.3–2.0 s (primary)

| Series | Metrics |
|--------|---------|
| `x = drift_A - drift_C` (signed) | {fmt_b(b13)} |
| `x = |drift_A|` | {fmt_b(b13_a)} |

Dominating ticks (`drift_A - drift_C`):
{dom_txt}

**Verdict 1.3–2.0 s (signed Δdrift):** {"BURSTY" if b13["bursty"] else "NOT bursty"}

### Context 1.0–5.0 s

| Series | Metrics |
|--------|---------|
| `drift_A - drift_C` | {fmt_b(b15)} |

### Whole-run comparison (avoid overclaim from short window)

| Series | Metrics |
|--------|---------|
| `drift_A - drift_C` (whole run) | {fmt_b(b_whole)} |

Whole-run B for |d(Δdrift)| path (per-tick |dx| of signed delta): **{b_whole['B']:.4f}** vs primary-window B **{b13['B']:.4f}**.

## Coincidence ω vs divergence

| Question | Result |
|----------|--------|
| At max `|d(Δdrift)/dt|` in 1.3–2.0 s | t = **{t_max_ddrift:.4f}** s; `|ω_truth|` = **{coincidence['at_max_abs_d_delta_dt_in_1p3_2p0']['abs_omega_truth']:.4f}** rad/s; `|Δdrift|` = {coincidence['at_max_abs_d_delta_dt_in_1p3_2p0']['abs_delta_drift_m']:.6g} m |
| First argmax `|ω_truth|` in 0–5 s | t = **{t_max_w:.4f}** s (tied also at ~2 s, ~4 s); `|ω|` = **{coincidence['at_max_abs_omega_truth_in_0_5']['abs_omega_truth']:.4f}** rad/s; `|Δdrift|` = {coincidence['at_max_abs_omega_truth_in_0_5']['abs_delta_drift_m']:.6g} m |
| At first interior `|ω|` peak | t = **{first_interior['t_s']:.4f}** s; `|ω|` = **{first_interior['abs_omega_truth']:.4f}** rad/s; `|Δdrift|` = {first_interior['abs_delta_drift_m']:.6g} m; `|d(Δdrift)/dt|` = {first_interior['abs_d_delta_dt']:.6g} m/s |
| Lag in 1–5 s (`t_div - t_ω`) | **{lag_s:+.4f}** s (argmax `|ω|` @ {t_s[idx_w_15]:.4f} s vs argmax `|dΔdrift/dt|` @ {t_s[idx_d_15]:.4f} s) |

Figure: `fig_slalom_omega_vs_ddrift.png` (0–5 s, band 1.3–2.0 s).

## Short factual summary

- CSV `yaw_rate`: **{"DEAD (all zeros)" if yaw_rate_dead else "populated"}**.
- Burstiness in 1.3–2.0 s on signed `(drift_A - drift_C)`: **{"bursty" if b13["bursty"] else "not bursty"}** (B={b13['B']:.4f}, top3_share={b13['top3_share']:.4f}); whole-run B={b_whole['B']:.4f}.
- ω peak vs divergence-rate peak (1–5 s lag): **{lag_s:+.4f} s** — coincidence metrics only.
"""
    OUT_MD.write_text(md, encoding="utf-8")
    OUT_JSON.write_text(json.dumps(results, indent=2), encoding="utf-8")

    print("=== SUMMARY ===")
    print(f"yaw_rate CSV dead: {yaw_rate_dead}")
    print(
        f"1.3-2.0s signed Delta-drift: "
        f"{'BURSTY' if b13['bursty'] else 'NOT bursty'} "
        f"(B={b13['B']:.4f}, top3_share={b13['top3_share']:.4f}); "
        f"whole-run B={b_whole['B']:.4f}"
    )
    print(
        f"1.3-2.0s |drift_A|: "
        f"{'BURSTY' if b13_a['bursty'] else 'NOT bursty'} "
        f"(B={b13_a['B']:.4f}, top3_share={b13_a['top3_share']:.4f})"
    )
    print(
        f"at max |d(Delta)/dt| in 1.3-2s (t={t_max_ddrift:.4f}s): "
        f"|omega_truth|={coincidence['at_max_abs_d_delta_dt_in_1p3_2p0']['abs_omega_truth']:.4f} rad/s"
    )
    print(
        f"at first |omega| argmax in 0-5s (t={t_max_w:.4f}s, tied peaks): "
        f"|Delta_drift|={coincidence['at_max_abs_omega_truth_in_0_5']['abs_delta_drift_m']:.6g} m"
    )
    print(
        f"at first interior |omega| peak (t={first_interior['t_s']:.4f}s): "
        f"|Delta_drift|={first_interior['abs_delta_drift_m']:.6g} m, "
        f"|dDelta/dt|={first_interior['abs_d_delta_dt']:.6g} m/s"
    )
    print(f"lag (t_div - t_omega) in 1-5s: {lag_s:+.4f} s")
    print(f"wrote {OUT_JSON}")
    print(f"wrote {OUT_MD}")
    print(f"wrote {OUT_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
