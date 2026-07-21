#!/usr/bin/env python3
"""Overlay innov composition break (ctrl vs latch) against ||omega||.

Cheap check before K_bias decompose: is the COMPOSITION_BREAK colocated with
the first interior slalom yaw-rate peak (~2 s), or does it precede it?
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"
BENCH = ROOT / "docs/benchmarks"

# SLALOM truth kinematics (same as burstiness autopsy)
OMEGA = 2.0 * np.pi / 4.0  # rad/s
YAW_AMP = 0.13750987  # rad


def omega_truth(t: np.ndarray) -> np.ndarray:
    return YAW_AMP * OMEGA * np.cos(OMEGA * t)


def innov_vec(df: pd.DataFrame) -> np.ndarray:
    """Prefer explicit innov_y/z; else reconstruct from components if present."""
    cols = set(df.columns)
    if {"innov_y_mps", "innov_z_mps"} <= cols:
        return np.column_stack(
            [df["innov_y_mps"].to_numpy(float), df["innov_z_mps"].to_numpy(float)]
        )
    if {"y_y", "y_z"} <= cols:
        return np.column_stack([df["y_y"].to_numpy(float), df["y_z"].to_numpy(float)])
    # residual lateral / vertical naming variants
    for a, b in [
        ("innov_lat_mps", "innov_vert_mps"),
        ("nhc_innov_y", "nhc_innov_z"),
    ]:
        if {a, b} <= cols:
            return np.column_stack([df[a].to_numpy(float), df[b].to_numpy(float)])
    raise KeyError(f"No innov y/z columns in {sorted(cols)}")


def cosine_rows(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    na = np.linalg.norm(a, axis=1)
    nb = np.linalg.norm(b, axis=1)
    den = na * nb
    out = np.full(len(a), np.nan)
    m = den > 1e-15
    out[m] = np.sum(a[m] * b[m], axis=1) / den[m]
    return out


def first_crossing(t: np.ndarray, cos: np.ndarray, thr: float, sustain_n: int) -> float | None:
    """First time cos drops below thr and stays below for sustain_n samples."""
    below = cos < thr
    for i in range(len(t) - sustain_n + 1):
        if below[i : i + sustain_n].all():
            return float(t[i])
    return None


def main() -> None:
    ctrl_a = pd.read_csv(OUT / "ctrl_nhc_block_audit.csv")
    latch_a = pd.read_csv(OUT / "latch_nhc_block_audit.csv")
    # align on timestamp
    m = ctrl_a.merge(latch_a, on="timestamp_s", suffixes=("_c", "_l"))
    t = m["timestamp_s"].to_numpy(float)

    # build innov from whichever naming exists (post-merge columns)
    def pick(suffix: str) -> np.ndarray:
        candidates = [
            (f"innov_y_mps_{suffix}", f"innov_z_mps_{suffix}"),
            (f"y_y_{suffix}", f"y_z_{suffix}"),
            (f"innov_lat_mps_{suffix}", f"innov_vert_mps_{suffix}"),
        ]
        for a, b in candidates:
            if a in m.columns and b in m.columns:
                return np.column_stack([m[a].to_numpy(float), m[b].to_numpy(float)])
        # try without suffix if merge didn't rename (identical names impossible)
        raise KeyError(
            "innov columns after merge: "
            + ", ".join(c for c in m.columns if "innov" in c.lower() or c.startswith("y_"))
        )

    yc = pick("c")
    yl = pick("l")
    cos = cosine_rows(yc, yl)
    nc = np.linalg.norm(yc, axis=1)
    nl = np.linalg.norm(yl, axis=1)

    # post-latch window
    post = (t >= 0.39) & (t <= 2.5)
    tp, cosp, ncp, nlp = t[post], cos[post], nc[post], nl[post]

    # fine break detectors
    t_cos09 = first_crossing(tp, cosp, 0.9, sustain_n=3)
    t_cos05 = first_crossing(tp, cosp, 0.5, sustain_n=3)
    t_cos00 = first_crossing(tp, cosp, 0.0, sustain_n=3)

    # rolling median cos (0.1 s ≈ 10 samples @100 Hz; audit may be NHC rate)
    # use ~5-sample median for robustness
    win = 5
    roll = pd.Series(cosp).rolling(win, center=True, min_periods=1).median().to_numpy()
    t_roll05 = first_crossing(tp, roll, 0.5, sustain_n=3)
    t_roll00 = first_crossing(tp, roll, 0.0, sustain_n=3)

    # largest drop in cos (1st difference)
    dcos = np.diff(cosp)
    i_drop = int(np.nanargmin(dcos)) if len(dcos) else 0
    t_max_drop = float(tp[i_drop + 1]) if len(tp) > 1 else None
    max_drop = float(dcos[i_drop]) if len(dcos) else None

    # telemetry omega
    telem_c = pd.read_csv(BENCH / "slalom_pattbias_ctrl_s71_telemetry.csv")
    telem_l = pd.read_csv(BENCH / "slalom_pattbias_latch_s71_telemetry.csv")
    for te in (telem_c, telem_l):
        te["t"] = te["time_us"] * 1e-6

    def abs_omega(te: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
        tt = te["t"].to_numpy(float)
        if "yaw_rate" in te.columns and np.nanmax(np.abs(te["yaw_rate"])) > 1e-9:
            w = te["yaw_rate"].to_numpy(float)
        else:
            w = omega_truth(tt)
        return tt, np.abs(w)

    tt_c, aw_c = abs_omega(telem_c)
    tt_l, aw_l = abs_omega(telem_l)
    # truth for reference
    aw_truth = np.abs(omega_truth(tt_c))

    # omega peaks in [0.39, 2.5]
    def peaks(tt: np.ndarray, aw: np.ndarray, t0: float, t1: float) -> dict:
        msk = (tt >= t0) & (tt <= t1)
        if not msk.any():
            return {}
        i = int(np.argmax(aw[msk]))
        idx = np.where(msk)[0][i]
        # first interior peak near 2s: local max of truth cos phase
        # also rising onset: when |w| exceeds 50% of peak toward 2s
        peak_t = float(tt[idx])
        peak_v = float(aw[idx])
        # onset: first t in [1.0, peak] where aw >= 0.5 * peak_at_2
        # use truth peak at 2.0
        t2 = 2.0
        i2 = int(np.argmin(np.abs(tt - t2)))
        w2 = float(aw[i2])
        m_on = (tt >= 1.0) & (tt <= t2) & (aw >= 0.5 * w2)
        t_onset = float(tt[m_on][0]) if m_on.any() else None
        return {
            "argmax_abs_omega_t": peak_t,
            "argmax_abs_omega": peak_v,
            "abs_omega_at_2.0": w2,
            "onset_half_peak_toward_2s_t": t_onset,
        }

    om_stats = peaks(tt_c, aw_c if np.nanmax(aw_c) > 1e-9 else aw_truth, 0.39, 2.5)

    # |omega| at each break candidate
    def omega_at(t_break: float | None) -> dict | None:
        if t_break is None:
            return None
        i = int(np.argmin(np.abs(tt_c - t_break)))
        return {
            "t": t_break,
            "abs_omega_meas_or_truth": float(
                aw_c[i] if np.nanmax(aw_c) > 1e-9 else aw_truth[i]
            ),
            "abs_omega_truth": float(aw_truth[i]),
            "frac_of_peak_at_2s": float(
                (aw_c[i] if np.nanmax(aw_c) > 1e-9 else aw_truth[i])
                / om_stats.get("abs_omega_at_2.0", 1.0)
            ),
            "dt_to_omega_peak_2s": float(2.0 - t_break),
        }

    # phase split conditioned on omega (not just clock)
    # high-omega: |w| >= 0.5 * |w|(2s); low: below, within [0.39, 2.5]
    w2 = om_stats["abs_omega_at_2.0"]
    # interpolate omega onto audit timestamps
    aw_on_t = np.interp(tp, tt_c, aw_c if np.nanmax(aw_c) > 1e-9 else aw_truth)
    high = aw_on_t >= 0.5 * w2
    low = ~high

    def phase_summary(mask: np.ndarray, label: str) -> dict:
        if not mask.any():
            return {"label": label, "n": 0}
        return {
            "label": label,
            "n": int(mask.sum()),
            "t_span": [float(tp[mask].min()), float(tp[mask].max())],
            "median_cos": float(np.nanmedian(cosp[mask])),
            "mean_cos": float(np.nanmean(cosp[mask])),
            "frac_cos_lt_0": float(np.nanmean(cosp[mask] < 0)),
            "mean_innov_c": float(np.nanmean(ncp[mask])),
            "mean_innov_l": float(np.nanmean(nlp[mask])),
            "mean_norm_ratio": float(np.nanmean(nlp[mask] / np.maximum(ncp[mask], 1e-15))),
        }

    # also clock bins for continuity
    early_clock = (tp >= 0.39) & (tp < 1.5)
    late_clock = (tp >= 1.5) & (tp <= 2.0)

    # bias escape on high vs low omega (from audit dx)
    dx_c = m.loc[post, "dx_bias_gz_c"].to_numpy(float) if "dx_bias_gz_c" in m.columns else None
    dx_l = m.loc[post, "dx_bias_gz_l"].to_numpy(float) if "dx_bias_gz_l" in m.columns else None
    if dx_c is None:
        # try without merge suffix
        if "dx_bias_gz" in ctrl_a.columns:
            dx_map_c = dict(zip(ctrl_a["timestamp_s"], ctrl_a["dx_bias_gz"]))
            dx_map_l = dict(zip(latch_a["timestamp_s"], latch_a["dx_bias_gz"]))
            dx_c = np.array([dx_map_c[x] for x in tp])
            dx_l = np.array([dx_map_l[x] for x in tp])

    omega_cond = {
        "low_omega": phase_summary(low & (tp <= 2.5), "low_|w|<0.5*peak2"),
        "high_omega": phase_summary(high & (tp <= 2.5), "high_|w|>=0.5*peak2"),
        "clock_early": phase_summary(early_clock, "clock_[0.39,1.5)"),
        "clock_late": phase_summary(late_clock, "clock_[1.5,2.0]"),
    }
    if dx_c is not None:
        for key, mask in [
            ("low_omega", low),
            ("high_omega", high),
            ("clock_early", early_clock),
            ("clock_late", late_clock),
        ]:
            omega_cond[key]["sum_dx_bias_gz_c"] = float(dx_c[mask].sum())
            omega_cond[key]["sum_dx_bias_gz_l"] = float(dx_l[mask].sum())

    # prior chronology anchors
    prior = {
        "interior_omega_peak_t": 2.0,
        "xcorr_tau_peak_focus_0_8s": 1.93,
        "xcorr_tau_peak_analysis_0_20s": 1.96,
        "max_abs_ddrift_dt_in_1.3_2.0": 1.61,
        "note": "From slalom_a_vs_c_omega_burstiness / omega_xcorr (seed 71).",
    }

    breaks = {
        "first_cos_lt_0.9_sustain3": omega_at(t_cos09),
        "first_cos_lt_0.5_sustain3": omega_at(t_cos05),
        "first_cos_lt_0.0_sustain3": omega_at(t_cos00),
        "first_roll5_median_cos_lt_0.5_sustain3": omega_at(t_roll05),
        "first_roll5_median_cos_lt_0.0_sustain3": omega_at(t_roll00),
        "max_single_step_cos_drop": {
            "t": t_max_drop,
            "dcos": max_drop,
            **(omega_at(t_max_drop) or {}),
        },
    }

    # primary break time: first sustained cos<0.5 (clear leave from parallel)
    t_break = t_roll05 if t_roll05 is not None else t_cos05
    coincidence = {
        "primary_break_def": "first rolling-median(5) cos<0.5 sustained 3 samples",
        "t_break": t_break,
        "dt_break_minus_1.5_bin": None if t_break is None else float(t_break - 1.5),
        "dt_omega_peak_minus_break": None if t_break is None else float(2.0 - t_break),
        "dt_break_minus_ddrift_peak_1.61": None if t_break is None else float(t_break - 1.61),
        "break_at_frac_of_omega_peak": None
        if t_break is None
        else omega_at(t_break)["frac_of_peak_at_2s"],
        "omega_onset_half_peak_t": om_stats.get("onset_half_peak_toward_2s_t"),
        "dt_break_minus_omega_onset": (
            None
            if t_break is None or om_stats.get("onset_half_peak_toward_2s_t") is None
            else float(t_break - om_stats["onset_half_peak_toward_2s_t"])
        ),
    }

    # verdict
    if t_break is None:
        label = "NO_CLEAR_BREAK"
        reading = "Could not find sustained cos drop; check innov columns."
    else:
        frac = coincidence["break_at_frac_of_omega_peak"]
        dt_peak = coincidence["dt_omega_peak_minus_break"]
        # colocated with rising limb / peak if break within ~0.3s of onset or peak,
        # and |w| already material (>=0.4 of peak)
        near_peak = dt_peak is not None and 0.0 <= dt_peak <= 0.6
        on_rising = frac is not None and frac >= 0.4
        high_vs_low = omega_cond["high_omega"]["median_cos"] < omega_cond["low_omega"]["median_cos"] - 0.3
        if near_peak and on_rising and high_vs_low:
            label = "BREAK_COLOCATED_WITH_ANGULAR_STIMULUS"
            reading = (
                "Composition break sits on the rising limb / approach of the first interior "
                "||ω|| peak (~2 s), not at an arbitrary clock cut. High-ω ticks show the "
                "composition break; low-ω post-latch do not. Condition K_bias decompose on "
                "ω-stimulus, not only clock [1.5,2]."
            )
        elif high_vs_low and on_rising:
            label = "BREAK_ON_RISING_OMEGA_PARTIAL"
            reading = (
                "Break precedes the ||ω|| peak but occurs once |ω| is already material; "
                "ω-conditioned split still separates composition better than chance. "
                "Prefer ω-gated K_bias decompose alongside clock window."
            )
        else:
            label = "BREAK_NOT_EXPLAINED_BY_OMEGA_PEAK_ALONE"
            reading = (
                "Fine break time does not sit cleanly on the angular stimulus peak/onset. "
                "Do not force the 'frozen Z + new turn → bias' story; keep clock window "
                "and still run K_bias decompose as empirical next step."
            )

    report = {
        "omega_stats_0.39_2.5": om_stats,
        "prior_anchors": prior,
        "break_candidates": breaks,
        "coincidence": coincidence,
        "omega_conditioned_phases": omega_cond,
        "verdict": {"label": label, "reading": reading},
        "figure": str(OUT / "fig_innov_break_vs_omega.png"),
    }

    # figure
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax0, ax1, ax2 = axes
    ax0.plot(tt_c, aw_truth, "k-", lw=1.2, label="|ω|_truth")
    if np.nanmax(aw_c) > 1e-9:
        ax0.plot(tt_c, aw_c, "C0-", alpha=0.7, lw=0.8, label="|ω|_telem ctrl")
    ax0.axvline(2.0, color="k", ls="--", alpha=0.5, label="ω peak t=2.0")
    if om_stats.get("onset_half_peak_toward_2s_t") is not None:
        ax0.axvline(
            om_stats["onset_half_peak_toward_2s_t"],
            color="C1",
            ls=":",
            label=f"onset 0.5·peak @{om_stats['onset_half_peak_toward_2s_t']:.3f}",
        )
    if t_break is not None:
        ax0.axvline(t_break, color="C3", lw=1.5, label=f"innov break @{t_break:.3f}")
    ax0.axvline(1.5, color="gray", ls="--", alpha=0.4, label="clock bin 1.5")
    ax0.set_ylabel("|ω| rad/s")
    ax0.legend(loc="upper right", fontsize=8)
    ax0.set_xlim(0.3, 2.5)
    ax0.set_title("Innov composition break vs ||ω|| (ctrl vs latch, seed 71)")

    ax1.plot(tp, cosp, "C0-", lw=0.8, alpha=0.5, label="cos(y_c,y_l)")
    ax1.plot(tp, roll, "C0-", lw=1.5, label="roll median-5")
    ax1.axhline(0.9, color="gray", ls=":", alpha=0.5)
    ax1.axhline(0.0, color="gray", ls="-", alpha=0.3)
    ax1.axvline(1.5, color="gray", ls="--", alpha=0.4)
    if t_break is not None:
        ax1.axvline(t_break, color="C3", lw=1.5)
    ax1.set_ylabel("cos innov")
    ax1.set_ylim(-1.05, 1.05)
    ax1.legend(loc="lower left", fontsize=8)

    ax2.plot(tp, ncp, "C0-", lw=1, label="‖y‖ ctrl")
    ax2.plot(tp, nlp, "C3-", lw=1, label="‖y‖ latch")
    ax2.axvline(1.5, color="gray", ls="--", alpha=0.4)
    ax2.axvline(2.0, color="k", ls="--", alpha=0.5)
    if t_break is not None:
        ax2.axvline(t_break, color="C3", lw=1.5)
    ax2.set_ylabel("‖innov‖")
    ax2.set_xlabel("t [s]")
    ax2.legend(loc="upper left", fontsize=8)

    fig.tight_layout()
    fig_path = OUT / "fig_innov_break_vs_omega.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)

    out_json = OUT / "innov_break_vs_omega.json"
    out_json.write_text(json.dumps(report, indent=2), encoding="utf-8")

    # markdown brief
    md = OUT / "innov_break_vs_omega.md"
    md.write_text(
        "\n".join(
            [
                "# Innov composition break vs ‖ω‖",
                "",
                f"**Verdict:** `{label}`",
                "",
                reading,
                "",
                "## Fine break time",
                "",
                f"- Primary: t = **{t_break}** s "
                f"(rolling median-5 cos < 0.5, sustain 3)",
                f"- Clock bin used earlier: 1.5 s "
                f"(Δ = {coincidence['dt_break_minus_1.5_bin']})",
                f"- ‖ω‖ peak (interior): 2.0 s "
                f"(Δ_peak−break = {coincidence['dt_omega_peak_minus_break']})",
                f"- Onset |ω|≥0.5·peak→2s: "
                f"{om_stats.get('onset_half_peak_toward_2s_t')}",
                f"- |ω|/peak at break: "
                f"{coincidence['break_at_frac_of_omega_peak']}",
                f"- Prior max|dΔdrift/dt| @ 1.61 s: "
                f"Δ(break−1.61) = {coincidence['dt_break_minus_ddrift_peak_1.61']}",
                "",
                "## ω-conditioned vs clock",
                "",
                f"| Split | n | median cos | mean ‖y‖_L / ‖y‖_C | Σdx_bias_gz latch |",
                f"|-------|---|------------|---------------------|-------------------|",
                *[
                    (
                        f"| {omega_cond[k]['label']} | {omega_cond[k]['n']} | "
                        f"{omega_cond[k].get('median_cos', float('nan')):.3f} | "
                        f"{omega_cond[k].get('mean_norm_ratio', float('nan')):.2f} | "
                        f"{omega_cond[k].get('sum_dx_bias_gz_l', float('nan')):+.4f} |"
                    )
                    for k in ("low_omega", "high_omega", "clock_early", "clock_late")
                ],
                "",
                f"Figure: `{fig_path.name}`",
                f"JSON: `{out_json.name}`",
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(json.dumps(report["verdict"], indent=2))
    print("t_break", t_break)
    print("wrote", out_json)
    print("wrote", fig_path)


if __name__ == "__main__":
    main()
