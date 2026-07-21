#!/usr/bin/env python3
"""Does filter vel_NED already dirty in [0.4, 1.69] before NHC explosion?

Hypothesis: attitude loop → f_va contaminates vel_NED silently; slalom turn
only reveals it as body lat/vert later.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"

V = 50.0 / 3.6
OMEGA = 2.0 * np.pi / 4.0
YAW_AMP = 3.0 / (V * OMEGA)
BASE = np.pi / 2.0

# chronology anchors
T_LATCH = 0.39
T_ONSET = 1.34
T_BREAK = 1.59
T_EXPLODE0 = 1.69
T_WIN0, T_WIN1 = 0.40, 1.69


def wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def truth_at(t: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    yaw = BASE + YAW_AMP * np.sin(OMEGA * t)
    vn = V * np.cos(yaw)
    ve = V * np.sin(yaw)
    vd = np.zeros_like(t)
    return yaw, vn, ve, vd


def arm_series(arm: str) -> pd.DataFrame:
    telem = pd.read_csv(ROOT / f"docs/benchmarks/slalom_pattbias_{arm}_s71_telemetry.csv")
    telem["t"] = telem["time_us"].astype(float) * 1e-6
    m = (telem["t"] >= T_WIN0) & (telem["t"] <= T_WIN1)
    d = telem.loc[m].copy()
    t = d["t"].to_numpy(float)
    yaw_t, vn_t, ve_t, vd_t = truth_at(t)
    vn_f = d["vel_x"].to_numpy(float)
    ve_f = d["vel_y"].to_numpy(float)
    vd_f = d["vel_z"].to_numpy(float)
    yaw_f = d["yaw"].to_numpy(float)

    # errors in NED
    evn = vn_f - vn_t
    eve = ve_f - ve_t
    evd = vd_f - vd_t
    e_horiz = np.hypot(evn, eve)

    # along / cross relative to truth heading
    along_f = vn_f * np.cos(yaw_t) + ve_f * np.sin(yaw_t)
    cross_f = -vn_f * np.sin(yaw_t) + ve_f * np.cos(yaw_t)
    along_err = along_f - V  # truth along = V
    cross_err = cross_f  # truth cross = 0

    dyaw = wrap(yaw_f - yaw_t)

    return pd.DataFrame(
        {
            "t": t,
            "yaw_truth": yaw_t,
            "yaw_filter": yaw_f,
            "dyaw_deg": np.rad2deg(dyaw),
            "vn_truth": vn_t,
            "ve_truth": ve_t,
            "vn_filter": vn_f,
            "ve_filter": ve_f,
            "vd_filter": vd_f,
            "err_vn": evn,
            "err_ve": eve,
            "err_vd": evd,
            "err_horiz": e_horiz,
            "along_filter": along_f,
            "cross_filter": cross_f,
            "along_err": along_err,
            "cross_err": cross_err,
            "bias_gz": d["bias_gz"].to_numpy(float),
            "drift_m": d["drift_m"].to_numpy(float),
        }
    )


def phase_stats(df: pd.DataFrame, t0: float, t1: float, half_open: bool = True) -> dict:
    if half_open:
        m = (df["t"] >= t0) & (df["t"] < t1)
    else:
        m = (df["t"] >= t0) & (df["t"] <= t1)
    g = df.loc[m]
    if g.empty:
        return {"n": 0}
    t = g["t"].to_numpy()
    ce = g["cross_err"].to_numpy()
    eh = g["err_horiz"].to_numpy()
    # growth rate of |cross| and horiz via linear fit on signed cross and |horiz|
    if len(t) >= 5:
        slope_cross = float(np.polyfit(t - t[0], ce, 1)[0])
        slope_abs_cross = float(np.polyfit(t - t[0], np.abs(ce), 1)[0])
        slope_horiz = float(np.polyfit(t - t[0], eh, 1)[0])
    else:
        slope_cross = slope_abs_cross = slope_horiz = float("nan")
    return {
        "n": int(len(g)),
        "t_span": [float(g["t"].min()), float(g["t"].max())],
        "cross_start": float(ce[0]),
        "cross_end": float(ce[-1]),
        "cross_delta": float(ce[-1] - ce[0]),
        "max_abs_cross": float(np.max(np.abs(ce))),
        "mean_abs_cross": float(np.mean(np.abs(ce))),
        "horiz_start": float(eh[0]),
        "horiz_end": float(eh[-1]),
        "horiz_delta": float(eh[-1] - eh[0]),
        "max_err_horiz": float(np.max(eh)),
        "slope_cross_mps2": slope_cross,
        "slope_abs_cross_mps2": slope_abs_cross,
        "slope_horiz_mps2": slope_horiz,
        "dyaw_deg_end": float(g["dyaw_deg"].iloc[-1]),
        "max_abs_dyaw_deg": float(np.max(np.abs(g["dyaw_deg"]))),
    }


def main() -> None:
    report: dict = {"window_s": [T_WIN0, T_WIN1], "arms": {}, "verdict": {}}
    series = {}
    phases = {
        "P_early_loop": (0.40, 1.34, True),  # latch/post early → onset
        "P_rise_pre_break": (1.34, 1.59, True),
        "P_break_to_explode": (1.59, 1.69, False),
        "P_full_pre_explode": (0.40, 1.69, False),
    }

    for arm in ("ctrl", "latch"):
        df = arm_series(arm)
        series[arm] = df
        df.to_csv(OUT / f"vel_ned_pre_explosion_{arm}.csv", index=False)
        arm_rep = {"phases": {}}
        for name, (t0, t1, ho) in phases.items():
            arm_rep["phases"][name] = phase_stats(df, t0, t1, ho)
        report["arms"][arm] = arm_rep

    # continuous growth? latch full window
    L = report["arms"]["latch"]["phases"]["P_full_pre_explode"]
    C = report["arms"]["ctrl"]["phases"]["P_full_pre_explode"]
    L_early = report["arms"]["latch"]["phases"]["P_early_loop"]
    L_rise = report["arms"]["latch"]["phases"]["P_rise_pre_break"]
    L_late = report["arms"]["latch"]["phases"]["P_break_to_explode"]

    grows = L["horiz_delta"] > 0.05 or L["max_abs_cross"] > 0.3
    continuous = (
        L_early["slope_abs_cross_mps2"] > 0
        or L_rise["slope_abs_cross_mps2"] > 0
        or L["slope_horiz_mps2"] > 0.05
    )
    # already dirty before explode: |cross| at 1.69 comparable to explosion window (~1.6)
    dirty_at_gate = abs(L["cross_end"]) > 0.5 or L["horiz_end"] > 0.5

    if grows and dirty_at_gate and continuous:
        label = "VEL_NED_DIRTY_BEFORE_NHC_EXPLOSION"
        reading = (
            "Filter vel_NED error (esp. cross-track vs truth heading) already grows "
            "continuously in [0.4, 1.69] and is material by the start of the NHC "
            "explosion window. The turn reveals pre-existing vel contamination; "
            "intervention point is early (attitude / f_va path), not NHC @ 1.7s."
        )
    elif dirty_at_gate:
        label = "VEL_NED_DIRTY_AT_GATE_GROWTH_UNCLEAR"
        reading = (
            "vel_NED is dirty by t=1.69 but growth pattern needs care; see phase slopes."
        )
    else:
        label = "VEL_NED_NOT_PREDIRTY"
        reading = (
            "vel_NED error does not clearly pre-accumulate before 1.69 — would weaken "
            "the silent f_va-pollution story."
        )

    report["verdict"] = {
        "label": label,
        "reading": reading,
        "latch_full": L,
        "ctrl_full": C,
        "latch_phases_slopes_abs_cross": {
            "early": L_early["slope_abs_cross_mps2"],
            "rise": L_rise["slope_abs_cross_mps2"],
            "break_to_explode": L_late["slope_abs_cross_mps2"],
        },
        "implication": (
            "If VEL_NED_DIRTY_BEFORE_NHC_EXPLOSION: full chain is "
            "NHC Jacobian sign → attitude loop → f_va pollutes vel_NED silently → "
            "heading turn reveals as v_lat → NHC innov explodes. Intervene early on "
            "attitude/f_va, not at NHC explosion."
        ),
    }

    # figure
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for arm, color in ("ctrl", "C0"), ("latch", "C3"):
        df = series[arm]
        axes[0].plot(df["t"], df["err_vn"], color=color, lw=0.9, label=f"{arm} e_vn")
        axes[0].plot(df["t"], df["err_ve"], color=color, lw=0.9, ls="--", label=f"{arm} e_ve")
        axes[1].plot(df["t"], df["cross_err"], color=color, lw=1.2, label=arm)
        axes[2].plot(df["t"], df["err_horiz"], color=color, lw=1.2, label=arm)
        axes[3].plot(df["t"], df["dyaw_deg"], color=color, lw=1.0, label=arm)
    for ax in axes:
        for x, lab in (
            (T_LATCH, "latch"),
            (T_ONSET, "onset"),
            (T_BREAK, "break"),
            (T_EXPLODE0, "explode"),
        ):
            ax.axvline(x, color="gray", ls=":", alpha=0.5)
    axes[0].set_ylabel("e_vn / e_ve")
    axes[0].legend(fontsize=7, ncol=2)
    axes[0].set_title("vel_NED filter−truth before NHC explosion [0.4 → 1.69]")
    axes[1].set_ylabel("cross-track err")
    axes[1].legend(fontsize=8)
    axes[1].axhline(0, color="gray", lw=0.5)
    axes[2].set_ylabel("|e_horiz|")
    axes[2].legend(fontsize=8)
    axes[3].set_ylabel("Δyaw [deg]")
    axes[3].set_xlabel("t [s]")
    axes[3].legend(fontsize=8)
    fig.tight_layout()
    fig_path = OUT / "fig_vel_ned_pre_explosion.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "vel_ned_pre_explosion.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )

    # md
    lines = [
        "# vel_NED dirty before NHC explosion? [0.4 → 1.69]s",
        "",
        f"**Verdict:** `{label}`",
        "",
        reading,
        "",
        "## Full window summary",
        "",
        "| Arm | cross start→end | Δcross | max|cross| | |e_h| start→end | slope |e_h| |",
        "|-----|-----------------|--------|------------|-----------------|-------------|",
    ]
    for arm in ("ctrl", "latch"):
        p = report["arms"][arm]["phases"]["P_full_pre_explode"]
        lines.append(
            f"| {arm} | {p['cross_start']:+.3f}→{p['cross_end']:+.3f} | "
            f"{p['cross_delta']:+.3f} | {p['max_abs_cross']:.3f} | "
            f"{p['horiz_start']:.3f}→{p['horiz_end']:.3f} | {p['slope_horiz_mps2']:+.3f} |"
        )
    lines += [
        "",
        "## Latch phases",
        "",
        "| Phase | cross start→end | slope |cross| | max|cross| | |e_h| end |",
        "|-------|-----------------|---------------|------------|----------|",
    ]
    for name in ("P_early_loop", "P_rise_pre_break", "P_break_to_explode"):
        p = report["arms"]["latch"]["phases"][name]
        lines.append(
            f"| {name} | {p['cross_start']:+.3f}→{p['cross_end']:+.3f} | "
            f"{p['slope_abs_cross_mps2']:+.4f} | {p['max_abs_cross']:.3f} | "
            f"{p['horiz_end']:.3f} |"
        )
    lines += [
        "",
        "## Implication",
        "",
        report["verdict"]["implication"],
        "",
        f"Figure: `{fig_path.name}`",
        "",
    ]
    (OUT / "vel_ned_pre_explosion.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report["verdict"], indent=2))


if __name__ == "__main__":
    main()
