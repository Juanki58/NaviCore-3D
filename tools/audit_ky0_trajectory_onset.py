#!/usr/bin/env python3
"""K_y0 / P_att trajectory from latch fire → 1.54s — gradual vs jump?

Hypothesis: K_y0 sign split at ~1.55 is the late symptom of P divergence
accumulating via Joseph under dx_att_z≡0 since latch fire (~0.39s), same
family as att→bias_gz escape (cut one channel → cov dynamics shift).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"
T_END = 1.54
# latch fire from prior HATT_C runs (~cand1); refine from data
T_FIRE_HINT = 0.39

NEED = [
    "k_att_y0",
    "k_att_y1",
    "k_att_z0",
    "k_att_z1",
    "P_pre_att_yy",
    "P_pre_att_xx",
    "P_pre_att_zz",
    "P_pre_att_xz",
    "P_pre_att_yz",
    "P_pre_att_xy",
    "s_yy",
    "s_yz",
    "s_zz",
    "s_inv_yy",
    "s_inv_yz",
    "s_inv_zz",
    "dx_att_z_rad",
    "dx_att_z_raw",
    "dx_att_y_rad",
    "innov_y_mps",
    "innov_z_mps",
]


def load(arm: str) -> pd.DataFrame:
    d = pd.read_csv(OUT / f"{arm}_nhc_block_audit.csv")
    miss = [c for c in NEED if c not in d.columns]
    if miss:
        raise KeyError(f"{arm} missing {miss} — rebuild + rerun audits")
    return d


def find_latch_fire(latch: pd.DataFrame) -> float:
    """First tick where applied dx_z≈0 but raw≠0 (λ active), or hint."""
    z = latch["dx_att_z_rad"].to_numpy(float)
    raw = latch["dx_att_z_raw"].to_numpy(float)
    t = latch["timestamp_s"].to_numpy(float)
    for i in range(len(t)):
        if abs(z[i]) < 1e-15 and abs(raw[i]) > 1e-9:
            return float(t[i])
    return T_FIRE_HINT


def rel_diff(a, b):
    den = np.maximum(np.maximum(np.abs(a), np.abs(b)), 1e-30)
    return np.abs(a - b) / den


def main() -> None:
    C = load("ctrl")
    L = load("latch")
    t_fire = find_latch_fire(L)

    # align on common timestamps in [fire, T_END]
    def window(df):
        m = (df["timestamp_s"] >= t_fire - 0.02) & (df["timestamp_s"] <= T_END + 0.005)
        return df.loc[m].copy().reset_index(drop=True)

    Cw, Lw = window(C), window(L)
    # inner join on rounded t
    Cw["t_r"] = Cw["timestamp_s"].round(6)
    Lw["t_r"] = Lw["timestamp_s"].round(6)
    m = Cw.merge(Lw, on="t_r", suffixes=("_c", "_l"))
    if len(m) < 10:
        raise RuntimeError(f"too few aligned ticks: {len(m)}")

    t = m["t_r"].to_numpy(float)
    ky0_c = m["k_att_y0_c"].to_numpy(float)
    ky0_l = m["k_att_y0_l"].to_numpy(float)
    ky1_c = m["k_att_y1_c"].to_numpy(float)
    ky1_l = m["k_att_y1_l"].to_numpy(float)
    pyy_c = m["P_pre_att_yy_c"].to_numpy(float)
    pyy_l = m["P_pre_att_yy_l"].to_numpy(float)
    pzz_c = m["P_pre_att_zz_c"].to_numpy(float)
    pzz_l = m["P_pre_att_zz_l"].to_numpy(float)
    pyz_c = m["P_pre_att_yz_c"].to_numpy(float)
    pyz_l = m["P_pre_att_yz_l"].to_numpy(float)
    pxz_c = m["P_pre_att_xz_c"].to_numpy(float)
    pxz_l = m["P_pre_att_xz_l"].to_numpy(float)
    pxy_c = m["P_pre_att_xy_c"].to_numpy(float)
    pxy_l = m["P_pre_att_xy_l"].to_numpy(float)

    # cumulative |dx_z| applied (ctrl) vs withheld (latch raw)
    cum_dxz_c = np.cumsum(np.abs(m["dx_att_z_rad_c"].to_numpy(float)))
    cum_dxz_raw_l = np.cumsum(np.abs(m["dx_att_z_raw_l"].to_numpy(float)))
    cum_withheld = np.cumsum(np.abs(m["dx_att_z_raw_l"].to_numpy(float)))  # latch never applies

    rd_ky0 = rel_diff(ky0_c, ky0_l)
    rd_pyy = rel_diff(pyy_c, pyy_l)
    rd_pzz = rel_diff(pzz_c, pzz_l)
    d_ky0 = ky0_l - ky0_c
    d_pyy = pyy_l - pyy_c

    # Sign: when do arms disagree on sign(K_y0)?
    sign_disagree = (np.sign(ky0_c) * np.sign(ky0_l)) < 0
    # ignore near-zero
    sign_disagree &= (np.abs(ky0_c) > 1e-6) | (np.abs(ky0_l) > 1e-6)
    first_sign_disagree = float(t[np.argmax(sign_disagree)]) if sign_disagree.any() else None
    # ctrl crosses zero
    ctrl_zero_cross = None
    for i in range(1, len(t)):
        if ky0_c[i - 1] * ky0_c[i] < 0:
            ctrl_zero_cross = float(t[i])
            break
    latch_zero_cross = None
    for i in range(1, len(t)):
        if ky0_l[i - 1] * ky0_l[i] < 0:
            latch_zero_cross = float(t[i])
            break

    # Gradual vs jump: max single-tick |Δ(d_ky0)| / total |d_ky0| span
    dd = np.diff(d_ky0)
    span = max(np.max(np.abs(d_ky0)) - np.min(np.abs(d_ky0)), np.ptp(d_ky0), 1e-30)
    # use range of d_ky0
    d_range = float(np.ptp(d_ky0)) if len(d_ky0) else 0.0
    max_step = float(np.max(np.abs(dd))) if len(dd) else 0.0
    max_step_frac = max_step / max(d_range, 1e-30)
    i_max_step = int(np.argmax(np.abs(dd))) + 1 if len(dd) else 0

    # When does relΔ K_y0 first exceed 10%, 50%?
    def first_above(series, thr):
        hit = np.where(series >= thr)[0]
        return float(t[hit[0]]) if len(hit) else None

    # Early vs late growth of |d_ky0|
    mid = 0.5 * (t_fire + T_END)
    early = t < mid
    late = ~early
    growth_early = float(np.ptp(d_ky0[early])) if early.any() else 0.0
    growth_late = float(np.ptp(d_ky0[late])) if late.any() else 0.0

    # Correlation |d_ky0| vs cum withheld Z
    abs_dky0 = np.abs(d_ky0)
    if np.std(abs_dky0) > 1e-15 and np.std(cum_withheld) > 1e-15:
        corr_withheld = float(np.corrcoef(abs_dky0, cum_withheld)[0, 1])
    else:
        corr_withheld = float("nan")
    if np.std(abs_dky0) > 1e-15 and np.std(np.abs(d_pyy)) > 1e-15:
        corr_dpyy = float(np.corrcoef(abs_dky0, np.abs(d_pyy))[0, 1])
    else:
        corr_dpyy = float("nan")

    # Verdict
    if max_step_frac < 0.25 and first_above(rd_ky0, 0.10) is not None:
        t10 = first_above(rd_ky0, 0.10)
        if t10 is not None and t10 < T_END - 0.3:
            label = "KY0_GRADUAL_P_DIVERGENCE"
            reading = (
                "K_y0 latch−ctrl diverges gradually from early after latch fire; "
                "the late sign split is the symptom, not a discrete jump. Consistent "
                "with Joseph/P accumulation under withheld dx_att_z (same family as "
                "channel-cut → cov dynamics shift)."
            )
        else:
            label = "KY0_GRADUAL_LATE_ONSET"
            reading = (
                "No single-tick cliff in ΔK_y0, but material divergence concentrates "
                "later in the window — still accumulation, onset delayed."
            )
    elif max_step_frac >= 0.4:
        label = "KY0_JUMP_DOMINATED"
        reading = (
            f"Largest single-tick step in ΔK_y0 is {max_step_frac:.0%} of the "
            f"range (@t={t[i_max_step]:.3f}) — more jump-like than the silent "
            "accumulation pattern."
        )
    else:
        label = "KY0_MIXED_TRAJECTORY"
        reading = "Trajectory neither cleanly gradual nor jump-dominated."

    next_step = (
        "If GRADUAL: the design target is early P/att dynamics under open Z, not "
        "the 1.55 sign flip. Map which P terms (yy / cross / zz) track ΔK_y0."
    )

    ticks = pd.DataFrame(
        {
            "t": t,
            "ky0_c": ky0_c,
            "ky0_l": ky0_l,
            "d_ky0": d_ky0,
            "rd_ky0": rd_ky0,
            "ky1_c": ky1_c,
            "ky1_l": ky1_l,
            "pyy_c": pyy_c,
            "pyy_l": pyy_l,
            "d_pyy": d_pyy,
            "rd_pyy": rd_pyy,
            "pzz_c": pzz_c,
            "pzz_l": pzz_l,
            "rd_pzz": rd_pzz,
            "pyz_c": pyz_c,
            "pyz_l": pyz_l,
            "pxz_c": pxz_c,
            "pxz_l": pxz_l,
            "cum_dxz_applied_ctrl": cum_dxz_c,
            "cum_dxz_raw_latch": cum_dxz_raw_l,
            "sign_disagree": sign_disagree.astype(int),
        }
    )
    ticks.to_csv(OUT / "ky0_trajectory_onset_ticks.csv", index=False)

    # downsample table for md: every ~0.05s + fire + end + sign events
    targets = list(np.arange(t_fire, T_END + 1e-9, 0.05))
    targets += [t_fire, T_END, 1.34, 1.54]
    if first_sign_disagree:
        targets.append(first_sign_disagree)
    if ctrl_zero_cross:
        targets.append(ctrl_zero_cross)
    targets = sorted(set(round(x, 3) for x in targets))
    rows_md = []
    for tt in targets:
        i = int(np.argmin(np.abs(t - tt)))
        if abs(t[i] - tt) > 0.02:
            continue
        rows_md.append(i)

    report = {
        "t_fire_s": t_fire,
        "t_end_s": T_END,
        "n_ticks": int(len(t)),
        "first_sign_disagree_s": first_sign_disagree,
        "ctrl_ky0_zero_cross_s": ctrl_zero_cross,
        "latch_ky0_zero_cross_s": latch_zero_cross,
        "first_rd_ky0_10pct_s": first_above(rd_ky0, 0.10),
        "first_rd_ky0_50pct_s": first_above(rd_ky0, 0.50),
        "first_rd_pyy_10pct_s": first_above(rd_pyy, 0.10),
        "max_step_frac_of_dky0_range": max_step_frac,
        "max_step_t_s": float(t[i_max_step]) if len(t) else None,
        "growth_early_ptp_dky0": growth_early,
        "growth_late_ptp_dky0": growth_late,
        "corr_abs_dky0_vs_cum_withheld_z": corr_withheld,
        "corr_abs_dky0_vs_abs_dpyy": corr_dpyy,
        "end_ky0_c": float(ky0_c[-1]),
        "end_ky0_l": float(ky0_l[-1]),
        "end_pyy_c": float(pyy_c[-1]),
        "end_pyy_l": float(pyy_l[-1]),
        "verdict": {
            "label": label,
            "reading": reading,
            "next": next_step,
        },
    }

    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    axes[0].plot(t, ky0_c, "C0", lw=1.2, label="ctrl K_y0")
    axes[0].plot(t, ky0_l, "C3", lw=1.2, label="latch K_y0")
    axes[0].axhline(0, color="gray", lw=0.4)
    axes[0].set_ylabel("K_y0")
    axes[0].set_title(f"K[ATT_Y, innov_y] from latch fire ({t_fire:.3f}s) → {T_END}s")
    axes[0].legend(fontsize=8)

    axes[1].plot(t, pyy_c, "C0", label="ctrl P_yy")
    axes[1].plot(t, pyy_l, "C3", label="latch P_yy")
    axes[1].plot(t, pzz_c, "C0", ls="--", alpha=0.7, label="ctrl P_zz")
    axes[1].plot(t, pzz_l, "C3", ls="--", alpha=0.7, label="latch P_zz")
    axes[1].set_ylabel("P diag")
    axes[1].legend(fontsize=7, ncol=2)

    axes[2].plot(t, d_ky0, "C2", label="ΔK_y0 (L−C)")
    axes[2].plot(t, d_pyy * 10, "C4", label="ΔP_yy×10")
    axes[2].axhline(0, color="gray", lw=0.4)
    axes[2].set_ylabel("latch−ctrl")
    axes[2].legend(fontsize=8)

    axes[3].plot(t, rd_ky0, "C2", label="relΔ K_y0")
    axes[3].plot(t, rd_pyy, "C4", label="relΔ P_yy")
    axes[3].plot(t, rd_pzz, "C5", ls="--", label="relΔ P_zz")
    axes[3].plot(t, cum_withheld / max(cum_withheld[-1], 1e-30), "k", alpha=0.5, label="cum|dx_z_raw|_L norm")
    axes[3].set_ylabel("relΔ / norm")
    axes[3].set_xlabel("t [s]")
    axes[3].legend(fontsize=7, ncol=2)

    for ax in axes:
        ax.axvline(t_fire, color="orange", ls="--", alpha=0.7, label=None)
        if first_sign_disagree:
            ax.axvline(first_sign_disagree, color="purple", ls=":", alpha=0.7)
        if ctrl_zero_cross:
            ax.axvline(ctrl_zero_cross, color="C0", ls=":", alpha=0.5)
    fig.tight_layout()
    fig_path = OUT / "fig_ky0_trajectory_onset.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "ky0_trajectory_onset.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )

    lines = [
        "# K_y0 / P_att trajectory — latch fire → 1.54s",
        "",
        f"**Verdict:** `{label}`",
        "",
        reading,
        "",
        f"**Next:** {next_step}",
        "",
        f"- Latch fire (first applied dx_z≡0 with raw≠0): **t={t_fire:.4f}s**",
        f"- First sign(K_y0) disagree: **{first_sign_disagree}**",
        f"- Ctrl K_y0 zero-cross: **{ctrl_zero_cross}**",
        f"- First relΔ K_y0 ≥10%: **{report['first_rd_ky0_10pct_s']}**",
        f"- First relΔ K_y0 ≥50%: **{report['first_rd_ky0_50pct_s']}**",
        f"- Max single-tick |Δ(ΔK_y0)| / range: **{max_step_frac:.2%}** @ t={report['max_step_t_s']}",
        f"- ptp(ΔK_y0) early/late half: {growth_early:.3e} / {growth_late:.3e}",
        f"- corr(|ΔK_y0|, cum|dx_z_raw|_latch) = **{corr_withheld:+.3f}**",
        f"- corr(|ΔK_y0|, |ΔP_yy|) = **{corr_dpyy:+.3f}**",
        "",
        "## Sparse ticks",
        "",
        "| t | Ky0_c | Ky0_l | ΔKy0 | relΔ | Pyy_c | Pyy_l | relΔPyy |",
        "|---|-------|-------|------|------|-------|-------|---------|",
    ]
    for i in rows_md:
        lines.append(
            f"| {t[i]:.3f} | {ky0_c[i]:+.4e} | {ky0_l[i]:+.4e} | {d_ky0[i]:+.4e} | "
            f"{rd_ky0[i]:.3f} | {pyy_c[i]:.4e} | {pyy_l[i]:.4e} | {rd_pyy[i]:.3f} |"
        )
    lines += ["", f"Figure: `{fig_path.name}`", ""]
    (OUT / "ky0_trajectory_onset.md").write_text("\n".join(lines), encoding="utf-8")

    print(json.dumps(report["verdict"], indent=2))
    print("---")
    print(json.dumps({k: report[k] for k in report if k != "verdict" and k != "figure"}, indent=2))


if __name__ == "__main__":
    main()
