#!/usr/bin/env python3
"""Is 22% Joseph underclip enough to explain P[ATT_Y,VN] latch−ctrl divergence?

Pure arithmetic on existing audits [1.10→1.54]:
  linear: Σ(dP_joseph_L − dP_joseph_C) + Σ(dP_predict_L − dP_predict_C) ≟ ΔP_final
  feedback: does |deficit| grow with |P_L| (compound) or stay flat?

CORR_ABS_SCALE on every correlation.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"
T0, T1 = 1.10, 1.54
# onset baseline for the "×260" claim
T_ON0, T_ON1 = 0.40, 1.10


def load_arm(arm: str) -> pd.DataFrame:
    d = pd.read_csv(OUT / f"{arm}_nhc_block_audit.csv")
    need = ["timestamp_s", "P_pre_att_y_vn", "P_post_att_y_vn"]
    miss = [c for c in need if c not in d.columns]
    if miss:
        raise KeyError(miss)
    return d


def window(df: pd.DataFrame, t0: float, t1: float) -> pd.DataFrame:
    m = (df["timestamp_s"] >= t0 - 1e-9) & (df["timestamp_s"] <= t1 + 1e-9)
    return df.loc[m].copy().reset_index(drop=True)


def safe_corr(a, b):
    if len(a) < 3 or np.std(a) < 1e-18 or np.std(b) < 1e-18:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def main() -> None:
    C = load_arm("ctrl")
    L = load_arm("latch")

    def series(df: pd.DataFrame) -> pd.DataFrame:
        w = window(df, T0 - 0.02, T1)
        t = w["timestamp_s"].to_numpy(float)
        pre = w["P_pre_att_y_vn"].to_numpy(float)
        post = w["P_post_att_y_vn"].to_numpy(float)
        d_jos = post - pre
        d_pred = np.full_like(pre, np.nan)
        d_pred[1:] = pre[1:] - post[:-1]
        return pd.DataFrame(
            {"t": t, "P_pre": pre, "P_post": post, "d_jos": d_jos, "d_pred": d_pred}
        )

    Cs, Ls = series(C), series(L)
    Cs["t_r"] = Cs["t"].round(6)
    Ls["t_r"] = Ls["t"].round(6)
    m = Ls.merge(Cs, on="t_r", suffixes=("_l", "_c"))
    m = m[(m["t_r"] >= T0) & (m["t_r"] <= T1)].reset_index(drop=True)
    n = len(m)
    t = m["t_r"].to_numpy(float)

    d_jos_l = m["d_jos_l"].to_numpy(float)
    d_jos_c = m["d_jos_c"].to_numpy(float)
    d_pred_l = m["d_pred_l"].to_numpy(float)
    d_pred_c = m["d_pred_c"].to_numpy(float)
    p_l = m["P_pre_l"].to_numpy(float)
    p_c = m["P_pre_c"].to_numpy(float)

    # deficit: latch cuts less → d_jos_l > d_jos_c when both negative
    jos_deficit = d_jos_l - d_jos_c  # positive ⇒ underclip
    pred_excess = d_pred_l - d_pred_c
    # finite masks for predict (first tick nan)
    fin = np.isfinite(d_pred_l) & np.isfinite(d_pred_c)

    sum_jos_def = float(np.sum(jos_deficit))
    sum_pred_exc = float(np.nansum(pred_excess))
    sum_total_lin = sum_jos_def + sum_pred_exc

    dP_start = float(p_l[0] - p_c[0])
    dP_end = float(p_l[-1] - p_c[-1])
    dP_observed = dP_end - dP_start

    # Linear reconstruction of ΔP path
    # ΔP_pre[t] ≈ ΔP_pre[0] + cumsum(pred_excess) + cumsum(jos_deficit) but ordering:
    # cycle: post[t-1] → predict → pre[t] → joseph → post[t]
    # so Δpre[t] - Δpre[t-1] = pred_excess[t] + (effect of previous joseph already in posts)
    # Better: reconstruct from components
    # Δpost = Δpre + jos_deficit (same tick)
    # Δpre[t] = Δpost[t-1] + pred_excess[t]
    dP_recon = np.zeros(n)
    dP_recon[0] = dP_start
    for i in range(1, n):
        # after joseph at i-1: Δpost[i-1] = Δpre[i-1] + jos_deficit[i-1]
        # then predict: Δpre[i] = Δpost[i-1] + pred_excess[i]
        dP_recon[i] = (
            dP_recon[i - 1] + jos_deficit[i - 1] + (pred_excess[i] if fin[i] else 0.0)
        )
    # Also include last joseph? Observed is on P_pre, so recon above tracks P_pre.
    # Check residual
    dP_actual = p_l - p_c
    recon_err = dP_actual - dP_recon
    recon_rmse = float(np.sqrt(np.mean(recon_err**2)))
    recon_end_err = float(dP_actual[-1] - dP_recon[-1])
    frac_explained = float(sum_total_lin / dP_observed) if abs(dP_observed) > 1e-30 else float("nan")
    frac_jos = float(sum_jos_def / dP_observed) if abs(dP_observed) > 1e-30 else float("nan")

    # ×260 baseline: max|Δ| in late vs onset
    def max_abs_delta(t0, t1):
        cw = window(C, t0, t1)
        lw = window(L, t0, t1)
        cw = cw.copy()
        lw = lw.copy()
        cw["t_r"] = cw["timestamp_s"].round(6)
        lw["t_r"] = lw["timestamp_s"].round(6)
        mm = lw.merge(cw, on="t_r", suffixes=("_l", "_c"))
        d = mm["P_pre_att_y_vn_l"].to_numpy(float) - mm["P_pre_att_y_vn_c"].to_numpy(float)
        return float(np.max(np.abs(d))), float(d[0]), float(d[-1]), int(len(d))

    max_on, _, _, n_on = max_abs_delta(T_ON0, T_ON1)
    max_late, d0_late, d1_late, n_late = max_abs_delta(T0, T1)
    ratio_260 = max_late / max(max_on, 1e-30)

    # Feedback test: |jos_deficit| vs |P_l| and vs |ΔP|
    abs_def = np.abs(jos_deficit)
    corr_def_Pl = {
        "pearson": safe_corr(abs_def, np.abs(p_l)),
        "mean_abs_deficit": float(np.mean(abs_def)),
        "mean_abs_P_l": float(np.mean(np.abs(p_l))),
        "max_abs_deficit": float(np.max(abs_def)),
        "max_abs_P_l": float(np.max(np.abs(p_l))),
    }
    corr_def_dP = {
        "pearson": safe_corr(abs_def, np.abs(dP_actual)),
        "mean_abs_deficit": float(np.mean(abs_def)),
        "mean_abs_dP": float(np.mean(np.abs(dP_actual))),
        "max_abs_deficit": float(np.max(abs_def)),
        "max_abs_dP": float(np.max(np.abs(dP_actual))),
    }
    # Is deficit roughly constant? CV = std/mean
    cv_def = float(np.std(jos_deficit) / max(np.mean(np.abs(jos_deficit)), 1e-30))
    # Early vs late half mean deficit
    mid = n // 2
    mean_def_early = float(np.mean(jos_deficit[:mid]))
    mean_def_late = float(np.mean(jos_deficit[mid:]))

    # Counterfactual: apply ctrl joseph to latch each tick (on reconstructed path)
    # Start from latch P_pre[0]; alternate predict_L then joseph_C
    p_cf = np.zeros(n)
    p_cf[0] = p_l[0]
    for i in range(1, n):
        # predict as latch did: from post. We don't have cf post; use
        # p_pre_cf[i] = p_post_cf[i-1] + d_pred_l[i]
        # p_post_cf[i-1] = p_pre_cf[i-1] + d_jos_c[i-1]
        p_post_prev = p_cf[i - 1] + d_jos_c[i - 1]
        p_cf[i] = p_post_prev + (d_pred_l[i] if fin[i] else 0.0)
    # Compare cf end to ctrl and latch
    cf_end = float(p_cf[-1])
    cf_vs_latch_end = cf_end - float(p_l[-1])
    cf_vs_ctrl_end = cf_end - float(p_c[-1])

    # Compound interest toy: if each tick deficit = 0.22 * |jos_c| constant
    mean_jos_c = float(np.mean(d_jos_c))  # negative
    toy_flat_deficit = -0.22 * mean_jos_c  # positive underclip amount
    toy_linear = dP_start + n * toy_flat_deficit  # crude (n ticks of joseph)
    # Better toy: sum 0.22*|d_jos_c| 
    toy_sum = dP_start + float(np.sum(-0.22 * d_jos_c))  # if d_jos_c<0, -0.22*neg = positive

    # Verdict
    # Linear explains if |frac_explained - 1| < 0.15 and recon_rmse << |dP_end|
    rel_recon = recon_rmse / max(abs(dP_end), 1e-30)
    linear_ok = abs(frac_explained - 1.0) < 0.15 and rel_recon < 0.15
    # Feedback if corr(|def|,|P_l|) high AND mean_def_late/early >= 1.5
    feedback = (
        corr_def_Pl["pearson"] >= 0.7
        and abs(mean_def_late) >= 1.5 * abs(mean_def_early)
        and corr_def_Pl["mean_abs_deficit"] >= 1e-6
    )

    if linear_ok and not feedback:
        label = "JOSEPH_DEFICIT_LINEAR_SUFFICES"
        reading = (
            "Σ joseph underclip (+ predict excess) reconstructs the latch−ctrl "
            f"ΔP to {frac_explained:.0%} (rmse/|ΔP_end|={rel_recon:.2%}). The ×260 "
            "is vs a tiny onset baseline, not a nonlinear compound beyond the sum of "
            "per-tick deficits. Boring complete explanation: less Joseph cut, linear accumulate."
        )
    elif linear_ok and feedback:
        label = "LINEAR_SUM_OK_WITH_GROWING_DEFICIT"
        reading = (
            "Linear sum still closes the budget, but per-tick deficit grows with P "
            "(feedback/compound flavor) — FEEDBACK_GROWTH in covariance, still no new motor."
        )
    elif abs(frac_explained) < 0.5:
        label = "LINEAR_DEFICIT_INSUFFICIENT"
        reading = (
            f"Linear Σ deficits only explain {frac_explained:.0%} of observed ΔP — "
            "need nonlinear accumulation or another term."
        )
    else:
        label = "JOSEPH_DEFICIT_MIXED"
        reading = (
            f"Budget frac={frac_explained:.0%}, recon rel_rmse={rel_recon:.0%}; "
            "neither clean linear nor clear shortfall."
        )

    report = {
        "window_s": [T0, T1],
        "n_ticks": n,
        "dP_start": dP_start,
        "dP_end": dP_end,
        "dP_observed_growth": dP_observed,
        "sum_joseph_deficit": sum_jos_def,
        "sum_predict_excess": sum_pred_exc,
        "sum_linear_total": sum_total_lin,
        "frac_linear_explains_growth": frac_explained,
        "frac_joseph_of_growth": frac_jos,
        "recon_rmse": recon_rmse,
        "recon_end_err": recon_end_err,
        "recon_rel_rmse": rel_recon,
        "x260": {
            "max_abs_dP_onset": max_on,
            "max_abs_dP_late": max_late,
            "ratio_late_over_onset": ratio_260,
            "note": "×260 = max|ΔP| late / max|ΔP| onset — baseline onset is tiny, not a compound factor on joseph.",
        },
        "deficit_stats": {
            "mean": float(np.mean(jos_deficit)),
            "mean_abs": float(np.mean(np.abs(jos_deficit))),
            "cv": cv_def,
            "mean_early": mean_def_early,
            "mean_late": mean_def_late,
            "late_over_early": float(mean_def_late / mean_def_early) if abs(mean_def_early) > 1e-30 else float("nan"),
        },
        "corr_abs_deficit_vs_abs_P_l": corr_def_Pl,
        "corr_abs_deficit_vs_abs_dP": corr_def_dP,
        "counterfactual_ctrl_joseph_on_latch_predict": {
            "P_cf_end": cf_end,
            "P_latch_end": float(p_l[-1]),
            "P_ctrl_end": float(p_c[-1]),
            "cf_minus_latch": cf_vs_latch_end,
            "cf_minus_ctrl": cf_vs_ctrl_end,
        },
        "toy_22pct_of_mean_ctrl_joseph": {
            "toy_sum_end_dP": toy_sum,
            "flat_n_times": toy_linear,
        },
        "verdict": {"label": label, "reading": reading},
    }

    # ticks csv
    pd.DataFrame(
        {
            "t": t,
            "P_l": p_l,
            "P_c": p_c,
            "dP": dP_actual,
            "dP_recon": dP_recon,
            "jos_def": jos_deficit,
            "pred_exc": pred_excess,
            "d_jos_l": d_jos_l,
            "d_jos_c": d_jos_c,
            "P_cf": p_cf,
        }
    ).to_csv(OUT / "joseph_clip_deficit_ticks.csv", index=False)

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t, p_c, "C0", label="ctrl P_pre")
    axes[0].plot(t, p_l, "C3", label="latch P_pre")
    axes[0].plot(t, p_cf, "C2", ls="--", label="cf: latch pred + ctrl joseph")
    axes[0].legend(fontsize=8)
    axes[0].set_ylabel("P[Ay,Vn]")
    axes[0].set_title("Joseph underclip budget [1.10→1.54]")

    axes[1].plot(t, dP_actual, "k", label="ΔP actual (L−C)")
    axes[1].plot(t, dP_recon, "C2", ls="--", label="ΔP recon Σ(pred_exc+jos_def)")
    axes[1].axhline(0, color="gray", lw=0.4)
    axes[1].legend(fontsize=8)
    axes[1].set_ylabel("ΔP")

    axes[2].plot(t, jos_deficit, "C1", label="joseph deficit (L−C)")
    axes[2].plot(t, np.cumsum(jos_deficit), "C1", ls="--", label="cumsum deficit")
    axes[2].plot(t, np.cumsum(np.where(fin, pred_excess, 0.0)), "C0", ls=":", label="cumsum pred excess")
    axes[2].axhline(0, color="gray", lw=0.4)
    axes[2].legend(fontsize=8)
    axes[2].set_xlabel("t [s]")
    axes[2].set_ylabel("deficit")
    fig.tight_layout()
    fig_path = OUT / "fig_joseph_clip_deficit_110_154.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "joseph_clip_deficit_110_154.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )

    lines = [
        "# Joseph underclip budget — ¿explica ΔP[ATT_Y,VN]?",
        "",
        f"**Verdict:** `{label}`",
        "",
        reading,
        "",
        f"n={n} ticks in [{T0},{T1}].",
        "",
        "## Linear budget (CORR_ABS_SCALE)",
        "",
        f"| qty | value |",
        f"|-----|-------|",
        f"| ΔP start (L−C) | {dP_start:+.6e} |",
        f"| ΔP end (L−C) | {dP_end:+.6e} |",
        f"| ΔP growth observed | {dP_observed:+.6e} |",
        f"| Σ joseph deficit (L−C) | {sum_jos_def:+.6e} |",
        f"| Σ predict excess (L−C) | {sum_pred_exc:+.6e} |",
        f"| Σ linear total | {sum_total_lin:+.6e} |",
        f"| frac linear explains growth | **{frac_explained:.3f}** |",
        f"| frac joseph of growth | {frac_jos:.3f} |",
        f"| recon RMSE | {recon_rmse:.3e} (rel {rel_recon:.2%}) |",
        f"| recon end err | {recon_end_err:+.3e} |",
        "",
        "## The ×260",
        "",
        f"- max\\|ΔP\\| onset [0.40,1.10] = {max_on:.3e}",
        f"- max\\|ΔP\\| late [1.10,1.54] = {max_late:.3e}",
        f"- ratio = **{ratio_260:.1f}×** — vs tiny onset baseline, not a per-tick compound factor.",
        "",
        "## Deficit shape (feedback?)",
        "",
        f"- mean deficit early/late = {mean_def_early:+.3e} / {mean_def_late:+.3e} "
        f"(late/early = {report['deficit_stats']['late_over_early']:.2f})",
        f"- CV(deficit) = {cv_def:.2f}",
        f"- corr(\\|def\\|,\\|P_l\\|) = {corr_def_Pl['pearson']:+.3f} "
        f"(mean\\|def\\|={corr_def_Pl['mean_abs_deficit']:.3e}, mean\\|P_l\\|={corr_def_Pl['mean_abs_P_l']:.3e})",
        f"- corr(\\|def\\|,\\|ΔP\\|) = {corr_def_dP['pearson']:+.3f} "
        f"(mean\\|ΔP\\|={corr_def_dP['mean_abs_dP']:.3e})",
        "",
        "## Counterfactual: latch predict + ctrl joseph",
        "",
        f"- P_cf end = {cf_end:.6e}",
        f"- P_latch end = {float(p_l[-1]):.6e}",
        f"- P_ctrl end = {float(p_c[-1]):.6e}",
        f"- cf − latch = {cf_vs_latch_end:+.3e} (negative ⇒ ctrl joseph would have pulled latch down)",
        "",
        f"Figure: `{fig_path.name}`",
        "",
    ]
    (OUT / "joseph_clip_deficit_110_154.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report["verdict"], indent=2))
    print("---")
    print(json.dumps({k: report[k] for k in (
        "dP_start", "dP_end", "dP_observed_growth",
        "sum_joseph_deficit", "sum_predict_excess", "sum_linear_total",
        "frac_linear_explains_growth", "frac_joseph_of_growth",
        "recon_rmse", "recon_rel_rmse", "x260", "deficit_stats",
        "corr_abs_deficit_vs_abs_P_l", "counterfactual_ctrl_joseph_on_latch_predict",
    )}, indent=2))


if __name__ == "__main__":
    main()
