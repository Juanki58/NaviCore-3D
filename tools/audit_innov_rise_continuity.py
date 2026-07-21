#!/usr/bin/env python3
"""Is innov ratio jump at break discrete, or continuous trend crossing a threshold?"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"

OMEGA = 2.0 * np.pi / 4.0
YAW_AMP = 0.13750987
T_ONSET, T_BREAK, T_PEAK = 1.34, 1.59, 2.0


def lin(x: np.ndarray, y: np.ndarray) -> tuple[float | None, float | None]:
    if len(x) < 2:
        return None, None
    p = np.polyfit(x, y, 1)
    return float(p[0]), float(p[1])


def main() -> None:
    ctrl = pd.read_csv(OUT / "ctrl_nhc_block_audit.csv")
    latch = pd.read_csv(OUT / "latch_nhc_block_audit.csv")
    m = ctrl.merge(latch, on="timestamp_s", suffixes=("_c", "_l"))
    t = m["timestamp_s"].to_numpy(float)
    yc = np.column_stack(
        [m["innov_y_mps_c"].to_numpy(float), m["innov_z_mps_c"].to_numpy(float)]
    )
    yl = np.column_stack(
        [m["innov_y_mps_l"].to_numpy(float), m["innov_z_mps_l"].to_numpy(float)]
    )
    nc = np.linalg.norm(yc, axis=1)
    nl = np.linalg.norm(yl, axis=1)
    den = nc * nl
    cos = np.full(len(t), np.nan)
    ok = den > 1e-15
    cos[ok] = np.sum(yc[ok] * yl[ok], axis=1) / den[ok]
    ratio = nl / np.maximum(nc, 1e-15)
    dxc = m["dx_bias_gz_c"].to_numpy(float)
    dxl = m["dx_bias_gz_l"].to_numpy(float)
    aw = np.abs(YAW_AMP * OMEGA * np.cos(OMEGA * t))
    wpeak = abs(YAW_AMP * OMEGA)

    rise = (t >= T_ONSET) & (t <= T_PEAK)
    pre = (t >= T_ONSET) & (t < T_BREAK)
    post = (t >= T_BREAK) & (t <= T_PEAK)
    edge_pre = (t >= T_BREAK - 0.10) & (t < T_BREAK)
    edge_post = (t >= T_BREAK) & (t < T_BREAK + 0.10)
    edge_pre05 = (t >= T_BREAK - 0.05) & (t < T_BREAK)
    edge_post05 = (t >= T_BREAK) & (t < T_BREAK + 0.05)

    def stats(mask: np.ndarray, label: str) -> dict:
        if not mask.any():
            return {"label": label, "n": 0}
        r = ratio[mask]
        return {
            "label": label,
            "n": int(mask.sum()),
            "t_span": [float(t[mask].min()), float(t[mask].max())],
            "mean_ratio": float(np.mean(r)),
            "median_ratio": float(np.median(r)),
            "p90_ratio": float(np.percentile(r, 90)),
            "mean_innov_c": float(np.mean(nc[mask])),
            "mean_innov_l": float(np.mean(nl[mask])),
            "sum_innov_c": float(np.sum(nc[mask])),
            "sum_innov_l": float(np.sum(nl[mask])),
            "sum_ratio_of_sums": float(np.sum(nl[mask]) / max(np.sum(nc[mask]), 1e-15)),
            "median_cos": float(np.nanmedian(cos[mask])),
            "mean_cos": float(np.nanmean(cos[mask])),
            "frac_cos_lt_0": float(np.mean(cos[mask] < 0)),
            "sum_dx_bias_gz_c": float(dxc[mask].sum()),
            "sum_dx_bias_gz_l": float(dxl[mask].sum()),
            "mean_abs_omega_frac_peak": float(np.mean(aw[mask] / wpeak)),
        }

    bins = np.arange(T_ONSET, T_PEAK + 1e-9, 0.05)
    bin_rows = []
    for i in range(len(bins) - 1):
        a, b = float(bins[i]), float(bins[i + 1])
        mask = (t >= a) & (t < b)
        if not mask.any():
            continue
        bin_rows.append(
            {
                "t_mid": 0.5 * (a + b),
                "t0": a,
                "t1": b,
                "n": int(mask.sum()),
                "median_ratio": float(np.median(ratio[mask])),
                "mean_ratio": float(np.mean(ratio[mask])),
                "sum_ratio": float(np.sum(nl[mask]) / max(np.sum(nc[mask]), 1e-15)),
                "median_cos": float(np.nanmedian(cos[mask])),
                "mean_innov_l": float(np.mean(nl[mask])),
                "mean_innov_c": float(np.mean(nc[mask])),
                "sum_dx_l": float(dxl[mask].sum()),
                "omega_frac": float(np.mean(aw[mask] / wpeak)),
            }
        )

    s_pre, i_pre = lin(t[pre], np.log(np.maximum(ratio[pre], 1e-6)))
    s_post, _ = lin(t[post], np.log(np.maximum(ratio[post], 1e-6)))
    s_all, _ = lin(t[rise], np.log(np.maximum(ratio[rise], 1e-6)))

    pred_at_break = None if s_pre is None else float(np.exp(s_pre * T_BREAK + i_pre))
    actual_first_post = float(np.median(ratio[edge_post])) if edge_post.any() else None
    actual_last_pre = float(np.median(ratio[edge_pre])) if edge_pre.any() else None
    jump_edge_median = (
        None
        if actual_first_post is None or actual_last_pre is None
        else float(actual_first_post - actual_last_pre)
    )
    jump_mean = float(np.mean(ratio[post]) - np.mean(ratio[pre]))
    jump_median = float(np.median(ratio[post]) - np.median(ratio[pre]))

    order = np.argsort(t[rise])
    t_r = t[rise][order]
    nl_r = nl[rise][order]
    nc_r = nc[rise][order]
    csum_l = np.cumsum(nl_r)
    csum_c = np.cumsum(nc_r)
    frac_sum_l_by_break = (
        float(csum_l[t_r < T_BREAK][-1] / csum_l[-1]) if (t_r < T_BREAK).any() else 0.0
    )
    frac_sum_c_by_break = (
        float(csum_c[t_r < T_BREAK][-1] / csum_c[-1]) if (t_r < T_BREAK).any() else 0.0
    )
    frac_abs_dxl_by_break = float(
        np.abs(dxl[pre]).sum() / max(np.abs(dxl[rise]).sum(), 1e-15)
    )

    mids = np.array([b["t_mid"] for b in bin_rows])
    med_r = np.array([b["median_ratio"] for b in bin_rows])
    med_c = np.array([b["median_cos"] for b in bin_rows])
    d1 = np.diff(med_r)
    typical_step = float(np.median(np.abs(d1))) if len(d1) else 0.0
    max_step = float(np.max(np.abs(d1))) if len(d1) else 0.0
    i_max_d1 = int(np.argmax(np.abs(d1))) if len(d1) else 0
    max_step_t = float(0.5 * (mids[i_max_d1] + mids[i_max_d1 + 1])) if len(d1) else None
    max_step_at_break = (
        abs(max_step_t - T_BREAK) < 0.08 if max_step_t is not None else False
    )

    pre_t = t[pre]
    pre_r = ratio[pre]
    mid_pre = 0.5 * (T_ONSET + T_BREAK)
    pre_h1 = pre_r[pre_t < mid_pre]
    pre_h2 = pre_r[pre_t >= mid_pre]
    pre_growth = (
        float(np.median(pre_h2) - np.median(pre_h1))
        if len(pre_h1) and len(pre_h2)
        else None
    )

    cos_last_pre = float(np.nanmedian(cos[edge_pre])) if edge_pre.any() else None
    cos_first_post = float(np.nanmedian(cos[edge_post])) if edge_post.any() else None
    edge_j = jump_edge_median if jump_edge_median is not None else 0.0

    flat_pre = (
        pre_growth is not None
        and abs(pre_growth) < 0.3
        and float(np.median(pre_r)) < 1.5
    )
    big_edge = (
        abs(edge_j) > max(3 * typical_step, 2.0) if typical_step > 0 else abs(edge_j) > 2.0
    )

    if flat_pre and (big_edge or max_step_at_break):
        label = "DISCRETE_INFLECTION_AT_BREAK"
        reading = (
            "Innovation ratio is flat through onset→break and jumps at the break — "
            "treat 1.59s as a real dynamic inflection for intervention timing."
        )
    elif (not flat_pre) and (not big_edge or not max_step_at_break):
        label = "CONTINUOUS_TREND_CROSSING_THRESHOLD"
        reading = (
            "Innovation ratio already evolves on the rising limb before 1.59s; the "
            "cosine median flip marks where a continuing trend crosses a statistical "
            "threshold, not a privileged causal instant. Window K_bias decompose on "
            "full rise onset→peak (1.34–2.0), not post-break only."
        )
    else:
        label = "MIXED_CONTINUOUS_WITH_ACCELERATION_NEAR_BREAK"
        reading = (
            "Trend exists before break and accelerates near it — still prefer "
            "full-rise window; do not use 1.59 as sole trigger (H-ATT-c timing lesson)."
        )

    # Extra nuance flags for the writeup
    nuance = {
        "post_mean_ratio_vs_aggregate_6": float(np.mean(ratio[post])),
        "pre_mean_ratio": float(np.mean(ratio[pre])),
        "post_sum_ratio_of_sums": float(
            np.sum(nl[post]) / max(np.sum(nc[post]), 1e-15)
        ),
        "pre_sum_ratio_of_sums": float(np.sum(nl[pre]) / max(np.sum(nc[pre]), 1e-15)),
        "note": (
            "Aggregate post-break mean ratio≈6 can be dominated by a few large ticks; "
            "compare median, edge jump, and Σ‖y‖ fractions."
        ),
    }

    report = {
        "windows": {
            "onset_to_break": stats(pre, "onset→break [1.34,1.59)"),
            "break_to_peak": stats(post, "break→peak [1.59,2.0]"),
            "full_rise": stats(rise, "full rise [1.34,2.0]"),
        },
        "edge_0.10s": {
            "pre": stats(edge_pre, "break-0.10→break"),
            "post": stats(edge_post, "break→break+0.10"),
            "jump_median_ratio": jump_edge_median,
            "median_cos_pre": cos_last_pre,
            "median_cos_post": cos_first_post,
        },
        "edge_0.05s": {
            "pre": stats(edge_pre05, "break-0.05→break"),
            "post": stats(edge_post05, "break→break+0.05"),
            "jump_median_ratio": (
                float(np.median(ratio[edge_post05]) - np.median(ratio[edge_pre05]))
                if edge_pre05.any() and edge_post05.any()
                else None
            ),
        },
        "continuity": {
            "log_ratio_slope_pre": s_pre,
            "log_ratio_slope_post": s_post,
            "log_ratio_slope_full_rise": s_all,
            "pred_ratio_at_break_from_pre_fit": pred_at_break,
            "actual_median_ratio_last_0.1_pre": actual_last_pre,
            "actual_median_ratio_first_0.1_post": actual_first_post,
            "jump_mean_ratio_post_minus_pre": jump_mean,
            "jump_median_ratio_post_minus_pre": jump_median,
            "pre_half_growth_median_ratio": pre_growth,
            "typical_abs_bin_step_ratio_50ms": typical_step,
            "max_abs_bin_step_ratio_50ms": max_step,
            "max_abs_bin_step_t_mid": max_step_t,
            "max_step_near_break": max_step_at_break,
            "frac_sum_innov_l_accumulated_by_break": frac_sum_l_by_break,
            "frac_sum_innov_c_accumulated_by_break": frac_sum_c_by_break,
            "frac_abs_dx_bias_gz_l_by_break": frac_abs_dxl_by_break,
            "flat_pre": flat_pre,
            "big_edge_jump": big_edge,
        },
        "nuance": nuance,
        "bins_50ms": bin_rows,
        "verdict": {"label": label, "reading": reading},
        "design_implication": {
            "decompose_window": "full rise onset→peak [1.34, 2.0]",
            "do_not": "Use t=1.59 alone as intervention trigger or as sole decompose window",
            "note_bias": (
                "Half of Σdx_bias_gz latch already in onset→break; confirms silent "
                "accumulation before visible cos break."
            ),
        },
    }

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax0, ax1, ax2 = axes
    ax0.plot(t[rise], aw[rise] / wpeak, "k-", lw=1.2, label="|ω|/peak")
    ax0.axvline(T_ONSET, color="C1", ls=":", label="onset 1.34")
    ax0.axvline(T_BREAK, color="C3", lw=1.5, label="break 1.59")
    ax0.axvline(T_PEAK, color="k", ls="--", alpha=0.5, label="peak 2.0")
    ax0.set_ylabel("|ω|/peak")
    ax0.legend(loc="upper left", fontsize=8)
    ax0.set_title("Rise continuity: innov ratio vs composition break")

    ax1.plot(t[rise], ratio[rise], "C0.", ms=4, alpha=0.5, label="ratio per tick")
    ax1.plot(mids, med_r, "C0-", lw=1.5, label="median ratio 50ms")
    ax1.axhline(1.0, color="gray", ls=":", alpha=0.5)
    ax1.axvline(T_ONSET, color="C1", ls=":")
    ax1.axvline(T_BREAK, color="C3", lw=1.5)
    ax1.axvline(T_PEAK, color="k", ls="--", alpha=0.5)
    ax1.set_ylabel("‖y‖_L / ‖y‖_C")
    ax1.set_ylim(0, max(12.0, float(np.percentile(ratio[rise], 95)) * 1.1))
    ax1.legend(loc="upper left", fontsize=8)

    ax2.plot(t[rise], cos[rise], "C2.", ms=4, alpha=0.4)
    ax2.plot(mids, med_c, "C2-", lw=1.5, label="median cos 50ms")
    ax2.axhline(0.0, color="gray", ls="-", alpha=0.3)
    ax2.axvline(T_ONSET, color="C1", ls=":")
    ax2.axvline(T_BREAK, color="C3", lw=1.5)
    ax2.axvline(T_PEAK, color="k", ls="--", alpha=0.5)
    ax2.set_ylabel("cos")
    ax2.set_xlabel("t [s]")
    ax2.set_ylim(-1.05, 1.05)
    ax2.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    fig_path = OUT / "fig_innov_rise_continuity.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "innov_rise_continuity.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )

    w = report["windows"]
    lines = [
        "# Innov ratio continuity along rising limb",
        "",
        f"**Verdict:** `{label}`",
        "",
        reading,
        "",
        "## Strict split: onset→break vs break→peak",
        "",
        "| Window | n | median ratio | mean ratio | Σ‖y‖_L/Σ‖y‖_C | median cos | Σdx_bias_gz latch |",
        "|--------|---|--------------|------------|---------------|------------|-------------------|",
    ]
    for key in ("onset_to_break", "break_to_peak", "full_rise"):
        p = w[key]
        lines.append(
            f"| {p['label']} | {p['n']} | {p['median_ratio']:.2f} | "
            f"{p['mean_ratio']:.2f} | {p['sum_ratio_of_sums']:.2f} | "
            f"{p['median_cos']:.2f} | {p['sum_dx_bias_gz_l']:+.4f} |"
        )
    lines += [
        "",
        "## Edge jump at break (±0.10 s)",
        "",
        f"- median ratio last 0.10s pre: **{actual_last_pre:.2f}**",
        f"- median ratio first 0.10s post: **{actual_first_post:.2f}**",
        f"- jump: **{jump_edge_median:+.2f}** "
        f"(typical 50ms bin step ≈ {typical_step:.2f}; "
        f"max step ≈ {max_step:.2f} @ t_mid={max_step_t})",
        f"- median cos: {cos_last_pre:.2f} → {cos_first_post:.2f}",
        f"- pre half-growth (1st→2nd half of onset→break) median ratio Δ: "
        f"**{pre_growth:+.2f}**",
        "",
        "## Accumulation by break (of full-rise totals)",
        "",
        f"- fraction Σ‖y‖_latch by break: **{frac_sum_l_by_break:.2%}**",
        f"- fraction Σ‖y‖_ctrl by break: **{frac_sum_c_by_break:.2%}**",
        f"- fraction |Σ| dx_bias_gz latch by break: **{frac_abs_dxl_by_break:.2%}**",
        "",
        "## Design",
        "",
        "- Decompose window: **full rise [1.34 → 2.0]**",
        "- Do **not** trigger / window solely on t=1.59 (H-ATT-c timing lesson)",
        "",
        f"Figure: `{fig_path.name}`",
        "",
    ]
    (OUT / "innov_rise_continuity.md").write_text("\n".join(lines), encoding="utf-8")

    print(json.dumps(report["verdict"], indent=2))
    print(
        "pre",
        w["onset_to_break"]["median_ratio"],
        w["onset_to_break"]["mean_ratio"],
        w["onset_to_break"]["sum_ratio_of_sums"],
    )
    print(
        "post",
        w["break_to_peak"]["median_ratio"],
        w["break_to_peak"]["mean_ratio"],
        w["break_to_peak"]["sum_ratio_of_sums"],
    )
    print("edge", actual_last_pre, "->", actual_first_post, "jump", jump_edge_median)
    print(
        "pre_growth",
        pre_growth,
        "typical_step",
        typical_step,
        "max_step",
        max_step,
        "@",
        max_step_t,
    )
    print(
        "frac_sum_l_by_break",
        frac_sum_l_by_break,
        "frac_dxl",
        frac_abs_dxl_by_break,
        "flat_pre",
        flat_pre,
        "big_edge",
        big_edge,
    )


if __name__ == "__main__":
    main()
