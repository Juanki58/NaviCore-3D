#!/usr/bin/env python3
"""Is path_scale just ‖y‖ seen through K, or a separate gain-coupling effect?

Also one more R3 subdivision (R3a1/a2, R3b1/b2) before closing R3 verdict.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

OUT = Path(__file__).resolve().parents[1] / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"

WINDOWS = {
    "R1": (1.34, 1.59, "[t0,t1)"),
    "R2": (1.59, 1.74, "[t0,t1)"),
    "R3": (1.74, 2.00, "[t0,t1]"),
    "full_rise": (1.34, 2.00, "[t0,t1]"),
}
R3_SUB2 = {
    "R3a1": (1.74, 1.805, "[t0,t1)"),
    "R3a2": (1.805, 1.87, "[t0,t1)"),
    "R3b1": (1.87, 1.935, "[t0,t1)"),
    "R3b2": (1.935, 2.00, "[t0,t1]"),
}


def mask(t: np.ndarray, t0: float, t1: float, ho: str) -> np.ndarray:
    return (t >= t0) & (t < t1) if ho == "[t0,t1)" else (t >= t0) & (t <= t1)


def per_tick(d: pd.DataFrame) -> pd.DataFrame:
    out = d.copy()
    vv = out["dx_bias_gz_via_vel"].to_numpy(float)
    va = out["dx_bias_gz_via_att"].to_numpy(float)
    out["path_scale_tick"] = np.abs(vv) + np.abs(va)
    out["net_abs"] = np.abs(out["dx_bias_gz"].to_numpy(float))
    out["cancel_tick"] = out["net_abs"] / np.maximum(out["path_scale_tick"], 1e-30)
    # gain-ish: path_scale / ‖y‖  (how much path activates per unit innov)
    y = out["innov_norm_mps"].to_numpy(float)
    out["path_per_innov"] = out["path_scale_tick"] / np.maximum(y, 1e-12)
    return out


def corr_stats(x: np.ndarray, y: np.ndarray) -> dict:
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 5:
        return {"n": int(m.sum()), "pearson": float("nan"), "spearman": float("nan")}
    xx, yy = x[m], y[m]
    # spearman via rank
    rx = pd.Series(xx).rank().to_numpy()
    ry = pd.Series(yy).rank().to_numpy()
    return {
        "n": int(m.sum()),
        "pearson": float(np.corrcoef(xx, yy)[0, 1]) if np.std(xx) > 0 and np.std(yy) > 0 else float("nan"),
        "spearman": float(np.corrcoef(rx, ry)[0, 1]) if np.std(rx) > 0 and np.std(ry) > 0 else float("nan"),
        "mean_x": float(np.mean(xx)),
        "mean_y": float(np.mean(yy)),
        "ratio_mean_x_over_mean_y": float(np.mean(xx) / max(np.mean(yy), 1e-30)),
    }


def win_report(d: pd.DataFrame, m: np.ndarray) -> dict:
    if not m.any():
        return {"n": 0}
    ps = d.loc[m, "path_scale_tick"].to_numpy(float)
    y = d.loc[m, "innov_norm_mps"].to_numpy(float)
    ppi = d.loc[m, "path_per_innov"].to_numpy(float)
    tot = d.loc[m, "dx_bias_gz"].to_numpy(float)
    vv = d.loc[m, "dx_bias_gz_via_vel"].to_numpy(float)
    va = d.loc[m, "dx_bias_gz_via_att"].to_numpy(float)
    c = corr_stats(ps, y)
    # residual of linear model path_scale ~ a * ‖y‖
    if np.std(y) > 1e-15:
        a = float(np.dot(ps, y) / np.dot(y, y))
        resid = ps - a * y
        frac_var_explained = float(1.0 - np.var(resid) / max(np.var(ps), 1e-30))
    else:
        a = float("nan")
        frac_var_explained = float("nan")
    return {
        "n": int(m.sum()),
        "corr_path_scale_vs_innov": c,
        "lin_gain_a_path_eq_a_times_innov": a,
        "frac_var_path_explained_by_innov": frac_var_explained,
        "mean_path_per_innov": float(np.mean(ppi)),
        "median_path_per_innov": float(np.median(ppi)),
        "std_path_per_innov": float(np.std(ppi)),
        "sum_total": float(tot.sum()),
        "sum_via_vel": float(vv.sum()),
        "sum_via_att": float(va.sum()),
        "path_scale_sumabs": float(np.abs(vv).sum() + np.abs(va).sum()),
        "mean_innov": float(np.mean(y)),
        "mean_path_scale": float(np.mean(ps)),
    }


def main() -> None:
    dc = per_tick(pd.read_csv(OUT / "ctrl_nhc_block_audit.csv"))
    dl = per_tick(pd.read_csv(OUT / "latch_nhc_block_audit.csv"))

    report: dict = {"windows": {}, "r3_sub2": {}, "latch_vs_ctrl_ppi": {}, "verdict": {}}

    for arm, d in ("ctrl", dc), ("latch", dl):
        t = d["timestamp_s"].to_numpy(float)
        report["windows"][arm] = {
            w: win_report(d, mask(t, *WINDOWS[w])) for w in WINDOWS
        }
        report["r3_sub2"][arm] = {
            w: win_report(d, mask(t, *R3_SUB2[w])) for w in R3_SUB2
        }

    # Does latch inflate path_per_innov beyond innov alone?
    for w in WINDOWS:
        Lc = report["windows"]["latch"][w]
        Cc = report["windows"]["ctrl"][w]
        if Lc.get("n", 0) == 0 or Cc.get("n", 0) == 0:
            continue
        report["latch_vs_ctrl_ppi"][w] = {
            "ratio_mean_innov_L_over_C": Lc["mean_innov"] / max(Cc["mean_innov"], 1e-30),
            "ratio_mean_path_scale_L_over_C": Lc["mean_path_scale"]
            / max(Cc["mean_path_scale"], 1e-30),
            "ratio_mean_ppi_L_over_C": Lc["mean_path_per_innov"]
            / max(Cc["mean_path_per_innov"], 1e-30),
            "pearson_latch": Lc["corr_path_scale_vs_innov"]["pearson"],
            "pearson_ctrl": Cc["corr_path_scale_vs_innov"]["pearson"],
            "frac_var_explained_latch": Lc["frac_var_path_explained_by_innov"],
            "frac_var_explained_ctrl": Cc["frac_var_path_explained_by_innov"],
        }

    # Verdict on path_scale vs innov
    r2 = report["latch_vs_ctrl_ppi"].get("R2", {})
    fr = report["latch_vs_ctrl_ppi"].get("full_rise", {})
    pear_l_r2 = r2.get("pearson_latch", float("nan"))
    ppi_ratio_r2 = r2.get("ratio_mean_ppi_L_over_C", float("nan"))
    path_ratio_r2 = r2.get("ratio_mean_path_scale_L_over_C", float("nan"))
    innov_ratio_r2 = r2.get("ratio_mean_innov_L_over_C", float("nan"))
    var_exp = r2.get("frac_var_explained_latch", float("nan"))

    # TRACKS if high corr and latch path_scale ratio ≈ innov ratio (ppi ~ stable)
    tracks = (
        np.isfinite(pear_l_r2)
        and pear_l_r2 >= 0.85
        and np.isfinite(ppi_ratio_r2)
        and 0.7 <= ppi_ratio_r2 <= 1.4
    )
    diverges = (
        np.isfinite(ppi_ratio_r2)
        and (ppi_ratio_r2 > 1.5 or ppi_ratio_r2 < 0.67)
    ) or (np.isfinite(var_exp) and var_exp < 0.5)

    if tracks and not diverges:
        label = "PATH_SCALE_TRACKS_INNOV"
        reading = (
            "Per-tick path_scale follows ‖y‖ closely in R2 under latch; "
            "path_per_innov stays ~stable vs ctrl. The R2 path_scale explosion is "
            "the same innov-magnitude explosion propagating through both H paths — "
            "not a separate K-coupling mechanism to attack. Intervention focus stays "
            "on why innov explodes at ~1.69–1.79s, not on cutting K_bias paths."
        )
    elif diverges:
        label = "PATH_SCALE_DIVERGES_FROM_INNOV"
        reading = (
            "path_scale grows more/less than ‖y‖ alone explains (path_per_innov "
            "shifts under latch). Gain-coupling / P·H geometry contributes beyond "
            "raw innov magnitude — worth a separate lever from innov itself."
        )
    else:
        label = "PATH_SCALE_PARTIAL_TRACK"
        reading = (
            "Mixed: some tracking of ‖y‖ but not clean enough to collapse the "
            "question. Report corr, ppi ratios, and R3 sub2 before designing."
        )

    # R3 further split
    r3_note = []
    for w in R3_SUB2:
        L = report["r3_sub2"]["latch"][w]
        if L.get("n", 0) == 0:
            continue
        r3_note.append(
            {
                "sub": w,
                "sum_total": L["sum_total"],
                "sum_via_vel": L["sum_via_vel"],
                "sum_via_att": L["sum_via_att"],
                "mean_innov": L["mean_innov"],
                "mean_path_scale": L["mean_path_scale"],
                "mean_ppi": L["mean_path_per_innov"],
            }
        )
    # homogeneity: sign of via_att across R3a1/a2/b1/b2
    signs = [
        np.sign(report["r3_sub2"]["latch"][w]["sum_via_att"])
        for w in R3_SUB2
        if report["r3_sub2"]["latch"][w].get("n", 0) > 0
    ]
    r3_homogeneous = len(set(float(s) for s in signs)) <= 1

    report["verdict"] = {
        "label": label,
        "reading": reading,
        "r2_ratios": {
            "pearson_path_vs_innov_latch": pear_l_r2,
            "innov_L_over_C": innov_ratio_r2,
            "path_scale_L_over_C": path_ratio_r2,
            "ppi_L_over_C": ppi_ratio_r2,
            "frac_var_explained_latch": var_exp,
        },
        "r3_sub2_homogeneous_via_att_sign": r3_homogeneous,
        "r3_sub2_latch": r3_note,
        "design": (
            "If PATH_SCALE_TRACKS_INNOV: do not attack K_bias/path_scale; return to "
            "innov explosion mechanism at 1.69–1.79. If DIVERGES: preregister gain/"
            "coupling lever separately."
        ),
    }

    # figure
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for arm, d, color in ("ctrl", dc, "C0"), ("latch", dl, "C3"):
        m = mask(d["timestamp_s"].to_numpy(float), 1.34, 2.0, "[t0,t1]")
        tt = d.loc[m, "timestamp_s"]
        axes[0].plot(tt, d.loc[m, "innov_norm_mps"], color=color, lw=1, label=arm)
        axes[1].plot(tt, d.loc[m, "path_scale_tick"], color=color, lw=1, label=arm)
        axes[2].plot(tt, d.loc[m, "path_per_innov"], color=color, lw=1, label=arm)
    for ax in axes:
        for x in (1.59, 1.74, 1.69, 1.79):
            ax.axvline(x, color="gray", ls="--", alpha=0.4)
    axes[0].set_ylabel("‖y‖")
    axes[0].legend(fontsize=8)
    axes[0].set_title("path_scale vs innov — does scale track ‖y‖?")
    axes[1].set_ylabel("|via_v|+|via_a|")
    axes[2].set_ylabel("path_scale / ‖y‖")
    axes[2].set_xlabel("t [s]")
    fig.tight_layout()
    fig_path = OUT / "fig_path_scale_vs_innov.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "path_scale_vs_innov.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )

    # md
    lines = [
        "# path_scale vs ‖y‖ — same explosion or separate coupling?",
        "",
        f"**Verdict:** `{label}`",
        "",
        reading,
        "",
        "## R2 latch vs ctrl (key)",
        "",
        f"- pearson(path_scale, ‖y‖) latch: **{pear_l_r2:.3f}**",
        f"- mean‖y‖ L/C: **{innov_ratio_r2:.2f}×**",
        f"- mean path_scale L/C: **{path_ratio_r2:.2f}×**",
        f"- mean (path_scale/‖y‖) L/C: **{ppi_ratio_r2:.2f}×**",
        f"- frac var(path) explained by ‖y‖ (latch R2): **{var_exp:.2f}**",
        "",
        "## Correlations by window",
        "",
        "| Arm | Window | n | pearson | spearman | mean ppi | frac var expl. |",
        "|-----|--------|---|---------|----------|----------|----------------|",
    ]
    for arm in ("ctrl", "latch"):
        for w in WINDOWS:
            p = report["windows"][arm][w]
            c = p["corr_path_scale_vs_innov"]
            lines.append(
                f"| {arm} | {w} | {p['n']} | {c['pearson']:.3f} | {c['spearman']:.3f} | "
                f"{p['mean_path_per_innov']:.3f} | {p['frac_var_path_explained_by_innov']:.2f} |"
            )
    lines += [
        "",
        "## Latch/ctrl ratios",
        "",
        "| Window | ‖y‖ L/C | path_scale L/C | ppi L/C |",
        "|--------|---------|----------------|---------|",
    ]
    for w, c in report["latch_vs_ctrl_ppi"].items():
        lines.append(
            f"| {w} | {c['ratio_mean_innov_L_over_C']:.2f} | "
            f"{c['ratio_mean_path_scale_L_over_C']:.2f} | "
            f"{c['ratio_mean_ppi_L_over_C']:.2f} |"
        )
    lines += [
        "",
        f"## R3 second split (homogeneous via_att sign? **{r3_homogeneous}**)",
        "",
        "| Arm | Sub | n | Σ total | Σ via_vel | Σ via_att | mean‖y‖ | mean path_scale |",
        "|-----|-----|---|---------|-----------|-----------|---------|-----------------|",
    ]
    for arm in ("ctrl", "latch"):
        for w in R3_SUB2:
            p = report["r3_sub2"][arm][w]
            lines.append(
                f"| {arm} | {w} | {p['n']} | {p['sum_total']:+.5f} | "
                f"{p['sum_via_vel']:+.5f} | {p['sum_via_att']:+.5f} | "
                f"{p['mean_innov']:.3f} | {p['mean_path_scale']:.4f} |"
            )
    lines += [
        "",
        "## Design",
        "",
        report["verdict"]["design"],
        "",
        f"Figure: `{fig_path.name}`",
        "",
    ]
    (OUT / "path_scale_vs_innov.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report["verdict"], indent=2))


if __name__ == "__main__":
    main()
