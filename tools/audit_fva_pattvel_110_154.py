#!/usr/bin/env python3
"""Does per-tick ΔP[ATT_Y,VEL_N] track f_va[VN,ATT_Y] in [1.10→1.54]?

Hypothesis: same f_va that dirties vel_NED also builds P_att–vel in predict;
under latch Joseph no longer clips via dx_z → cross cov grows → K_y0 moves.

ALWAYS report corr WITH absolute magnitudes (protocol lesson CORR_ABS_SCALE).
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"
T0, T1 = 1.10, 1.54
SEED = 71
T2 = 3.736646e-6
TMAX = 0.65

NEED = [
    "P_pre_att_y_vn",
    "P_post_att_y_vn",
    "f_va_vn_atty",
    "f_va_vn_attx",
    "f_va_vn_attz",
    "a_nav_n",
    "a_nav_e",
    "k_att_y0",
    "dx_att_z_rad",
    "dx_att_z_raw",
]

sys.path.insert(0, str(ROOT))
from run_all_benchmarks import run_benchmark  # noqa: E402


def run_arm(name: str, *, lam: float, gate: float | None) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    audit = OUT / f"{name}_nhc_block_audit.csv"
    if audit.exists():
        audit.unlink()
    env = os.environ.copy()
    env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = str(audit)
    r = run_benchmark(
        f"FvaPattVel {name}",
        "SLALOM",
        seed=SEED,
        imu_mode="ideal",
        nhc_jacobian="correct",
        nhc_att_z_forget=lam if gate else 0.0,
        nhc_att_z_forget_gate=gate if gate else 0.0,
        nhc_att_z_forget_tmax=TMAX if gate else None,
        archive_suffix=f"pattbias_{name}_s{SEED}",
        env=env,
    )
    if r.error:
        raise RuntimeError(r.error)
    return audit


def safe_corr(a, b):
    if len(a) < 3 or np.std(a) < 1e-18 or np.std(b) < 1e-18:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def arm_series(arm: str) -> pd.DataFrame:
    d = pd.read_csv(OUT / f"{arm}_nhc_block_audit.csv")
    miss = [c for c in NEED if c not in d.columns]
    if miss:
        raise KeyError(f"{arm} missing {miss}")
    m = (d["timestamp_s"] >= T0 - 0.02) & (d["timestamp_s"] <= T1 + 0.005)
    w = d.loc[m].copy().reset_index(drop=True)
    t = w["timestamp_s"].to_numpy(float)
    p_pre = w["P_pre_att_y_vn"].to_numpy(float)
    p_post = w["P_post_att_y_vn"].to_numpy(float)
    fva = w["f_va_vn_atty"].to_numpy(float)  # F[VEL_N, ATT_Y], includes −dt

    # Joseph cut this tick
    d_joseph = p_post - p_pre
    # Predict rebuild proxy: P_pre[t] − P_post[t−1]
    d_pred = np.zeros_like(p_pre)
    d_pred[0] = np.nan
    d_pred[1:] = p_pre[1:] - p_post[:-1]
    # Net tick-to-tick on pre
    d_net = np.zeros_like(p_pre)
    d_net[0] = np.nan
    d_net[1:] = p_pre[1:] - p_pre[:-1]

    out = pd.DataFrame(
        {
            "t": t,
            "P_pre_ayn": p_pre,
            "P_post_ayn": p_post,
            "dP_joseph": d_joseph,
            "dP_predict": d_pred,
            "dP_net_pre": d_net,
            "f_va_vn_atty": fva,
            "f_va_vn_attx": w["f_va_vn_attx"].to_numpy(float),
            "f_va_vn_attz": w["f_va_vn_attz"].to_numpy(float),
            "a_nav_n": w["a_nav_n"].to_numpy(float),
            "a_nav_e": w["a_nav_e"].to_numpy(float),
            "k_y0": w["k_att_y0"].to_numpy(float),
            "dx_z": w["dx_att_z_rad"].to_numpy(float),
            "dx_z_raw": w["dx_att_z_raw"].to_numpy(float),
        }
    )
    out = out[(out["t"] >= T0) & (out["t"] <= T1)].reset_index(drop=True)
    out.to_csv(OUT / f"fva_pattvel_{arm}_ticks.csv", index=False)
    return out


def corr_pack(name: str, x: np.ndarray, y: np.ndarray) -> dict:
    """Correlation with mandatory absolute-scale fields."""
    mask = np.isfinite(x) & np.isfinite(y)
    x, y = x[mask], y[mask]
    return {
        "pair": name,
        "n": int(len(x)),
        "pearson": safe_corr(x, y),
        "mean_abs_x": float(np.mean(np.abs(x))) if len(x) else float("nan"),
        "mean_abs_y": float(np.mean(np.abs(y))) if len(y) else float("nan"),
        "max_abs_x": float(np.max(np.abs(x))) if len(x) else float("nan"),
        "max_abs_y": float(np.max(np.abs(y))) if len(y) else float("nan"),
        "ptp_x": float(np.ptp(x)) if len(x) else float("nan"),
        "ptp_y": float(np.ptp(y)) if len(y) else float("nan"),
        # scale ratio: is |x| material vs |y|? (not dimensionless — diagnostic only)
        "mean_abs_x_over_mean_abs_y": (
            float(np.mean(np.abs(x)) / max(np.mean(np.abs(y)), 1e-30)) if len(x) else float("nan")
        ),
    }


def analyze_arm(arm: str, df: pd.DataFrame) -> dict:
    fva = df["f_va_vn_atty"].to_numpy(float)
    d_pred = df["dP_predict"].to_numpy(float)
    d_jos = df["dP_joseph"].to_numpy(float)
    d_net = df["dP_net_pre"].to_numpy(float)
    # product proxy: f_va * sign consistency (first-order Φ term scale)
    # Also |a| as driver of |f_va|
    a_h = np.hypot(df["a_nav_n"].to_numpy(float), df["a_nav_e"].to_numpy(float))

    packs = [
        corr_pack("dP_predict vs f_va_vn_atty", d_pred, fva),
        corr_pack("dP_predict vs |f_va|", d_pred, np.abs(fva)),
        corr_pack("dP_net_pre vs f_va_vn_atty", d_net, fva),
        corr_pack("dP_joseph vs f_va_vn_atty", d_jos, fva),
        corr_pack("|dP_predict| vs |f_va|", np.abs(d_pred), np.abs(fva)),
        corr_pack("|dP_predict| vs |a|_h", np.abs(d_pred), a_h),
        corr_pack("dP_joseph vs dx_z_applied", d_jos, df["dx_z"].to_numpy(float)),
        corr_pack("dP_joseph vs dx_z_raw", d_jos, df["dx_z_raw"].to_numpy(float)),
    ]
    return {
        "corr_packs": packs,
        "mean_abs_f_va_vn_atty": float(np.mean(np.abs(fva))),
        "mean_abs_dP_predict": float(np.nanmean(np.abs(d_pred))),
        "mean_abs_dP_joseph": float(np.mean(np.abs(d_jos))),
        "sum_dP_predict": float(np.nansum(d_pred)),
        "sum_dP_joseph": float(np.sum(d_jos)),
        "end_P_pre": float(df["P_pre_ayn"].iloc[-1]),
        "start_P_pre": float(df["P_pre_ayn"].iloc[0]),
        "end_k_y0": float(df["k_y0"].iloc[-1]),
        "start_k_y0": float(df["k_y0"].iloc[0]),
    }


def main() -> None:
    reuse = "--reuse-audit" in sys.argv
    if not reuse:
        print("Running ctrl…")
        run_arm("ctrl", lam=0.0, gate=None)
        print("Running latch…")
        run_arm("latch", lam=1.0, gate=T2)

    arms = {a: arm_series(a) for a in ("ctrl", "latch")}
    stats = {a: analyze_arm(a, df) for a, df in arms.items()}

    # Latch−ctrl: does Δ(dP_predict) track? And is joseph cut weaker under latch?
    L, C = arms["latch"], arms["ctrl"]
    # align
    L = L.copy()
    C = C.copy()
    L["t_r"] = L["t"].round(6)
    C["t_r"] = C["t"].round(6)
    m = L.merge(C, on="t_r", suffixes=("_l", "_c"))
    d_pred_L = m["dP_predict_l"].to_numpy(float)
    d_pred_C = m["dP_predict_c"].to_numpy(float)
    d_jos_L = m["dP_joseph_l"].to_numpy(float)
    d_jos_C = m["dP_joseph_c"].to_numpy(float)
    fva_L = m["f_va_vn_atty_l"].to_numpy(float)
    fva_C = m["f_va_vn_atty_c"].to_numpy(float)

    # f_va should be ~same (same a, attitude slowly diverges) — check
    fva_cmp = corr_pack("f_va latch vs ctrl", fva_L, fva_C)
    # Excess predict growth latch vs ctrl
    excess_pred = d_pred_L - d_pred_C
    excess_jos = d_jos_L - d_jos_C  # less negative cut under latch?
    pack_excess = corr_pack("excess_dP_predict vs f_va_latch", excess_pred, fva_L)

    # Materiality: mean|dP_predict| vs mean|P| and vs mean|f_va|
    def material(arm_stats):
        mp = arm_stats["mean_abs_dP_predict"]
        mf = arm_stats["mean_abs_f_va_vn_atty"]
        # dP and f_va have different units (P is cov, f_va is dimensionless-ish *P scale)
        # Compare joseph cut magnitude latch vs ctrl
        return {
            "mean_abs_dP_predict": mp,
            "mean_abs_dP_joseph": arm_stats["mean_abs_dP_joseph"],
            "mean_abs_f_va": mf,
            "P_growth_pre": arm_stats["end_P_pre"] - arm_stats["start_P_pre"],
        }

    # Primary pack for latch
    latch_pred_fva = next(
        p for p in stats["latch"]["corr_packs"] if p["pair"] == "dP_predict vs f_va_vn_atty"
    )
    latch_abs = next(
        p for p in stats["latch"]["corr_packs"] if p["pair"] == "|dP_predict| vs |f_va|"
    )
    ctrl_pred_fva = next(
        p for p in stats["ctrl"]["corr_packs"] if p["pair"] == "dP_predict vs f_va_vn_atty"
    )

    # Joseph cut comparison: latch should cut less (sum_joseph closer to 0 or less negative relative to growth)
    jos_ratio = stats["latch"]["mean_abs_dP_joseph"] / max(
        stats["ctrl"]["mean_abs_dP_joseph"], 1e-30
    )

    # Verdict with scale gate
    # Pass material: mean|dP_predict| > 1e-6 (cov units) AND |pearson|>=0.7
    # AND not "f_va identical but dP differs only by joseph" without f_va track
    mat_ok = latch_pred_fva["mean_abs_x"] >= 1e-6 and latch_pred_fva["mean_abs_y"] >= 1e-6
    corr_ok = abs(latch_pred_fva["pearson"]) >= 0.70 or abs(latch_abs["pearson"]) >= 0.70
    # Joseph weaker under latch?
    joseph_weaker = jos_ratio < 0.85 or (
        abs(stats["latch"]["sum_dP_joseph"]) < abs(stats["ctrl"]["sum_dP_joseph"]) * 0.85
    )

    if mat_ok and corr_ok and joseph_weaker:
        label = "FVA_BUILDS_PATTVEL_JOSEPH_UNDERCLIPS"
        reading = (
            "Per-tick predict rebuild of P[ATT_Y,VN] tracks f_va[VN,ATT_Y] with "
            "material absolute scale; under latch the Joseph cut on that block is "
            "weaker than ctrl — unified f_va mechanism (state dirt + cov cross growth)."
        )
    elif mat_ok and corr_ok:
        label = "FVA_TRACKS_DP_JOSEPH_UNCLEAR"
        reading = (
            "dP_predict tracks f_va with material scale, but Joseph cut latch vs ctrl "
            "is not clearly weaker — partial support for unified f_va→P_av story."
        )
    elif corr_ok and not mat_ok:
        label = "FVA_CORR_SCALE_TRAP"
        reading = (
            "Correlation present but absolute |dP| or |f_va| below materiality — "
            "another CORR_ABS_SCALE trap; do not claim f_va builds P_av from this."
        )
    else:
        label = "FVA_NOT_PATTVEL_DRIVER"
        reading = (
            "dP[ATT_Y,VN] does not track f_va materially — look elsewhere for "
            "P_av growth (other Φ terms / H / multi-path FPFᵀ)."
        )

    report = {
        "window_s": [T0, T1],
        "index_note": "f_va[i][j] = discrete F row vel_i, col att_j (includes −dt); "
        "f_va_vn_atty = f_va[0][1] for P[ATT_Y,VEL_N] pair.",
        "method_lesson": "CORR_ABS_SCALE: every pearson reported with mean|x|, mean|y|, max|x|, max|y|.",
        "arms": {a: {k: v for k, v in s.items()} for a, s in stats.items()},
        "f_va_latch_vs_ctrl": fva_cmp,
        "excess_predict_vs_fva": pack_excess,
        "joseph_abs_ratio_latch_over_ctrl": jos_ratio,
        "materiality": {a: material(stats[a]) for a in ("ctrl", "latch")},
        "verdict": {
            "label": label,
            "reading": reading,
            "latch_pearson_dPpred_fva": latch_pred_fva["pearson"],
            "latch_mean_abs_dPpred": latch_pred_fva["mean_abs_x"],
            "latch_mean_abs_fva": latch_pred_fva["mean_abs_y"],
            "ctrl_pearson_dPpred_fva": ctrl_pred_fva["pearson"],
            "joseph_weaker_under_latch": joseph_weaker,
        },
    }

    # figure
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    t = m["t_r"].to_numpy(float)
    axes[0].plot(t, m["P_pre_ayn_c"], "C0", label="ctrl P_pre[Ay,Vn]")
    axes[0].plot(t, m["P_pre_ayn_l"], "C3", label="latch P_pre[Ay,Vn]")
    axes[0].legend(fontsize=8)
    axes[0].set_ylabel("P")
    axes[0].set_title("P[ATT_Y,VEL_N] + f_va [1.10→1.54]")

    axes[1].plot(t, m["dP_predict_c"], "C0", label="ctrl dP_predict")
    axes[1].plot(t, m["dP_predict_l"], "C3", label="latch dP_predict")
    axes[1].plot(t, m["dP_joseph_c"], "C0", ls="--", label="ctrl dP_joseph")
    axes[1].plot(t, m["dP_joseph_l"], "C3", ls="--", label="latch dP_joseph")
    axes[1].axhline(0, color="gray", lw=0.4)
    axes[1].legend(fontsize=7, ncol=2)
    axes[1].set_ylabel("ΔP / tick")

    axes[2].plot(t, m["f_va_vn_atty_c"], "C0", label="ctrl f_va[Vn,Ay]")
    axes[2].plot(t, m["f_va_vn_atty_l"], "C3", label="latch f_va[Vn,Ay]")
    axes[2].legend(fontsize=8)
    axes[2].set_ylabel("f_va")

    axes[3].plot(t, m["k_y0_c"], "C0", label="ctrl K_y0")
    axes[3].plot(t, m["k_y0_l"], "C3", label="latch K_y0")
    axes[3].axhline(0, color="gray", lw=0.4)
    axes[3].legend(fontsize=8)
    axes[3].set_ylabel("K_y0")
    axes[3].set_xlabel("t [s]")
    fig.tight_layout()
    fig_path = OUT / "fig_fva_pattvel_110_154.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "fva_pattvel_110_154.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )

    def fmt_pack(p):
        return (
            f"| {p['pair']} | {p['pearson']:+.3f} | {p['mean_abs_x']:.3e} | "
            f"{p['mean_abs_y']:.3e} | {p['max_abs_x']:.3e} | {p['max_abs_y']:.3e} |"
        )

    lines = [
        "# f_va vs ΔP[ATT_Y,VEL_N] [1.10→1.54]",
        "",
        f"**Verdict:** `{label}`",
        "",
        reading,
        "",
        "**CORR_ABS_SCALE:** toda correlación abajo lleva mean|·| y max|·|.",
        "",
        f"Index: `f_va_vn_atty` = F discrete [VEL_N, ATT_Y] (= `f_va[0][1]`, incl. −dt).",
        f"`dP_predict` = P_pre[t] − P_post[t−1] (rebuild entre NHC).",
        "",
        "## Latch corr packs",
        "",
        "| pair | pearson | mean\\|x\\| | mean\\|y\\| | max\\|x\\| | max\\|y\\| |",
        "|------|---------|---------|---------|--------|--------|",
    ]
    for p in stats["latch"]["corr_packs"]:
        lines.append(fmt_pack(p))
    lines += [
        "",
        "## Ctrl corr packs (primary)",
        "",
        "| pair | pearson | mean\\|x\\| | mean\\|y\\| | max\\|x\\| | max\\|y\\| |",
        "|------|---------|---------|---------|--------|--------|",
        fmt_pack(ctrl_pred_fva),
        fmt_pack(next(p for p in stats["ctrl"]["corr_packs"] if p["pair"] == "|dP_predict| vs |f_va|")),
        "",
        "## Arm budgets",
        "",
        "| Arm | Σ dP_predict | Σ dP_joseph | mean\\|dP_pred\\| | mean\\|dP_jos\\| | ΔP_pre |",
        "|-----|--------------|-------------|-----------------|----------------|--------|",
    ]
    for a in ("ctrl", "latch"):
        s = stats[a]
        lines.append(
            f"| {a} | {s['sum_dP_predict']:+.3e} | {s['sum_dP_joseph']:+.3e} | "
            f"{s['mean_abs_dP_predict']:.3e} | {s['mean_abs_dP_joseph']:.3e} | "
            f"{s['end_P_pre'] - s['start_P_pre']:+.3e} |"
        )
    lines += [
        "",
        f"- f_va latch vs ctrl pearson = {fva_cmp['pearson']:+.3f} "
        f"(mean|L|={fva_cmp['mean_abs_x']:.3e}, mean|C|={fva_cmp['mean_abs_y']:.3e})",
        f"- mean\\|dP_joseph\\| latch/ctrl = {jos_ratio:.3f}",
        f"- excess dP_predict vs f_va_L: pearson={pack_excess['pearson']:+.3f}, "
        f"mean|excess|={pack_excess['mean_abs_x']:.3e}",
        "",
        f"Figure: `{fig_path.name}`",
        "",
    ]
    (OUT / "fva_pattvel_110_154.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report["verdict"], indent=2))
    print("---")
    print(json.dumps(stats["latch"]["corr_packs"][:3], indent=2))
    print("joseph_ratio", jos_ratio)
    print("fva_cmp", fva_cmp)


if __name__ == "__main__":
    main()
