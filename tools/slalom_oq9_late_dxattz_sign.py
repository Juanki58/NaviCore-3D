#!/usr/bin/env python3
"""OQ9: dx_att_z sign persistence A vs C in late 14–25 s (same check as tick-0 / early).

Distinguishes "same persistent H_att-sign loop" vs "something else that also grows".
Does NOT rename FEEDBACK_CONTINUES_OMEGA_DECOUPLED — data only.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "docs" / "benchmarks" / "jacobian_imu_ab"
AUDIT_A = OUT / "slalom_cellA_jcorrect_imuideal_s71_nhc_block_audit.csv"
AUDIT_C = OUT / "slalom_cellC_jlegacy_imuideal_s71_nhc_block_audit.csv"
PROV = OUT / "slalom_a_vs_c_kp_postfix_provenance.json"

OUT_JSON = OUT / "slalom_oq9_late_dxattz_sign.json"
OUT_MD = OUT / "slalom_oq9_late_dxattz_sign.md"
OUT_PNG = OUT / "fig_slalom_oq9_late_dxattz_sign.png"

WINDOWS = {
    "early_0_0p75s": (0.0, 0.75),
    "early_0_4s": (0.0, 4.0),
    "mid_4_14s": (4.0, 14.0),
    "late_14_25s": (14.0, 25.0),
    "late_22_25s": (22.0, 25.0),
}


def load_merged() -> pd.DataFrame:
    a = pd.read_csv(AUDIT_A)
    c = pd.read_csv(AUDIT_C)
    a["t_ms"] = (a["timestamp_s"] * 1000).round().astype(int)
    c["t_ms"] = (c["timestamp_s"] * 1000).round().astype(int)
    return a.merge(c, on="t_ms", suffixes=("_A", "_C"))


def sign_metrics(za: np.ndarray, zc: np.ndarray) -> dict:
    # ignore exact zeros for opposite/same
    nz = (np.abs(za) > 0) & (np.abs(zc) > 0)
    if nz.sum() < 5:
        return {
            "n": int(len(za)),
            "n_nonzero_both": int(nz.sum()),
            "opposite_frac": None,
            "same_frac": None,
            "sign_corr": None,
            "ratio_median": None,
            "ratio_near_minus1_frac": None,
        }
    sa = np.sign(za[nz])
    sc = np.sign(zc[nz])
    opp = float(np.mean(sa * sc < 0))
    same = float(np.mean(sa * sc > 0))
    # product ratio za/zc — for exact negation ≈ -1
    ratio = za[nz] / zc[nz]
    near_m1 = float(np.mean(np.abs(ratio + 1.0) < 0.25))  # within 25% of -1
    if np.std(za[nz]) > 0 and np.std(zc[nz]) > 0:
        # Pearson on signed values
        corr = float(np.corrcoef(za[nz], zc[nz])[0, 1])
    else:
        corr = float("nan")
    return {
        "n": int(len(za)),
        "n_nonzero_both": int(nz.sum()),
        "opposite_frac": opp,
        "same_frac": same,
        "sign_corr": corr,
        "ratio_median": float(np.median(ratio)),
        "ratio_p10": float(np.percentile(ratio, 10)),
        "ratio_p90": float(np.percentile(ratio, 90)),
        "ratio_near_minus1_frac": near_m1,
        "rms_A": float(np.sqrt(np.mean(za**2))),
        "rms_C": float(np.sqrt(np.mean(zc**2))),
        "rms_A_over_C": float(np.sqrt(np.mean(za**2)) / max(np.sqrt(np.mean(zc**2)), 1e-30)),
    }


def analyze_window(m: pd.DataFrame, t0: float, t1: float) -> dict:
    t = m["timestamp_s_A"].to_numpy(dtype=float)
    w = m[(t >= t0) & (t <= t1)].reset_index(drop=True)
    tw = w["timestamp_s_A"].to_numpy(dtype=float)
    za = w["dx_att_z_rad_A"].to_numpy(dtype=float)
    zc = w["dx_att_z_rad_C"].to_numpy(dtype=float)
    sm = sign_metrics(za, zc)

    cum_a = np.cumsum(za)
    cum_c = np.cumsum(zc)
    sep = cum_a - cum_c
    abs_sep = np.abs(sep)

    # Growth shape of |cum sep| — same spirit as early FEEDBACK_GROWTH check
    if len(tw) > 10:
        mid = 0.5 * (t0 + t1)
        se = abs_sep
        dsep = np.diff(se)
        dt = np.diff(tw)
        rate = dsep / np.maximum(dt, 1e-9)
        te_r = tw[1:]
        rate_1 = float(np.mean(rate[te_r <= mid])) if np.any(te_r <= mid) else float("nan")
        rate_2 = float(np.mean(rate[te_r > mid])) if np.any(te_r > mid) else float("nan")
        rate_ratio = (
            float(rate_2 / rate_1) if np.isfinite(rate_1) and abs(rate_1) > 1e-30 else float("nan")
        )
        coef = np.polyfit(tw - tw[0], se, 1)
        se_hat = coef[0] * (tw - tw[0]) + coef[1]
        ss_res = float(np.sum((se - se_hat) ** 2))
        ss_tot = float(np.sum((se - np.mean(se)) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    else:
        rate_1 = rate_2 = rate_ratio = r2 = float("nan")

    # Does |dx_att_z| dominate attitude correction energy vs dx_att_x/y?
    dax_a = w["dx_att_x_rad_A"].to_numpy(dtype=float)
    day_a = w["dx_att_y_rad_A"].to_numpy(dtype=float)
    e_z = float(np.sum(za**2))
    e_xy = float(np.sum(dax_a**2 + day_a**2))
    e_tot = e_z + e_xy

    # Coupling to P_pp_C growth: corr(|dx_att_z_A|, dP_pp_C/dt) in window
    ppc = w["P_pre_pp_frob_C"].to_numpy(dtype=float)
    if len(tw) > 5:
        dppc_dt = np.gradient(ppc, tw)
        if np.std(np.abs(za)) > 0 and np.std(dppc_dt) > 0:
            corr_dz_dPpc = float(np.corrcoef(np.abs(za), dppc_dt)[0, 1])
        else:
            corr_dz_dPpc = float("nan")
        innov = w["innov_norm_mps_A"].to_numpy(dtype=float)
        if np.std(innov) > 0 and np.std(dppc_dt) > 0:
            corr_innov_dPpc = float(np.corrcoef(innov, dppc_dt)[0, 1])
        else:
            corr_innov_dPpc = float("nan")
    else:
        corr_dz_dPpc = corr_innov_dPpc = float("nan")

    return {
        "t0": t0,
        "t1": t1,
        "sign": sm,
        "cum_sep_start": float(abs_sep[0]) if len(abs_sep) else None,
        "cum_sep_end": float(abs_sep[-1]) if len(abs_sep) else None,
        "cum_sep_end_over_start": (
            float(abs_sep[-1] / max(abs_sep[0], 1e-30)) if len(abs_sep) else None
        ),
        "cum_A_end": float(cum_a[-1]) if len(cum_a) else None,
        "cum_C_end": float(cum_c[-1]) if len(cum_c) else None,
        "mean_dabs_sep_dt_first_half": rate_1,
        "mean_dabs_sep_dt_second_half": rate_2,
        "rate_ratio_second_over_first": rate_ratio,
        "r2_linear_abs_sep_vs_t": r2,
        "att_energy_frac_z_A": float(e_z / max(e_tot, 1e-30)),
        "corr_abs_dx_att_z_A_vs_dP_pp_C_dt": corr_dz_dPpc,
        "corr_innov_A_vs_dP_pp_C_dt": corr_innov_dPpc,
        "series": {
            "t_s": tw.tolist(),
            "dx_att_z_A": za.tolist(),
            "dx_att_z_C": zc.tolist(),
            "cum_A": cum_a.tolist(),
            "cum_C": cum_c.tolist(),
            "abs_sep": abs_sep.tolist(),
        },
    }


def classify_late(early: dict, late: dict) -> dict:
    """Data-driven tags — not a verdict rename."""
    es = early["sign"]
    ls = late["sign"]
    # Early reference: high opposite, ratio near -1, strong A>>C rms
    early_persistent = bool(
        es.get("opposite_frac") is not None
        and es["opposite_frac"] >= 0.6
        and es.get("ratio_near_minus1_frac", 0) >= 0.3
    )
    late_opp = ls.get("opposite_frac")
    late_near = ls.get("ratio_near_minus1_frac")
    late_corr = ls.get("sign_corr")

    if late_opp is None:
        tag = "INCONCLUSIVE"
        note = "insufficient nonzero samples"
    elif late_opp >= 0.6 and late_near is not None and late_near >= 0.25:
        tag = "SIGN_STILL_OPPOSITE_PERSISTENT"
        note = (
            "Late dx_att_z remains mostly opposite A vs C with ratios near −1 — "
            "compatible with same persistent H_att-sign mechanism (not proven alone)."
        )
    elif late_opp >= 0.6 and (late_near is None or late_near < 0.25):
        tag = "SIGN_OPPOSITE_BUT_RATIO_NOT_LOCKED"
        note = (
            "Opposite signs often, but za/zc not clustered near −1 — "
            "sign family may persist without the early lockstep negation."
        )
    elif late_opp < 0.4:
        tag = "SIGN_NO_LONGER_OPPOSITE"
        note = (
            "Opposite-sign fraction collapsed — weak evidence for the same "
            "persistent H_att-sign loop; other drivers of late P_pp growth more plausible."
        )
    else:
        tag = "SIGN_MIXED"
        note = "Intermediate opposite frac — do not claim same-loop or new-mechanism yet."

    # Dominance: is att energy still in z? is z coupled to P_pp_C growth?
    z_dom = late.get("att_energy_frac_z_A", 0) >= 0.5
    coupled = (
        late.get("corr_abs_dx_att_z_A_vs_dP_pp_C_dt") is not None
        and abs(late["corr_abs_dx_att_z_A_vs_dP_pp_C_dt"]) >= 0.3
    )

    return {
        "early_looked_persistent_sign": early_persistent,
        "late_sign_tag": tag,
        "late_sign_note": note,
        "late_att_energy_still_mostly_z": bool(z_dom),
        "late_abs_dx_att_z_coupled_to_dP_pp_C": bool(coupled),
        "rename_verdict": False,
        "discipline": (
            "Do not rename FEEDBACK_CONTINUES_OMEGA_DECOUPLED from this file alone; "
            "report late_sign_tag as the discriminating datum."
        ),
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    prov = json.loads(PROV.read_text(encoding="utf-8")) if PROV.exists() else {}
    m = load_merged()

    windows = {}
    for name, (t0, t1) in WINDOWS.items():
        windows[name] = analyze_window(m, t0, t1)
        # drop series from non-plot windows in json later

    early = windows["early_0_0p75s"]
    late = windows["late_14_25s"]
    clf = classify_late(early, late)

    # Figure: early vs late dx_att_z and cumsep
    fig, axes = plt.subplots(3, 2, figsize=(11, 8))
    for col, key, title in (
        (0, "early_0_0p75s", "Early 0–0.75 s"),
        (1, "late_14_25s", "Late 14–25 s"),
    ):
        s = windows[key]["series"]
        t = np.asarray(s["t_s"])
        axes[0, col].plot(t, s["dx_att_z_A"], label="A", lw=0.8)
        axes[0, col].plot(t, s["dx_att_z_C"], label="C", lw=0.8)
        axes[0, col].set_title(title)
        axes[0, col].set_ylabel("dx_att_z")
        axes[0, col].legend(fontsize=7)
        axes[0, col].grid(True, alpha=0.3)

        axes[1, col].plot(t, s["cum_A"], label="ΣA")
        axes[1, col].plot(t, s["cum_C"], label="ΣC")
        axes[1, col].set_ylabel("Σ dx_att_z")
        axes[1, col].legend(fontsize=7)
        axes[1, col].grid(True, alpha=0.3)

        axes[2, col].plot(t, s["abs_sep"], color="C3")
        axes[2, col].set_ylabel("|ΣA−ΣC|")
        axes[2, col].set_xlabel("t (s)")
        axes[2, col].grid(True, alpha=0.3)

    fig.suptitle(
        f"dx_att_z sign check — late tag: {clf['late_sign_tag']}",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=140)
    plt.close(fig)

    # JSON without bulky series except early+late
    dump_windows = {}
    for name, w in windows.items():
        d = {k: v for k, v in w.items() if k != "series"}
        dump_windows[name] = d

    results = {
        "provenance_ref": str(PROV.name) if PROV.exists() else None,
        "audit_sha_A": prov.get("cells", {}).get("A", {}).get("audit_sha256_16"),
        "audit_sha_C": prov.get("cells", {}).get("C", {}).get("audit_sha256_16"),
        "windows": dump_windows,
        "classification": clf,
        "status": "DISCRIMINATING_DATUM_ONLY_NO_VERDICT_RENAME",
    }
    with OUT_JSON.open("w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    lines = [
        "# OQ9 — late dx_att_z sign (same check as early / tick 0)",
        "",
        "**No verdict rename.** Discriminating datum only for "
        "`FEEDBACK_CONTINUES_OMEGA_DECOUPLED` working hypothesis.",
        "",
        f"Audits sha A={results['audit_sha_A']}, C={results['audit_sha_C']}.  ",
        f"**Figure:** `{OUT_PNG.name}`  ",
        "",
        "## Sign metrics by window",
        "",
        "| Window | opp frac | same frac | sign corr | ratio median | "
        "frac ratio≈−1 | rms A/C |",
        "|--------|----------|-----------|-----------|--------------|"
        "----------------|---------|",
    ]
    for name in WINDOWS:
        s = windows[name]["sign"]
        lines.append(
            f"| {name} | {s.get('opposite_frac')} | {s.get('same_frac')} | "
            f"{s.get('sign_corr')} | {s.get('ratio_median')} | "
            f"{s.get('ratio_near_minus1_frac')} | {s.get('rms_A_over_C')} |"
        )

    lines += [
        "",
        "## Cumulative |ΣA−ΣC| growth",
        "",
        "| Window | |sep| start→end | end/start | rate 2nd/1st half | R² linear |",
        "|--------|----------------|-----------|------------------|-----------|",
    ]
    for name in ("early_0_0p75s", "early_0_4s", "late_14_25s", "late_22_25s"):
        w = windows[name]
        lines.append(
            f"| {name} | {w['cum_sep_start']:.4g}→{w['cum_sep_end']:.4g} | "
            f"{w['cum_sep_end_over_start']:.4g} | "
            f"{w['rate_ratio_second_over_first']:.3g} | "
            f"{w['r2_linear_abs_sep_vs_t']:.3f} |"
        )

    le = late
    lines += [
        "",
        "## Does dx_att_z still dominate / couple to P_pp_C late?",
        "",
        f"- att energy fraction in z (A), 14–25 s: "
        f"**{le['att_energy_frac_z_A']:.3f}**",
        f"- corr(|dx_att_z_A|, dP_pp_C/dt): "
        f"**{le['corr_abs_dx_att_z_A_vs_dP_pp_C_dt']:.3f}**",
        f"- corr(innov_A, dP_pp_C/dt): "
        f"**{le['corr_innov_A_vs_dP_pp_C_dt']:.3f}**",
        "",
        "## Discriminating tag (not a rename)",
        "",
        f"**`{clf['late_sign_tag']}`**",
        "",
        clf["late_sign_note"],
        "",
        f"- early looked persistent: {clf['early_looked_persistent_sign']}",
        f"- late att energy mostly z: {clf['late_att_energy_still_mostly_z']}",
        f"- late |dx_att_z| coupled to dP_pp_C: {clf['late_abs_dx_att_z_coupled_to_dP_pp_C']}",
        "",
        clf["discipline"],
        "",
        "## Reading guide",
        "",
        "- `SIGN_STILL_OPPOSITE_PERSISTENT` → strengthens same-loop reading "
        "(still not sufficient alone with P/burst checks).",
        "- `SIGN_NO_LONGER_OPPOSITE` → weakens same-loop; late P_pp growth likely "
        "needs another driver name.",
        "- `SIGN_MIXED` / `RATIO_NOT_LOCKED` → keep working hypothesis; do not "
        "preregister late success criteria yet.",
        "",
    ]
    OUT_MD.write_text("\n".join(lines), encoding="utf-8")
    print(OUT_MD.read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
