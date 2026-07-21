#!/usr/bin/env python3
"""Candidate order: (2) S conditioning first [0.40→1.10], then (1) P_att–vel if needed.

S already in NHC audit (s_cond, s_eig*, s_yy/yz/zz, s_inv_*).
P_att–vel scalar elements need rebuild if S is stable.
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
T_FIRE = 0.40
T_ONSET_END = 1.10  # before relΔ K_y0 ≥10%
T_WIDE_END = 1.54
SEED = 71
T2 = 3.736646e-6
TMAX = 0.65

S_COLS = [
    "s_yy",
    "s_yz",
    "s_zz",
    "s_inv_yy",
    "s_inv_yz",
    "s_inv_zz",
    "s_eigmin",
    "s_eigmax",
    "s_cond",
    "k_att_y0",
    "dx_att_z_raw",
    "dx_att_z_rad",
]

PATTVEL_COLS = [
    "P_pre_att_y_vn",
    "P_pre_att_y_ve",
    "P_pre_att_y_vd",
    "P_pre_att_z_vn",
    "P_pre_att_z_ve",
    "P_pre_att_z_vd",
    "P_pre_vel_att_frob",
]

sys.path.insert(0, str(ROOT))


def rel_diff(a, b):
    den = np.maximum(np.maximum(np.abs(a), np.abs(b)), 1e-30)
    return np.abs(a - b) / den


def align(C: pd.DataFrame, L: pd.DataFrame, t0: float, t1: float) -> pd.DataFrame:
    def w(df):
        m = (df["timestamp_s"] >= t0 - 1e-6) & (df["timestamp_s"] <= t1 + 1e-6)
        out = df.loc[m].copy()
        out["t_r"] = out["timestamp_s"].round(6)
        return out

    return w(C).merge(w(L), on="t_r", suffixes=("_c", "_l"))


def analyze_s(m: pd.DataFrame) -> dict:
    t = m["t_r"].to_numpy(float)
    out = {"t": t}
    for col in ("s_cond", "s_eigmin", "s_eigmax", "s_yy", "s_yz", "s_zz", "s_inv_yy", "s_inv_yz", "s_inv_zz"):
        c = m[f"{col}_c"].to_numpy(float)
        l = m[f"{col}_l"].to_numpy(float)
        out[f"{col}_c"] = c
        out[f"{col}_l"] = l
        out[f"rd_{col}"] = rel_diff(c, l)
        out[f"d_{col}"] = l - c

    ky0_c = m["k_att_y0_c"].to_numpy(float)
    ky0_l = m["k_att_y0_l"].to_numpy(float)
    out["ky0_c"] = ky0_c
    out["ky0_l"] = ky0_l
    out["rd_ky0"] = rel_diff(ky0_c, ky0_l)
    out["d_ky0"] = ky0_l - ky0_c
    out["cum_z_raw_l"] = np.cumsum(np.abs(m["dx_att_z_raw_l"].to_numpy(float)))

    # summary stats on onset window
    def stats(prefix):
        rd = out[f"rd_{prefix}"]
        return {
            "mean_relΔ": float(np.mean(rd)),
            "max_relΔ": float(np.max(rd)),
            "mean_c": float(np.mean(out[f"{prefix}_c"])),
            "mean_l": float(np.mean(out[f"{prefix}_l"])),
            "end_c": float(out[f"{prefix}_c"][-1]),
            "end_l": float(out[f"{prefix}_l"][-1]),
        }

    summary = {k: stats(k) for k in ("s_cond", "s_yy", "s_yz", "s_zz", "s_inv_yy", "s_inv_yz", "s_inv_zz")}
    # correlations of |ΔK_y0| with |ΔS_*|
    abs_dky0 = np.abs(out["d_ky0"])
    corrs = {}
    for k in ("s_cond", "s_yz", "s_inv_yz", "s_yy", "s_zz"):
        d = np.abs(out[f"d_{k}"])
        if np.std(abs_dky0) > 1e-15 and np.std(d) > 1e-15:
            corrs[f"corr_|dky0|_vs_|d_{k}|"] = float(np.corrcoef(abs_dky0, d)[0, 1])
        else:
            corrs[f"corr_|dky0|_vs_|d_{k}|"] = float("nan")

    # Absolute-scale verdict (avoid P_yy-style trap: high relΔ/corr on ~0 cross-terms).
    max_rd_cond = summary["s_cond"]["max_relΔ"]
    max_rd_syz = summary["s_yz"]["max_relΔ"]
    max_rd_sinv_yz = summary["s_inv_yz"]["max_relΔ"]
    max_abs_d_cond = float(np.max(np.abs(out["d_s_cond"])))
    max_abs_d_syz = float(np.max(np.abs(out["d_s_yz"])))
    mean_abs_syy = float(np.mean(np.abs(out["s_yy_c"])))
    syz_vs_syy = max_abs_d_syz / max(mean_abs_syy, 1e-30)
    corr_syz = corrs.get("corr_|dky0|_vs_|d_s_yz|", 0.0)

    # Material S move: cond changes ≥5% OR |Δs_yz| ≥ 5% of mean|s_yy|
    material = max_rd_cond >= 0.05 or syz_vs_syy >= 0.05
    if not material:
        label = "S_STABLE"
        reading = (
            "S diagonals/cond essentially identical latch vs ctrl; s_yz absolute "
            f"Δ max={max_abs_d_syz:.2e} ≪ mean|s_yy|={mean_abs_syy:.2e} "
            f"(relΔ peaks near-zero are noise). Candidate (2) out — proceed to P_att–vel."
        )
    elif material and abs(corr_syz) >= 0.85:
        label = "S_DIVERGES"
        reading = (
            "S moves materially latch vs ctrl and tracks ΔK_y0 — candidate (2) alive."
        )
    else:
        label = "S_MILD"
        reading = (
            "S shows some latch−ctrl difference but weak/ambiguous track of ΔK_y0; "
            "still check P_att–vel."
        )

    return {
        "series": out,
        "summary": summary,
        "corrs": corrs,
        "verdict": {
            "label": label,
            "reading": reading,
            "max_relΔ_s_cond": max_rd_cond,
            "max_relΔ_s_yz": max_rd_syz,
            "max_relΔ_s_inv_yz": max_rd_sinv_yz,
            "max_abs_d_s_yz": max_abs_d_syz,
            "max_abs_d_s_cond": max_abs_d_cond,
            "syz_delta_over_mean_syy": syz_vs_syy,
        },
    }


def analyze_pattvel(m: pd.DataFrame) -> dict:
    t = m["t_r"].to_numpy(float)
    keys = [
        "P_pre_att_y_vn",
        "P_pre_att_y_ve",
        "P_pre_att_y_vd",
        "P_pre_att_z_vn",
        "P_pre_att_z_ve",
        "P_pre_att_z_vd",
        "P_pre_vel_att_frob",
    ]
    series = {"t": t}
    summary = {}
    ky0_c = m["k_att_y0_c"].to_numpy(float)
    ky0_l = m["k_att_y0_l"].to_numpy(float)
    d_ky0 = ky0_l - ky0_c
    abs_dky0 = np.abs(d_ky0)
    corrs = {}
    for k in keys:
        c = m[f"{k}_c"].to_numpy(float)
        l = m[f"{k}_l"].to_numpy(float)
        series[f"{k}_c"] = c
        series[f"{k}_l"] = l
        series[f"d_{k}"] = l - c
        series[f"rd_{k}"] = rel_diff(c, l)
        summary[k] = {
            "mean_relΔ": float(np.mean(series[f"rd_{k}"])),
            "max_relΔ": float(np.max(series[f"rd_{k}"])),
            "end_c": float(c[-1]),
            "end_l": float(l[-1]),
            "ptp_d": float(np.ptp(l - c)),
        }
        d = np.abs(l - c)
        if np.std(abs_dky0) > 1e-15 and np.std(d) > 1e-15:
            corrs[f"corr_|dky0|_vs_|d_{k}|"] = float(np.corrcoef(abs_dky0, d)[0, 1])
        else:
            corrs[f"corr_|dky0|_vs_|d_{k}|"] = float("nan")

    # pick best correlating P_att_y–vel component
    best = max(
        (k for k in keys if k.startswith("P_pre_att_y")),
        key=lambda k: abs(corrs.get(f"corr_|dky0|_vs_|d_{k}|", 0.0)),
    )
    best_corr = corrs[f"corr_|dky0|_vs_|d_{best}|"]
    max_rd_y = max(summary[k]["max_relΔ"] for k in keys if "att_y" in k)

    if max_rd_y >= 0.20 and abs(best_corr) >= 0.85:
        label = "PATT_VEL_CARRIES_KY0"
        reading = (
            f"P_att_y–vel diverges and tracks ΔK_y0 (best {best}, corr={best_corr:+.3f}). "
            "Candidate (1): attitude–velocity cross in P moves pitch gain without P_yy."
        )
    elif max_rd_y >= 0.20:
        label = "PATT_VEL_DIVERGES_WEAK_TRACK"
        reading = (
            "P_att–vel blocks diverge but tracking of ΔK_y0 is weaker — still the "
            "leading structural candidate over S."
        )
    else:
        label = "PATT_VEL_STABLE"
        reading = (
            "P_att–vel also stable — need H rows / other P paths (e.g. bias–att)."
        )

    return {
        "series": series,
        "summary": summary,
        "corrs": corrs,
        "best_att_y_vel": best,
        "verdict": {"label": label, "reading": reading, "best_corr": best_corr},
    }


def run_arms_with_pattvel() -> None:
    from run_all_benchmarks import run_benchmark

    for name, lam, gate in (("ctrl", 0.0, None), ("latch", 1.0, T2)):
        audit = OUT / f"{name}_nhc_block_audit.csv"
        if audit.exists():
            audit.unlink()
        env = os.environ.copy()
        env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = str(audit)
        r = run_benchmark(
            f"SvsPattVel {name}",
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


def main() -> None:
    C = pd.read_csv(OUT / "ctrl_nhc_block_audit.csv")
    L = pd.read_csv(OUT / "latch_nhc_block_audit.csv")
    for col in S_COLS:
        if col not in C.columns or col not in L.columns:
            raise KeyError(f"missing S col {col}")

    m_onset = align(C, L, T_FIRE, T_ONSET_END)
    m_wide = align(C, L, T_FIRE, T_WIDE_END)
    s_onset = analyze_s(m_onset)
    s_wide = analyze_s(m_wide)

    report = {
        "windows": {"onset": [T_FIRE, T_ONSET_END], "wide": [T_FIRE, T_WIDE_END]},
        "S_onset": {
            "summary": s_onset["summary"],
            "corrs": s_onset["corrs"],
            "verdict": s_onset["verdict"],
        },
        "S_wide": {
            "summary": s_wide["summary"],
            "corrs": s_wide["corrs"],
            "verdict": s_wide["verdict"],
        },
    }

    # figure S
    sw = s_wide["series"]
    tw = sw["t"]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(tw, sw["s_cond_c"], "C0", label="ctrl s_cond")
    axes[0].plot(tw, sw["s_cond_l"], "C3", label="latch s_cond")
    axes[0].set_ylabel("s_cond")
    axes[0].legend(fontsize=8)
    axes[0].set_title("NHC S latch vs ctrl — fire→1.54")

    axes[1].plot(tw, sw["s_yz_c"], "C0", label="ctrl s_yz")
    axes[1].plot(tw, sw["s_yz_l"], "C3", label="latch s_yz")
    axes[1].plot(tw, sw["s_inv_yz_c"], "C0", ls="--", label="ctrl Sinv_yz")
    axes[1].plot(tw, sw["s_inv_yz_l"], "C3", ls="--", label="latch Sinv_yz")
    axes[1].set_ylabel("s_yz / S⁻¹_yz")
    axes[1].legend(fontsize=7, ncol=2)

    axes[2].plot(tw, sw["rd_s_cond"], "C2", label="relΔ s_cond")
    axes[2].plot(tw, sw["rd_s_yz"], "C1", label="relΔ s_yz")
    axes[2].plot(tw, sw["rd_ky0"], "k", label="relΔ K_y0")
    axes[2].axvline(T_ONSET_END, color="gray", ls="--", alpha=0.6)
    axes[2].set_ylabel("relΔ")
    axes[2].set_xlabel("t [s]")
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.axvline(T_FIRE, color="orange", ls="--", alpha=0.6)
    fig.tight_layout()
    fig_s = OUT / "fig_s_onset_ky0.png"
    fig.savefig(fig_s, dpi=140)
    plt.close(fig)
    report["figure_s"] = str(fig_s)

    # If S stable/mild → need P_att–vel columns
    need_p = s_onset["verdict"]["label"] in ("S_STABLE", "S_MILD")
    has_p = all(c in C.columns for c in PATTVEL_COLS)

    if need_p and not has_p:
        report["pattvel_status"] = "MISSING_COLS_NEED_REBUILD"
        report["verdict"] = {
            "label": s_onset["verdict"]["label"],
            "reading": s_onset["verdict"]["reading"]
            + " P_att–vel columns not in audit yet — instrument + rerun.",
            "next": "Add P[ATT_Y/Z, VEL_*] to NHC audit; rerun ctrl/latch; re-run this script.",
        }
    elif need_p and has_p:
        p_onset = analyze_pattvel(m_onset)
        p_wide = analyze_pattvel(m_wide)
        report["P_attvel_onset"] = {
            "summary": p_onset["summary"],
            "corrs": p_onset["corrs"],
            "verdict": p_onset["verdict"],
            "best": p_onset["best_att_y_vel"],
        }
        report["P_attvel_wide"] = {
            "summary": p_wide["summary"],
            "corrs": p_wide["corrs"],
            "verdict": p_wide["verdict"],
            "best": p_wide["best_att_y_vel"],
        }
        # figure P
        pw = p_wide["series"]
        fig2, axes2 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        for key, color_suffix in (
            ("P_pre_att_y_vn", "vn"),
            ("P_pre_att_y_ve", "ve"),
            ("P_pre_att_y_vd", "vd"),
        ):
            axes2[0].plot(pw["t"], pw[f"{key}_c"], label=f"ctrl {color_suffix}")
            axes2[0].plot(pw["t"], pw[f"{key}_l"], ls="--", label=f"latch {color_suffix}")
        axes2[0].set_ylabel("P[ATT_Y,VEL]")
        axes2[0].legend(fontsize=6, ncol=3)
        axes2[0].set_title("P_att_y–vel latch vs ctrl")

        for key in ("P_pre_att_z_vn", "P_pre_att_z_ve", "P_pre_att_z_vd"):
            axes2[1].plot(pw["t"], pw[f"{key}_c"], label=key[-2:] + "_c")
            axes2[1].plot(pw["t"], pw[f"{key}_l"], ls="--", label=key[-2:] + "_l")
        axes2[1].set_ylabel("P[ATT_Z,VEL]")
        axes2[1].legend(fontsize=6, ncol=3)

        axes2[2].plot(pw["t"], rel_diff(m_wide["k_att_y0_c"], m_wide["k_att_y0_l"]), "k", label="relΔ K_y0")
        best = p_wide["best_att_y_vel"]
        axes2[2].plot(pw["t"], pw[f"rd_{best}"], "C3", label=f"relΔ {best}")
        axes2[2].plot(pw["t"], pw["rd_P_pre_vel_att_frob"], "C2", ls="--", label="relΔ ‖P_va‖F")
        axes2[2].legend(fontsize=8)
        axes2[2].set_xlabel("t [s]")
        for ax in axes2:
            ax.axvline(T_FIRE, color="orange", ls="--", alpha=0.6)
            ax.axvline(T_ONSET_END, color="gray", ls="--", alpha=0.6)
        fig2.tight_layout()
        fig_p = OUT / "fig_pattvel_onset_ky0.png"
        fig2.savefig(fig_p, dpi=140)
        plt.close(fig2)
        report["figure_p"] = str(fig_p)
        report["verdict"] = {
            "label": f"{s_onset['verdict']['label']}__{p_onset['verdict']['label']}",
            "reading": s_onset["verdict"]["reading"] + " " + p_onset["verdict"]["reading"],
            "next": "If PATT_VEL carries: freeze which component; design implication = open-Z keeps P_av bounded.",
        }
    else:
        # S diverges — stop at S
        report["verdict"] = {
            "label": s_onset["verdict"]["label"],
            "reading": s_onset["verdict"]["reading"],
            "next": "Decompose which of s_yz / S⁻¹ / cond tracks ΔK_y0; defer P_att–vel.",
        }

    # write ticks csv for S onset
    pd.DataFrame(
        {
            "t": s_onset["series"]["t"],
            "s_cond_c": s_onset["series"]["s_cond_c"],
            "s_cond_l": s_onset["series"]["s_cond_l"],
            "s_yz_c": s_onset["series"]["s_yz_c"],
            "s_yz_l": s_onset["series"]["s_yz_l"],
            "rd_s_cond": s_onset["series"]["rd_s_cond"],
            "rd_s_yz": s_onset["series"]["rd_s_yz"],
            "rd_ky0": s_onset["series"]["rd_ky0"],
            "d_ky0": s_onset["series"]["d_ky0"],
        }
    ).to_csv(OUT / "s_onset_ky0_ticks.csv", index=False)

    (OUT / "s_vs_pattvel_onset.json").write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")

    # markdown
    vo = report["verdict"]
    lines = [
        "# S vs P_att–vel — onset [0.40→1.10] (wide to 1.54)",
        "",
        f"**Verdict:** `{vo['label']}`",
        "",
        vo["reading"],
        "",
        f"**Next:** {vo['next']}",
        "",
        "## S — onset summary",
        "",
        "| qty | mean relΔ | max relΔ | end ctrl | end latch |",
        "|-----|-----------|----------|----------|-----------|",
    ]
    for k, st in s_onset["summary"].items():
        lines.append(
            f"| {k} | {st['mean_relΔ']:.4f} | {st['max_relΔ']:.4f} | "
            f"{st['end_c']:+.4e} | {st['end_l']:+.4e} |"
        )
    lines += ["", "### corrs |ΔK_y0| vs |ΔS| (onset)", ""]
    for k, v in s_onset["corrs"].items():
        lines.append(f"- {k} = **{v:+.3f}**")

    if "P_attvel_onset" in report:
        lines += ["", "## P_att–vel — onset summary", ""]
        lines += [
            "| qty | mean relΔ | max relΔ | ptp(Δ) | corr|dky0| |",
            "|-----|-----------|----------|--------|-----------|",
        ]
        for k, st in report["P_attvel_onset"]["summary"].items():
            corr = report["P_attvel_onset"]["corrs"].get(f"corr_|dky0|_vs_|d_{k}|", float("nan"))
            lines.append(
                f"| {k} | {st['mean_relΔ']:.4f} | {st['max_relΔ']:.4f} | "
                f"{st['ptp_d']:+.3e} | {corr:+.3f} |"
            )

    lines += ["", f"Figure S: `{fig_s.name}`", ""]
    if "figure_p" in report:
        lines.append(f"Figure P: `{Path(report['figure_p']).name}`")
    (OUT / "s_vs_pattvel_onset.md").write_text("\n".join(lines), encoding="utf-8")

    print(json.dumps(report["verdict"], indent=2))
    print("--- S onset ---")
    print(json.dumps(report["S_onset"], indent=2))
    if "P_attvel_onset" in report:
        print("--- P onset ---")
        print(json.dumps(report["P_attvel_onset"]["verdict"], indent=2))
        print(json.dumps(report["P_attvel_onset"]["corrs"], indent=2))
    elif report.get("pattvel_status"):
        print(report["pattvel_status"])


if __name__ == "__main__":
    if "--rerun-arms" in sys.argv:
        run_arms_with_pattvel()
    main()
