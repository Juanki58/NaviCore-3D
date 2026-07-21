#!/usr/bin/env python3
"""OQ9 follow-up: late-regime P A vs C trajectories + burstiness in 22–25 s.

Does NOT close FEEDBACK_CONTINUES_OMEGA_DECOUPLED — supplies the two missing
checks before that label can be more than a working hypothesis.
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
TELEM_A = REPO / "docs/benchmarks/slalom_cellA_jcorrect_imuideal_s71_telemetry.csv"
TELEM_C = REPO / "docs/benchmarks/slalom_cellC_jlegacy_imuideal_s71_telemetry.csv"
PROV = OUT / "slalom_a_vs_c_kp_postfix_provenance.json"

OUT_JSON = OUT / "slalom_oq9_late_p_and_burstiness.json"
OUT_MD = OUT / "slalom_oq9_late_p_and_burstiness.md"
OUT_PNG = OUT / "fig_slalom_oq9_late_p_and_burstiness.png"

# Same burstiness thresholds as slalom_a_vs_c_omega_burstiness.py / GAP-3 spirit
THRESH_TOP3_SHARE = 0.5
THRESH_B = 0.25

LATE = (14.0, 25.0)
BURST_WIN = (22.0, 25.0)
# also report finer sub-windows if bursty
EARLY_REF = (0.0, 4.0)


def burstiness_metrics(x: np.ndarray, t_s: np.ndarray) -> dict:
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
    dominating = []
    for rank, j in enumerate(top3_idx, start=1):
        dominating.append(
            {
                "rank": rank,
                "t_start_s": float(t_s[j]),
                "t_end_s": float(t_s[j + 1]),
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
        "thresholds": {"B": THRESH_B, "top3_share": THRESH_TOP3_SHARE},
        "dominating_ticks": dominating,
    }


def load_merged() -> pd.DataFrame:
    a = pd.read_csv(AUDIT_A)
    c = pd.read_csv(AUDIT_C)
    a["t_ms"] = (a["timestamp_s"] * 1000).round().astype(int)
    c["t_ms"] = (c["timestamp_s"] * 1000).round().astype(int)
    return a.merge(c, on="t_ms", suffixes=("_A", "_C"))


def load_telem(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["t_s"] = df["time_us"].astype(float) * 1e-6
    return df


def p_snapshot(m: pd.DataFrame, t0: float, t1: float) -> dict:
    t = m["timestamp_s_A"].to_numpy()
    w = m[(t >= t0) & (t <= t1)]
    if len(w) == 0:
        return {"n": 0, "t0": t0, "t1": t1}

    def mean(col):
        return float(w[col].mean())

    def end(col):
        return float(w[col].iloc[-1])

    def start(col):
        return float(w[col].iloc[0])

    out = {"n": int(len(w)), "t0": t0, "t1": t1}
    for block in ("aa", "vv", "pv", "pp"):
        ca = f"P_pre_{block}_frob_A"
        cc = f"P_pre_{block}_frob_C"
        out[f"P_{block}_A_mean"] = mean(ca)
        out[f"P_{block}_C_mean"] = mean(cc)
        out[f"P_{block}_A_start"] = start(ca)
        out[f"P_{block}_C_start"] = start(cc)
        out[f"P_{block}_A_end"] = end(ca)
        out[f"P_{block}_C_end"] = end(cc)
        out[f"P_{block}_A_growth"] = end(ca) - start(ca)
        out[f"P_{block}_C_growth"] = end(cc) - start(cc)
        out[f"P_{block}_C_over_A_mean"] = mean(cc) / max(mean(ca), 1e-12)
        out[f"P_{block}_C_over_A_end"] = end(cc) / max(end(ca), 1e-12)
    return out


def mirror_test(m: pd.DataFrame, t0: float, t1: float) -> dict:
    """Is late P_C growth the 'stabilizing mirror' of A's feedback?

    Claims to test (must be data, not slogans):
    (1) Which block 'explodes' in C: aa / vv / pv / pp?
    (2) Does A stay tight on the same block while C grows (inverse scale)?
    (3) Cumulative log-ratio log(P_C/P_A) for that block — monotonic growth?
    (4) Correlation of innov_A with P_C growth rate (loose coupling check).
    """
    t = m["timestamp_s_A"].to_numpy(dtype=float)
    mask = (t >= t0) & (t <= t1)
    w = m.loc[mask].reset_index(drop=True)
    tw = w["timestamp_s_A"].to_numpy(dtype=float)

    blocks = {}
    for block in ("aa", "vv", "pv", "pp"):
        pa = w[f"P_pre_{block}_frob_A"].to_numpy(dtype=float)
        pc = w[f"P_pre_{block}_frob_C"].to_numpy(dtype=float)
        ratio = pc / np.maximum(pa, 1e-12)
        log_ratio = np.log10(np.maximum(ratio, 1e-30))
        # growth rates
        dpc = np.diff(pc)
        dpa = np.diff(pa)
        dt = np.diff(tw)
        rate_c = dpc / np.maximum(dt, 1e-9)
        rate_a = dpa / np.maximum(dt, 1e-9)
        # Is C growing while A shrinks or stays flat?
        mean_rate_c = float(np.mean(rate_c))
        mean_rate_a = float(np.mean(rate_a))
        # Monotonicity of log_ratio: fraction of positive steps
        dlog = np.diff(log_ratio)
        frac_log_up = float(np.mean(dlog > 0))
        blocks[block] = {
            "P_A_start": float(pa[0]),
            "P_A_end": float(pa[-1]),
            "P_C_start": float(pc[0]),
            "P_C_end": float(pc[-1]),
            "P_C_over_A_start": float(ratio[0]),
            "P_C_over_A_end": float(ratio[-1]),
            "P_C_end_over_start": float(pc[-1] / max(pc[0], 1e-12)),
            "P_A_end_over_start": float(pa[-1] / max(pa[0], 1e-12)),
            "mean_dP_dt_C": mean_rate_c,
            "mean_dP_dt_A": mean_rate_a,
            "frac_log10_C_over_A_increasing": frac_log_up,
            "log10_C_over_A_start": float(log_ratio[0]),
            "log10_C_over_A_end": float(log_ratio[-1]),
        }

    # Which block explodes most in C (end/start)?
    explode_block = max(
        blocks.keys(), key=lambda b: blocks[b]["P_C_end_over_start"]
    )
    # Mirror-like: that block has C growing a lot, A not (end/start_A ~<=1 or << C)
    eb = blocks[explode_block]
    mirror_like = bool(
        eb["P_C_end_over_start"] > 5.0
        and eb["P_A_end_over_start"] < 3.0
        and eb["P_C_over_A_end"] > 10.0
        and eb["frac_log10_C_over_A_increasing"] > 0.6
    )

    innov_a = w["innov_norm_mps_A"].to_numpy(dtype=float)
    # corr between innov_A and P_C of explode block (levels)
    pc = w[f"P_pre_{explode_block}_frob_C"].to_numpy(dtype=float)
    if np.std(innov_a) > 0 and np.std(pc) > 0:
        corr_innov_Pc = float(np.corrcoef(innov_a, pc)[0, 1])
    else:
        corr_innov_Pc = float("nan")

    return {
        "window": [t0, t1],
        "blocks": blocks,
        "explode_block_by_C_end_over_start": explode_block,
        "mirror_like_on_explode_block": mirror_like,
        "mirror_criteria": {
            "P_C_end_over_start_gt": 5.0,
            "P_A_end_over_start_lt": 3.0,
            "P_C_over_A_end_gt": 10.0,
            "frac_log_ratio_increasing_gt": 0.6,
        },
        "corr_innov_A_vs_P_C_explode_block": corr_innov_Pc,
        "reading_note": (
            "'Mirror of non-failure' would require: same H_att sign family still "
            "driving A loud innov while C's covariance on the explode block grows "
            "(uncertainty opens) rather than A/C sharing a new late mechanism. "
            "mirror_like=True is necessary but not sufficient for that claim."
        ),
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    prov = json.loads(PROV.read_text(encoding="utf-8")) if PROV.exists() else {}
    m = load_merged()
    ta = load_telem(TELEM_A)
    tc = load_telem(TELEM_C)
    n = min(len(ta), len(tc))
    ta, tc = ta.iloc[:n].reset_index(drop=True), tc.iloc[:n].reset_index(drop=True)

    # --- Burstiness 22–25 s on |drift_A| and drift_A-drift_C ---
    t = ta["t_s"].to_numpy(dtype=float)
    da = ta["drift_m"].to_numpy(dtype=float)
    dc = tc["drift_m"].to_numpy(dtype=float)
    abs_da = np.abs(da)
    ddelta = da - dc

    def win_series(series, t0, t1):
        mask = (t >= t0) & (t <= t1)
        return series[mask], t[mask]

    burst = {}
    for name, series in (
        ("abs_drift_A", abs_da),
        ("drift_A_minus_C", ddelta),
        ("abs_d_drift_A_dt", np.abs(np.gradient(da, t))),
    ):
        xs, ts = win_series(series, *BURST_WIN)
        # For abs_d_drift use the series itself as x (level), burstiness on diffs of level
        # For rate series, burstiness of the rate values' diffs is odd — prefer
        # burstiness of per-tick Δ of abs_drift / delta
        if name == "abs_d_drift_A_dt":
            # instead: burstiness of |Δ abs_drift| already covered by abs_drift_A
            continue
        burst[name] = burstiness_metrics(xs, ts)

    # Finer: within 22–25, find shortest interval containing 50% of sum|dx|
    xs, ts = win_series(abs_da, *BURST_WIN)
    dx = np.diff(xs)
    abs_dx = np.abs(dx)
    total = float(abs_dx.sum())
    concentration = None
    if total > 0 and len(abs_dx) > 5:
        # sliding window minimal width for ≥50% of sum|dx|
        target = 0.5 * total
        best = None
        csum = np.concatenate([[0.0], np.cumsum(abs_dx)])
        for i in range(len(abs_dx)):
            for j in range(i + 1, len(abs_dx) + 1):
                s = csum[j] - csum[i]
                if s >= target:
                    width = float(ts[j] - ts[i])  # ts aligned to points; interval i..j
                    if best is None or width < best["width_s"]:
                        best = {
                            "width_s": width,
                            "t_start_s": float(ts[i]),
                            "t_end_s": float(ts[j]),
                            "share": float(s / total),
                        }
                    break
        concentration = best

    # Also burstiness on whole late 14–25 and early 0–4 for context
    burst_context = {}
    for label, (t0, t1) in (
        ("early_0_4s", EARLY_REF),
        ("late_14_25s", LATE),
        ("burst_win_22_25s", BURST_WIN),
    ):
        xs, ts = win_series(abs_da, t0, t1)
        burst_context[label] = burstiness_metrics(xs, ts)

    # --- P trajectories ---
    p_early = p_snapshot(m, *EARLY_REF)
    p_late = p_snapshot(m, *LATE)
    p_burst = p_snapshot(m, *BURST_WIN)
    mirror = mirror_test(m, *LATE)

    # Cumulative log10(P_C/P_A) series for explode block (for figure)
    t_m = m["timestamp_s_A"].to_numpy(dtype=float)
    mask_late = (t_m >= LATE[0]) & (t_m <= LATE[1])
    eb = mirror["explode_block_by_C_end_over_start"]
    pa = m.loc[mask_late, f"P_pre_{eb}_frob_A"].to_numpy(dtype=float)
    pc = m.loc[mask_late, f"P_pre_{eb}_frob_C"].to_numpy(dtype=float)
    tw = t_m[mask_late]
    log_ratio = np.log10(np.maximum(pc / np.maximum(pa, 1e-12), 1e-30))

    # Figure
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=False)
    axes[0].plot(t, abs_da, label="|drift_A|")
    axes[0].axvspan(*BURST_WIN, color="C3", alpha=0.15, label="22–25 s")
    axes[0].axvline(14.0, color="0.4", ls="--", lw=0.8)
    axes[0].set_ylabel("|drift_A|")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title("OQ9 late checks — P mirror + 22–25s burstiness")

    # per-tick |Δdrift| in 22–25
    xs, ts = win_series(abs_da, *BURST_WIN)
    axes[1].bar(ts[1:], np.abs(np.diff(xs)), width=0.008, color="C3", alpha=0.8)
    axes[1].set_ylabel("|Δ|drift_A|| / tick")
    axes[1].set_xlabel("t (s) in 22–25")
    axes[1].grid(True, alpha=0.3)
    b22 = burst["abs_drift_A"]
    axes[1].set_title(
        f"22–25s burstiness: B={b22['B']:.3f}, top3={b22['top3_share']:.3f}, "
        f"bursty={b22['bursty']}"
    )

    for block, ls in (("aa", "-"), ("vv", "--"), ("pv", ":"), ("pp", "-.")):
        axes[2].semilogy(
            t_m[mask_late],
            np.maximum(m.loc[mask_late, f"P_pre_{block}_frob_A"], 1e-12),
            ls=ls,
            color="C0",
            alpha=0.85,
            label=f"A P_{block}" if block == "aa" else None,
        )
        axes[2].semilogy(
            t_m[mask_late],
            np.maximum(m.loc[mask_late, f"P_pre_{block}_frob_C"], 1e-12),
            ls=ls,
            color="C1",
            alpha=0.85,
            label=f"C P_{block}" if block == "aa" else None,
        )
    # legend manually
    from matplotlib.lines import Line2D

    axes[2].legend(
        handles=[
            Line2D([0], [0], color="C0", label="A"),
            Line2D([0], [0], color="C1", label="C"),
            Line2D([0], [0], color="0.3", ls="-", label="P_aa"),
            Line2D([0], [0], color="0.3", ls="--", label="P_vv"),
            Line2D([0], [0], color="0.3", ls=":", label="P_pv"),
            Line2D([0], [0], color="0.3", ls="-.", label="P_pp"),
        ],
        fontsize=7,
        ncol=3,
    )
    axes[2].set_ylabel("P_*_frob (log)")
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title(f"14–25 s P blocks — C explode block = P_{eb}")

    axes[3].plot(tw, log_ratio, color="C2")
    axes[3].set_ylabel(f"log10(P_{eb}_C / P_{eb}_A)")
    axes[3].set_xlabel("t (s)")
    axes[3].grid(True, alpha=0.3)
    axes[3].set_title(
        f"mirror_like={mirror['mirror_like_on_explode_block']} on P_{eb}"
    )
    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=140)
    plt.close(fig)

    # Working-hypothesis status — do NOT auto-close FEEDBACK_CONTINUES
    b = burst["abs_drift_A"]
    if b["bursty"] and concentration and concentration["width_s"] <= 0.5:
        drift_shape = "PUNCTUAL_HIGH_GAIN_SUSPECT"
        drift_note = (
            "≥50% of 22–25s |Δdrift| mass fits in ≤0.5 s — treat as possible "
            "superposed high-gain event, not pure continuous feedback."
        )
    elif b["bursty"]:
        drift_shape = "BURSTY_BUT_NOT_SUBSECOND"
        drift_note = (
            "Burstiness thresholds fired in 22–25s but concentration width >0.5 s "
            "— clustered accrual, not proven single-tick cliff."
        )
    else:
        drift_shape = "NOT_BURSTY_CONTINUOUS_LIKE"
        drift_note = (
            "22–25s fails burstiness thresholds — more consistent with distributed "
            "accrual (continuous-like) than a GAP-3-style cliff."
        )

    results = {
        "provenance_ref": str(PROV.name) if PROV.exists() else None,
        "audit_sha_A": prov.get("cells", {}).get("A", {}).get("audit_sha256_16"),
        "audit_sha_C": prov.get("cells", {}).get("C", {}).get("audit_sha256_16"),
        "status": "WORKING_HYPOTHESIS_CHECKS_ONLY",
        "prior_verdict_not_closed": "FEEDBACK_CONTINUES_OMEGA_DECOUPLED",
        "P_snapshots": {"early_0_4s": p_early, "late_14_25s": p_late, "win_22_25s": p_burst},
        "mirror_test_14_25s": mirror,
        "burstiness": {
            "abs_drift_A_22_25s": burst["abs_drift_A"],
            "drift_A_minus_C_22_25s": burst["drift_A_minus_C"],
            "context": burst_context,
            "half_mass_concentration_22_25s": concentration,
            "drift_shape_22_25s": drift_shape,
            "drift_shape_note": drift_note,
        },
        "implication_for_preregistration": (
            "Do not use FEEDBACK_CONTINUES_OMEGA_DECOUPLED as closed basis for a "
            "late-regime success criterion until mirror_like and drift_shape are "
            "accepted explicitly."
        ),
    }

    with OUT_JSON.open("w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    # Markdown
    lines = [
        "# OQ9 follow-up — late P A/C + burstiness 22–25 s",
        "",
        "**Status: working-hypothesis checks — does NOT close** "
        "`FEEDBACK_CONTINUES_OMEGA_DECOUPLED`.",
        "",
        f"Post-fix audits sha A={results['audit_sha_A']}, C={results['audit_sha_C']}.  ",
        f"**Figure:** `{OUT_PNG.name}`  ",
        "",
        "## 1. What “P_C explodes” means (numbers)",
        "",
        "Blocks logged: Frobenius norms `P_pre_aa`, `P_pre_vv`, `P_pre_pv`, `P_pre_pp` "
        "from NHC audit (pre-update each NHC tick).",
        "",
        "### Late window 14–25 s",
        "",
        "| Block | P_A start→end | P_C start→end | C end/start | C/A end |",
        "|-------|---------------|---------------|-------------|---------|",
    ]
    for block in ("aa", "vv", "pv", "pp"):
        binfo = mirror["blocks"][block]
        lines.append(
            f"| P_{block} | {binfo['P_A_start']:.4g}→{binfo['P_A_end']:.4g} | "
            f"{binfo['P_C_start']:.4g}→{binfo['P_C_end']:.4g} | "
            f"**{binfo['P_C_end_over_start']:.3g}** | "
            f"**{binfo['P_C_over_A_end']:.3g}** |"
        )

    lines += [
        "",
        f"**Largest C growth (end/start):** `P_{eb}` "
        f"(×{mirror['blocks'][eb]['P_C_end_over_start']:.3g}).",
        "",
        "### Early reference 0–4 s (same blocks)",
        "",
        "| Block | P_A mean | P_C mean | C/A mean |",
        "|-------|----------|----------|----------|",
    ]
    for block in ("aa", "vv", "pv", "pp"):
        lines.append(
            f"| P_{block} | {p_early[f'P_{block}_A_mean']:.4g} | "
            f"{p_early[f'P_{block}_C_mean']:.4g} | "
            f"{p_early[f'P_{block}_C_over_A_mean']:.3g} |"
        )

    lines += [
        "",
        "### “Mirror of non-failure” test (data, not slogan)",
        "",
        f"- explode block: **P_{eb}**",
        f"- mirror_like (criteria in JSON): "
        f"**{mirror['mirror_like_on_explode_block']}**",
        f"- frac log10(P_C/P_A) increasing: "
        f"{mirror['blocks'][eb]['frac_log10_C_over_A_increasing']:.3f}",
        f"- log10(C/A): {mirror['blocks'][eb]['log10_C_over_A_start']:.3f} → "
        f"{mirror['blocks'][eb]['log10_C_over_A_end']:.3f}",
        f"- corr(innov_A, P_C on explode block): "
        f"{mirror['corr_innov_A_vs_P_C_explode_block']:.3f}",
        "",
        mirror["reading_note"],
        "",
        "## 2. Burstiness of 22–25 s (|drift_A|)",
        "",
        f"Rule: **bursty** if top3_share > {THRESH_TOP3_SHARE} OR B > {THRESH_B} "
        f"(B = max\\|Δ\\| / Σ\\|Δ\\|).",
        "",
        f"| Window | B | top3_share | bursty | sum\\|Δ\\| |",
        f"|--------|---|------------|--------|---------|",
    ]
    for label, bm in burst_context.items():
        lines.append(
            f"| {label} | {bm['B']:.4f} | {bm['top3_share']:.4f} | "
            f"**{bm['bursty']}** | {bm['sum_abs_dx']:.3f} |"
        )

    lines += ["", "Dominating ticks in 22–25 s (`|drift_A|`):", ""]
    for d in burst["abs_drift_A"]["dominating_ticks"]:
        lines.append(
            f"- rank {d['rank']}: t∈[{d['t_start_s']:.3f},{d['t_end_s']:.3f}] s, "
            f"dx={d['dx']:.4f}, share={d['share']:.3f}"
        )

    if concentration:
        lines += [
            "",
            f"Shortest interval with ≥50% of 22–25s Σ\\|Δ\\|: "
            f"**{concentration['width_s']:.3f} s** "
            f"[{concentration['t_start_s']:.3f}, {concentration['t_end_s']:.3f}] "
            f"(share={concentration['share']:.2f})",
            "",
        ]

    lines += [
        f"**Drift-shape label: `{drift_shape}`**",
        "",
        drift_note,
        "",
        "## 3. What this does to the prior OQ9 verdict",
        "",
        "Prior label `FEEDBACK_CONTINUES_OMEGA_DECOUPLED` stays a "
        "**working hypothesis** until both checks are accepted:",
        "",
        f"1. P mirror: mirror_like={mirror['mirror_like_on_explode_block']} "
        f"on P_{eb} (see criteria).",
        f"2. 22–25s shape: `{drift_shape}` (bursty={b['bursty']}).",
        "",
        "**Not** a closed basis for a late-regime §11-style success criterion yet.",
        "",
    ]
    OUT_MD.write_text("\n".join(lines), encoding="utf-8")
    print(OUT_MD.read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
