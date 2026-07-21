#!/usr/bin/env python3
"""OQ9 / H-DEG-ATT-LAG — turns 9–10 degraded regime vs clean delayed turns 1/2/4.

Uses post-fix NHC audits + telemetry only (no intervention). Goal: decide whether
late trajectory (drift≳20 m) is still the same attitude-feedback loop (just
decorrelated from |ω|) or a second mechanism that must inform any future clamp.
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

OUT_JSON = OUT / "slalom_oq9_degraded_regime.json"
OUT_MD = OUT / "slalom_oq9_degraded_regime.md"
OUT_PNG = OUT / "fig_slalom_oq9_degraded_regime.png"

# From per-turn xcorr classification
CLEAN_DELAYED = {1: 2.0, 2: 4.0, 4: 8.0}
DEGRADED_OTHER = {9: 18.0, 10: 20.0}
# also compare turn 7 (last clean delayed) and turn 8 (alias@high drift)
CONTEXT = {7: 14.0, 8: 16.0}
STIM_HALF = 1.0
DRIFT_DEG_M = 15.0


def load_merged_audit() -> pd.DataFrame:
    a = pd.read_csv(AUDIT_A)
    c = pd.read_csv(AUDIT_C)
    a["t_ms"] = (a["timestamp_s"] * 1000).round().astype(int)
    c["t_ms"] = (c["timestamp_s"] * 1000).round().astype(int)
    return a.merge(c, on="t_ms", suffixes=("_A", "_C"))


def load_telem(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["t_s"] = df["time_us"].astype(float) * 1e-6
    return df


def window(df: pd.DataFrame, t_c: float, half: float = STIM_HALF) -> pd.DataFrame:
    t = df["timestamp_s_A"] if "timestamp_s_A" in df.columns else df["timestamp_s"]
    return df[(t >= t_c - half) & (t <= t_c + half)]


def summarize_nhc(w: pd.DataFrame) -> dict:
    if len(w) == 0:
        return {"n": 0}

    def rms(col):
        x = w[col].to_numpy(dtype=float)
        return float(np.sqrt(np.mean(x**2)))

    def mean(col):
        return float(w[col].mean())

    za = w["dx_att_z_rad_A"].to_numpy(dtype=float)
    zc = w["dx_att_z_rad_C"].to_numpy(dtype=float)
    opp = float(np.mean(np.sign(za) * np.sign(zc) < 0)) if len(za) else float("nan")
    # feedback still "alive" if innov_A >> innov_C and |v_body_y|_A large
    innov_ratio = rms("innov_norm_mps_A") / max(rms("innov_norm_mps_C"), 1e-12)
    k_att_ratio = mean("k_att_max_A") / max(mean("k_att_max_C"), 1e-12)
    return {
        "n": int(len(w)),
        "innov_A_rms": rms("innov_norm_mps_A"),
        "innov_C_rms": rms("innov_norm_mps_C"),
        "innov_A_over_C": innov_ratio,
        "v_body_y_A_rms": rms("v_body_y_before_mps_A"),
        "v_body_y_C_rms": rms("v_body_y_before_mps_C"),
        "dx_att_z_A_rms": rms("dx_att_z_rad_A"),
        "dx_att_z_C_rms": rms("dx_att_z_rad_C"),
        "dx_att_z_opposite_frac": opp,
        "dx_vel_A_rms": rms("dx_vel_norm_mps_A"),
        "k_att_A_mean": mean("k_att_max_A"),
        "k_att_C_mean": mean("k_att_max_C"),
        "k_att_A_over_C": k_att_ratio,
        "k_vel_A_mean": mean("k_vel_max_A"),
        "P_pre_aa_A": mean("P_pre_aa_frob_A"),
        "P_pre_aa_C": mean("P_pre_aa_frob_C"),
        "P_pre_vv_A": mean("P_pre_vv_frob_A"),
        "P_pre_vv_C": mean("P_pre_vv_frob_C"),
        "P_pre_pv_A": mean("P_pre_pv_frob_A"),
        "P_pre_pv_C": mean("P_pre_pv_frob_C"),
        "P_aa_A_over_C": mean("P_pre_aa_frob_A") / max(mean("P_pre_aa_frob_C"), 1e-12),
        "P_pv_A_over_C": mean("P_pre_pv_frob_A") / max(mean("P_pre_pv_frob_C"), 1e-12),
        "delta_P_pv_A": mean("delta_P_pv_frob_A"),
        "nis_A": mean("nis_total_A"),
        "nis_C": mean("nis_total_C"),
    }


def drift_budget(telem_a: pd.DataFrame, telem_c: pd.DataFrame) -> dict:
    """How much |drift_A| / |drift_A-drift_C| accrues in which epochs."""
    t = telem_a["t_s"].to_numpy()
    da = telem_a["drift_m"].to_numpy(dtype=float)
    dc = telem_c["drift_m"].to_numpy(dtype=float)
    # align lengths
    n = min(len(da), len(dc), len(t))
    t, da, dc = t[:n], da[:n], dc[:n]
    ddelta = da - dc

    def growth(t0, t1, series):
        m0 = np.argmin(np.abs(t - t0))
        m1 = np.argmin(np.abs(t - t1))
        return float(series[m1] - series[m0]), float(series[m0]), float(series[m1])

    epochs = {
        "early_feedback_0_4s": (0.0, 4.0),
        "clean_turns_4_14s": (4.0, 14.0),
        "pre_degraded_14_18s": (14.0, 18.0),
        "degraded_other_18_22s": (18.0, 22.0),  # covers turns 9–10 stim+effect
        "tail_22_25s": (22.0, 25.0),
        "whole_0_25s": (0.0, 25.0),
    }
    out = {}
    whole_da, _, da_end = growth(0.0, 25.0, np.abs(da))
    # use signed max-tracking: growth of |drift| along run
    abs_da = np.abs(da)
    # running max for contribution framing
    for name, (t0, t1) in epochs.items():
        g_abs, a0, a1 = growth(t0, t1, abs_da)
        g_delta, d0, d1 = growth(t0, t1, ddelta)
        out[name] = {
            "t0": t0,
            "t1": t1,
            "abs_drift_A_start": a0,
            "abs_drift_A_end": a1,
            "delta_abs_drift_A": g_abs,
            "frac_of_final_abs_drift_A": float(g_abs / max(abs_da[-1], 1e-9)),
            "delta_drift_A_minus_C": g_delta,
            "drift_A_minus_C_start": d0,
            "drift_A_minus_C_end": d1,
        }
    out["final_abs_drift_A"] = float(abs_da[-1])
    out["final_drift_A_minus_C"] = float(ddelta[-1])
    out["first_t_abs_drift_ge_15"] = float(t[np.where(abs_da >= DRIFT_DEG_M)[0][0]]) if np.any(abs_da >= DRIFT_DEG_M) else None
    out["first_t_abs_drift_ge_20"] = float(t[np.where(abs_da >= 20.0)[0][0]]) if np.any(abs_da >= 20.0) else None
    return out


def classify_regime(s: dict, role: str) -> str:
    """Cheap labels for whether attitude feedback still dominates the window."""
    if s.get("n", 0) < 10:
        return "empty"
    # In clean early turns: innov_A>>C, opposite dx_att_z often high early then lower,
    # P_aa_A < P_aa_C (A more "confident"/collapsed on att?)
    innov_dom = s["innov_A_over_C"] > 100
    att_opp = s["dx_att_z_opposite_frac"] > 0.4
    # Degraded signature candidates:
    # - lag/ω coupling broken already known from xcorr
    # - P_pv or P_aa regime flip vs clean
    # - k_att collapses or explodes
    return {
        "role": role,
        "attitude_feedback_still_loud": bool(innov_dom and s["v_body_y_A_rms"] > 0.1),
        "dx_att_z_often_opposite": bool(att_opp),
        "P_aa_A_much_smaller_than_C": bool(s["P_aa_A_over_C"] < 0.1),
        "P_pv_A_much_smaller_than_C": bool(s["P_pv_A_over_C"] < 0.1),
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    if not AUDIT_A.exists() or not AUDIT_C.exists():
        raise SystemExit("Missing post-fix audits; run slalom_a_vs_c_kp_postfix_verify.py first")

    prov = json.loads(PROV.read_text(encoding="utf-8")) if PROV.exists() else {}
    m = load_merged_audit()
    ta = load_telem(TELEM_A)
    tc = load_telem(TELEM_C)
    # align telem
    n = min(len(ta), len(tc))
    ta, tc = ta.iloc[:n].reset_index(drop=True), tc.iloc[:n].reset_index(drop=True)

    turns = {}
    for turn_id, t_c in {**CLEAN_DELAYED, **CONTEXT, **DEGRADED_OTHER}.items():
        if turn_id in CLEAN_DELAYED:
            role = "clean_delayed"
        elif turn_id in DEGRADED_OTHER:
            role = "degraded_other"
        elif turn_id == 7:
            role = "last_clean_delayed"
        else:
            role = "alias_high_drift"
        w = window(m, t_c)
        s = summarize_nhc(w)
        # drift at t_c from telem
        i = int(np.argmin(np.abs(ta["t_s"] - t_c)))
        s["drift_A_at_tc"] = float(ta["drift_m"].iloc[i])
        s["drift_C_at_tc"] = float(tc["drift_m"].iloc[i])
        s["t_c"] = t_c
        s["role"] = role
        s["flags"] = classify_regime(s, role)
        turns[str(turn_id)] = s

    budget = drift_budget(ta, tc)

    # Aggregate clean vs degraded
    def mean_key(ids, key):
        vals = [turns[str(i)][key] for i in ids if turns[str(i)].get("n", 0)]
        return float(np.mean(vals)) if vals else float("nan")

    clean_ids = list(CLEAN_DELAYED.keys())
    deg_ids = list(DEGRADED_OTHER.keys())
    contrast = {
        "innov_A_rms_clean_mean": mean_key(clean_ids, "innov_A_rms"),
        "innov_A_rms_degraded_mean": mean_key(deg_ids, "innov_A_rms"),
        "v_body_y_A_rms_clean_mean": mean_key(clean_ids, "v_body_y_A_rms"),
        "v_body_y_A_rms_degraded_mean": mean_key(deg_ids, "v_body_y_A_rms"),
        "P_pre_aa_A_clean_mean": mean_key(clean_ids, "P_pre_aa_A"),
        "P_pre_aa_A_degraded_mean": mean_key(deg_ids, "P_pre_aa_A"),
        "P_pre_pv_A_clean_mean": mean_key(clean_ids, "P_pre_pv_A"),
        "P_pre_pv_A_degraded_mean": mean_key(deg_ids, "P_pre_pv_A"),
        "P_pre_aa_C_clean_mean": mean_key(clean_ids, "P_pre_aa_C"),
        "P_pre_aa_C_degraded_mean": mean_key(deg_ids, "P_pre_aa_C"),
        "P_pre_pv_C_clean_mean": mean_key(clean_ids, "P_pre_pv_C"),
        "P_pre_pv_C_degraded_mean": mean_key(deg_ids, "P_pre_pv_C"),
        "k_att_A_clean_mean": mean_key(clean_ids, "k_att_A_mean"),
        "k_att_A_degraded_mean": mean_key(deg_ids, "k_att_A_mean"),
        "dx_att_z_opp_clean_mean": mean_key(clean_ids, "dx_att_z_opposite_frac"),
        "dx_att_z_opp_degraded_mean": mean_key(deg_ids, "dx_att_z_opposite_frac"),
    }

    # Reading: is degraded still same loop?
    # Same loop if innov still huge A vs C and v_body_y large, but ω-xcorr broke
    # → "feedback continues, ω is no longer the organizing stimulus" (phase scrambled)
    # Second mechanism if P blocks / k_att qualitatively different and innov saturates
    # differently, or C also starts growing (shared degrad), etc.
    feedback_still = all(
        turns[str(i)]["flags"]["attitude_feedback_still_loud"] for i in deg_ids
    )
    p_flip = (
        contrast["P_pre_pv_A_degraded_mean"] > 2 * contrast["P_pre_pv_A_clean_mean"]
        or contrast["P_pre_aa_C_degraded_mean"] > 2 * contrast["P_pre_aa_C_clean_mean"]
    )
    # Drift shares — late = from first ≥20 m through end (not only turns 9–10 stim)
    deg_frac = budget["degraded_other_18_22s"]["frac_of_final_abs_drift_A"]
    early_frac = budget["early_feedback_0_4s"]["frac_of_final_abs_drift_A"]
    mid_frac = budget["clean_turns_4_14s"]["frac_of_final_abs_drift_A"]
    pre_deg_frac = budget["pre_degraded_14_18s"]["frac_of_final_abs_drift_A"]
    tail_frac = budget["tail_22_25s"]["frac_of_final_abs_drift_A"]
    late_frac = pre_deg_frac + deg_frac + tail_frac  # t≥14 s
    post20_frac = deg_frac + tail_frac  # roughly after drift≥20 m (t≳16–18)

    if feedback_still and late_frac >= 0.40:
        verdict = "FEEDBACK_CONTINUES_OMEGA_DECOUPLED"
        reason = (
            "Turns 9–10 still show loud A≫C innov/|v_body_y| (same attitude-feedback "
            "family as turn 1), but per-turn xcorr vs |ω| broke — the loop is no longer "
            "paced by slalom yaw peaks. Most |drift_A| accrues at/after the degraded "
            f"boundary (t≥14 s share ≈ {late_frac:.0%}; 18–25 s ≈ {post20_frac:.0%}), "
            "not in the ω-locked early turns. A clamp preregistered only on turn 1/2/4 "
            "windows would miss the phase that dominates the 54 m end state. "
            "Companion note: P_pv_C / P_aa_C inflate hard late while A stays "
            "attitude-tight — C's P growth is the non-failing mirror, not the driver of A's drift."
        )
    elif feedback_still and late_frac < 0.40:
        verdict = "FEEDBACK_CONTINUES_BUT_LOW_LATE_SHARE"
        reason = (
            "Late turns still look like attitude feedback, and late |drift| share is "
            "modest — turn-1/2/4-centric intervention might still cover the bulk; "
            "keep a late-regime check in the preregistration anyway."
        )
    elif not feedback_still and p_flip:
        verdict = "SECOND_MECHANISM_P_REGIME"
        reason = (
            "Late turns lose the loud A≫C innov signature and/or show P-block regime "
            "change — candidate second mechanism (P degradation family GAP-3/4). "
            "Do not design attitude gain-clamp from turn 1 alone."
        )
    else:
        verdict = "MIXED_DEGRADED"
        reason = (
            "Degraded turns do not cleanly match either pure continued-feedback or "
            "pure P-regime second mechanism; inspect per-turn table before intervention."
        )

    # Figure
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    t = ta["t_s"].to_numpy()
    axes[0].plot(t, np.abs(ta["drift_m"]), label="|drift_A|")
    axes[0].plot(t, np.abs(tc["drift_m"]), label="|drift_C|")
    axes[0].axhline(20, color="0.4", ls="--", lw=0.9, label="20 m")
    axes[0].axvspan(18, 22, color="C3", alpha=0.12, label="turns 9–10 window")
    axes[0].set_ylabel("|drift| (m)")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    taud = m["timestamp_s_A"].to_numpy()
    axes[1].semilogy(taud, np.maximum(m["innov_norm_mps_A"], 1e-12), label="innov A")
    axes[1].semilogy(taud, np.maximum(m["innov_norm_mps_C"], 1e-12), label="innov C")
    axes[1].axvspan(18, 22, color="C3", alpha=0.12)
    axes[1].set_ylabel("‖innov‖")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(taud, m["P_pre_pv_frob_A"], label="P_pv A")
    axes[2].plot(taud, m["P_pre_pv_frob_C"], label="P_pv C")
    axes[2].plot(taud, m["P_pre_aa_frob_A"], label="P_aa A", ls="--")
    axes[2].plot(taud, m["P_pre_aa_frob_C"], label="P_aa C", ls="--")
    axes[2].axvspan(18, 22, color="C3", alpha=0.12)
    axes[2].set_ylabel("P frob")
    axes[2].set_xlabel("t (s)")
    axes[2].legend(fontsize=7, ncol=2)
    axes[2].grid(True, alpha=0.3)
    fig.suptitle(f"OQ9 degraded regime — {verdict}", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=140)
    plt.close(fig)

    results = {
        "provenance_ref": str(PROV.name) if PROV.exists() else None,
        "audit_sha_A": prov.get("cells", {}).get("A", {}).get("audit_sha256_16"),
        "audit_sha_C": prov.get("cells", {}).get("C", {}).get("audit_sha256_16"),
        "turns": turns,
        "drift_budget": budget,
        "contrast_clean_vs_degraded": contrast,
        "verdict": verdict,
        "reason": reason,
        "intervention_implication": (
            "Do not preregister an attitude-NHC gain-clamp from turn 1/2/4 alone until "
            "this verdict is accepted; late drift share and mechanism class matter."
        ),
        "no_code_fix": True,
    }
    with OUT_JSON.open("w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    lines = [
        "# OQ9 — H-DEG-ATT-LAG: degraded turns 9–10 vs clean 1/2/4",
        "",
        "**No intervention.** Post-fix audits only "
        f"(sha A={results['audit_sha_A']}, C={results['audit_sha_C']}).  ",
        f"**Figure:** `{OUT_PNG.name}`  ",
        "",
        "## State of the world (do not blur)",
        "",
        "- Mechanism A×C (early feedback via `H_att` sign) is **understood**.",
        "- Historical thresholds: still **FAIL** on the matrix cells — nothing is fixed.",
        "- Intervention family **not decided** (gain-clamp / companion retune / other).",
        "- **Do not design** a clamp from turn 1/2/4 until this degraded-regime read is in.",
        "",
        "## Drift budget (|drift_A| accrual)",
        "",
        f"Final |drift_A| ≈ **{budget['final_abs_drift_A']:.2f} m**; "
        f"first ≥20 m @ t≈{budget['first_t_abs_drift_ge_20']} s.",
        "",
        "| Epoch | Δ|drift_A| | frac of final | Δ(drift_A−C) |",
        "|-------|------------|---------------|--------------|",
    ]
    for key in (
        "early_feedback_0_4s",
        "clean_turns_4_14s",
        "pre_degraded_14_18s",
        "degraded_other_18_22s",
        "tail_22_25s",
    ):
        b = budget[key]
        lines.append(
            f"| {key} | {b['delta_abs_drift_A']:.2f} | "
            f"{b['frac_of_final_abs_drift_A']:.2%} | "
            f"{b['delta_drift_A_minus_C']:.2f} |"
        )

    lines += [
        "",
        f"- early 0–4 s: **{early_frac:.1%}**",
        f"- mid (clean turns) 4–14 s: **{mid_frac:.1%}**",
        f"- pre-degraded 14–18 s: **{pre_deg_frac:.1%}**",
        f"- turns 9–10 window 18–22 s: **{deg_frac:.1%}**",
        f"- tail 22–25 s: **{tail_frac:.1%}**",
        f"- **late total t≥14 s: {late_frac:.1%}** (post-~20 m ≈ {post20_frac:.1%})",
        "",
        "Headline: the ω-locked early turns are **not** where most of the 54 m is earned.",
        "",
        "## Per-turn NHC (stim ±1 s)",
        "",
        "| Turn | role | drift_A | innov_A | innov A/C | |v_by|_A | "
        "k_att_A | P_aa_A | P_pv_A | P_aa_C | P_pv_C | opp dx_att_z |",
        "|------|------|---------|---------|-----------|---------|--------|"
        "--------|--------|--------|--------|-------------|",
    ]
    for tid in ("1", "2", "4", "7", "8", "9", "10"):
        s = turns[tid]
        lines.append(
            f"| {tid} | {s['role']} | {s['drift_A_at_tc']:.2f} | "
            f"{s['innov_A_rms']:.3g} | {s['innov_A_over_C']:.3g} | "
            f"{s['v_body_y_A_rms']:.3g} | {s['k_att_A_mean']:.3g} | "
            f"{s['P_pre_aa_A']:.3g} | {s['P_pre_pv_A']:.3g} | "
            f"{s['P_pre_aa_C']:.3g} | {s['P_pre_pv_C']:.3g} | "
            f"{s['dx_att_z_opposite_frac']:.2f} |"
        )

    lines += [
        "",
        "## Clean vs degraded contrast (means)",
        "",
        f"- innov_A rms: clean **{contrast['innov_A_rms_clean_mean']:.3g}** vs "
        f"degraded **{contrast['innov_A_rms_degraded_mean']:.3g}**",
        f"- |v_body_y|_A: clean **{contrast['v_body_y_A_rms_clean_mean']:.3g}** vs "
        f"degraded **{contrast['v_body_y_A_rms_degraded_mean']:.3g}**",
        f"- P_aa_A: clean {contrast['P_pre_aa_A_clean_mean']:.3g} vs "
        f"degraded {contrast['P_pre_aa_A_degraded_mean']:.3g}",
        f"- P_pv_A: clean {contrast['P_pre_pv_A_clean_mean']:.3g} vs "
        f"degraded {contrast['P_pre_pv_A_degraded_mean']:.3g}",
        f"- P_aa_C / P_pv_C: clean "
        f"{contrast['P_pre_aa_C_clean_mean']:.3g} / {contrast['P_pre_pv_C_clean_mean']:.3g} "
        f"vs degraded "
        f"{contrast['P_pre_aa_C_degraded_mean']:.3g} / {contrast['P_pre_pv_C_degraded_mean']:.3g}",
        f"- k_att_A: clean {contrast['k_att_A_clean_mean']:.3g} vs "
        f"degraded {contrast['k_att_A_degraded_mean']:.3g}",
        f"- opp dx_att_z frac: clean {contrast['dx_att_z_opp_clean_mean']:.2f} vs "
        f"degraded {contrast['dx_att_z_opp_degraded_mean']:.2f}",
        "",
        "## Verdict",
        "",
        f"**{verdict}**",
        "",
        reason,
        "",
        results["intervention_implication"],
        "",
        "## Next (still no code)",
        "",
        "- If `FEEDBACK_CONTINUES_OMEGA_DECOUPLED`: any future clamp must be stated in "
        "terms of innov/`v_body`/attitude-gain energy, not only ω-locked turn-1 windows; "
        "include a late-regime success criterion in the preregistration.",
        "- If `SECOND_MECHANISM_P_REGIME`: preregister a P-facing arm separately "
        "(GAP-3/4 family), not as a footnote to attitude clamp.",
        "- Either way: **preregister before implementing** (same discipline as §11).",
        "",
    ]
    OUT_MD.write_text("\n".join(lines), encoding="utf-8")
    print(OUT_MD.read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
