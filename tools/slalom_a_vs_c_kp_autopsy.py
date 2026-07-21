#!/usr/bin/env python3
"""SLALOM A vs C — NHC K/P autopsy on turn 1 (+ controls 2, 4).

Answers: where in ω→att→v_body→NHC→v_ned→pos the H_att sign flip
(correct = −legacy) amplifies drift ~143× vs attenuating it.

Requires nhc_block_audit CSVs from re-runs with
NAVICORE_NHC_BLOCK_AUDIT_CSV set.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
OUT_DIR = REPO / "docs" / "benchmarks" / "jacobian_imu_ab"
SIM = REPO / "build" / "NaviCore3D_Sim.exe"
SEED = 71

AUDIT_A = OUT_DIR / "slalom_cellA_jcorrect_imuideal_s71_nhc_block_audit.csv"
AUDIT_C = OUT_DIR / "slalom_cellC_jlegacy_imuideal_s71_nhc_block_audit.csv"
TELEM_A = REPO / "docs/benchmarks/slalom_cellA_jcorrect_imuideal_s71_telemetry.csv"
TELEM_C = REPO / "docs/benchmarks/slalom_cellC_jlegacy_imuideal_s71_telemetry.csv"

OUT_JSON = OUT_DIR / "slalom_a_vs_c_kp_autopsy.json"
OUT_MD = OUT_DIR / "slalom_a_vs_c_kp_autopsy.md"
OUT_PNG = OUT_DIR / "fig_slalom_kp_autopsy_turn1.png"

# Turn centers (‖ω‖ peaks) — delayed family
TURNS = {
    1: {"t_c": 2.0, "role": "primary"},
    2: {"t_c": 4.0, "role": "control"},
    4: {"t_c": 8.0, "role": "control"},
}
STIM_HALF = 1.0
EFFECT_LAG = 2.0  # characteristic lag from xcorr
EFFECT_HALF = 1.0


def hatt_correct(vx, vy, vz):
    """H_att rows (correct mode): H0=[-vz,0,vx], H1=[vy,-vx,0]."""
    return np.array([[-vz, 0.0, vx], [vy, -vx, 0.0]], dtype=float)


def hatt_legacy(vx, vy, vz):
    """H_att rows (legacy bug): −correct."""
    return -hatt_correct(vx, vy, vz)


def run_cell_audited(jacobian: str, cell: str, audit_path: Path) -> None:
    """Single Sim run: telemetry archived by run_benchmark with audit env."""
    sys.path.insert(0, str(REPO))
    from run_all_benchmarks import run_benchmark

    env_audit = str(audit_path)
    # Monkey-patch: set env for child process — run_benchmark uses subprocess
    # so we need run_benchmark to pass env, or call Sim ourselves + copy CSV.
    # Simplest: call Sim with audit env and known csv path from main.
    audit_path.parent.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = env_audit
    # Discover default telemetry path after run via run_benchmark internals
    # Override: run Sim, then use run_benchmark's archive by re-invoking with env.
    # Check how run_benchmark launches:
    import run_all_benchmarks as rab

    # Patch temporarily
    old_run = rab.subprocess.run

    def run_with_env(*args, **kwargs):
        kwargs = dict(kwargs)
        e = kwargs.get("env")
        if e is None:
            e = os.environ.copy()
        else:
            e = dict(e)
        e["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = env_audit
        kwargs["env"] = e
        return old_run(*args, **kwargs)

    rab.subprocess.run = run_with_env  # type: ignore
    try:
        suffix = f"cell{cell}_j{jacobian}_imuideal_s{SEED}"
        r = run_benchmark(
            f"{cell} SLALOM",
            "SLALOM",
            seed=SEED,
            imu_mode="ideal",
            nhc_jacobian=jacobian,
            archive_suffix=suffix,
        )
        print(f"cell {cell}: drift={r.metrics[0].measured if r.metrics else '?'}")
    finally:
        rab.subprocess.run = old_run  # type: ignore


def window_mask(t, t0, t1):
    return (t >= t0) & (t <= t1)


def summarize_window(df: pd.DataFrame, t0: float, t1: float) -> dict:
    m = window_mask(df["timestamp_s"].to_numpy(), t0, t1)
    w = df.loc[m]
    if len(w) == 0:
        return {"n": 0}
    def rms(col):
        x = w[col].to_numpy(dtype=float)
        return float(np.sqrt(np.mean(x**2))) if len(x) else float("nan")
    def mean_abs(col):
        return float(np.mean(np.abs(w[col].to_numpy(dtype=float))))
    def peak(col):
        return float(np.max(np.abs(w[col].to_numpy(dtype=float))))

    # Reconstruct H_att norms from v_body
    vx = w["v_body_x_before_mps"].to_numpy(dtype=float)
    vy = w["v_body_y_before_mps"].to_numpy(dtype=float)
    vz = w["v_body_z_before_mps"].to_numpy(dtype=float)
    h_corr_frob = []
    h_leg_frob = []
    for i in range(len(vx)):
        hc = hatt_correct(vx[i], vy[i], vz[i])
        hl = hatt_legacy(vx[i], vy[i], vz[i])
        h_corr_frob.append(float(np.linalg.norm(hc, "fro")))
        h_leg_frob.append(float(np.linalg.norm(hl, "fro")))
        # verify sign flip
        assert np.allclose(hc, -hl), "H_att correct != -legacy"

    return {
        "n": int(len(w)),
        "t0": t0,
        "t1": t1,
        "innov_norm_rms": rms("innov_norm_mps"),
        "innov_norm_peak": peak("innov_norm_mps"),
        "innov_y_rms": rms("innov_y_mps"),
        "innov_z_rms": rms("innov_z_mps"),
        "k_att_max_mean": float(w["k_att_max"].mean()),
        "k_att_max_peak": peak("k_att_max"),
        "k_vel_max_mean": float(w["k_vel_max"].mean()),
        "k_vel_max_peak": peak("k_vel_max"),
        "dx_att_norm_rms": rms("dx_att_norm_rad"),
        "dx_att_norm_peak": peak("dx_att_norm_rad"),
        "dx_att_z_rms": rms("dx_att_z_rad"),
        "dx_att_z_mean_signed": float(w["dx_att_z_rad"].mean()),
        "dx_vel_norm_rms": rms("dx_vel_norm_mps"),
        "dx_vel_norm_peak": peak("dx_vel_norm_mps"),
        "dx_pos_norm_rms": rms("dx_pos_norm_m"),
        "dv_body_y_rms": rms("dv_body_y_mps"),
        "dv_body_z_rms": rms("dv_body_z_mps"),
        "v_body_y_rms": rms("v_body_y_before_mps"),
        "v_body_z_rms": rms("v_body_z_before_mps"),
        "P_pre_aa_frob_mean": float(w["P_pre_aa_frob"].mean()),
        "P_pre_vv_frob_mean": float(w["P_pre_vv_frob"].mean()),
        "P_pre_pv_frob_mean": float(w["P_pre_pv_frob"].mean()),
        "delta_P_aa_frob_mean": float(w["delta_P_aa_frob"].mean()),
        "delta_P_vv_frob_mean": float(w["delta_P_vv_frob"].mean()),
        "delta_P_pv_frob_mean": float(w["delta_P_pv_frob"].mean()),
        "H_att_frob_mean": float(np.mean(h_corr_frob)),  # same mag either mode
        "nis_total_mean": float(w["nis_total"].mean()),
    }


def align_and_diff(a: pd.DataFrame, c: pd.DataFrame) -> pd.DataFrame:
    """Inner-join on timestamp (should match)."""
    # round to ms for join safety
    a = a.copy()
    c = c.copy()
    a["t_ms"] = (a["timestamp_s"] * 1000).round().astype(int)
    c["t_ms"] = (c["timestamp_s"] * 1000).round().astype(int)
    m = a.merge(c, on="t_ms", suffixes=("_A", "_C"))
    return m


def chain_localization(merged: pd.DataFrame, t0: float, t1: float) -> dict:
    """Locate where A and C diverge in the NHC chain within a window."""
    t = merged["timestamp_s_A"].to_numpy()
    m = window_mask(t, t0, t1)
    w = merged.loc[m]
    if len(w) < 5:
        return {"n": 0}

    innov_a = w["innov_norm_mps_A"].to_numpy(dtype=float)
    innov_c = w["innov_norm_mps_C"].to_numpy(dtype=float)
    dxatt_a = w["dx_att_norm_rad_A"].to_numpy(dtype=float)
    dxatt_c = w["dx_att_norm_rad_C"].to_numpy(dtype=float)
    dxatt_z_a = w["dx_att_z_rad_A"].to_numpy(dtype=float)
    dxatt_z_c = w["dx_att_z_rad_C"].to_numpy(dtype=float)
    dxvel_a = w["dx_vel_norm_mps_A"].to_numpy(dtype=float)
    dxvel_c = w["dx_vel_norm_mps_C"].to_numpy(dtype=float)
    katt_a = w["k_att_max_A"].to_numpy(dtype=float)
    katt_c = w["k_att_max_C"].to_numpy(dtype=float)
    kvel_a = w["k_vel_max_A"].to_numpy(dtype=float)
    kvel_c = w["k_vel_max_C"].to_numpy(dtype=float)

    # Sign opposition of dx_att_z (primary yaw coupling for planar slalom)
    # corr of signed corrections: negative ⇒ opposite direction
    if np.std(dxatt_z_a) > 1e-12 and np.std(dxatt_z_c) > 1e-12:
        sign_corr_dz = float(np.corrcoef(dxatt_z_a, dxatt_z_c)[0, 1])
    else:
        sign_corr_dz = float("nan")
    opposite_frac = float(np.mean(np.sign(dxatt_z_a) * np.sign(dxatt_z_c) < 0))

    # H_att reconstruction: for each tick, check that applying −H flips expected K direction
    # Ratio of magnitudes
    def ratio(a, c):
        ca = np.mean(np.abs(c))
        return float(np.mean(np.abs(a)) / ca) if ca > 1e-15 else float("inf")

    # Reconstruct predicted dx_att direction from H_att * (via innov as proxy):
    # For same P,R roughly: K_att ∝ P_aa H_att^T → sign(K_att) follows sign(H_att)
    vx = w["v_body_x_before_mps_A"].to_numpy(dtype=float)
    # Use A v_body for both H reconstructions (kinematics nearly shared early)
    vy_a = w["v_body_y_before_mps_A"].to_numpy(dtype=float)
    vz_a = w["v_body_z_before_mps_A"].to_numpy(dtype=float)
    # H0_att[2] = ±v_x  (yaw column of lateral NHC row) — key planar coupling
    h0_yaw_corr = vx  # correct: +vx
    h0_yaw_leg = -vx  # legacy: -vx

    out = {
        "n": int(len(w)),
        "innov_norm_ratio_A_over_C": ratio(innov_a, innov_c),
        "dx_att_norm_ratio_A_over_C": ratio(dxatt_a, dxatt_c),
        "dx_att_z_ratio_A_over_C": ratio(dxatt_z_a, dxatt_z_c),
        "dx_vel_norm_ratio_A_over_C": ratio(dxvel_a, dxvel_c),
        "k_att_max_ratio_A_over_C": ratio(katt_a, katt_c),
        "k_vel_max_ratio_A_over_C": ratio(kvel_a, kvel_c),
        "dx_att_z_sign_corr_A_vs_C": sign_corr_dz,
        "dx_att_z_opposite_sign_frac": opposite_frac,
        "mean_abs_innov_A": float(np.mean(np.abs(innov_a))),
        "mean_abs_innov_C": float(np.mean(np.abs(innov_c))),
        "mean_abs_dx_att_A": float(np.mean(np.abs(dxatt_a))),
        "mean_abs_dx_att_C": float(np.mean(np.abs(dxatt_c))),
        "mean_abs_dx_vel_A": float(np.mean(np.abs(dxvel_a))),
        "mean_abs_dx_vel_C": float(np.mean(np.abs(dxvel_c))),
        "H0_att_yaw_col_is_negation": True,  # by construction correct=+vx, legacy=-vx
        "note_H_yaw": (
            "correct H0_att[2]=+v_x; legacy H0_att[2]=-v_x "
            "(planar NHC lateral↔yaw coupling flips)"
        ),
    }

    # Localization heuristic
    innov_close = abs(np.log10(out["innov_norm_ratio_A_over_C"] + 1e-15)) < 0.3  # <2x
    att_flip = opposite_frac > 0.6 or (np.isfinite(sign_corr_dz) and sign_corr_dz < -0.3)
    att_amp = out["dx_att_norm_ratio_A_over_C"] > 1.5
    vel_amp = out["dx_vel_norm_ratio_A_over_C"] > 1.5

    if att_flip and att_amp:
        locus = (
            "H_att_sign → K_att → dx_att "
            "(attitude correction opposite and larger under correct J)"
        )
    elif att_flip and not att_amp and vel_amp:
        locus = (
            "H_att_sign → wrong-way dx_att → growing innov/v_body → dx_vel "
            "(amplification downstream of attitude)"
        )
    elif not innov_close and vel_amp:
        locus = (
            "innov already diverged in window — feedback loop already running; "
            "look earlier for first H_att sign effect"
        )
    else:
        locus = "mixed / inspect time series"

    out["locus"] = locus
    out["flags"] = {
        "innov_magnitudes_similar": bool(innov_close),
        "dx_att_z_opposite": bool(att_flip),
        "dx_att_amplified_A": bool(att_amp),
        "dx_vel_amplified_A": bool(vel_amp),
    }
    return out


def telem_drift_at(path: Path, t_s: float) -> float:
    df = pd.read_csv(path)
    t = df["time_us"].astype(float) * 1e-6
    i = int(np.argmin(np.abs(t - t_s)))
    return float(df["drift_m"].iloc[i])


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    force = "--force-rerun" in sys.argv
    need = force or not AUDIT_A.exists() or not AUDIT_C.exists()
    if need:
        if not SIM.exists():
            print(f"ERROR: missing {SIM}", file=sys.stderr)
            return 1
        run_cell_audited("correct", "A", AUDIT_A)
        run_cell_audited("legacy", "C", AUDIT_C)

    a = pd.read_csv(AUDIT_A)
    c = pd.read_csv(AUDIT_C)
    merged = align_and_diff(a, c)

    results = {
        "audits": {"A": str(AUDIT_A.name), "C": str(AUDIT_C.name)},
        "H_att_formulas": {
            "correct": "H0=[-vz,0,+vx], H1=[+vy,-vx,0]",
            "legacy": "H0=[+vz,0,-vx], H1=[-vy,+vx,0]  (= -correct)",
        },
        "turns": {},
        "gap4_link": (
            "Same NHC H_att sign family as GAP-4 / bf2bfbd Jacobian fix; "
            "E2E A×C shows corrected sign amplifies drift vs legacy on SLALOM"
        ),
    }

    for turn_id, meta in TURNS.items():
        tc = meta["t_c"]
        stim = (tc - STIM_HALF, tc + STIM_HALF)
        effect = (tc + EFFECT_LAG - EFFECT_HALF, tc + EFFECT_LAG + EFFECT_HALF)
        # also early window from first divergence
        early = (1.3, 3.0) if turn_id == 1 else stim

        sa = summarize_window(a, *stim)
        sc = summarize_window(c, *stim)
        ea = summarize_window(a, *effect)
        ec = summarize_window(c, *effect)
        loc_stim = chain_localization(merged, *stim)
        loc_effect = chain_localization(merged, *effect)
        loc_early = chain_localization(merged, *early) if turn_id == 1 else None

        results["turns"][str(turn_id)] = {
            "role": meta["role"],
            "t_c": tc,
            "stim_window": stim,
            "effect_window": effect,
            "drift_A_at_tc": telem_drift_at(TELEM_A, tc),
            "drift_C_at_tc": telem_drift_at(TELEM_C, tc),
            "drift_A_at_effect": telem_drift_at(TELEM_A, tc + EFFECT_LAG),
            "drift_C_at_effect": telem_drift_at(TELEM_C, tc + EFFECT_LAG),
            "stim_A": sa,
            "stim_C": sc,
            "effect_A": ea,
            "effect_C": ec,
            "locus_stim": loc_stim,
            "locus_effect": loc_effect,
            "locus_early": loc_early,
        }

    # Whole early chain for turn 1 narrative: 1.3–5 s
    results["turn1_extended_1p3_5s"] = chain_localization(merged, 1.3, 5.0)

    # Figure: turn 1 time series
    t = merged["timestamp_s_A"].to_numpy()
    m = window_mask(t, 1.0, 5.5)
    w = merged.loc[m]
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(w["timestamp_s_A"], w["innov_norm_mps_A"], label="A correct")
    axes[0].plot(w["timestamp_s_A"], w["innov_norm_mps_C"], label="C legacy")
    axes[0].set_ylabel("‖innov‖")
    axes[0].legend(fontsize=8)
    axes[0].axvspan(1.0, 3.0, color="C0", alpha=0.08)
    axes[0].axvspan(3.0, 5.0, color="C3", alpha=0.08)
    axes[0].set_title("Turn1: stim [1–3] / effect [3–5]")

    axes[1].plot(w["timestamp_s_A"], w["dx_att_z_rad_A"], label="A dx_att_z")
    axes[1].plot(w["timestamp_s_A"], w["dx_att_z_rad_C"], label="C dx_att_z")
    axes[1].set_ylabel("dx_att_z (rad)")
    axes[1].legend(fontsize=8)

    axes[2].plot(w["timestamp_s_A"], w["k_att_max_A"], label="A k_att_max")
    axes[2].plot(w["timestamp_s_A"], w["k_att_max_C"], label="C k_att_max")
    axes[2].set_ylabel("k_att_max")
    axes[2].legend(fontsize=8)

    axes[3].plot(w["timestamp_s_A"], w["dx_vel_norm_mps_A"], label="A ‖dx_vel‖")
    axes[3].plot(w["timestamp_s_A"], w["dx_vel_norm_mps_C"], label="C ‖dx_vel‖")
    axes[3].set_ylabel("‖dx_vel‖")
    axes[3].set_xlabel("t (s)")
    axes[3].legend(fontsize=8)
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.axvline(2.0, color="0.4", ls="--", lw=0.8)
        ax.axvline(4.0, color="0.4", ls=":", lw=0.8)
    fig.suptitle("SLALOM A vs C NHC K/P — turn 1 window", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=140)
    plt.close(fig)

    # Verdict text
    t1 = results["turns"]["1"]
    loc = t1["locus_stim"]
    loc_e = t1["locus_effect"]
    verdict_lines = [
        f"Turn1 stim locus: {loc.get('locus', '?')}",
        f"Turn1 effect locus: {loc_e.get('locus', '?')}",
        f"dx_att_z opposite frac (stim)={loc.get('dx_att_z_opposite_sign_frac')}",
        f"dx_att ratio A/C (stim)={loc.get('dx_att_norm_ratio_A_over_C')}",
        f"dx_vel ratio A/C (stim)={loc.get('dx_vel_norm_ratio_A_over_C')}",
        f"innov ratio A/C (stim)={loc.get('innov_norm_ratio_A_over_C')}",
    ]
    results["verdict_bullets"] = verdict_lines

    with OUT_JSON.open("w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    # Markdown
    lines = [
        "# SLALOM A vs C — NHC K/P autopsy (turn 1 + controls 2/4)",
        "",
        "Question that closes the Jacobian thread: **why does corrected H_att "
        "amplify ~143× vs legacy**, given the same ω kinematics?",
        "",
        f"**H_att correct:** `{results['H_att_formulas']['correct']}`  ",
        f"**H_att legacy:** `{results['H_att_formulas']['legacy']}`  ",
        f"**Figure:** `{OUT_PNG.name}`  ",
        "",
        "## GAP-4 link",
        "",
        results["gap4_link"],
        "",
        "Same sign family as the NHC Jacobian that opened this investigation "
        "(bf2bfbd / GAP-4 attitude coupling). E2E A×C is the system-level "
        "consequence of that sign on SLALOM.",
        "",
    ]

    for turn_id in ("1", "2", "4"):
        tr = results["turns"][turn_id]
        lines += [
            f"## Turn {turn_id} ({tr['role']}) — t_c={tr['t_c']} s",
            "",
            f"- drift A/C @ t_c: **{tr['drift_A_at_tc']:.3f}** / {tr['drift_C_at_tc']:.3f} m",
            f"- drift A/C @ t_c+2s: **{tr['drift_A_at_effect']:.3f}** / "
            f"{tr['drift_C_at_effect']:.3f} m",
            "",
            "### Stim window `[t_c±1]`",
            "",
            "| Metric | A (correct) | C (legacy) | A/C |",
            "|--------|-------------|------------|-----|",
        ]
        sa, sc = tr["stim_A"], tr["stim_C"]
        loc = tr["locus_stim"]

        def row(name, ka, kc):
            ratio = (ka / kc) if kc and abs(kc) > 1e-15 else float("nan")
            lines.append(f"| {name} | {ka:.4g} | {kc:.4g} | {ratio:.3g} |")

        if sa.get("n", 0) and sc.get("n", 0):
            row("innov_norm_rms", sa["innov_norm_rms"], sc["innov_norm_rms"])
            row("k_att_max_mean", sa["k_att_max_mean"], sc["k_att_max_mean"])
            row("k_vel_max_mean", sa["k_vel_max_mean"], sc["k_vel_max_mean"])
            row("dx_att_norm_rms", sa["dx_att_norm_rms"], sc["dx_att_norm_rms"])
            row("dx_att_z_rms", sa["dx_att_z_rms"], sc["dx_att_z_rms"])
            row("dx_vel_norm_rms", sa["dx_vel_norm_rms"], sc["dx_vel_norm_rms"])
            row("P_pre_aa_frob", sa["P_pre_aa_frob_mean"], sc["P_pre_aa_frob_mean"])
            row("P_pre_pv_frob", sa["P_pre_pv_frob_mean"], sc["P_pre_pv_frob_mean"])
            row("|v_body_y| rms", sa["v_body_y_rms"], sc["v_body_y_rms"])

        lines += [
            "",
            f"**Locus (stim):** {loc.get('locus', '?')}  ",
            f"dx_att_z opposite-sign frac: **{loc.get('dx_att_z_opposite_sign_frac', float('nan')):.2f}**; "
            f"sign corr: {loc.get('dx_att_z_sign_corr_A_vs_C', float('nan')):.3f}",
            "",
            "### Effect window `[t_c+2±1]`",
            "",
        ]
        le = tr["locus_effect"]
        ea, ec = tr["effect_A"], tr["effect_C"]
        if ea.get("n") and ec.get("n"):
            lines += [
                "| Metric | A | C | A/C |",
                "|--------|---|---|-----|",
            ]
            def row2(name, ka, kc):
                ratio = (ka / kc) if kc and abs(kc) > 1e-15 else float("nan")
                lines.append(f"| {name} | {ka:.4g} | {kc:.4g} | {ratio:.3g} |")

            row2("innov_norm_rms", ea["innov_norm_rms"], ec["innov_norm_rms"])
            row2("dx_att_norm_rms", ea["dx_att_norm_rms"], ec["dx_att_norm_rms"])
            row2("dx_vel_norm_rms", ea["dx_vel_norm_rms"], ec["dx_vel_norm_rms"])
            row2("|v_body_y| rms", ea["v_body_y_rms"], ec["v_body_y_rms"])
        lines += [
            "",
            f"**Locus (effect):** {le.get('locus', '?')}",
            "",
        ]

    ext = results["turn1_extended_1p3_5s"]
    lines += [
        "## Chain reading (turn 1, 1.3–5 s)",
        "",
        f"- locus: **{ext.get('locus')}**",
        f"- innov A/C: {ext.get('innov_norm_ratio_A_over_C')}",
        f"- dx_att A/C: {ext.get('dx_att_norm_ratio_A_over_C')}",
        f"- dx_vel A/C: {ext.get('dx_vel_norm_ratio_A_over_C')}",
        f"- dx_att_z opposite frac: {ext.get('dx_att_z_opposite_sign_frac')}",
        f"- locus flags: {ext.get('flags')}",
        "",
        "## Why correct amplifies (mechanistic)",
        "",
        "1. `H_att_correct = −H_att_legacy` (only attitude columns of NHC H).",
        "2. `K = P Hᵀ S⁻¹` ⇒ attitude block of K flips sign with H_att "
        "(vel rows of H unchanged).",
        "3. `dx_att = K y` ⇒ yaw/roll correction applied **opposite** under "
        "correct vs legacy for the same innov.",
        "4. On SLALOM, legacy’s “wrong” sign happens to **oppose** the "
        "attitude error that NHC innov implies from the ideal kinematic "
        "trajectory + filter lag — damping the loop. Correct sign **agrees** "
        "with the linearization and, when innov is contaminated by "
        "prediction error during the turn, drives attitude further from "
        "truth → `v_body` lateral grows → innov grows → positive feedback "
        "into `v_ned` → integrated drift.",
        "5. Numbers above must show: early opposite `dx_att_z`, then growing "
        "`innov` / `dx_vel` / `|v_body_y|` under A vs flat under C.",
        "",
        "## OQ9 / T6 (not this autopsy)",
        "",
        "- **T6:** open alias-shift miss in clean regime — not structural here.",
        "- **H-DEG-ATT-LAG (OQ9):** turns 9–10 when drift_A ≳ 20 m — parked.",
        "",
        "## Verdict bullets",
        "",
    ]
    for b in verdict_lines:
        lines.append(f"- {b}")
    lines.append("")

    OUT_MD.write_text("\n".join(lines), encoding="utf-8")
    print(OUT_MD.read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
