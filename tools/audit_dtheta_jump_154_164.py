#!/usr/bin/env python3
"""Why does ||δθ|| jump ~7.5× near t=1.59? Autopsy [1.54, 1.64] latch vs ctrl.

Decompose: K_att jump vs innov (y) jump vs product.
Also separate: state attitude error vs truth vs per-tick ||dx_att|| from NHC.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"
T0, T1 = 1.54, 1.64
T_BREAK = 1.59

V = 50.0 / 3.6
OMEGA = 2.0 * np.pi / 4.0
YAW_AMP = 3.0 / (V * OMEGA)
BASE = np.pi / 2.0


def wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def build_arm(arm: str) -> pd.DataFrame:
    audit = pd.read_csv(OUT / f"{arm}_nhc_block_audit.csv")
    telem = pd.read_csv(ROOT / f"docs/benchmarks/slalom_pattbias_{arm}_s71_telemetry.csv")
    telem["t"] = telem["time_us"].astype(float) * 1e-6

    m = (audit["timestamp_s"] >= T0) & (audit["timestamp_s"] <= T1)
    a = audit.loc[m].copy()
    t = a["timestamp_s"].to_numpy(float)

    # NHC innov (feeds dx_att via K)
    iy = a["innov_y_mps"].to_numpy(float)
    iz = a["innov_z_mps"].to_numpy(float)
    y_norm = a["innov_norm_mps"].to_numpy(float)

    dx_x = a["dx_att_x_rad"].to_numpy(float)
    dx_y = a["dx_att_y_rad"].to_numpy(float)
    dx_z = a["dx_att_z_rad"].to_numpy(float)
    dx_norm = a["dx_att_norm_rad"].to_numpy(float)
    k_att = a["k_att_max"].to_numpy(float)

    # Effective gain proxy: ||dx_att|| / ||y||  (same units as K * innov → att)
    # Note: K has mixed units; this is |δθ| / |y| scale
    g_eff = dx_norm / np.maximum(y_norm, 1e-12)

    # Identity residual unavailable without full K rows; use consistency:
    # if K~const, dx_norm ∝ y_norm; correlate and scale fit
    # Also compare g_eff to k_att_max (order-of-magnitude / relative jump)

    # State attitude error vs truth
    yaw_f = np.interp(t, telem["t"], telem["yaw"].to_numpy(float))
    roll_f = np.interp(t, telem["t"], telem["roll"].to_numpy(float))
    pitch_f = np.interp(t, telem["t"], telem["pitch"].to_numpy(float))
    yaw_t = BASE + YAW_AMP * np.sin(OMEGA * t)
    dyaw = wrap(yaw_f - yaw_t)
    dtheta_state = np.sqrt(roll_f**2 + pitch_f**2 + dyaw**2)

    p_aa = a["P_pre_aa_frob"].to_numpy(float)
    p_az_bg = a["P_pre_att_z_bias_gz"].to_numpy(float)

    return pd.DataFrame(
        {
            "t": t,
            "innov_y": iy,
            "innov_z": iz,
            "y_norm": y_norm,
            "dx_att_x": dx_x,
            "dx_att_y": dx_y,
            "dx_att_z": dx_z,
            "dx_att_norm": dx_norm,
            "k_att_max": k_att,
            "g_eff_dx_over_y": g_eff,
            "dtheta_state_rad": dtheta_state,
            "dtheta_state_deg": np.rad2deg(dtheta_state),
            "dyaw_deg": np.rad2deg(dyaw),
            "roll_deg": np.rad2deg(roll_f),
            "pitch_deg": np.rad2deg(pitch_f),
            "P_pre_aa_frob": p_aa,
            "P_pre_att_z_bias_gz": p_az_bg,
        }
    )


def jump_analysis(df: pd.DataFrame) -> dict:
    """Find largest single-tick jump in dx_att_norm and dtheta_state; classify cause."""
    t = df["t"].to_numpy()
    dx = df["dx_att_norm"].to_numpy()
    y = df["y_norm"].to_numpy()
    k = df["k_att_max"].to_numpy()
    g = df["g_eff_dx_over_y"].to_numpy()
    st = df["dtheta_state_deg"].to_numpy()

    # single-step ratios (avoid /0)
    def step_ratio(x):
        r = np.ones(len(x))
        r[1:] = np.abs(x[1:]) / np.maximum(np.abs(x[:-1]), 1e-15)
        return r

    r_dx = step_ratio(dx)
    r_y = step_ratio(y)
    r_k = step_ratio(k)
    r_g = step_ratio(g)
    r_st = step_ratio(st)

    i_dx = int(np.argmax(r_dx[1:])) + 1 if len(r_dx) > 1 else 0
    i_st = int(np.argmax(r_st[1:])) + 1 if len(r_st) > 1 else 0

    def classify_at(i: int, label: str) -> dict:
        if i <= 0:
            return {"label": "no_jump", "i": i}
        # relative growth of factors
        ry, rk, rg = r_y[i], r_k[i], r_g[i]
        rdx = r_dx[i]
        # dominant factor
        if rk >= 1.5 * max(ry, 1.0) and rk >= 1.8:
            cause = "K_JUMP"
        elif ry >= 1.5 * max(rk, 1.0) and ry >= 1.8:
            cause = "Y_JUMP"
        elif rdx >= 1.8 and ry >= 1.3 and rk >= 1.3:
            cause = "PRODUCT"
        elif rdx >= 1.8 and max(ry, rk) < 1.5:
            cause = "PRODUCT_OR_OTHER"
        elif rdx < 1.5:
            cause = "NO_CLEAN_DX_JUMP"
        else:
            # whichever grows more
            cause = "Y_JUMP" if ry >= rk else "K_JUMP" if rk > ry else "PRODUCT"

        return {
            "series": label,
            "t": float(t[i]),
            "t_prev": float(t[i - 1]),
            "cause": cause,
            "ratio_dx": float(rdx),
            "ratio_y": float(ry),
            "ratio_k_att_max": float(rk),
            "ratio_g_eff": float(rg),
            "dx_prev": float(dx[i - 1]),
            "dx_now": float(dx[i]),
            "y_prev": float(y[i - 1]),
            "y_now": float(y[i]),
            "k_prev": float(k[i - 1]),
            "k_now": float(k[i]),
            "g_prev": float(g[i - 1]),
            "g_now": float(g[i]),
        }

    # window means before/after break
    pre = df["t"] < T_BREAK
    post = df["t"] >= T_BREAK

    def mean_block(mask, col):
        return float(df.loc[mask, col].mean()) if mask.any() else float("nan")

    # algebraic proxy: dx ≈ g_eff * y  by construction of g_eff
    # check pearson dx vs y and vs k
    def corr(a, b):
        if np.std(a) < 1e-30 or np.std(b) < 1e-30:
            return float("nan")
        return float(np.corrcoef(a, b)[0, 1])

    return {
        "max_step_dx_att": classify_at(i_dx, "dx_att_norm"),
        "max_step_dtheta_state": {
            "t": float(t[i_st]),
            "t_prev": float(t[i_st - 1]) if i_st > 0 else None,
            "ratio": float(r_st[i_st]),
            "deg_prev": float(st[i_st - 1]) if i_st > 0 else None,
            "deg_now": float(st[i_st]),
        },
        "pre_break_means": {
            "dx_att_norm": mean_block(pre, "dx_att_norm"),
            "y_norm": mean_block(pre, "y_norm"),
            "k_att_max": mean_block(pre, "k_att_max"),
            "g_eff": mean_block(pre, "g_eff_dx_over_y"),
            "dtheta_state_deg": mean_block(pre, "dtheta_state_deg"),
            "P_aa_frob": mean_block(pre, "P_pre_aa_frob"),
        },
        "post_break_means": {
            "dx_att_norm": mean_block(post, "dx_att_norm"),
            "y_norm": mean_block(post, "y_norm"),
            "k_att_max": mean_block(post, "k_att_max"),
            "g_eff": mean_block(post, "g_eff_dx_over_y"),
            "dtheta_state_deg": mean_block(post, "dtheta_state_deg"),
            "P_aa_frob": mean_block(post, "P_pre_aa_frob"),
        },
        "ratios_post_over_pre": {
            "dx_att_norm": mean_block(post, "dx_att_norm")
            / max(mean_block(pre, "dx_att_norm"), 1e-15),
            "y_norm": mean_block(post, "y_norm") / max(mean_block(pre, "y_norm"), 1e-15),
            "k_att_max": mean_block(post, "k_att_max")
            / max(mean_block(pre, "k_att_max"), 1e-15),
            "g_eff": mean_block(post, "g_eff_dx_over_y")
            / max(mean_block(pre, "g_eff_dx_over_y"), 1e-15),
            "dtheta_state_deg": mean_block(post, "dtheta_state_deg")
            / max(mean_block(pre, "dtheta_state_deg"), 1e-15),
            "P_aa_frob": mean_block(post, "P_pre_aa_frob")
            / max(mean_block(pre, "P_pre_aa_frob"), 1e-15),
        },
        "pearson_dx_vs_y": corr(dx, y),
        "pearson_dx_vs_k": corr(dx, k),
        "pearson_dx_vs_g": corr(dx, g),
        "note_identity": (
            "Full K_att rows not in audit; g_eff:=||dx_att||/||y|| is the realized "
            "gain scale. k_att_max is max|K| over att rows — relative jumps comparable."
        ),
    }


def main() -> None:
    report = {"window_s": [T0, T1], "break_s": T_BREAK, "arms": {}}
    frames = {}
    for arm in ("ctrl", "latch"):
        df = build_arm(arm)
        frames[arm] = df
        df.to_csv(OUT / f"dtheta_jump_{arm}_ticks.csv", index=False)
        report["arms"][arm] = jump_analysis(df)

    L = report["arms"]["latch"]
    C = report["arms"]["ctrl"]

    # Overall verdict
    cause = L["max_step_dx_att"]["cause"]
    latch_has_dx_jump = L["max_step_dx_att"].get("ratio_dx", 1) >= 1.8
    ctrl_has_dx_jump = C["max_step_dx_att"].get("ratio_dx", 1) >= 1.8
    latch_state_jump = L["max_step_dtheta_state"]["ratio"] >= 1.8
    ctrl_state_jump = C["max_step_dtheta_state"]["ratio"] >= 1.8

    ratios = L["ratios_post_over_pre"]
    if ratios["y_norm"] >= 2.0 and ratios["k_att_max"] < 1.5:
        window_cause = "Y_JUMP_WINDOW"
    elif ratios["k_att_max"] >= 2.0 and ratios["y_norm"] < 1.5:
        window_cause = "K_JUMP_WINDOW"
    elif ratios["y_norm"] >= 1.5 and ratios["k_att_max"] >= 1.5:
        window_cause = "PRODUCT_WINDOW"
    elif ratios["g_eff"] >= 2.0 and ratios["y_norm"] < 1.5:
        window_cause = "G_EFF_JUMP_WINDOW"
    else:
        window_cause = cause

    if latch_has_dx_jump and not ctrl_has_dx_jump:
        ctrl_note = "CTRL_NO_DX_JUMP — latch-specific / cascade-critical, not scenario accident"
    elif latch_has_dx_jump and ctrl_has_dx_jump:
        ctrl_note = "CTRL_ALSO_JUMPS — check if weaker; may be scenario kinematics shared"
    else:
        ctrl_note = (
            "NO_CLEAN_SINGLE_TICK_DX_JUMP — state ||δθ|| may ramp across several ticks; "
            "use pre/post means and substructure"
        )

    # substructure: half-split window
    for arm, df in frames.items():
        mid = 0.5 * (T0 + T1)
        h1 = df[df["t"] < mid]
        h2 = df[df["t"] >= mid]
        report["arms"][arm]["halves"] = {
            "first": {
                "t_span": [float(h1["t"].min()), float(h1["t"].max())],
                "mean_dx": float(h1["dx_att_norm"].mean()),
                "mean_y": float(h1["y_norm"].mean()),
                "mean_k": float(h1["k_att_max"].mean()),
                "mean_dtheta_deg": float(h1["dtheta_state_deg"].mean()),
            },
            "second": {
                "t_span": [float(h2["t"].min()), float(h2["t"].max())],
                "mean_dx": float(h2["dx_att_norm"].mean()),
                "mean_y": float(h2["y_norm"].mean()),
                "mean_k": float(h2["k_att_max"].mean()),
                "mean_dtheta_deg": float(h2["dtheta_state_deg"].mean()),
            },
        }

    report["verdict"] = {
        "label": window_cause,
        "max_step_cause_latch": cause,
        "ctrl_comparison": ctrl_note,
        "latch_ratios_post_over_pre": ratios,
        "ctrl_ratios_post_over_pre": C["ratios_post_over_pre"],
        "reading": (
            f"Window post/pre (latch): dx×{ratios['dx_att_norm']:.2f}, "
            f"y×{ratios['y_norm']:.2f}, k_att_max×{ratios['k_att_max']:.2f}, "
            f"g_eff×{ratios['g_eff']:.2f}, state||δθ||×{ratios['dtheta_state_deg']:.2f}. "
            f"Max single-tick dx cause: {cause}. {ctrl_note}."
        ),
        "design": (
            "If K_JUMP: preregister gain clamp on NHC→att (ZUPT/GNSS pattern). "
            "If Y_JUMP: step back — what feeds innov at that tick (state vel already dirty?). "
            "If PRODUCT: conditional on both. "
            "If state||δθ|| jumps without dx spike: accumulation / predict path, not one NHC update."
        ),
    }

    # figure
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for arm, color in ("ctrl", "C0"), ("latch", "C3"):
        df = frames[arm]
        axes[0].plot(df["t"], df["dx_att_norm"], color=color, lw=1.2, label=f"{arm} ||dx_att||")
        axes[1].plot(df["t"], df["y_norm"], color=color, lw=1.2, label=f"{arm} ||y||")
        axes[2].plot(df["t"], df["k_att_max"], color=color, lw=1.2, label=f"{arm} k_att_max")
        axes[2].plot(
            df["t"], df["g_eff_dx_over_y"], color=color, ls="--", lw=1.0, label=f"{arm} g_eff"
        )
        axes[3].plot(
            df["t"], df["dtheta_state_deg"], color=color, lw=1.2, label=f"{arm} ||δθ||_state°"
        )
    for ax in axes:
        ax.axvline(T_BREAK, color="gray", ls="--", alpha=0.6)
        ax.legend(fontsize=7, ncol=2)
    axes[0].set_ylabel("||dx_att||")
    axes[0].set_title("δθ jump autopsy [1.54→1.64] — K vs y vs state error")
    axes[1].set_ylabel("||y|| NHC")
    axes[2].set_ylabel("K / g_eff")
    axes[3].set_ylabel("||δθ||_state°")
    axes[3].set_xlabel("t [s]")
    fig.tight_layout()
    fig_path = OUT / "fig_dtheta_jump_154_164.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "dtheta_jump_154_164.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    # markdown tables
    lines = [
        "# ‖δθ‖ jump autopsy [1.54→1.64] — K vs y vs state",
        "",
        f"**Verdict:** `{report['verdict']['label']}`",
        "",
        report["verdict"]["reading"],
        "",
        report["verdict"]["design"],
        "",
        "## Ratios post-break / pre-break (means)",
        "",
        "| Arm | ‖dx_att‖ | ‖y‖ | k_att_max | g_eff | ‖δθ‖_state | P_aa_frob |",
        "|-----|----------|-----|-----------|-------|------------|-----------|",
    ]
    for arm in ("ctrl", "latch"):
        r = report["arms"][arm]["ratios_post_over_pre"]
        lines.append(
            f"| {arm} | {r['dx_att_norm']:.2f}× | {r['y_norm']:.2f}× | "
            f"{r['k_att_max']:.2f}× | {r['g_eff']:.2f}× | "
            f"{r['dtheta_state_deg']:.2f}× | {r['P_aa_frob']:.2f}× |"
        )
    lines += [
        "",
        "## Max single-tick jump (latch)",
        "",
    ]
    ms = L["max_step_dx_att"]
    lines += [
        f"- cause: **{ms.get('cause')}** @ t={ms.get('t')}",
        f"- ‖dx_att‖: {ms.get('dx_prev'):.4e} → {ms.get('dx_now'):.4e} ({ms.get('ratio_dx'):.2f}×)",
        f"- ‖y‖: {ms.get('y_prev'):.4f} → {ms.get('y_now'):.4f} ({ms.get('ratio_y'):.2f}×)",
        f"- k_att_max: {ms.get('k_prev'):.4f} → {ms.get('k_now'):.4f} ({ms.get('ratio_k_att_max'):.2f}×)",
        f"- g_eff: {ms.get('g_prev'):.4e} → {ms.get('g_now'):.4e} ({ms.get('ratio_g_eff'):.2f}×)",
        "",
        "## Tick table — latch",
        "",
        "| t | ‖y‖ | k_att_max | g_eff | ‖dx_att‖ | dx_z | ‖δθ‖_state° | P_aa |",
        "|---|-----|-----------|-------|----------|------|-------------|------|",
    ]
    for _, r in frames["latch"].iterrows():
        lines.append(
            f"| {r.t:.3f} | {r.y_norm:.4f} | {r.k_att_max:.4f} | {r.g_eff_dx_over_y:.4e} | "
            f"{r.dx_att_norm:.4e} | {r.dx_att_z:+.3e} | {r.dtheta_state_deg:.3f} | "
            f"{r.P_pre_aa_frob:.4e} |"
        )
    lines += [
        "",
        "## Tick table — ctrl",
        "",
        "| t | ‖y‖ | k_att_max | g_eff | ‖dx_att‖ | ‖δθ‖_state° |",
        "|---|-----|-----------|-------|----------|-------------|",
    ]
    for _, r in frames["ctrl"].iterrows():
        lines.append(
            f"| {r.t:.3f} | {r.y_norm:.4f} | {r.k_att_max:.4f} | {r.g_eff_dx_over_y:.4e} | "
            f"{r.dx_att_norm:.4e} | {r.dtheta_state_deg:.3f} |"
        )
    lines += [
        "",
        f"Figure: `{fig_path.name}`",
        "",
        L["note_identity"] if "note_identity" in L else report["arms"]["latch"].get(
            "note_identity", ""
        ),
        "",
    ]
    # fix note access
    lines[-2] = report["arms"]["latch"].get(
        "note_identity",
        "Full K rows not logged; g_eff:=||dx||/||y|| is realized gain scale.",
    )
    # actually jump_analysis puts note inside the returned dict at top level of arm
    # I put note_identity inside jump_analysis return - good
    (OUT / "dtheta_jump_154_164.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report["verdict"], indent=2))
    print("LATCH max step", L["max_step_dx_att"])
    print("LATCH ratios", ratios)
    print("CTRL ratios", C["ratios_post_over_pre"])


if __name__ == "__main__":
    main()
