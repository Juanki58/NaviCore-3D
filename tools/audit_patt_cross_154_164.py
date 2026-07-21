#!/usr/bin/env python3
"""Priority (1): P_att cross blocks XZ/YZ in [1.54,1.64] latch vs ctrl.

ATT_X=roll, ATT_Y=pitch, ATT_Z=yaw.
If cross blocks large/grow → covariance escape within attitude.
Else → step to f_va/predict X/Y.
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
T0, T1 = 1.54, 1.64
T_BREAK = 1.59
SEED = 71
T2 = 3.736646e-6
TMAX = 0.65

sys.path.insert(0, str(ROOT))
from run_all_benchmarks import run_benchmark  # noqa: E402

NEED = [
    "P_pre_att_xx",
    "P_pre_att_yy",
    "P_pre_att_zz",
    "P_pre_att_xz",
    "P_pre_att_yz",
    "P_pre_att_xy",
    "dx_att_x_rad",
    "dx_att_y_rad",
    "dx_att_z_rad",
    "innov_norm_mps",
]


def run_arm(name: str, *, lam: float, gate: float | None) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    audit = OUT / f"{name}_nhc_block_audit.csv"
    if audit.exists():
        audit.unlink()
    env = os.environ.copy()
    env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = str(audit)
    r = run_benchmark(
        f"PattCross {name}",
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


def corr_norm(p_xz, p_xx, p_zz):
    """Correlation ρ = Pxz / sqrt(Pxx Pzz)."""
    den = np.sqrt(np.maximum(p_xx, 0) * np.maximum(p_zz, 0))
    return p_xz / np.maximum(den, 1e-30)


def analyze(arm: str, path: Path) -> dict:
    d = pd.read_csv(path)
    miss = [c for c in NEED if c not in d.columns]
    if miss:
        raise KeyError(f"{arm} missing {miss}")
    m = (d["timestamp_s"] >= T0) & (d["timestamp_s"] <= T1)
    w = d.loc[m].copy()
    t = w["timestamp_s"].to_numpy(float)

    pxz = w["P_pre_att_xz"].to_numpy(float)
    pyz = w["P_pre_att_yz"].to_numpy(float)
    pxy = w["P_pre_att_xy"].to_numpy(float)
    pxx = w["P_pre_att_xx"].to_numpy(float)
    pyy = w["P_pre_att_yy"].to_numpy(float)
    pzz = w["P_pre_att_zz"].to_numpy(float)
    rho_xz = corr_norm(pxz, pxx, pzz)
    rho_yz = corr_norm(pyz, pyy, pzz)

    dx_x = w["dx_att_x_rad"].to_numpy(float)
    dx_y = w["dx_att_y_rad"].to_numpy(float)
    dx_z = w["dx_att_z_rad"].to_numpy(float)

    pre = t < T_BREAK
    post = t >= T_BREAK

    def block(mask):
        return {
            "n": int(mask.sum()),
            "mean_P_xz": float(np.mean(pxz[mask])),
            "mean_P_yz": float(np.mean(pyz[mask])),
            "mean_P_xy": float(np.mean(pxy[mask])),
            "mean_rho_xz": float(np.mean(rho_xz[mask])),
            "mean_rho_yz": float(np.mean(rho_yz[mask])),
            "max_abs_P_xz": float(np.max(np.abs(pxz[mask]))),
            "max_abs_P_yz": float(np.max(np.abs(pyz[mask]))),
            "mean_P_xx": float(np.mean(pxx[mask])),
            "mean_P_yy": float(np.mean(pyy[mask])),
            "mean_P_zz": float(np.mean(pzz[mask])),
            "mean_abs_dx_x": float(np.mean(np.abs(dx_x[mask]))),
            "mean_abs_dx_y": float(np.mean(np.abs(dx_y[mask]))),
            "mean_abs_dx_z": float(np.mean(np.abs(dx_z[mask]))),
            "sum_dx_y": float(np.sum(dx_y[mask])),
            "sum_dx_x": float(np.sum(dx_x[mask])),
        }

    ticks = pd.DataFrame(
        {
            "t": t,
            "P_xz": pxz,
            "P_yz": pyz,
            "P_xy": pxy,
            "rho_xz": rho_xz,
            "rho_yz": rho_yz,
            "P_xx": pxx,
            "P_yy": pyy,
            "P_zz": pzz,
            "dx_x": dx_x,
            "dx_y": dx_y,
            "dx_z": dx_z,
            "y_norm": w["innov_norm_mps"].to_numpy(float),
        }
    )
    ticks.to_csv(OUT / f"patt_cross_{arm}_ticks.csv", index=False)

    return {
        "pre": block(pre),
        "post": block(post),
        "full": block(np.ones(len(t), dtype=bool)),
        "ticks_csv": str(OUT / f"patt_cross_{arm}_ticks.csv"),
        "growth": {
            "P_yz_post_over_pre": block(post)["mean_P_yz"]
            / max(abs(block(pre)["mean_P_yz"]), 1e-30)
            * np.sign(block(post)["mean_P_yz"] or 1),
            "abs_P_yz_post_over_pre": block(post)["max_abs_P_yz"]
            / max(block(pre)["max_abs_P_yz"], 1e-30),
            "abs_P_xz_post_over_pre": block(post)["max_abs_P_xz"]
            / max(block(pre)["max_abs_P_xz"], 1e-30),
            "rho_yz_post_minus_pre": block(post)["mean_rho_yz"] - block(pre)["mean_rho_yz"],
            "rho_xz_post_minus_pre": block(post)["mean_rho_xz"] - block(pre)["mean_rho_xz"],
        },
    }


def main() -> None:
    reuse = "--reuse-audit" in sys.argv
    if not reuse:
        print("Running ctrl…")
        run_arm("ctrl", lam=0.0, gate=None)
        print("Running latch…")
        run_arm("latch", lam=1.0, gate=T2)

    report = {"window_s": [T0, T1], "convention": "ATT_X=roll, ATT_Y=pitch, ATT_Z=yaw", "arms": {}}
    for arm in ("ctrl", "latch"):
        report["arms"][arm] = analyze(arm, OUT / f"{arm}_nhc_block_audit.csv")

    L = report["arms"]["latch"]
    C = report["arms"]["ctrl"]

    # Verdict thresholds
    # "large" cross: |ρ| > 0.3 or |P_yz| comparable to sqrt(Pyy Pzz)*0.3
    latch_rho_yz = abs(L["full"]["mean_rho_yz"])
    latch_rho_xz = abs(L["full"]["mean_rho_xz"])
    grows = (
        L["growth"]["abs_P_yz_post_over_pre"] >= 1.5
        or abs(L["growth"]["rho_yz_post_minus_pre"]) >= 0.15
    )
    large = latch_rho_yz >= 0.3 or latch_rho_xz >= 0.3
    # vs ctrl: latch amplified?
    latch_vs_ctrl_yz = abs(L["full"]["mean_P_yz"]) / max(abs(C["full"]["mean_P_yz"]), 1e-30)

    # Also: is dx_y (pitch correction) material under latch?
    dx_y_material = L["full"]["mean_abs_dx_y"] > 1e-4

    if large and grows:
        label = "PATT_CROSS_ESCAPE"
        reading = (
            "P_att cross blocks (esp. pitch–yaw P_yz / ρ_yz) are material and grow "
            "across the break under latch — covariance escape within attitude, same "
            "pattern as att→bias_gz. Freezing dx_att_z does not isolate pitch."
        )
        next_step = "Design must address P_att cross (or joint att axes), not Z-only."
    elif large and not grows:
        label = "PATT_CROSS_LARGE_FLAT"
        reading = (
            "P_att cross blocks are material but do not surge at the break — "
            "present as standing coupling; may still mediate leakage but not the "
            "timing of the pitch ramp alone."
        )
        next_step = "Check whether standing cross + rising innov explains pitch; else f_va/predict."
    elif not large and dx_y_material:
        label = "PATT_CROSS_SMALL_DX_Y_ACTIVE"
        reading = (
            "P_att cross blocks small; but NHC still applies dx_att_y (pitch) — "
            "direct NHC X/Y path, not Z-cross covariance."
        )
        next_step = "Priority (3) elevated: autopsy dx_att_x/y vs innov; then f_va if needed."
    else:
        label = "PATT_CROSS_SMALL"
        reading = (
            "P_att cross XZ/YZ small/flat — does not explain pitch growth via "
            "covariance escape from latched Z. Proceed to (2) f_va/predict X/Y."
        )
        next_step = "Priority (2): f_va / quaternion predict coupling into pitch."

    report["verdict"] = {
        "label": label,
        "reading": reading,
        "next": next_step,
        "latch_mean_rho_yz": L["full"]["mean_rho_yz"],
        "latch_mean_rho_xz": L["full"]["mean_rho_xz"],
        "latch_abs_P_yz_growth": L["growth"]["abs_P_yz_post_over_pre"],
        "latch_vs_ctrl_abs_P_yz": latch_vs_ctrl_yz,
        "latch_mean_abs_dx_y": L["full"]["mean_abs_dx_y"],
        "latch_mean_abs_dx_z": L["full"]["mean_abs_dx_z"],
    }

    # figure
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for arm, color in ("ctrl", "C0"), ("latch", "C3"):
        tdf = pd.read_csv(OUT / f"patt_cross_{arm}_ticks.csv")
        axes[0].plot(tdf["t"], tdf["P_yz"], color=color, lw=1.2, label=f"{arm} P_yz")
        axes[0].plot(tdf["t"], tdf["P_xz"], color=color, lw=1.0, ls="--", label=f"{arm} P_xz")
        axes[1].plot(tdf["t"], tdf["rho_yz"], color=color, lw=1.2, label=f"{arm} ρ_yz")
        axes[1].plot(tdf["t"], tdf["rho_xz"], color=color, lw=1.0, ls="--", label=f"{arm} ρ_xz")
        axes[2].plot(tdf["t"], tdf["dx_y"], color=color, lw=1.2, label=f"{arm} dx_att_y")
        axes[2].plot(tdf["t"], tdf["dx_z"], color=color, lw=1.0, ls=":", label=f"{arm} dx_att_z")
    for ax in axes:
        ax.axvline(T_BREAK, color="gray", ls="--", alpha=0.5)
        ax.legend(fontsize=7, ncol=2)
        ax.axhline(0, color="gray", lw=0.4)
    axes[0].set_ylabel("P cross")
    axes[0].set_title("P_att cross (pitch–yaw / roll–yaw) [1.54→1.64]")
    axes[1].set_ylabel("ρ")
    axes[2].set_ylabel("dx_att")
    axes[2].set_xlabel("t [s]")
    fig.tight_layout()
    fig_path = OUT / "fig_patt_cross_154_164.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "patt_cross_154_164.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    lines = [
        "# P_att cross blocks [1.54→1.64] — priority (1)",
        "",
        f"**Verdict:** `{label}`",
        "",
        reading,
        "",
        f"**Next:** {next_step}",
        "",
        "Convention: ATT_X=roll, ATT_Y=pitch, ATT_Z=yaw.",
        "",
        "## Summary",
        "",
        "| Arm | mean P_yz | mean ρ_yz | mean P_xz | mean ρ_xz | mean|dx_y| | mean|dx_z| |",
        "|-----|-----------|-----------|-----------|-----------|-------------|-------------|",
    ]
    for arm in ("ctrl", "latch"):
        f = report["arms"][arm]["full"]
        lines.append(
            f"| {arm} | {f['mean_P_yz']:+.4e} | {f['mean_rho_yz']:+.3f} | "
            f"{f['mean_P_xz']:+.4e} | {f['mean_rho_xz']:+.3f} | "
            f"{f['mean_abs_dx_y']:.3e} | {f['mean_abs_dx_z']:.3e} |"
        )
    lines += [
        "",
        "## Latch pre vs post break",
        "",
        "| Phase | P_yz | ρ_yz | P_xz | ρ_xz | |dx_y| |",
        "|-------|------|------|------|------|-------|",
        f"| pre | {L['pre']['mean_P_yz']:+.4e} | {L['pre']['mean_rho_yz']:+.3f} | "
        f"{L['pre']['mean_P_xz']:+.4e} | {L['pre']['mean_rho_xz']:+.3f} | "
        f"{L['pre']['mean_abs_dx_y']:.3e} |",
        f"| post | {L['post']['mean_P_yz']:+.4e} | {L['post']['mean_rho_yz']:+.3f} | "
        f"{L['post']['mean_P_xz']:+.4e} | {L['post']['mean_rho_xz']:+.3f} | "
        f"{L['post']['mean_abs_dx_y']:.3e} |",
        "",
        "## Tick table — latch",
        "",
        "| t | P_yz | ρ_yz | P_xz | ρ_xz | dx_y | dx_z | ‖y‖ |",
        "|---|------|------|------|------|------|------|-----|",
    ]
    lt = pd.read_csv(OUT / "patt_cross_latch_ticks.csv")
    for _, r in lt.iterrows():
        lines.append(
            f"| {r.t:.3f} | {r.P_yz:+.4e} | {r.rho_yz:+.3f} | {r.P_xz:+.4e} | "
            f"{r.rho_xz:+.3f} | {r.dx_y:+.3e} | {r.dx_z:+.3e} | {r.y_norm:.3f} |"
        )
    lines += ["", f"Figure: `{fig_path.name}`", ""]
    (OUT / "patt_cross_154_164.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report["verdict"], indent=2))


if __name__ == "__main__":
    main()
