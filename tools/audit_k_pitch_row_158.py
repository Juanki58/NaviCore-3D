#!/usr/bin/env python3
"""Does K[ATT_Y,:] differ latch vs ctrl around t≈1.58? Or only y?

Latch zeros dx_att_z post-hoc after δx=Ky; K/Joseph unchanged in-tick.
If K_pitch row identical → sign-flip of dx_y is innov composition (trajectory).
If K_pitch row differs → prior-state divergence already moved the gain (still not
in-tick Z↔Y coupling inside one solve).
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
T_FLIP = 1.58  # ctrl dx_y sign change neighborhood
SEED = 71
T2 = 3.736646e-6
TMAX = 0.65

NEED = [
    "k_att_y0",
    "k_att_y1",
    "k_att_z0",
    "k_att_z1",
    "dx_att_y_via_innov_y",
    "dx_att_y_via_innov_z",
    "dx_att_z_raw",
    "dx_att_y_rad",
    "dx_att_z_rad",
    "innov_y_mps",
    "innov_z_mps",
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
        f"KPitchRow {name}",
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


def rel_diff(a: float, b: float) -> float:
    den = max(abs(a), abs(b), 1e-30)
    return abs(a - b) / den


def analyze() -> dict:
    arms = {}
    for arm in ("ctrl", "latch"):
        d = pd.read_csv(OUT / f"{arm}_nhc_block_audit.csv")
        miss = [c for c in NEED if c not in d.columns]
        if miss:
            raise KeyError(f"{arm} missing {miss} — rebuild sim")
        m = (d["timestamp_s"] >= T0) & (d["timestamp_s"] <= T1)
        w = d.loc[m].copy().reset_index(drop=True)
        # consistency: dx_y ≈ via_y + via_z
        recon = w["dx_att_y_via_innov_y"] + w["dx_att_y_via_innov_z"]
        resid = (w["dx_att_y_rad"] - recon).abs().max()
        ticks = pd.DataFrame(
            {
                "t": w["timestamp_s"],
                "innov_y": w["innov_y_mps"],
                "innov_z": w["innov_z_mps"],
                "k_y0": w["k_att_y0"],
                "k_y1": w["k_att_y1"],
                "k_z0": w["k_att_z0"],
                "k_z1": w["k_att_z1"],
                "dx_y": w["dx_att_y_rad"],
                "dx_y_via_y": w["dx_att_y_via_innov_y"],
                "dx_y_via_z": w["dx_att_y_via_innov_z"],
                "dx_z": w["dx_att_z_rad"],
                "dx_z_raw": w["dx_att_z_raw"],
                "recon_resid_max": resid,
            }
        )
        ticks.to_csv(OUT / f"k_pitch_row_{arm}_ticks.csv", index=False)
        arms[arm] = ticks

    L, C = arms["latch"], arms["ctrl"]
    t = L["t"].to_numpy(float)
    assert np.allclose(t, C["t"].to_numpy(float), atol=1e-6)

    # relative diffs latch vs ctrl
    def series_rd(col):
        return np.array([rel_diff(a, b) for a, b in zip(L[col], C[col])])

    rd = {
        "k_y0": series_rd("k_y0"),
        "k_y1": series_rd("k_y1"),
        "k_z0": series_rd("k_z0"),
        "k_z1": series_rd("k_z1"),
        "innov_y": series_rd("innov_y"),
        "innov_z": series_rd("innov_z"),
        "dx_y": series_rd("dx_y"),
    }

    # focus tick nearest 1.58 and first ctrl sign-change of dx_y
    i_flip = int(np.argmin(np.abs(t - T_FLIP)))
    dx_c = C["dx_y"].to_numpy(float)
    sign_change = None
    for i in range(1, len(dx_c)):
        if dx_c[i - 1] * dx_c[i] < 0:
            sign_change = i
            break

    def snap(i: int) -> dict:
        return {
            "t": float(t[i]),
            "ctrl": {
                "k_y0": float(C.loc[i, "k_y0"]),
                "k_y1": float(C.loc[i, "k_y1"]),
                "k_z0": float(C.loc[i, "k_z0"]),
                "k_z1": float(C.loc[i, "k_z1"]),
                "innov_y": float(C.loc[i, "innov_y"]),
                "innov_z": float(C.loc[i, "innov_z"]),
                "dx_y": float(C.loc[i, "dx_y"]),
                "dx_y_via_y": float(C.loc[i, "dx_y_via_y"]),
                "dx_y_via_z": float(C.loc[i, "dx_y_via_z"]),
                "dx_z": float(C.loc[i, "dx_z"]),
                "dx_z_raw": float(C.loc[i, "dx_z_raw"]),
            },
            "latch": {
                "k_y0": float(L.loc[i, "k_y0"]),
                "k_y1": float(L.loc[i, "k_y1"]),
                "k_z0": float(L.loc[i, "k_z0"]),
                "k_z1": float(L.loc[i, "k_z1"]),
                "innov_y": float(L.loc[i, "innov_y"]),
                "innov_z": float(L.loc[i, "innov_z"]),
                "dx_y": float(L.loc[i, "dx_y"]),
                "dx_y_via_y": float(L.loc[i, "dx_y_via_y"]),
                "dx_y_via_z": float(L.loc[i, "dx_y_via_z"]),
                "dx_z": float(L.loc[i, "dx_z"]),
                "dx_z_raw": float(L.loc[i, "dx_z_raw"]),
            },
            "rel_diff": {k: float(rd[k][i]) for k in rd},
        }

    # Counterfactual: ctrl K × latch y and latch K × ctrl y at flip tick
    def counterfactual(i: int) -> dict:
        c, l = C.loc[i], L.loc[i]
        dx_ck_ly = c["k_y0"] * l["innov_y"] + c["k_y1"] * l["innov_z"]
        dx_lk_cy = l["k_y0"] * c["innov_y"] + l["k_y1"] * c["innov_z"]
        return {
            "dx_y_ctrl": float(c["dx_y"]),
            "dx_y_latch": float(l["dx_y"]),
            "ctrl_K_times_latch_y": float(dx_ck_ly),
            "latch_K_times_ctrl_y": float(dx_lk_cy),
            # which swap recovers latch sign/magnitude better?
            "err_swap_y_vs_latch": float(abs(dx_ck_ly - l["dx_y"])),
            "err_swap_K_vs_latch": float(abs(dx_lk_cy - l["dx_y"])),
            "err_swap_y_vs_ctrl": float(abs(dx_lk_cy - c["dx_y"])),
            "err_swap_K_vs_ctrl": float(abs(dx_ck_ly - c["dx_y"])),
        }

    i_use = sign_change if sign_change is not None else i_flip
    # mean relative diffs over window
    mean_rd = {k: float(np.mean(v)) for k, v in rd.items()}
    max_rd = {k: float(np.max(v)) for k, v in rd.items()}

    # Verdict thresholds
    # K "identical" if max rel diff on k_y0/k_y1 < 5% near flip and mean < 10%
    k_near = max(rd["k_y0"][i_use], rd["k_y1"][i_use])
    y_near = max(rd["innov_y"][i_use], rd["innov_z"][i_use])
    cf = counterfactual(i_use)

    if k_near < 0.05 and y_near >= 0.20:
        label = "K_PITCH_SAME_Y_DIFFERS"
        reading = (
            "K[ATT_Y,:] nearly identical latch vs ctrl at the flip tick; innov "
            "differs. Post-hoc Z zero-out does not rewrite the pitch gain in-tick — "
            "dx_y sign divergence is carried by y (trajectory), not by an inconsistent "
            "joint K solve."
        )
        next_step = (
            "Trace why innov_y/z composition diverges so ctrl reverses dx_y — "
            "state/H path feeding y, not K coupling from λ."
        )
    elif k_near >= 0.20 and cf["err_swap_K_vs_latch"] < cf["err_swap_y_vs_latch"]:
        label = "K_PITCH_DIFFERS_DOMINANT"
        reading = (
            "K[ATT_Y,:] already differs materially at the flip (prior P/H divergence). "
            "Not in-tick Z-row mutilation of one solve — gain itself has drifted."
        )
        next_step = "Ask when K_pitch first diverges (onset) vs when y diverges."
    elif y_near > k_near and cf["err_swap_y_vs_latch"] <= cf["err_swap_K_vs_latch"]:
        label = "Y_DOMINATES_K_SECONDARY"
        reading = (
            "Both K and y differ, but swapping y (holding K) recovers the arm "
            "difference better — innov composition is the primary driver of the "
            "dx_y sign split; K drift is secondary trajectory effect."
        )
        next_step = "Decompose innov_y vs innov_z contributions to the sign flip."
    else:
        label = "K_AND_Y_BOTH_MOVE"
        reading = (
            "K_pitch and innov both differ at the flip; counterfactual does not "
            "cleanly pick one. Still: λ does not edit K in-tick (code+audit)."
        )
        next_step = "Widen onset window; track first divergence of K_y vs y."

    # Also: in-tick identity — latch dx_z_raw should equal Kz·y; applied dx_z=0
    latch_z_zero = bool(np.allclose(L["dx_z"].to_numpy(float), 0.0, atol=1e-12))

    report = {
        "window_s": [T0, T1],
        "flip_tick": snap(i_use),
        "tick_at_1_58": snap(i_flip),
        "ctrl_sign_change_index": sign_change,
        "mean_rel_diff_latch_vs_ctrl": mean_rd,
        "max_rel_diff_latch_vs_ctrl": max_rd,
        "counterfactual_at_flip": cf,
        "latch_dx_z_applied_all_zero": latch_z_zero,
        "implementation_note": (
            "λ zeros δx[ATT_Z] after δx=Ky; K and Joseph use full-system K. "
            "Any K difference latch↔ctrl is from diverged P/H/state, not from "
            "editing the pitch row inside one update."
        ),
        "verdict": {
            "label": label,
            "reading": reading,
            "next": next_step,
            "k_pitch_rel_diff_at_flip": k_near,
            "innov_rel_diff_at_flip": y_near,
        },
    }

    # figure
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    axes[0].plot(t, C["k_y0"], "C0", label="ctrl K_y0")
    axes[0].plot(t, L["k_y0"], "C3", label="latch K_y0")
    axes[0].plot(t, C["k_y1"], "C0", ls="--", label="ctrl K_y1")
    axes[0].plot(t, L["k_y1"], "C3", ls="--", label="latch K_y1")
    axes[0].set_ylabel("K[ATT_Y]")
    axes[0].set_title("Pitch gain row K[ATT_Y,:] latch vs ctrl")
    axes[0].legend(fontsize=7, ncol=2)

    axes[1].plot(t, C["innov_y"], "C0", label="ctrl y_lat")
    axes[1].plot(t, L["innov_y"], "C3", label="latch y_lat")
    axes[1].plot(t, C["innov_z"], "C0", ls="--", label="ctrl y_vert")
    axes[1].plot(t, L["innov_z"], "C3", ls="--", label="latch y_vert")
    axes[1].set_ylabel("innov m/s")
    axes[1].legend(fontsize=7, ncol=2)

    axes[2].plot(t, C["dx_y"], "C0", label="ctrl dx_y")
    axes[2].plot(t, L["dx_y"], "C3", label="latch dx_y")
    axes[2].plot(t, C["dx_y_via_y"], "C0", ls=":", label="ctrl via y_lat")
    axes[2].plot(t, C["dx_y_via_z"], "C0", ls="--", label="ctrl via y_vert")
    axes[2].plot(t, L["dx_y_via_y"], "C3", ls=":", label="latch via y_lat")
    axes[2].plot(t, L["dx_y_via_z"], "C3", ls="--", label="latch via y_vert")
    axes[2].axhline(0, color="gray", lw=0.4)
    axes[2].set_ylabel("dx_att_y")
    axes[2].legend(fontsize=6, ncol=3)

    axes[3].plot(t, rd["k_y0"], "C2", label="relΔ K_y0")
    axes[3].plot(t, rd["k_y1"], "C2", ls="--", label="relΔ K_y1")
    axes[3].plot(t, rd["innov_y"], "C1", label="relΔ innov_y")
    axes[3].plot(t, rd["innov_z"], "C1", ls="--", label="relΔ innov_z")
    axes[3].set_ylabel("rel |L−C|/max")
    axes[3].set_xlabel("t [s]")
    axes[3].legend(fontsize=7, ncol=2)

    for ax in axes:
        ax.axvline(T_FLIP, color="gray", ls="--", alpha=0.5)
        if sign_change is not None:
            ax.axvline(t[sign_change], color="purple", ls=":", alpha=0.6)
    fig.tight_layout()
    fig_path = OUT / "fig_k_pitch_row_158.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "k_pitch_row_158.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    fl = report["flip_tick"]
    lines = [
        "# K[ATT_Y,:] at pitch sign-flip — latch vs ctrl",
        "",
        f"**Verdict:** `{label}`",
        "",
        reading,
        "",
        f"**Next:** {next_step}",
        "",
        report["implementation_note"],
        "",
        f"Flip/focus tick t={fl['t']:.3f}s "
        f"(ctrl sign-change index={sign_change})",
        "",
        "## At focus tick",
        "",
        "| | ctrl | latch | relΔ |",
        "|--|------|-------|------|",
        f"| K_y0 | {fl['ctrl']['k_y0']:+.6e} | {fl['latch']['k_y0']:+.6e} | {fl['rel_diff']['k_y0']:.3f} |",
        f"| K_y1 | {fl['ctrl']['k_y1']:+.6e} | {fl['latch']['k_y1']:+.6e} | {fl['rel_diff']['k_y1']:.3f} |",
        f"| innov_y | {fl['ctrl']['innov_y']:+.6f} | {fl['latch']['innov_y']:+.6f} | {fl['rel_diff']['innov_y']:.3f} |",
        f"| innov_z | {fl['ctrl']['innov_z']:+.6f} | {fl['latch']['innov_z']:+.6f} | {fl['rel_diff']['innov_z']:.3f} |",
        f"| dx_y | {fl['ctrl']['dx_y']:+.6e} | {fl['latch']['dx_y']:+.6e} | {fl['rel_diff']['dx_y']:.3f} |",
        f"| dx_y via y | {fl['ctrl']['dx_y_via_y']:+.6e} | {fl['latch']['dx_y_via_y']:+.6e} | |",
        f"| dx_y via z | {fl['ctrl']['dx_y_via_z']:+.6e} | {fl['latch']['dx_y_via_z']:+.6e} | |",
        f"| dx_z (applied) | {fl['ctrl']['dx_z']:+.6e} | {fl['latch']['dx_z']:+.6e} | |",
        f"| dx_z_raw | {fl['ctrl']['dx_z_raw']:+.6e} | {fl['latch']['dx_z_raw']:+.6e} | |",
        "",
        "## Counterfactual (focus tick)",
        "",
        f"- ctrl K × latch y → dx_y = {cf['ctrl_K_times_latch_y']:+.6e}",
        f"- latch K × ctrl y → dx_y = {cf['latch_K_times_ctrl_y']:+.6e}",
        f"- err(swap y → latch) = {cf['err_swap_y_vs_latch']:.3e}",
        f"- err(swap K → latch) = {cf['err_swap_K_vs_latch']:.3e}",
        "",
        "## Mean relΔ over [1.54,1.64]",
        "",
        "| qty | mean relΔ | max relΔ |",
        "|-----|-----------|----------|",
    ]
    for k in ("k_y0", "k_y1", "k_z0", "k_z1", "innov_y", "innov_z", "dx_y"):
        lines.append(f"| {k} | {mean_rd[k]:.3f} | {max_rd[k]:.3f} |")

    lines += [
        "",
        "## Tick table",
        "",
        "| t | Ky0_c | Ky0_l | Ky1_c | Ky1_l | iy_c | iy_l | iz_c | iz_l | dxy_c | dxy_l |",
        "|---|-------|-------|-------|-------|------|------|------|------|-------|-------|",
    ]
    for i in range(len(t)):
        lines.append(
            f"| {t[i]:.3f} | {C.loc[i,'k_y0']:+.4e} | {L.loc[i,'k_y0']:+.4e} | "
            f"{C.loc[i,'k_y1']:+.4e} | {L.loc[i,'k_y1']:+.4e} | "
            f"{C.loc[i,'innov_y']:+.3f} | {L.loc[i,'innov_y']:+.3f} | "
            f"{C.loc[i,'innov_z']:+.3f} | {L.loc[i,'innov_z']:+.3f} | "
            f"{C.loc[i,'dx_y']:+.3e} | {L.loc[i,'dx_y']:+.3e} |"
        )
    lines += ["", f"Figure: `{fig_path.name}`", ""]
    (OUT / "k_pitch_row_158.md").write_text("\n".join(lines), encoding="utf-8")
    return report


def main() -> None:
    reuse = "--reuse-audit" in sys.argv
    if not reuse:
        print("Running ctrl…")
        run_arm("ctrl", lam=0.0, gate=None)
        print("Running latch…")
        run_arm("latch", lam=1.0, gate=T2)
    report = analyze()
    print(json.dumps(report["verdict"], indent=2))
    print("---")
    print(json.dumps(report["flip_tick"], indent=2))
    print(json.dumps(report["counterfactual_at_flip"], indent=2))
    print(json.dumps(report["mean_rel_diff_latch_vs_ctrl"], indent=2))


if __name__ == "__main__":
    main()
