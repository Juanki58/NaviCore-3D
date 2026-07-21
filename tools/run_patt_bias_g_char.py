#!/usr/bin/env python3
"""Characterize P_att–bias_g vs bias escape (protocol §13).

Arms: ctrl (no latch) vs latch λ=1 @ T2. SLALOM A jcorrect seed 71.
Do not design intervention until characterization verdict is scored.
"""

from __future__ import annotations

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "docs" / "benchmarks" / "jacobian_imu_ab" / "patt_bias_g"
SEED = 71
T2 = 3.736646e-6
TMAX = 0.65
LATCH_T = 0.39

sys.path.insert(0, str(REPO))
from run_all_benchmarks import run_benchmark  # noqa: E402


def run_arm(name: str, *, lam: float, gate: float | None) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    audit = OUT / f"{name}_nhc_block_audit.csv"
    if audit.exists():
        audit.unlink()
    env = os.environ.copy()
    env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = str(audit)
    r = run_benchmark(
        f"PattBias {name}",
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
    if not audit.is_file():
        raise FileNotFoundError(audit)
    telem = REPO / "docs" / "benchmarks" / f"slalom_pattbias_{name}_s{SEED}_telemetry.csv"
    return audit, telem, r


def analyze(audit_path: Path, telem_path: Path, name: str) -> dict:
    a = pd.read_csv(audit_path)
    t = pd.read_csv(telem_path)
    t["t_s"] = t["time_us"].astype(float) * 1e-6

    need = [
        "P_pre_att_z_bias_gz",
        "P_pre_att_bias_g_frob",
        "dx_bias_gz",
        "k_bias_gz",
        "dx_att_z_rad",
        "innov_norm_mps",
    ]
    missing = [c for c in need if c not in a.columns]
    if missing:
        raise KeyError(f"{name}: missing columns {missing} — rebuild sim with §13 audit")

    ta = a["timestamp_s"].to_numpy(float)
    p_az = a["P_pre_att_z_bias_gz"].to_numpy(float)
    p_frob = a["P_pre_att_bias_g_frob"].to_numpy(float)
    dx_bg = a["dx_bias_gz"].to_numpy(float)
    k_bg = a["k_bias_gz"].to_numpy(float)
    dx_az = a["dx_att_z_rad"].to_numpy(float)

    # telemetría bias
    tt = t["t_s"].to_numpy(float)
    bias = t["bias_gz"].to_numpy(float)

    def win(t0, t1):
        m = (ta >= t0) & (ta <= t1)
        return m

    m_pre = win(0.0, LATCH_T)
    m_post = win(LATCH_T, 2.0)

    def stats(mask, x):
        xx = x[mask]
        if len(xx) < 3:
            return {}
        return {
            "n": int(mask.sum()),
            "mean": float(np.mean(xx)),
            "median": float(np.median(xx)),
            "std": float(np.std(xx)),
            "sum": float(np.sum(xx)),
            "start": float(xx[0]),
            "end": float(xx[-1]),
        }

    # corr P vs dx_bias in post window
    def corr(mask, x, y):
        xx, yy = x[mask], y[mask]
        if len(xx) < 5 or np.std(xx) < 1e-30 or np.std(yy) < 1e-30:
            return float("nan")
        return float(np.corrcoef(xx, yy)[0, 1])

    # sign agreement frac
    def sign_agree(mask, x, y):
        xx, yy = x[mask], y[mask]
        both = (xx != 0) & (yy != 0)
        if both.sum() == 0:
            return float("nan")
        return float(np.mean(np.sign(xx[both]) == np.sign(yy[both])))

    # bias telem slope post
    mb = (tt >= LATCH_T) & (tt <= 2.0)
    if mb.sum() >= 5:
        slope_bias = float(np.polyfit(tt[mb] - tt[mb][0], bias[mb], 1)[0])
        delta_bias = float(bias[mb][-1] - bias[mb][0])
    else:
        slope_bias, delta_bias = float("nan"), float("nan")

    return {
        "name": name,
        "pre_latch": {
            "P_az_bgz": stats(m_pre, p_az),
            "P_att_bias_frob": stats(m_pre, p_frob),
            "dx_bias_gz": stats(m_pre, dx_bg),
            "dx_att_z": stats(m_pre, dx_az),
        },
        "post_latch_0.39_2": {
            "P_az_bgz": stats(m_post, p_az),
            "P_att_bias_frob": stats(m_post, p_frob),
            "dx_bias_gz": stats(m_post, dx_bg),
            "dx_att_z": stats(m_post, dx_az),
            "k_bias_gz": stats(m_post, k_bg),
            "corr_P_vs_dx_bias": corr(m_post, p_az, dx_bg),
            "sign_agree_P_vs_dx_bias": sign_agree(m_post, p_az, dx_bg),
            "corr_P_vs_k_bias": corr(m_post, p_az, k_bg),
            "telem_bias_gz_slope": slope_bias,
            "telem_bias_gz_delta": delta_bias,
        },
        "_series": {
            "ta": ta,
            "p_az": p_az,
            "p_frob": p_frob,
            "dx_bg": dx_bg,
            "dx_az": dx_az,
            "k_bg": k_bg,
            "tt": tt,
            "bias": bias,
        },
    }


def score(ctrl: dict, latch: dict) -> dict:
    c = ctrl["post_latch_0.39_2"]
    l = latch["post_latch_0.39_2"]
    # Characterization PASS criteria (§13.2)
    sign_ok = (
        np.isfinite(l.get("sign_agree_P_vs_dx_bias", np.nan))
        and l["sign_agree_P_vs_dx_bias"] >= 0.7
    )
    corr_ok = (
        np.isfinite(l.get("corr_P_vs_dx_bias", np.nan))
        and abs(l["corr_P_vs_dx_bias"]) >= 0.3
    )
    # Block material / reorganizes vs ctrl
    c_frob = c["P_att_bias_frob"].get("mean", 0) or 0
    l_frob = l["P_att_bias_frob"].get("mean", 0) or 0
    c_paz = c["P_az_bgz"].get("mean", 0) or 0
    l_paz = l["P_az_bgz"].get("mean", 0) or 0
    material = abs(l_paz) > 1e-8 or l_frob > 1e-8
    reorganizes = abs(l_paz - c_paz) > 0.5 * max(abs(c_paz), 1e-12) or (
        l_frob > 1.5 * c_frob if c_frob > 0 else l_frob > 1e-8
    )
    # Escape present under latch (sanity)
    escape = abs(l.get("telem_bias_gz_slope") or 0) > 2.0 * abs(
        c.get("telem_bias_gz_slope") or 0
    )

    # Strict: predictive corr AND P that differs under latch.
    # Escape alone with unchanged P → innov-amplified K_bias path (not P migration).
    char_pass = bool(corr_ok and material and reorganizes and escape)
    char_weak = bool(material and escape and not char_pass)
    return {
        "sign_agree_latch": l.get("sign_agree_P_vs_dx_bias"),
        "corr_P_dx_latch": l.get("corr_P_vs_dx_bias"),
        "corr_P_dx_ctrl": c.get("corr_P_vs_dx_bias"),
        "P_az_bgz_mean_ctrl": c_paz,
        "P_az_bgz_mean_latch": l_paz,
        "P_frob_mean_ctrl": c_frob,
        "P_frob_mean_latch": l_frob,
        "material_block": material,
        "reorganizes_vs_ctrl": reorganizes,
        "escape_amplified": escape,
        "CHAR_PASS": char_pass,
        "CHAR_WEAK": char_weak,
        "reading": (
            "innov_amplified_K_bias"
            if (escape and not reorganizes and not corr_ok)
            else ("p_block_predictive" if char_pass else "inconclusive")
        ),
        "note": (
            "PASS unlocks P_att-bias intervention. "
            "WEAK/innov-amplified: do not zero-P blind; decompose K_bias next."
        ),
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    print("=== ctrl ===")
    audit_c, telem_c, _ = run_arm("ctrl", lam=0.0, gate=None)
    print("=== latch λ=1 T2 ===")
    audit_l, telem_l, _ = run_arm("latch", lam=1.0, gate=T2)

    ctrl = analyze(audit_c, telem_c, "ctrl")
    latch = analyze(audit_l, telem_l, "latch")
    verdict = score(ctrl, latch)

    # figure
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    for ax, key, ylab, title in [
        (axes[0, 0], "p_az", "P[ATT_Z,BIAS_GZ] pre", "P_att_z_bias_gz"),
        (axes[0, 1], "dx_bg", "dx_bias_gz (NHC)", "dx_bias_gz"),
        (axes[1, 0], "p_frob", "‖P_att,bias_g‖_F", "P_att_bias_g frob"),
        (axes[1, 1], "bias", "bias_gz state", "bias_gz telemetry"),
    ]:
        for arm, color in [(ctrl, "C0"), (latch, "C1")]:
            s = arm["_series"]
            if key == "bias":
                ax.plot(s["tt"], s["bias"], label=arm["name"], color=color, lw=1.1)
            else:
                y = {"p_az": s["p_az"], "dx_bg": s["dx_bg"], "p_frob": s["p_frob"]}[key]
                ax.plot(s["ta"], y, label=arm["name"], color=color, lw=1.1)
        ax.axvline(LATCH_T, color="red", ls="--", lw=0.8)
        ax.set_xlim(0, 2)
        ax.set_ylabel(ylab)
        ax.set_title(title)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    fig_path = OUT / "fig_patt_bias_g_ctrl_vs_latch.png"
    fig.savefig(fig_path, dpi=140)

    # strip series for json
    for d in (ctrl, latch):
        d.pop("_series", None)

    report = {
        "protocol": "docs/diagnostics/18-jacobian-imu-ab-protocol.md §13",
        "timestamp": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "seed": SEED,
        "ctrl": ctrl,
        "latch": latch,
        "verdict": verdict,
        "figure": str(fig_path),
    }
    outj = OUT / "patt_bias_g_char_report.json"
    outj.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("\n======== §13 CHARACTERIZATION VERDICT ========")
    print(json.dumps(verdict, indent=2))
    print(f"Wrote {outj}")
    print(f"Wrote {fig_path}")
    return 0 if verdict["CHAR_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
