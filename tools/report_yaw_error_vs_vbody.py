#!/usr/bin/env python3
"""Honest close: -V sin(Δyaw) falsified; cross-track vel explains v_lat."""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

OUT = Path(__file__).resolve().parents[1] / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"
V = 50.0 / 3.6
OMEGA = 2.0 * np.pi / 4.0
YAW_AMP = 3.0 / (V * OMEGA)
BASE = np.pi / 2


def wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def main() -> None:
    arm = "latch"
    a = pd.read_csv(OUT / f"{arm}_nhc_block_audit.csv")
    t = a[(a.timestamp_s >= 1.69) & (a.timestamp_s <= 1.79)].copy()
    telem = pd.read_csv(
        Path(__file__).resolve().parents[1]
        / f"docs/benchmarks/slalom_pattbias_{arm}_s71_telemetry.csv"
    )
    telem["t"] = telem.time_us * 1e-6
    ts = t.timestamp_s.to_numpy(float)
    yaw_f = np.interp(ts, telem.t, telem.yaw)
    roll_f = np.interp(ts, telem.t, telem.roll)
    pitch_f = np.interp(ts, telem.t, telem.pitch)
    yaw_t = BASE + YAW_AMP * np.sin(OMEGA * ts)
    vn = np.interp(ts, telem.t, telem.vel_x)
    ve = np.interp(ts, telem.t, telem.vel_y)
    dyaw = wrap(yaw_f - yaw_t)
    along = vn * np.cos(yaw_t) + ve * np.sin(yaw_t)
    cross = -vn * np.sin(yaw_t) + ve * np.cos(yaw_t)
    fl = t.v_body_y_before_mps.to_numpy(float)
    fv = t.v_body_z_before_mps.to_numpy(float)
    pred_sin = -V * np.sin(dyaw)

    prev = json.loads((OUT / "yaw_error_vs_vbody.json").read_text(encoding="utf-8"))
    verdict = {
        "label": "CASCADE_VIA_VEL_STATE_NOT_INSTANT_YAW_PROJECTION",
        "reading": (
            "The simple equation filter_v_lat ≈ -V·sin(Δyaw) is FALSIFIED for latch: "
            f"Δyaw only ~0.3–1.2° predicts |v_lat|≲{V * np.sin(np.deg2rad(1.2)):.2f} m/s "
            f"but observed max is {np.max(np.abs(fl)):.2f} m/s. "
            "Instead filter_v_lat ≈ filter NED cross-track vs truth heading "
            f"(pearson {np.corrcoef(fl, cross)[0,1]:.2f}, "
            f"frac var {1 - np.var(fl - cross) / np.var(fl):.2f}) — "
            "the lateral innov is the already-corrupted velocity state, not an "
            "instantaneous yaw misprojection of forward speed. "
            "filter_v_vert still tracks attitude projection of speed (roll/pitch; "
            "att_only pearson ~0.985) with partial amplitude. "
            "Cascade holds end-to-end, but the proximate equation at explosion time is: "
            "attitude loop → polluted vel_NED → body lat/vert via NHC, "
            "not v_lat=V·sin(Δψ) alone."
        ),
        "falsified": "filter_v_lat ≈ -V sin(Δyaw) as sole quantitative link in [1.69,1.79] latch",
        "supported": [
            "v_lat ≈ v_cross_track_filter (truth-heading frame)",
            "v_vert shape ≈ truth_vel projected with filter roll/pitch",
            "A_CASCADE (truth body≈0) still stands",
        ],
        "metrics": {
            "dyaw_deg_range": [
                float(np.rad2deg(dyaw.min())),
                float(np.rad2deg(dyaw.max())),
            ],
            "max_abs_pred_Vsin": float(np.max(np.abs(pred_sin))),
            "max_abs_filter_v_lat": float(np.max(np.abs(fl))),
            "pearson_vlat_vs_cross": float(np.corrcoef(fl, cross)[0, 1]),
            "frac_var_vlat_vs_cross": float(1 - np.var(fl - cross) / np.var(fl)),
            "pearson_vlat_vs_Vsin": float(np.corrcoef(fl, pred_sin)[0, 1]),
            "roll_deg_range": [
                float(np.rad2deg(roll_f.min())),
                float(np.rad2deg(roll_f.max())),
            ],
            "pitch_deg_range": [
                float(np.rad2deg(pitch_f.min())),
                float(np.rad2deg(pitch_f.max())),
            ],
            "filter_along_range": [float(along.min()), float(along.max())],
        },
    }
    prev["verdict"]["latch"] = verdict
    prev["verdict"]["note"] = (
        "Do not claim yaw-sin closes the equation; claim cascade via velocity-state "
        "pollution + attitude projection on vert."
    )
    (OUT / "yaw_error_vs_vbody.json").write_text(json.dumps(prev, indent=2), encoding="utf-8")

    df = pd.DataFrame(
        {
            "t_s": ts,
            "dyaw_deg": np.rad2deg(dyaw),
            "roll_deg": np.rad2deg(roll_f),
            "pitch_deg": np.rad2deg(pitch_f),
            "filter_along": along,
            "filter_cross": cross,
            "filter_v_lat": fl,
            "pred_Vsin": pred_sin,
            "filter_v_vert": fv,
            "resid_lat_minus_cross": fl - cross,
            "resid_lat_minus_Vsin": fl - pred_sin,
        }
    )
    df.to_csv(OUT / "yaw_error_vs_vbody_latch_ticks.csv", index=False)

    m = verdict["metrics"]
    lines = [
        "# Yaw error ↔ filter v_body — quantitative check",
        "",
        f"**Verdict (latch):** `{verdict['label']}`",
        "",
        verdict["reading"],
        "",
        "## Key numbers",
        "",
        f"- Δyaw: [{m['dyaw_deg_range'][0]:.2f}, {m['dyaw_deg_range'][1]:.2f}]°",
        f"- max|-V sinΔψ|: **{m['max_abs_pred_Vsin']:.3f}** m/s vs max|filter_v_lat|: "
        f"**{m['max_abs_filter_v_lat']:.3f}** m/s "
        f"(~{m['max_abs_filter_v_lat'] / max(m['max_abs_pred_Vsin'], 1e-9):.0f}× larger)",
        f"- pearson(v_lat, filter cross-track): **{m['pearson_vlat_vs_cross']:.3f}**, "
        f"frac var **{m['frac_var_vlat_vs_cross']:.3f}**",
        f"- pearson(v_lat, -V sinΔψ): **{m['pearson_vlat_vs_Vsin']:.3f}** (fails)",
        f"- roll range: [{m['roll_deg_range'][0]:.1f}, {m['roll_deg_range'][1]:.1f}]°; "
        f"pitch: [{m['pitch_deg_range'][0]:.1f}, {m['pitch_deg_range'][1]:.1f}]°",
        "",
        "## Tick table — latch",
        "",
        "| t | Δyaw° | roll° | pitch° | v_cross | v_lat | −VsinΔψ | v_vert |",
        "|---|-------|-------|--------|---------|-------|---------|--------|",
    ]
    for _, r in df.iterrows():
        lines.append(
            f"| {r.t_s:.3f} | {r.dyaw_deg:+.2f} | {r.roll_deg:+.1f} | {r.pitch_deg:+.1f} | "
            f"{r.filter_cross:+.3f} | {r.filter_v_lat:+.3f} | {r.pred_Vsin:+.3f} | "
            f"{r.filter_v_vert:+.3f} |"
        )
    lines += [
        "",
        "## Implication for cascade close",
        "",
        "- Qualitative cascade (Jacobian → attitude loop → innov explosion) **still holds**.",
        "- Quantitative link at explosion time is **not** instant yaw projection of V.",
        "- Proximate: **polluted filter vel_NED** (cross-track ≈ v_lat) + **roll/pitch** feeding v_vert.",
        "- Intervention should target the cascade **before** velocity is already wrong "
        "(onset/early attitude path), not assume fixing Δyaw alone at t≈1.7 kills v_lat.",
        "",
        "Figure: `fig_yaw_error_vs_vbody.png`",
        "",
    ]
    (OUT / "yaw_error_vs_vbody.md").write_text("\n".join(lines), encoding="utf-8")

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(ts, np.rad2deg(dyaw), "C3", label="Δyaw°")
    axes[0].plot(ts, np.rad2deg(roll_f), "C1", label="roll°")
    axes[0].plot(ts, np.rad2deg(pitch_f), "C2", label="pitch°")
    axes[0].legend(fontsize=8)
    axes[0].set_ylabel("deg")
    axes[0].set_title("Cascade close: Δyaw too small for v_lat; cross-track vel explains it")
    axes[1].plot(ts, fl, "C3", lw=1.5, label="filter v_lat")
    axes[1].plot(ts, cross, "k--", lw=1.2, label="filter NED cross-track")
    axes[1].plot(ts, pred_sin, "C0:", lw=1.2, label="-V sin(Δyaw)")
    axes[1].legend(fontsize=8)
    axes[1].set_ylabel("m/s")
    axes[1].axhline(0, color="gray", lw=0.5)
    axes[2].plot(ts, fv, "C3", label="filter v_vert")
    axes[2].legend(fontsize=8)
    axes[2].set_ylabel("m/s")
    axes[2].set_xlabel("t [s]")
    axes[2].axhline(0, color="gray", lw=0.5)
    fig.tight_layout()
    fig.savefig(OUT / "fig_yaw_error_vs_vbody.png", dpi=140)
    plt.close(fig)
    print(verdict["label"])
    print("max Vsin", m["max_abs_pred_Vsin"], "max vlat", m["max_abs_filter_v_lat"])
    print("pearson cross", m["pearson_vlat_vs_cross"], "frac", m["frac_var_vlat_vs_cross"])


if __name__ == "__main__":
    main()
