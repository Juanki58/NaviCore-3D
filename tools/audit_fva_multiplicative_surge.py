#!/usr/bin/env python3
"""Is vel_NED surge [1.59→1.69] multiplicative: attitude error × high |a_corr|?

f_va = -dt * R_bn * [a_corr×]  →  δv ~ dt * |a| * |δθ|
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"

V = 50.0 / 3.6
OMEGA = 2.0 * np.pi / 4.0
YAW_AMP = 3.0 / (V * OMEGA)
BASE = np.pi / 2.0
A_LAT_MAX = 3.0  # TC04_MAX_LATERAL_ACCEL
G = 9.80665
DT = 0.01

T0, T1 = 1.34, 1.69
T_BREAK = 1.59


def wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def truth_kinematics(t: np.ndarray):
    phase = OMEGA * t
    yaw = BASE + YAW_AMP * np.sin(phase)
    yaw_rate = YAW_AMP * OMEGA * np.cos(phase)
    # horiz accel magnitude in NED = V * |yaw_rate|
    a_horiz = V * np.abs(yaw_rate)  # = A_LAT_MAX * |cos(phase)|
    a_n = -V * np.sin(yaw) * yaw_rate
    a_e = V * np.cos(yaw) * yaw_rate
    # ideal IMU specific force body (level yaw-only):
    # specific_ned = [a_n, a_e, a_d+g]; body ≈ [forward specific, 0, ~g] with lateral from...
    # For yaw-only DCM, body_x gets along-track specific, body_y gets cross of accel
    # Exact: specific_ned = (a_n, a_e, g) if a_d=0 and D-down gravity +g
    # v_body from earlier: level yaw → a_body_lat from rotating accel
    c, s = np.cos(yaw), np.sin(yaw)
    # NED→body (yaw-only): bx = an*c+ae*s, by=-an*s+ae*c, bz = g
    spec_n, spec_e, spec_d = a_n, a_e, G
    a_fwd = spec_n * c + spec_e * s
    a_lat = -spec_n * s + spec_e * c
    a_vert = spec_d
    a_corr_norm = np.sqrt(a_fwd**2 + a_lat**2 + a_vert**2)
    a_corr_horiz_body = np.hypot(a_fwd, a_lat)
    return {
        "yaw": yaw,
        "yaw_rate": yaw_rate,
        "a_horiz_ned": a_horiz,
        "a_fwd_body": a_fwd,
        "a_lat_body": a_lat,
        "a_vert_body": a_vert,
        "a_corr_norm": a_corr_norm,
        "a_corr_horiz_body": a_corr_horiz_body,
    }


def arm_frame(arm: str) -> pd.DataFrame:
    telem = pd.read_csv(ROOT / f"docs/benchmarks/slalom_pattbias_{arm}_s71_telemetry.csv")
    telem["t"] = telem["time_us"].astype(float) * 1e-6
    m = (telem["t"] >= T0) & (telem["t"] <= T1)
    d = telem.loc[m].copy()
    t = d["t"].to_numpy(float)
    tk = truth_kinematics(t)

    yaw_f = d["yaw"].to_numpy(float)
    roll_f = d["roll"].to_numpy(float)
    pitch_f = d["pitch"].to_numpy(float)
    dyaw = wrap(yaw_f - tk["yaw"])
    # attitude error mag (truth roll=pitch=0)
    datt = np.sqrt(roll_f**2 + pitch_f**2 + dyaw**2)

    vn_f = d["vel_x"].to_numpy(float)
    ve_f = d["vel_y"].to_numpy(float)
    yaw_t = tk["yaw"]
    cross = -vn_f * np.sin(yaw_t) + ve_f * np.cos(yaw_t)

    # telem accel proxies
    fwd_acc = d["fwd_accel"].to_numpy(float)
    vert_acc = d["vert_accel"].to_numpy(float)
    telem_a_norm = np.sqrt(fwd_acc**2 + vert_acc**2)  # incomplete (no lat channel)

    # multiplicative proxies
    prod_truth_a_horiz_x_datt = tk["a_horiz_ned"] * datt
    prod_a_corr_x_datt = tk["a_corr_norm"] * datt
    # per-tick |δv| scale from f_va: dt * |a| * |δθ|
    dv_scale = DT * tk["a_corr_norm"] * datt

    # finite-diff of |cross|
    abs_cross = np.abs(cross)
    d_abs_cross = np.diff(abs_cross, prepend=abs_cross[0]) / DT

    return pd.DataFrame(
        {
            "t": t,
            "a_horiz_ned_truth": tk["a_horiz_ned"],
            "a_corr_norm_truth_imu": tk["a_corr_norm"],
            "a_corr_horiz_body_truth": tk["a_corr_horiz_body"],
            "a_lat_body_truth": tk["a_lat_body"],
            "fwd_accel_telem": fwd_acc,
            "vert_accel_telem": vert_acc,
            "telem_a_norm_incomplete": telem_a_norm,
            "datt_rad": datt,
            "datt_deg": np.rad2deg(datt),
            "dyaw_deg": np.rad2deg(dyaw),
            "roll_deg": np.rad2deg(roll_f),
            "pitch_deg": np.rad2deg(pitch_f),
            "cross_err": cross,
            "abs_cross": abs_cross,
            "d_abs_cross_dt": d_abs_cross,
            "prod_a_horiz_x_datt": prod_truth_a_horiz_x_datt,
            "prod_a_corr_x_datt": prod_a_corr_x_datt,
            "dv_scale_fva": dv_scale,
        }
    )


def phase(df: pd.DataFrame, t0: float, t1: float) -> dict:
    m = (df["t"] >= t0) & (df["t"] <= t1)
    g = df.loc[m]
    if g.empty:
        return {"n": 0}
    return {
        "n": int(len(g)),
        "mean_a_horiz": float(g["a_horiz_ned_truth"].mean()),
        "max_a_horiz": float(g["a_horiz_ned_truth"].max()),
        "mean_a_corr": float(g["a_corr_norm_truth_imu"].mean()),
        "mean_datt_deg": float(g["datt_deg"].mean()),
        "max_datt_deg": float(g["datt_deg"].max()),
        "mean_prod": float(g["prod_a_horiz_x_datt"].mean()),
        "max_prod": float(g["prod_a_horiz_x_datt"].max()),
        "cross_start": float(g["cross_err"].iloc[0]),
        "cross_end": float(g["cross_err"].iloc[-1]),
        "delta_abs_cross": float(g["abs_cross"].iloc[-1] - g["abs_cross"].iloc[0]),
        "mean_d_abs_cross_dt": float(g["d_abs_cross_dt"].mean()),
        "pearson_dabs_vs_prod": float(
            np.corrcoef(g["d_abs_cross_dt"], g["prod_a_horiz_x_datt"])[0, 1]
        )
        if len(g) > 5
        else float("nan"),
        "pearson_dabs_vs_a": float(
            np.corrcoef(g["d_abs_cross_dt"], g["a_horiz_ned_truth"])[0, 1]
        )
        if len(g) > 5
        else float("nan"),
        "pearson_dabs_vs_datt": float(
            np.corrcoef(g["d_abs_cross_dt"], g["datt_rad"])[0, 1]
        )
        if len(g) > 5
        else float("nan"),
    }


def main() -> None:
    report = {"window_s": [T0, T1], "arms": {}}
    frames = {}
    for arm in ("ctrl", "latch"):
        df = arm_frame(arm)
        frames[arm] = df
        df.to_csv(OUT / f"fva_multiplicative_{arm}.csv", index=False)
        report["arms"][arm] = {
            "rise_pre_break": phase(df, 1.34, 1.59),
            "surge_window": phase(df, 1.59, 1.69),
            "full": phase(df, 1.34, 1.69),
        }

    L_rise = report["arms"]["latch"]["rise_pre_break"]
    L_surge = report["arms"]["latch"]["surge_window"]

    # Does |a| jump in surge window?
    a_ratio = L_surge["mean_a_horiz"] / max(L_rise["mean_a_horiz"], 1e-9)
    datt_ratio = L_surge["mean_datt_deg"] / max(L_rise["mean_datt_deg"], 1e-9)
    prod_ratio = L_surge["mean_prod"] / max(L_rise["mean_prod"], 1e-9)
    cross_growth_ratio = abs(L_surge["delta_abs_cross"]) / max(
        abs(L_rise["delta_abs_cross"]), 1e-9
    )

    # Verdict logic
    a_jumps = a_ratio >= 1.3
    datt_jumps = datt_ratio >= 1.3
    prod_tracks = (
        report["arms"]["latch"]["surge_window"]["pearson_dabs_vs_prod"] >= 0.5
        or report["arms"]["latch"]["full"]["pearson_dabs_vs_prod"] >= 0.5
    )

    if a_jumps and datt_jumps and prod_ratio >= 1.5:
        label = "MULTIPLICATIVE_A_TIMES_DATT"
        reading = (
            "Both |a| and attitude error rise into the surge window; product grows "
            "enough to support f_va multiplicative firing (biased att × high accel)."
        )
    elif a_jumps and not datt_jumps:
        label = "ACCEL_DRIVEN_SURGE"
        reading = (
            "|a| rises into [1.59,1.69] while attitude error stays similar — surge "
            "driven mainly by acceleration multiplying an already-present att error."
        )
    elif datt_jumps and not a_jumps:
        label = "ATTITUDE_JUMP_DRIVEN_SURGE"
        reading = (
            "Attitude error jumps in the surge window while |a| is already high/"
            "smooth — not primarily an accel spike; look at latch/composition-break "
            "attitude dynamics."
        )
    elif prod_tracks:
        label = "PARTIAL_MULTIPLICATIVE"
        reading = (
            "d|cross|/dt correlates with |a|·|δθ| but phase ratios are mixed — "
            "multiplicative story partially supported."
        )
    else:
        label = "NOT_SIMPLE_MULTIPLICATIVE"
        reading = (
            "Surge not well explained by simultaneous |a| peak × |δθ|; need another "
            "mechanism (e.g. NHC updates themselves feeding vel during break)."
        )

    # Note: truth |a_horiz| = 3|cos(ωt)| is smooth toward peak at t=2 — check values
    report["verdict"] = {
        "label": label,
        "reading": reading,
        "ratios": {
            "a_horiz_mean_rise_vs_surge": [L_rise["mean_a_horiz"], L_surge["mean_a_horiz"]],
            "a_horiz_ratio_surge_over_rise": a_ratio,
            "datt_mean_deg_rise_vs_surge": [
                L_rise["mean_datt_deg"],
                L_surge["mean_datt_deg"],
            ],
            "datt_ratio_surge_over_rise": datt_ratio,
            "prod_ratio_surge_over_rise": prod_ratio,
            "delta_abs_cross_rise_vs_surge": [
                L_rise["delta_abs_cross"],
                L_surge["delta_abs_cross"],
            ],
            "cross_growth_ratio": cross_growth_ratio,
            "pearson_dabs_vs_prod_surge": L_surge["pearson_dabs_vs_prod"],
            "pearson_dabs_vs_a_surge": L_surge["pearson_dabs_vs_a"],
            "pearson_dabs_vs_datt_surge": L_surge["pearson_dabs_vs_datt"],
            "pearson_dabs_vs_prod_full": report["arms"]["latch"]["full"][
                "pearson_dabs_vs_prod"
            ],
        },
        "design_hint": (
            "If multiplicative: consider gating/damping f_va when |a| high AND "
            "attitude error energy already elevated — conditional, not blind Z-forget. "
            "If attitude-jump driven: stay on attitude-loop early intervention. "
            "If not multiplicative: re-examine NHC feedback into vel during break."
        ),
        "note_a_smooth": (
            "Truth |a_horiz|=3|cos(ωt)| rises smoothly toward peak at t=2.0; "
            "expect no cliff in |a| at 1.59 — if surge is sharp, attitude-side or "
            "update-side discontinuity is the sharper factor."
        ),
    }

    # figure
    df = frames["latch"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    axes[0].plot(df["t"], df["a_horiz_ned_truth"], "k-", lw=1.2, label="|a|_horiz truth")
    axes[0].plot(
        df["t"], df["a_corr_norm_truth_imu"], "C0--", lw=1.0, label="||a_corr|| truth IMU"
    )
    axes[0].plot(df["t"], np.abs(df["fwd_accel_telem"]), "C1:", lw=0.9, label="|fwd_accel| telem")
    axes[0].legend(fontsize=7)
    axes[0].set_ylabel("m/s²")
    axes[0].set_title("f_va multiplicative check — latch [1.34→1.69]")

    axes[1].plot(df["t"], df["datt_deg"], "C3", lw=1.2, label="||δθ|| deg")
    axes[1].plot(df["t"], np.abs(df["roll_deg"]), "C1--", lw=0.9, label="|roll|")
    axes[1].plot(df["t"], np.abs(df["dyaw_deg"]), "C2:", lw=0.9, label="|Δyaw|")
    axes[1].legend(fontsize=7)
    axes[1].set_ylabel("deg")

    axes[2].plot(df["t"], df["prod_a_horiz_x_datt"], "k-", lw=1.2, label="|a|_h · ||δθ||")
    axes[2].legend(fontsize=8)
    axes[2].set_ylabel("prod")

    axes[3].plot(df["t"], df["cross_err"], "C3", lw=1.2, label="cross-track err")
    axes[3].plot(df["t"], df["d_abs_cross_dt"], "C0--", lw=1.0, label="d|cross|/dt")
    axes[3].legend(fontsize=8)
    axes[3].set_ylabel("m/s , m/s²")
    axes[3].set_xlabel("t [s]")
    for ax in axes:
        ax.axvline(T_BREAK, color="gray", ls="--", alpha=0.6)
    fig.tight_layout()
    fig_path = OUT / "fig_fva_multiplicative_surge.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "fva_multiplicative_surge.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )

    v = report["verdict"]
    lines = [
        "# f_va multiplicative surge? |a| × attitude error [1.34→1.69]",
        "",
        f"**Verdict:** `{label}`",
        "",
        reading,
        "",
        v["note_a_smooth"],
        "",
        "## Latch phase comparison",
        "",
        "| Phase | mean |a|_h | mean ||δθ||° | mean prod | Δ|cross| | pearson d|c|/dt vs prod |",
        "|-------|------------|---------------|-----------|----------|--------------------------|",
        f"| rise [1.34,1.59] | {L_rise['mean_a_horiz']:.3f} | {L_rise['mean_datt_deg']:.2f} | "
        f"{L_rise['mean_prod']:.4f} | {L_rise['delta_abs_cross']:+.3f} | "
        f"{L_rise['pearson_dabs_vs_prod']:.3f} |",
        f"| surge [1.59,1.69] | {L_surge['mean_a_horiz']:.3f} | {L_surge['mean_datt_deg']:.2f} | "
        f"{L_surge['mean_prod']:.4f} | {L_surge['delta_abs_cross']:+.3f} | "
        f"{L_surge['pearson_dabs_vs_prod']:.3f} |",
        "",
        "## Ratios surge/rise",
        "",
        f"- |a|_h: **{a_ratio:.2f}×**",
        f"- ||δθ||: **{datt_ratio:.2f}×**",
        f"- prod |a|·||δθ||: **{prod_ratio:.2f}×**",
        f"- Δ|cross|: **{cross_growth_ratio:.2f}×**",
        "",
        "## Design hint",
        "",
        v["design_hint"],
        "",
        f"Figure: `{fig_path.name}`",
        "",
    ]
    (OUT / "fva_multiplicative_surge.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(v, indent=2))


if __name__ == "__main__":
    main()
