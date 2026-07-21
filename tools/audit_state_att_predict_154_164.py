#!/usr/bin/env python3
"""Priority (2): state attitude (q→Euler) + ω in predict window [1.54,1.64].

Hypothesis: latch freezes dx_att_z but leaves yaw error in q_att_; quaternion
integration of measured ω through biased yaw spreads into pitch between NHC
corrections. dx_att_y≈ctrl already argues against (3).

Uses existing telem + NHC audit (no rebuild). Ideal SLALOM: gyro=(0,0,ωz).
Predict rate proxy: Δeuler_telem − dx_att (NHC inject) per 10 ms tick.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"
T0, T1 = 1.54, 1.64
T_BREAK = 1.59
DT = 0.01

V = 50.0 / 3.6
OMEGA = 2.0 * np.pi / 4.0
YAW_AMP = 3.0 / (V * OMEGA)
BASE = np.pi / 2.0


def wrap(a: np.ndarray | float) -> np.ndarray | float:
    return (a + np.pi) % (2 * np.pi) - np.pi


def truth_yaw(t: np.ndarray) -> np.ndarray:
    return BASE + YAW_AMP * np.sin(OMEGA * t)


def truth_omega_z(t: np.ndarray) -> np.ndarray:
    return YAW_AMP * OMEGA * np.cos(OMEGA * t)


def load_arm(arm: str) -> pd.DataFrame:
    telem = pd.read_csv(ROOT / f"docs/benchmarks/slalom_pattbias_{arm}_s71_telemetry.csv")
    telem["t"] = telem["time_us"].astype(float) * 1e-6
    m = (telem["t"] >= T0 - DT) & (telem["t"] <= T1 + DT)
    d = telem.loc[m].copy().reset_index(drop=True)

    audit = pd.read_csv(OUT / f"{arm}_nhc_block_audit.csv")
    am = (audit["timestamp_s"] >= T0 - DT) & (audit["timestamp_s"] <= T1 + DT)
    a = audit.loc[am]

    t = d["t"].to_numpy(float)
    roll = d["roll"].to_numpy(float)
    pitch = d["pitch"].to_numpy(float)
    yaw = d["yaw"].to_numpy(float)
    yaw_t = truth_yaw(t)
    dyaw = wrap(yaw - yaw_t)

    # telem Δ per tick (includes predict + NHC in that cycle)
    d_roll = np.zeros_like(roll)
    d_pitch = np.zeros_like(pitch)
    d_yaw = np.zeros_like(yaw)
    d_roll[1:] = wrap(roll[1:] - roll[:-1])
    d_pitch[1:] = pitch[1:] - pitch[:-1]
    d_yaw[1:] = wrap(yaw[1:] - yaw[:-1])

    # NHC dx_att at telem times
    dx_x = np.interp(t, a["timestamp_s"], a["dx_att_x_rad"].to_numpy(float))
    dx_y = np.interp(t, a["timestamp_s"], a["dx_att_y_rad"].to_numpy(float))
    dx_z = np.interp(t, a["timestamp_s"], a["dx_att_z_rad"].to_numpy(float))

    # Predict proxy: state change minus NHC inject (GNSS does not write att here)
    pred_roll = d_roll - dx_x
    pred_pitch = d_pitch - dx_y
    pred_yaw = d_yaw - dx_z

    omega_z_meas = d["yaw_rate"].to_numpy(float)
    omega_z_truth = truth_omega_z(t)
    bgx = d["bias_gx"].to_numpy(float)
    bgy = d["bias_gy"].to_numpy(float)
    bgz = d["bias_gz"].to_numpy(float)
    # Ideal SLALOM: gyro_x=gyro_y=0 → ω_corr = -bias_g for x/y
    w_corr_x = 0.0 - bgx
    w_corr_y = 0.0 - bgy
    w_corr_z = omega_z_meas - bgz

    out = pd.DataFrame(
        {
            "t": t,
            "roll_deg": np.rad2deg(roll),
            "pitch_deg": np.rad2deg(pitch),
            "yaw_deg": np.rad2deg(yaw),
            "yaw_truth_deg": np.rad2deg(yaw_t),
            "dyaw_deg": np.rad2deg(dyaw),
            "droll_telem_mrad": d_roll * 1e3,
            "dpitch_telem_mrad": d_pitch * 1e3,
            "dyaw_telem_mrad": d_yaw * 1e3,
            "dx_att_x_mrad": dx_x * 1e3,
            "dx_att_y_mrad": dx_y * 1e3,
            "dx_att_z_mrad": dx_z * 1e3,
            "pred_roll_mrad": pred_roll * 1e3,
            "pred_pitch_mrad": pred_pitch * 1e3,
            "pred_yaw_mrad": pred_yaw * 1e3,
            "omega_z_meas": omega_z_meas,
            "omega_z_truth": omega_z_truth,
            "bias_gx": bgx,
            "bias_gy": bgy,
            "bias_gz": bgz,
            "w_corr_x": w_corr_x,
            "w_corr_y": w_corr_y,
            "w_corr_z": w_corr_z,
        }
    )
    # focus window
    out = out[(out["t"] >= T0) & (out["t"] <= T1)].reset_index(drop=True)
    out.to_csv(OUT / f"state_att_predict_{arm}_ticks.csv", index=False)
    return out


def window_stats(df: pd.DataFrame) -> dict:
    pre = df["t"] < T_BREAK
    post = df["t"] >= T_BREAK

    def blk(mask: np.ndarray) -> dict:
        d = df.loc[mask]
        return {
            "n": int(mask.sum()),
            "mean_dyaw_deg": float(d["dyaw_deg"].mean()),
            "mean_pitch_deg": float(d["pitch_deg"].mean()),
            "mean_roll_deg": float(d["roll_deg"].mean()),
            "end_dyaw_deg": float(d["dyaw_deg"].iloc[-1]) if len(d) else float("nan"),
            "end_pitch_deg": float(d["pitch_deg"].iloc[-1]) if len(d) else float("nan"),
            "end_roll_deg": float(d["roll_deg"].iloc[-1]) if len(d) else float("nan"),
            "sum_pred_pitch_mrad": float(d["pred_pitch_mrad"].sum()),
            "sum_pred_yaw_mrad": float(d["pred_yaw_mrad"].sum()),
            "sum_pred_roll_mrad": float(d["pred_roll_mrad"].sum()),
            "sum_dx_y_mrad": float(d["dx_att_y_mrad"].sum()),
            "sum_dx_z_mrad": float(d["dx_att_z_mrad"].sum()),
            "sum_dpitch_telem_mrad": float(d["dpitch_telem_mrad"].sum()),
            "sum_dyaw_telem_mrad": float(d["dyaw_telem_mrad"].sum()),
            "mean_abs_w_corr_x": float(np.abs(d["w_corr_x"]).mean()),
            "mean_abs_w_corr_y": float(np.abs(d["w_corr_y"]).mean()),
            "mean_omega_z": float(d["omega_z_meas"].mean()),
            "mean_bias_gz": float(d["bias_gz"].mean()),
        }

    return {"pre": blk(pre.to_numpy()), "post": blk(post.to_numpy()), "full": blk(np.ones(len(df), dtype=bool))}


def main() -> None:
    arms = {arm: load_arm(arm) for arm in ("ctrl", "latch")}
    L, C = arms["latch"], arms["ctrl"]

    # Align on t
    t = L["t"].to_numpy(float)
    assert np.allclose(t, C["t"].to_numpy(float), atol=1e-9)

    d_pitch = L["pitch_deg"].to_numpy() - C["pitch_deg"].to_numpy()
    d_roll = L["roll_deg"].to_numpy() - C["roll_deg"].to_numpy()
    d_dyaw = L["dyaw_deg"].to_numpy() - C["dyaw_deg"].to_numpy()  # latch−ctrl yaw error
    d_pred_p = L["pred_pitch_mrad"].to_numpy() - C["pred_pitch_mrad"].to_numpy()
    d_pred_y = L["pred_yaw_mrad"].to_numpy() - C["pred_yaw_mrad"].to_numpy()

    # Co-growth: does |Δyaw_error latch−ctrl| track |Δpitch latch−ctrl|?
    # Use absolute levels and changes from t0
    pitch_L0 = L["pitch_deg"].iloc[0]
    dyaw_L0 = L["dyaw_deg"].iloc[0]
    pitch_growth_L = L["pitch_deg"].to_numpy() - pitch_L0
    dyaw_growth_L = L["dyaw_deg"].to_numpy() - dyaw_L0
    pitch_growth_C = C["pitch_deg"].to_numpy() - C["pitch_deg"].iloc[0]
    dyaw_growth_C = C["dyaw_deg"].to_numpy() - C["dyaw_deg"].iloc[0]

    def safe_corr(a, b):
        if len(a) < 3 or np.std(a) < 1e-12 or np.std(b) < 1e-12:
            return float("nan")
        return float(np.corrcoef(a, b)[0, 1])

    stats = {arm: window_stats(df) for arm, df in arms.items()}

    # Fraction of pitch state change explained by predict proxy vs NHC
    def frac(arm_df):
        sp = float(arm_df["sum_pred_pitch_mrad"] if False else arm_df["pred_pitch_mrad"].sum())
        sd = float(arm_df["dx_att_y_mrad"].sum())
        st = float(arm_df["dpitch_telem_mrad"].sum())
        return {
            "sum_pred_pitch_mrad": sp,
            "sum_dx_y_mrad": sd,
            "sum_dpitch_telem_mrad": st,
            "pred_share_of_telem": sp / st if abs(st) > 1e-9 else float("nan"),
            "nhc_share_of_telem": sd / st if abs(st) > 1e-9 else float("nan"),
        }

    latch_pitch_budget = frac(L)
    ctrl_pitch_budget = frac(C)
    latch_yaw_budget = {
        "sum_pred_yaw_mrad": float(L["pred_yaw_mrad"].sum()),
        "sum_dx_z_mrad": float(L["dx_att_z_mrad"].sum()),
        "sum_dyaw_telem_mrad": float(L["dyaw_telem_mrad"].sum()),
    }
    ctrl_yaw_budget = {
        "sum_pred_yaw_mrad": float(C["pred_yaw_mrad"].sum()),
        "sum_dx_z_mrad": float(C["dx_att_z_mrad"].sum()),
        "sum_dyaw_telem_mrad": float(C["dyaw_telem_mrad"].sum()),
    }

    # Ideal SLALOM note
    gyro_xy_zero = True

    # Verdict logic
    # A: yaw state still diverges under latch (dyaw grows / differs from ctrl via predict)
    latch_yaw_predict_dom = abs(latch_yaw_budget["sum_pred_yaw_mrad"]) > 2.0 * max(
        abs(latch_yaw_budget["sum_dx_z_mrad"]), 1e-9
    )
    # B: pitch telem change dominated by predict proxy, not dx_y (or latch−ctrl pitch diff from pred)
    latch_pitch_pred_share = latch_pitch_budget["pred_share_of_telem"]
    # C: co-timing yaw growth ↔ pitch growth under latch
    corr_yaw_pitch_L = safe_corr(dyaw_growth_L, pitch_growth_L)
    corr_dpred = safe_corr(d_pred_y, d_pred_p)

    # Latch vs ctrl: does predict pitch diverge more under latch?
    sum_d_pred_pitch = float(d_pred_p.sum())
    sum_d_dx_y = float((L["dx_att_y_mrad"] - C["dx_att_y_mrad"]).sum())

    if (
        latch_yaw_predict_dom
        and abs(latch_pitch_pred_share) >= 0.4
        and abs(corr_yaw_pitch_L) >= 0.7
        and abs(sum_d_pred_pitch) > abs(sum_d_dx_y)
    ):
        label = "PREDICT_YAW_BIAS_COUPLES_PITCH"
        reading = (
            "Under latch, yaw state change is predict-dominated (dx_z≡0) while pitch "
            "telem Δ is largely explained by the predict proxy, not by anomalous NHC "
            "dx_y. Yaw-error growth and pitch growth co-time. Consistent with "
            "quaternion integration of ω through an already-biased q_att_."
        )
        next_step = (
            "Close (2): design must remove early yaw contamination in state, not only "
            "gate future dx_att_z. Optional: decompose ω_corr vs pure quat coupling "
            "(ideal: gx=gy=0 → residual via bias_g or q⊗δq)."
        )
    elif latch_yaw_predict_dom and abs(sum_d_pred_pitch) > abs(sum_d_dx_y):
        label = "PREDICT_PITCH_DIFF_YAW_OPEN"
        reading = (
            "Latch−ctrl pitch divergence is carried by the predict proxy more than by "
            "dx_att_y, and latch yaw evolves without NHC Z. Co-timing with pitch is "
            "weaker/ambiguous — still points to predict, mechanism of axis coupling TBD."
        )
        next_step = "Inspect ω_corr (esp. −bias_gx/gy) vs pure yaw-rate⊗biased-q coupling."
    elif abs(latch_pitch_pred_share) < 0.3 and abs(sum_d_dx_y) >= abs(sum_d_pred_pitch):
        label = "NHC_PITCH_STILL_DOMINATES"
        reading = (
            "Pitch state change still mostly from NHC dx_y, contrary to the "
            "predict-coupling reading — revisit (3) or budgeting assumptions."
        )
        next_step = "Re-check telem vs NHC timing alignment; reconsider (3)."
    else:
        label = "PREDICT_MIXED"
        reading = (
            "State yaw/pitch budgets are mixed — predict matters but the clean "
            "yaw-bias→pitch coupling signature is incomplete."
        )
        next_step = "Table-driven follow-up on pred_* residuals and bias_g."

    report = {
        "window_s": [T0, T1],
        "break_s": T_BREAK,
        "ideal_slalom_gyro_xy_zero": gyro_xy_zero,
        "note": (
            "Ideal SLALOM: measured gyro=(0,0,ωz). Pitch/roll rates in predict come "
            "from −bias_g and/or quaternion composition with non-level q_att_."
        ),
        "arms": stats,
        "pitch_budget": {"latch": latch_pitch_budget, "ctrl": ctrl_pitch_budget},
        "yaw_budget": {"latch": latch_yaw_budget, "ctrl": ctrl_yaw_budget},
        "latch_minus_ctrl": {
            "sum_pred_pitch_mrad": sum_d_pred_pitch,
            "sum_dx_y_mrad": sum_d_dx_y,
            "sum_pred_yaw_mrad": float(d_pred_y.sum()),
            "end_d_pitch_deg": float(d_pitch[-1]),
            "end_d_dyaw_deg": float(d_dyaw[-1]),
            "end_d_roll_deg": float(d_roll[-1]),
            "start_d_pitch_deg": float(d_pitch[0]),
            "start_d_dyaw_deg": float(d_dyaw[0]),
        },
        "co_timing": {
            "corr_latch_dyaw_growth_vs_pitch_growth": corr_yaw_pitch_L,
            "corr_ctrl_dyaw_growth_vs_pitch_growth": safe_corr(dyaw_growth_C, pitch_growth_C),
            "corr_latch_minus_ctrl_pred_yaw_vs_pred_pitch": corr_dpred,
            "latch_pitch_growth_deg": float(pitch_growth_L[-1]),
            "latch_dyaw_growth_deg": float(dyaw_growth_L[-1]),
            "ctrl_pitch_growth_deg": float(pitch_growth_C[-1]),
            "ctrl_dyaw_growth_deg": float(dyaw_growth_C[-1]),
        },
        "verdict": {
            "label": label,
            "reading": reading,
            "next": next_step,
            "latch_yaw_predict_dominated": latch_yaw_predict_dom,
            "latch_pitch_pred_share": latch_pitch_pred_share,
            "corr_yaw_pitch_growth_latch": corr_yaw_pitch_L,
        },
    }

    # Figure
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    axes[0].plot(t, L["dyaw_deg"], "C3", label="latch Δyaw")
    axes[0].plot(t, C["dyaw_deg"], "C0", label="ctrl Δyaw")
    axes[0].plot(t, L["pitch_deg"], "C3", ls="--", label="latch pitch")
    axes[0].plot(t, C["pitch_deg"], "C0", ls="--", label="ctrl pitch")
    axes[0].set_ylabel("deg")
    axes[0].set_title("State Euler error/levels [1.54→1.64]")
    axes[0].legend(fontsize=7, ncol=2)

    axes[1].plot(t, L["pred_pitch_mrad"], "C3", label="latch predΔpitch")
    axes[1].plot(t, C["pred_pitch_mrad"], "C0", label="ctrl predΔpitch")
    axes[1].plot(t, L["dx_att_y_mrad"], "C3", ls="--", label="latch dx_y")
    axes[1].plot(t, C["dx_att_y_mrad"], "C0", ls="--", label="ctrl dx_y")
    axes[1].set_ylabel("mrad/tick")
    axes[1].set_title("Pitch: predict proxy vs NHC dx_att_y")
    axes[1].legend(fontsize=7, ncol=2)

    axes[2].plot(t, L["pred_yaw_mrad"], "C3", label="latch predΔyaw")
    axes[2].plot(t, C["pred_yaw_mrad"], "C0", label="ctrl predΔyaw")
    axes[2].plot(t, L["dx_att_z_mrad"], "C3", ls="--", label="latch dx_z")
    axes[2].plot(t, C["dx_att_z_mrad"], "C0", ls="--", label="ctrl dx_z")
    axes[2].set_ylabel("mrad/tick")
    axes[2].set_title("Yaw: predict proxy vs NHC dx_att_z")
    axes[2].legend(fontsize=7, ncol=2)

    axes[3].plot(t, L["omega_z_meas"], "C3", label="latch ωz meas")
    axes[3].plot(t, C["omega_z_meas"], "C0", label="ctrl ωz meas")
    axes[3].plot(t, L["omega_z_truth"], "k", ls=":", label="truth ωz")
    axes[3].plot(t, L["w_corr_x"] * 100, "C3", ls="--", alpha=0.7, label="latch w_corr_x×100")
    axes[3].plot(t, L["w_corr_y"] * 100, "C1", ls="--", alpha=0.7, label="latch w_corr_y×100")
    axes[3].set_ylabel("rad/s")
    axes[3].set_xlabel("t [s]")
    axes[3].set_title("ω measured / corrected (gx=gy=0 ideal → w_corr_xy=−bias)")
    axes[3].legend(fontsize=7, ncol=3)

    for ax in axes:
        ax.axvline(T_BREAK, color="gray", ls="--", alpha=0.5)
        ax.axhline(0, color="gray", lw=0.4)
    fig.tight_layout()
    fig_path = OUT / "fig_state_att_predict_154_164.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "state_att_predict_154_164.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )

    lines = [
        "# State attitude + ω predict check [1.54→1.64] — priority (2)",
        "",
        f"**Verdict:** `{label}`",
        "",
        reading,
        "",
        f"**Next:** {next_step}",
        "",
        "Ideal SLALOM: measured `gyro=(0,0,ωz)`. `w_corr_x/y = −bias_g`.",
        "Predict proxy per tick: `Δeuler_telem − dx_att_NHC`.",
        "",
        "## Pitch budget (sum over window, mrad)",
        "",
        "| Arm | ΣΔpitch_telem | Σ predΔpitch | Σ dx_att_y | pred share |",
        "|-----|---------------|--------------|------------|------------|",
    ]
    for arm, b in (("latch", latch_pitch_budget), ("ctrl", ctrl_pitch_budget)):
        lines.append(
            f"| {arm} | {b['sum_dpitch_telem_mrad']:+.3f} | {b['sum_pred_pitch_mrad']:+.3f} | "
            f"{b['sum_dx_y_mrad']:+.3f} | {b['pred_share_of_telem']:+.2f} |"
        )
    lines += [
        "",
        "## Yaw budget (sum over window, mrad)",
        "",
        "| Arm | ΣΔyaw_telem | Σ predΔyaw | Σ dx_att_z |",
        "|-----|-------------|------------|------------|",
        f"| latch | {latch_yaw_budget['sum_dyaw_telem_mrad']:+.3f} | "
        f"{latch_yaw_budget['sum_pred_yaw_mrad']:+.3f} | "
        f"{latch_yaw_budget['sum_dx_z_mrad']:+.3f} |",
        f"| ctrl | {ctrl_yaw_budget['sum_dyaw_telem_mrad']:+.3f} | "
        f"{ctrl_yaw_budget['sum_pred_yaw_mrad']:+.3f} | "
        f"{ctrl_yaw_budget['sum_dx_z_mrad']:+.3f} |",
        "",
        "## Latch − ctrl",
        "",
        f"- Σ(predΔpitch): **{sum_d_pred_pitch:+.3f} mrad**",
        f"- Σ(dx_y): **{sum_d_dx_y:+.3f} mrad**",
        f"- end Δ(pitch): {float(d_pitch[-1]):+.3f}° (start {float(d_pitch[0]):+.3f}°)",
        f"- end Δ(Δyaw): {float(d_dyaw[-1]):+.3f}° (start {float(d_dyaw[0]):+.3f}°)",
        "",
        "## Co-timing",
        "",
        f"- corr(latch Δyaw growth, pitch growth) = **{corr_yaw_pitch_L:+.3f}**",
        f"- latch pitch growth = {float(pitch_growth_L[-1]):+.3f}°; "
        f"Δyaw growth = {float(dyaw_growth_L[-1]):+.3f}°",
        f"- ctrl pitch growth = {float(pitch_growth_C[-1]):+.3f}°; "
        f"Δyaw growth = {float(dyaw_growth_C[-1]):+.3f}°",
        "",
        "## Tick table — latch",
        "",
        "| t | Δyaw° | pitch° | roll° | predΔp | dx_y | predΔψ | dx_z | ωz |",
        "|---|-------|--------|-------|--------|------|--------|------|----|",
    ]
    for _, r in L.iterrows():
        lines.append(
            f"| {r.t:.3f} | {r.dyaw_deg:+.3f} | {r.pitch_deg:+.3f} | {r.roll_deg:+.3f} | "
            f"{r.pred_pitch_mrad:+.2f} | {r.dx_att_y_mrad:+.2f} | "
            f"{r.pred_yaw_mrad:+.2f} | {r.dx_att_z_mrad:+.2f} | {r.omega_z_meas:+.3f} |"
        )
    lines += [
        "",
        "## Tick table — ctrl (compact)",
        "",
        "| t | Δyaw° | pitch° | predΔp | dx_y | predΔψ | dx_z |",
        "|---|-------|--------|--------|------|--------|------|",
    ]
    for _, r in C.iterrows():
        lines.append(
            f"| {r.t:.3f} | {r.dyaw_deg:+.3f} | {r.pitch_deg:+.3f} | "
            f"{r.pred_pitch_mrad:+.2f} | {r.dx_att_y_mrad:+.2f} | "
            f"{r.pred_yaw_mrad:+.2f} | {r.dx_att_z_mrad:+.2f} |"
        )
    lines += ["", f"Figure: `{fig_path.name}`", ""]
    (OUT / "state_att_predict_154_164.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report["verdict"], indent=2))
    print("---")
    print(json.dumps(report["pitch_budget"], indent=2))
    print(json.dumps(report["yaw_budget"], indent=2))
    print(json.dumps(report["latch_minus_ctrl"], indent=2))
    print(json.dumps(report["co_timing"], indent=2))


if __name__ == "__main__":
    main()
