#!/usr/bin/env python3
"""Quantitative link: yaw error → filter v_body oscillation in [1.69, 1.79].

Hypothesis: with truth vel along heading and ~level, filter body lateral ≈
  -V * sin(Δyaw)  (and vertical from roll/pitch error / coupling).
Also decompose: attitude-only vs velocity-only projection.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"
T0, T1 = 1.69, 1.79

TC04_SPEED_MPS = 50.0 / 3.6
TC04_SLALOM_PERIOD_S = 4.0
TC04_MAX_LATERAL_ACCEL_MPS2 = 3.0
OMEGA = 2.0 * np.pi / TC04_SLALOM_PERIOD_S
YAW_AMP = TC04_MAX_LATERAL_ACCEL_MPS2 / (TC04_SPEED_MPS * OMEGA)


def find_base_course_rad() -> float:
    for p in ROOT.rglob("*.hpp"):
        try:
            text = p.read_text(encoding="utf-8", errors="ignore")
        except Exception:
            continue
        m = re.search(r"#define\s+TC04_BASE_COURSE_DEG\s+([0-9.]+)", text)
        if m:
            return float(np.deg2rad(float(m.group(1))))
        m = re.search(r"TC04_BASE_COURSE_DEG\s*=\s*([0-9.]+)", text)
        if m:
            return float(np.deg2rad(float(m.group(1))))
    # telemetry t=0 yaw ≈ 1.573 → ~90°; slalom often 90°
    return float(np.deg2rad(90.0))


def truth_yaw(t: np.ndarray, base: float) -> np.ndarray:
    return base + YAW_AMP * np.sin(OMEGA * t)


def euler_to_dcm_bn(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Body-from-NED? Match ins_ekf quat_to_dcm_bn convention via ZYX yaw-pitch-roll.

    For ned_to_body in slalom_scenario: body[i] = sum_j dcm[j][i] * ned[j]
    with dcm from quat. Here build DCM_bn such that v_body = C_nb @ v_ned
    with C_nb = R3(yaw)^T for yaw-only (standard aerospace: body = R_yaw^T * ned
    for level).
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    # R = Rz(yaw) Ry(pitch) Rx(roll) maps body→NED; NED→body = R^T
    # v_body = R^T v_ned
    n = len(yaw)
    C = np.zeros((n, 3, 3))
    for i in range(n):
        Rz = np.array([[cy[i], -sy[i], 0], [sy[i], cy[i], 0], [0, 0, 1]])
        Ry = np.array([[cp[i], 0, sp[i]], [0, 1, 0], [-sp[i], 0, cp[i]]])
        Rx = np.array([[1, 0, 0], [0, cr[i], -sr[i]], [0, sr[i], cr[i]]])
        R_bn = Rz @ Ry @ Rx  # body to NED
        C[i] = R_bn.T  # NED to body
    return C


def project_ned_to_body(
    vn: np.ndarray, ve: np.ndarray, vd: np.ndarray, C: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    v = np.column_stack([vn, ve, vd])
    vb = np.einsum("nij,nj->ni", C, v)
    return vb[:, 0], vb[:, 1], vb[:, 2]


def wrap_pi(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def analyze_arm(arm: str, base: float) -> dict:
    audit = pd.read_csv(OUT / f"{arm}_nhc_block_audit.csv")
    telem = pd.read_csv(ROOT / f"docs/benchmarks/slalom_pattbias_{arm}_s71_telemetry.csv")
    telem["t"] = telem["time_us"].astype(float) * 1e-6

    m = (audit["timestamp_s"] >= T0) & (audit["timestamp_s"] <= T1)
    a = audit.loc[m].copy()
    t = a["timestamp_s"].to_numpy(float)

    # interpolate filter attitude / vel from telem
    yaw_f = np.interp(t, telem["t"], telem["yaw"].to_numpy(float))
    roll_f = np.interp(t, telem["t"], telem["roll"].to_numpy(float))
    pitch_f = np.interp(t, telem["t"], telem["pitch"].to_numpy(float))
    vn_f = np.interp(t, telem["t"], telem["vel_x"].to_numpy(float))
    ve_f = np.interp(t, telem["t"], telem["vel_y"].to_numpy(float))
    vd_f = np.interp(t, telem["t"], telem["vel_z"].to_numpy(float))

    yaw_t = truth_yaw(t, base)
    # truth vel NED
    vn_t = TC04_SPEED_MPS * np.cos(yaw_t)
    ve_t = TC04_SPEED_MPS * np.sin(yaw_t)
    vd_t = np.zeros_like(t)

    dyaw = wrap_pi(yaw_f - yaw_t)

    # filter v_body from audit (ground truth of what NHC sees)
    f_lat = a["v_body_y_before_mps"].to_numpy(float)
    f_vert = a["v_body_z_before_mps"].to_numpy(float)
    f_fwd = a["v_body_x_before_mps"].to_numpy(float)

    # Simple yaw-only model: v_lat ≈ -V sin(Δyaw), v_fwd ≈ V cos(Δyaw)
    V = TC04_SPEED_MPS
    pred_lat_yaw = -V * np.sin(dyaw)
    pred_fwd_yaw = V * np.cos(dyaw)
    pred_vert_yaw = np.zeros_like(t)  # yaw-only → no vertical

    # Full: project truth vel with filter attitude (attitude error only)
    C_f = euler_to_dcm_bn(roll_f, pitch_f, yaw_f)
    _, att_lat, att_vert = project_ned_to_body(vn_t, ve_t, vd_t, C_f)
    att_fwd, _, _ = project_ned_to_body(vn_t, ve_t, vd_t, C_f)

    # Project filter vel with truth attitude (velocity error only; truth level)
    C_t = euler_to_dcm_bn(np.zeros_like(t), np.zeros_like(t), yaw_t)
    _, vel_lat, vel_vert = project_ned_to_body(vn_f, ve_f, vd_f, C_t)

    # Project filter vel with filter attitude (should ≈ audit v_body)
    recon_fwd, recon_lat, recon_vert = project_ned_to_body(vn_f, ve_f, vd_f, C_f)

    def fit_stats(y, yhat, name: str) -> dict:
        # y ≈ a * yhat  (scale) and pearson
        denom = float(np.dot(yhat, yhat))
        a = float(np.dot(y, yhat) / denom) if denom > 1e-18 else float("nan")
        resid = y - yhat
        resid_scaled = y - a * yhat
        return {
            "name": name,
            "pearson": float(np.corrcoef(y, yhat)[0, 1]) if np.std(y) > 0 and np.std(yhat) > 0 else float("nan"),
            "rmse": float(np.sqrt(np.mean(resid**2))),
            "rmse_best_scale": float(np.sqrt(np.mean(resid_scaled**2))),
            "best_scale": a,
            "frac_var_explained": float(1.0 - np.var(resid) / max(np.var(y), 1e-30)),
            "frac_var_explained_scaled": float(
                1.0 - np.var(resid_scaled) / max(np.var(y), 1e-30)
            ),
        }

    stats = {
        "lat_vs_minus_V_sin_dyaw": fit_stats(f_lat, pred_lat_yaw, "filter_v_lat vs -V sin(Δyaw)"),
        "lat_vs_att_only": fit_stats(f_lat, att_lat, "filter_v_lat vs (truth_vel @ filter_att)_lat"),
        "lat_vs_vel_only": fit_stats(f_lat, vel_lat, "filter_v_lat vs (filter_vel @ truth_att)_lat"),
        "vert_vs_att_only": fit_stats(f_vert, att_vert, "filter_v_vert vs (truth_vel @ filter_att)_vert"),
        "vert_vs_vel_only": fit_stats(f_vert, vel_vert, "filter_v_vert vs (filter_vel @ truth_att)_vert"),
        "recon_lat_vs_audit": fit_stats(f_lat, recon_lat, "audit_lat vs telem-recon_lat"),
        "fwd_vs_V_cos_dyaw": fit_stats(f_fwd, pred_fwd_yaw, "filter_v_fwd vs V cos(Δyaw)"),
    }

    # small-angle: v_lat ≈ -V * Δyaw
    pred_lat_small = -V * dyaw
    stats["lat_vs_minus_V_dyaw_small"] = fit_stats(
        f_lat, pred_lat_small, "filter_v_lat vs -V·Δyaw (small-angle)"
    )

    ticks = pd.DataFrame(
        {
            "t_s": t,
            "yaw_truth": yaw_t,
            "yaw_filter": yaw_f,
            "dyaw_rad": dyaw,
            "dyaw_deg": np.rad2deg(dyaw),
            "roll_filter": roll_f,
            "pitch_filter": pitch_f,
            "filter_v_fwd": f_fwd,
            "filter_v_lat": f_lat,
            "filter_v_vert": f_vert,
            "pred_lat_minus_V_sin_dyaw": pred_lat_yaw,
            "pred_lat_att_only": att_lat,
            "pred_vert_att_only": att_vert,
            "pred_lat_vel_only": vel_lat,
            "pred_vert_vel_only": vel_vert,
            "resid_lat_minus_pred_sin": f_lat - pred_lat_yaw,
            "resid_lat_minus_att_only": f_lat - att_lat,
            "resid_vert_minus_att_only": f_vert - att_vert,
        }
    )
    tick_path = OUT / f"yaw_error_vs_vbody_{arm}_ticks.csv"
    ticks.to_csv(tick_path, index=False)

    # verdict
    s = stats["lat_vs_minus_V_sin_dyaw"]
    s_att = stats["lat_vs_att_only"]
    s_vel = stats["lat_vs_vel_only"]
    if s["pearson"] >= 0.9 and s["frac_var_explained"] >= 0.8:
        label = "YAW_ERROR_EXPLAINS_VLAT"
        reading = (
            "filter_v_lat tracks -V·sin(Δyaw) tightly — spurious body-lateral is the "
            "projection of forward speed through yaw error, as predicted by the attitude "
            "cascade."
        )
    elif s_att["pearson"] >= 0.9 and s_att["frac_var_explained"] >= 0.8:
        label = "ATTITUDE_ERROR_EXPLAINS_VLAT"
        reading = (
            "filter_v_lat tracks truth-vel projected with filter attitude (roll/pitch/yaw); "
            "attitude error (not only pure yaw-sin) explains the oscillation."
        )
    elif s_vel["frac_var_explained"] > s_att["frac_var_explained"] + 0.2:
        label = "VEL_ERROR_DOMINATES"
        reading = "Filter velocity error dominates body-lateral more than attitude misprojection."
    else:
        label = "PARTIAL_YAW_LINK"
        reading = (
            "Yaw/attitude error correlates with v_lat but does not fully close the equation; "
            "see stats table."
        )

    return {
        "arm": arm,
        "n": int(len(t)),
        "tick_csv": str(tick_path),
        "dyaw_deg": {
            "mean": float(np.mean(dyaw) * 180 / np.pi),
            "std": float(np.std(dyaw) * 180 / np.pi),
            "min": float(np.min(dyaw) * 180 / np.pi),
            "max": float(np.max(dyaw) * 180 / np.pi),
        },
        "roll_pitch_rms_deg": {
            "roll": float(np.sqrt(np.mean(roll_f**2)) * 180 / np.pi),
            "pitch": float(np.sqrt(np.mean(pitch_f**2)) * 180 / np.pi),
        },
        "stats": stats,
        "verdict": {"label": label, "reading": reading},
        "ticks": ticks,
    }


def main() -> None:
    base = find_base_course_rad()
    # calibrate base so truth yaw matches telem at t~0 if needed
    telem0 = pd.read_csv(ROOT / "docs/benchmarks/slalom_pattbias_latch_s71_telemetry.csv")
    t0 = float(telem0["time_us"].iloc[0]) * 1e-6
    yaw0 = float(telem0["yaw"].iloc[0])
    # at t=0, truth yaw = base; if filter seeded from truth, yaw0 ≈ base
    if abs(wrap_pi(np.array([yaw0 - base]))[0]) > 0.2:
        # prefer telem-implied base at t=0 (seeded)
        base = yaw0 - YAW_AMP * np.sin(OMEGA * t0)

    report = {"window_s": [T0, T1], "base_course_rad": base, "V_mps": TC04_SPEED_MPS, "arms": {}}
    results = {}
    for arm in ("ctrl", "latch"):
        results[arm] = analyze_arm(arm, base)
        # strip ticks from json
        r = {k: v for k, v in results[arm].items() if k != "ticks"}
        # stats are dicts ok
        report["arms"][arm] = r

    report["verdict"] = {
        "latch": results["latch"]["verdict"],
        "ctrl": results["ctrl"]["verdict"],
        "comparison": {
            "latch_pearson_sin": report["arms"]["latch"]["stats"]["lat_vs_minus_V_sin_dyaw"][
                "pearson"
            ],
            "ctrl_pearson_sin": report["arms"]["ctrl"]["stats"]["lat_vs_minus_V_sin_dyaw"][
                "pearson"
            ],
            "latch_frac_var_sin": report["arms"]["latch"]["stats"]["lat_vs_minus_V_sin_dyaw"][
                "frac_var_explained"
            ],
            "latch_frac_var_att_only": report["arms"]["latch"]["stats"]["lat_vs_att_only"][
                "frac_var_explained"
            ],
            "latch_frac_var_vel_only": report["arms"]["latch"]["stats"]["lat_vs_vel_only"][
                "frac_var_explained"
            ],
        },
    }

    # figure latch
    dl = results["latch"]["ticks"]
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(dl["t_s"], dl["dyaw_deg"], "C3", lw=1.2, label="Δyaw (filt−truth) deg")
    axes[0].axhline(0, color="gray", lw=0.6)
    axes[0].set_ylabel("Δyaw [deg]")
    axes[0].legend(fontsize=8)
    axes[0].set_title("Yaw error vs filter v_body — latch [1.69, 1.79]")

    axes[1].plot(dl["t_s"], dl["filter_v_lat"], "C3", lw=1.2, label="filter v_lat")
    axes[1].plot(
        dl["t_s"], dl["pred_lat_minus_V_sin_dyaw"], "k--", lw=1.2, label="-V sin(Δyaw)"
    )
    axes[1].plot(
        dl["t_s"], dl["pred_lat_att_only"], "C0:", lw=1.2, label="truth_vel@filter_att"
    )
    axes[1].axhline(0, color="gray", lw=0.6)
    axes[1].set_ylabel("v_lat [m/s]")
    axes[1].legend(fontsize=8)

    axes[2].plot(dl["t_s"], dl["filter_v_vert"], "C3", lw=1.2, label="filter v_vert")
    axes[2].plot(
        dl["t_s"], dl["pred_vert_att_only"], "C0:", lw=1.2, label="truth_vel@filter_att"
    )
    axes[2].axhline(0, color="gray", lw=0.6)
    axes[2].set_ylabel("v_vert [m/s]")
    axes[2].set_xlabel("t [s]")
    axes[2].legend(fontsize=8)
    fig.tight_layout()
    fig_path = OUT / "fig_yaw_error_vs_vbody.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "yaw_error_vs_vbody.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    # md
    ls = report["arms"]["latch"]["stats"]
    lines = [
        "# Yaw error ↔ filter v_body — quantitative close of cascade",
        "",
        f"**Verdict (latch):** `{results['latch']['verdict']['label']}`",
        "",
        results["latch"]["verdict"]["reading"],
        "",
        f"Base course used: {base:.6f} rad; V = {TC04_SPEED_MPS:.4f} m/s",
        "",
        "## Fit quality (latch)",
        "",
        "| Model | pearson | frac var | RMSE |",
        "|-------|---------|----------|------|",
    ]
    for key in (
        "lat_vs_minus_V_sin_dyaw",
        "lat_vs_minus_V_dyaw_small",
        "lat_vs_att_only",
        "lat_vs_vel_only",
        "vert_vs_att_only",
        "vert_vs_vel_only",
        "fwd_vs_V_cos_dyaw",
        "recon_lat_vs_audit",
    ):
        s = ls[key]
        lines.append(
            f"| {s['name']} | {s['pearson']:.3f} | {s['frac_var_explained']:.3f} | {s['rmse']:.4f} |"
        )
    lines += [
        "",
        f"Δyaw latch: mean={report['arms']['latch']['dyaw_deg']['mean']:.2f}° "
        f"[{report['arms']['latch']['dyaw_deg']['min']:.2f}, "
        f"{report['arms']['latch']['dyaw_deg']['max']:.2f}]°",
        f"roll/pitch RMS: "
        f"{report['arms']['latch']['roll_pitch_rms_deg']['roll']:.2f}° / "
        f"{report['arms']['latch']['roll_pitch_rms_deg']['pitch']:.2f}°",
        "",
        "## Tick table — latch",
        "",
        "| t | Δyaw° | v_lat | −V sinΔψ | att_only_lat | v_vert | att_only_vert | resid_lat |",
        "|---|-------|-------|----------|--------------|--------|---------------|-----------|",
    ]
    for _, r in dl.iterrows():
        lines.append(
            f"| {r['t_s']:.3f} | {r['dyaw_deg']:+.2f} | {r['filter_v_lat']:+.3f} | "
            f"{r['pred_lat_minus_V_sin_dyaw']:+.3f} | {r['pred_lat_att_only']:+.3f} | "
            f"{r['filter_v_vert']:+.3f} | {r['pred_vert_att_only']:+.3f} | "
            f"{r['resid_lat_minus_pred_sin']:+.3f} |"
        )
    # ctrl summary
    cs = report["arms"]["ctrl"]["stats"]["lat_vs_minus_V_sin_dyaw"]
    lines += [
        "",
        "## Ctrl comparison",
        "",
        f"- pearson(-V sinΔψ): **{cs['pearson']:.3f}**, frac var **{cs['frac_var_explained']:.3f}**",
        f"- Δyaw ctrl mean={report['arms']['ctrl']['dyaw_deg']['mean']:.2f}° "
        f"range [{report['arms']['ctrl']['dyaw_deg']['min']:.2f}, "
        f"{report['arms']['ctrl']['dyaw_deg']['max']:.2f}]°",
        "",
        f"Figure: `{fig_path.name}`",
        "",
    ]
    (OUT / "yaw_error_vs_vbody.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report["verdict"], indent=2))


if __name__ == "__main__":
    main()
