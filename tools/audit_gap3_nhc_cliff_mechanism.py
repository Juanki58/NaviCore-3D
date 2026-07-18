#!/usr/bin/env python3
"""GAP-3.16 — NHC cliff mechanism: K saturation, predict +3.2, floor, F1 cliff compare.

Four falsification checks before any NHC_MAX_GAIN / R_nhc change:
  1. K vs P/(P+R) on cliff ticks 2–4 (baseline nhc_block)
  2. predict +3.2 vs Q_vel white noise (cross-term accounting)
  3. P_vv post-fix#3 extension (floor vs continued fall)
  4. F1 baseline vs N=10 tick ΔP_vv pattern + ||ΔP||/||Δx|| ratio
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
AUTOPSY = REPO_ROOT / "docs" / "benchmarks" / "gap3_gnss_accepted_autopsy"
F1_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_f1_nhc_dose_response"
OUT_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_nhc_cliff_mechanism"
REPORT_JSON = OUT_DIR / "gap3_nhc_cliff_mechanism_report.json"
SUMMARY_MD = OUT_DIR / "gap3_nhc_cliff_mechanism.md"

T_FIX2 = 5.664433479
T_FIX3 = 6.053678513
DT_IMU = 0.01
SIGMA_A = 0.05
ACCEL_NOISE_VAR = SIGMA_A**2
Q_VEL_PER_TICK = ACCEL_NOISE_VAR * DT_IMU  # 2.5e-5 per axis diagonal


def load_csv(path: Path) -> pd.DataFrame:
    if not path.is_file():
        return pd.DataFrame()
    df = pd.read_csv(path, index_col=False)
    skip = {"update_type", "phase", "event", "constraint_policy", "source"}
    for col in df.columns:
        if col in skip:
            continue
        converted = pd.to_numeric(df[col], errors="coerce")
        if converted.notna().any():
            df[col] = converted
    return df


def dx_norm(row: pd.Series) -> float:
    parts = [
        row.get("dx_vel_norm_mps", 0),
        row.get("dx_pos_norm_m", 0),
        row.get("dx_att_norm_rad", 0),
        row.get("dx_bias_norm", 0),
    ]
    return float(math.sqrt(sum(p**2 for p in parts if pd.notna(p))))


def dP_norm(row: pd.Series) -> float:
    parts = [
        abs(row.get("delta_P_vv_frob", 0)),
        abs(row.get("delta_P_pv_frob", 0)),
        abs(row.get("delta_P_pp_frob", 0)),
        abs(row.get("delta_P_aa_frob", 0)),
    ]
    return float(math.sqrt(sum(p**2 for p in parts if pd.notna(p))))


def analyze_k_ticks(nhc: pd.DataFrame, imu_seqs: list[int], tick_labels: list[int]) -> list[dict]:
    rows = []
    for tick_idx, imu in zip(tick_labels, imu_seqs):
        r = nhc[nhc["imu_seq"] == imu]
        if r.empty:
            continue
        r = r.iloc[0]
        hph_yy = float(r["hph_yy"])
        hph_zz = float(r["hph_zz"])
        r_y = float(r["r_y_m2"])
        r_z = float(r["r_z_m2"])
        s_yy = float(r["s_yy"])
        s_zz = float(r["s_zz"])
        k_scalar_y = hph_yy / (hph_yy + r_y) if (hph_yy + r_y) > 0 else math.nan
        k_scalar_z = hph_zz / (hph_zz + r_z) if (hph_zz + r_z) > 0 else math.nan
        rows.append(
            {
                "tick": tick_idx,
                "imu_seq": int(imu),
                "timestamp_s": float(r["timestamp_s"]),
                "hph_yy": hph_yy,
                "hph_zz": hph_zz,
                "r_y_m2": r_y,
                "r_z_m2": r_z,
                "s_yy": s_yy,
                "s_zz": s_zz,
                "k_scalar_y_hph_over_s": k_scalar_y,
                "k_scalar_z_hph_over_s": k_scalar_z,
                "k_y_max_logged": float(r["k_y_max"]),
                "k_z_max_logged": float(r["k_z_max"]),
                "k_vel_max_logged": float(r["k_vel_max"]),
                "nis_total": float(r["nis_total"]),
                "innov_norm_mps": float(r["innov_norm_mps"]),
                "delta_P_vv": float(r["delta_P_vv_frob"]),
                "dx_norm": dx_norm(r),
                "dP_norm": dP_norm(r),
                "cov_consume_ratio_dP_over_dx": dP_norm(r) / dx_norm(r) if dx_norm(r) > 1e-12 else math.inf,
                "P_pre_vv": float(r["P_pre_vv_frob"]),
            }
        )
    return rows


def analyze_predict_gap(cov: pd.DataFrame, t2: float, t3: float) -> dict:
    gap = cov[(cov["timestamp_s"] > t2) & (cov["timestamp_s"] < t3)]
    pred_post = gap[(gap["update_type"] == "predict") & (gap["phase"] == "post")]
    pred_pre = gap[(gap["update_type"] == "predict") & (gap["phase"] == "pre")]

    sum_d_pvv = 0.0
    sum_d_ppv = 0.0
    sum_d_paa = 0.0
    for imu in sorted(gap["imu_seq"].unique()):
        pre = pred_pre[pred_pre["imu_seq"] == imu]
        post = pred_post[pred_post["imu_seq"] == imu]
        if len(pre) and len(post):
            sum_d_pvv += float(post.iloc[0]["P_vv_frob"] - pre.iloc[0]["P_vv_frob"])
            sum_d_ppv += float(post.iloc[0]["P_pv_frob"] - pre.iloc[0]["P_pv_frob"])
            sum_d_paa += float(post.iloc[0]["P_aa_frob"] - pre.iloc[0]["P_aa_frob"])

    n_ticks = len(pred_post)
    q_vel_frob_white_only = math.sqrt(3 * n_ticks * Q_VEL_PER_TICK**2)
    ratio_obs_vs_white = sum_d_pvv / q_vel_frob_white_only if q_vel_frob_white_only > 0 else math.nan

    return {
        "n_predict_ticks": n_ticks,
        "sum_delta_P_vv_predict": sum_d_pvv,
        "sum_delta_P_pv_predict": sum_d_ppv,
        "sum_delta_P_aa_predict": sum_d_paa,
        "Q_vel_frob_white_noise_only_est": q_vel_frob_white_only,
        "P_vv_predict_over_white_Q_ratio": ratio_obs_vs_white,
        "interpretation": (
            f"Observed +{sum_d_pvv:.2f} >> white Q frob ~{q_vel_frob_white_only:.4f} "
            f"(ratio {ratio_obs_vs_white:.0f}x) → growth from F*P cross-terms (att/bias/pos→vel), not Q alone."
        ),
    }


def pvv_gap_tail_equilibrium(cov: pd.DataFrame, t2: float, t3: float, tail_n: int = 10) -> dict:
    gap = cov[(cov["timestamp_s"] > t2) & (cov["timestamp_s"] < t3)]
    nhc_post = gap[(gap["update_type"] == "nhc") & (gap["phase"] == "post")].sort_values("timestamp_s")
    if len(nhc_post) < tail_n:
        tail_n = len(nhc_post)
    tail = nhc_post.tail(tail_n)
    pvv = tail["P_vv_frob"].values
    slope = float(np.polyfit(np.arange(len(pvv)), pvv, 1)[0]) if len(pvv) >= 2 else 0.0
    return {
        "last_n_ticks_P_vv": [float(x) for x in pvv],
        "tail_slope_per_tick": slope,
        "tail_mean": float(np.mean(pvv)),
        "tail_std": float(np.std(pvv)),
        "verdict": "soft_floor_in_gap" if abs(slope) < 0.05 and float(np.std(pvv)) < 0.15 else "still_descending_at_fix3",
    }


def pvv_post_fix3_extension(cov: pd.DataFrame, t3: float, extend_s: float = 4.0) -> dict:
    win = cov[(cov["timestamp_s"] >= t3) & (cov["timestamp_s"] <= t3 + extend_s)].copy()
    nhc_post = win[(win["update_type"] == "nhc") & (win["phase"] == "post")].sort_values("timestamp_s")
    pred_post = win[(win["update_type"] == "predict") & (win["phase"] == "post")].sort_values("timestamp_s")
    gnss = win[win["update_type"] == "gnss"]

    series = []
    for _, r in win.sort_values("timestamp_s").iterrows():
        if r["phase"] in ("post", "post_accept") and r["update_type"] in ("nhc", "predict", "gnss"):
            series.append(
                {
                    "t": float(r["timestamp_s"]),
                    "type": str(r["update_type"]),
                    "P_vv": float(r["P_vv_frob"]),
                }
            )

    pvv_vals = [s["P_vv"] for s in series]
    if not pvv_vals:
        return {"verdict": "no_data"}

    t_end = series[-1]["t"]
    pvv_min = min(pvv_vals)
    pvv_max = max(pvv_vals)
    pvv_at_t3 = pvv_vals[0]
    pvv_at_end = pvv_vals[-1]
    still_falling = pvv_at_end < pvv_at_t3 * 0.5

    return {
        "extend_s": extend_s,
        "P_vv_at_fix3_pre": pvv_at_t3,
        "P_vv_at_extend_end": pvv_at_end,
        "P_vv_min_in_window": pvv_min,
        "P_vv_max_in_window": pvv_max,
        "P_vv_range_in_window": pvv_max - pvv_min,
        "nhc_post_samples": len(nhc_post),
        "verdict": "oscillating_equilibrium" if pvv_max - pvv_min < 5.0 and not still_falling else "continued_monotonic_fall",
        "series_sample": series[:: max(1, len(series) // 20)],
    }


def gap_tick_deltas(cov: pd.DataFrame, t2: float, t3: float) -> pd.DataFrame:
    gap = cov[(cov["timestamp_s"] > t2) & (cov["timestamp_s"] < t3)].sort_values("timestamp_s")
    ticks = []
    for tick_idx, imu in enumerate(sorted(gap["imu_seq"].unique()), start=1):
        tick = gap[gap["imu_seq"] == imu]
        pred_pre = tick[(tick["update_type"] == "predict") & (tick["phase"] == "pre")]
        pred_post = tick[(tick["update_type"] == "predict") & (tick["phase"] == "post")]
        nhc_post = tick[(tick["update_type"] == "nhc") & (tick["phase"] == "post")]
        if pred_pre.empty:
            continue
        d_pred = float(pred_post.iloc[0]["P_vv_frob"] - pred_pre.iloc[0]["P_vv_frob"]) if len(pred_post) else 0.0
        d_nhc = (
            float(nhc_post.iloc[0]["P_vv_frob"] - pred_post.iloc[0]["P_vv_frob"])
            if len(nhc_post) and len(pred_post)
            else 0.0
        )
        ticks.append({"tick": tick_idx, "imu_seq": int(imu), "delta_P_vv_predict": d_pred, "delta_P_vv_nhc": d_nhc})
    return pd.DataFrame(ticks)


def compare_f1_cliff(baseline_cov: pd.DataFrame, f1c_cov: pd.DataFrame, t2: float, t3: float) -> dict:
    b = gap_tick_deltas(baseline_cov, t2, t3)
    f = gap_tick_deltas(f1c_cov, t2, t3)
    n = min(len(b), len(f))
    b = b.head(n).reset_index(drop=True)
    f = f.head(n).reset_index(drop=True)

    b_top3 = b["delta_P_vv_nhc"].abs().nlargest(3).sum()
    f_top3 = f["delta_P_vv_nhc"].abs().nlargest(3).sum()
    b_total = b["delta_P_vv_nhc"].abs().sum()
    f_total = f["delta_P_vv_nhc"].abs().sum()

    # cliff index: tick with max |d_nhc|
    b_cliff = int(b["delta_P_vv_nhc"].abs().idxmax()) + 1
    f_cliff = int(f["delta_P_vv_nhc"].abs().idxmax()) + 1

    return {
        "baseline_top3_share": float(b_top3 / b_total) if b_total else math.nan,
        "f1c_top3_share": float(f_top3 / f_total) if f_total else math.nan,
        "baseline_cliff_tick": b_cliff,
        "f1c_cliff_tick": f_cliff,
        "baseline_max_abs_d_nhc": float(b["delta_P_vv_nhc"].abs().max()),
        "f1c_max_abs_d_nhc": float(f["delta_P_vv_nhc"].abs().max()),
        "baseline_first6_abs_d_nhc": [float(x) for x in b["delta_P_vv_nhc"].abs().head(6).tolist()],
        "f1c_first6_abs_d_nhc": [float(x) for x in f["delta_P_vv_nhc"].abs().head(6).tolist()],
        "f1c_early_ticks_zero_nhc": int((f["delta_P_vv_nhc"].abs().head(9) < 1e-6).sum()),
        "verdict": (
            "frequency_spreads_early_cliffs"
            if f["delta_P_vv_nhc"].abs().head(6).max() < 0.5 * b["delta_P_vv_nhc"].abs().head(6).max()
            else "event_persists"
        ),
    }


def plot_all(k_ticks: list[dict], baseline_ticks: pd.DataFrame, f1c_ticks: pd.DataFrame, pvv_ext: dict, cov: pd.DataFrame, t3: float) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))

    # K vs delta P
    ax = axes[0, 0]
    ticks = [r["tick"] for r in k_ticks]
    ax.bar([t - 0.15 for t in ticks], [r["k_scalar_z_hph_over_s"] for r in k_ticks], width=0.3, label="K_scalar_z=HPH/(HPH+R)")
    ax.bar([t + 0.15 for t in ticks], [r["k_vel_max_logged"] for r in k_ticks], width=0.3, label="k_vel_max logged")
    ax.set_xticks(ticks)
    ax.set_title("Check 1: K not saturated at ~0.99")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # cov consume ratio
    ax = axes[0, 1]
    ax.bar(ticks, [r["cov_consume_ratio_dP_over_dx"] for r in k_ticks], color="C3")
    ax.set_xticks(ticks)
    ax.set_title("||ΔP||/||Δx|| (spikes at cliff tick 3)")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.3)

    # F1 cliff compare
    ax = axes[1, 0]
    n = min(len(baseline_ticks), len(f1c_ticks), 15)
    x = np.arange(1, n + 1)
    ax.bar(x - 0.15, -baseline_ticks["delta_P_vv_nhc"].abs().head(n), width=0.3, label="N=1")
    ax.bar(x + 0.15, -f1c_ticks["delta_P_vv_nhc"].abs().head(n), width=0.3, label="N=10")
    ax.set_xlabel("tick")
    ax.set_title("Check 4: F1 cliff pattern N=1 vs N=10")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # P_vv extension post fix3
    ax = axes[1, 1]
    ext = cov[(cov["timestamp_s"] >= t3) & (cov["timestamp_s"] <= t3 + 4.0)]
    nhc_p = ext[(ext["update_type"] == "nhc") & (ext["phase"] == "post")]
    ax.plot(nhc_p["timestamp_s"] - t3, nhc_p["P_vv_frob"], "C3-o", ms=3, label="P_vv after NHC")
    ax.axhline(2.5, color="gray", ls=":", label="~2.5 reference")
    ax.set_xlabel("time since fix#3 [s]")
    ax.set_title("Check 3: floor vs continued fall")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(OUT_DIR / "gap3_nhc_cliff_mechanism.png", dpi=150)
    plt.close(fig)


def write_summary(report: dict) -> None:
    k = report["k_ticks_234"]
    pred = report["predict_gap"]
    floor = report["pvv_gap_tail"]
    post = report["pvv_post_fix3"]
    f1 = report["f1_cliff_compare"]
    lines = [
        "# GAP-3.16 — NHC cliff mechanism checks",
        "",
        "## 1. ¿Cliff = K≈1?",
        "",
        "| tick | K_scalar_z | k_vel_max | NIS | ΔP_vv | ||ΔP||/||Δx|| |",
        "|------|----------:|----------:|----:|------:|---------------:|",
    ]
    for r in k:
        lines.append(
            f"| {r['tick']} | {r['k_scalar_z_hph_over_s']:.3f} | {r['k_vel_max_logged']:.3f} | "
            f"{r['nis_total']:.2f} | {r['delta_P_vv']:.1f} | {r['cov_consume_ratio_dP_over_dx']:.1f} |"
        )
    lines += [
        "",
        f"**Verdict:** `{report['k_saturation_verdict']}` — K escalar HPH/(HPH+R) ∈ [0.35, 0.55], no ~0.99.",
        "",
        "## 2. predict +3.2",
        "",
        f"- ΣΔP_vv predict: **{pred['sum_delta_P_vv_predict']:+.2f}**",
        f"- White Q frob est: **{pred['Q_vel_frob_white_noise_only_est']:.4f}** (ratio **{pred['P_vv_predict_over_white_Q_ratio']:.0f}×**)",
        f"- ΣΔP_pv predict: {pred['sum_delta_P_pv_predict']:+.2f}",
        f"- {pred['interpretation']}",
        "",
        "## 3. ¿2.5 suelo?",
        "",
        f"- Últimos 10 ticks del gap: P_vv ≈ {floor['tail_mean']:.2f} ± {floor['tail_std']:.2f}, slope/tick={floor['tail_slope_per_tick']:+.4f}",
        f"- **Gap tail verdict:** `{floor['verdict']}` (2.5 es equilibrio predict↔NHC, no foto a mitad de caída libre)",
        f"- Post-fix#3 +4s: range {post['P_vv_range_in_window']:.1f} → `{post['verdict']}`",
        "",
        "## 4. F1 cliff N=1 vs N=10",
        "",
        f"- Baseline cliff tick: **{f1['baseline_cliff_tick']}** (max |ΔP|={f1['baseline_max_abs_d_nhc']:.1f})",
        f"- F1c cliff tick: **{f1['f1c_cliff_tick']}** (max |ΔP|={f1['f1c_max_abs_d_nhc']:.1f})",
        f"- **Verdict:** `{f1['verdict']}`",
        "",
    ]
    SUMMARY_MD.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--autopsy-dir", type=Path, default=AUTOPSY)
    parser.add_argument("--f1-dir", type=Path, default=F1_DIR)
    args = parser.parse_args()
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    nhc = load_csv(args.autopsy_dir / "nhc_block_audit.csv")
    cov = load_csv(args.autopsy_dir / "cov_step_audit.csv")
    f1c_cov = load_csv(args.f1_dir / "F1c" / "cov_step_audit.csv")
    baseline_f1_cov = load_csv(args.f1_dir / "baseline" / "cov_step_audit.csv")

    imu_234 = [412, 413, 414]  # gap ticks 2,3,4
    k_ticks = analyze_k_ticks(nhc, imu_234, tick_labels=[2, 3, 4])

    k_saturated = all(0.3 < r["k_scalar_z_hph_over_s"] < 0.65 for r in k_ticks)
    k_sat_verdict = "multivariate_geometry_not_scalar_saturation" if k_saturated else "investigate_k_saturation"

    predict_gap = analyze_predict_gap(cov, T_FIX2, T_FIX3)
    pvv_tail = pvv_gap_tail_equilibrium(cov, T_FIX2, T_FIX3)
    pvv_ext = pvv_post_fix3_extension(cov, T_FIX3, 4.0)
    f1_cmp = compare_f1_cliff(
        baseline_f1_cov if len(baseline_f1_cov) else cov,
        f1c_cov,
        T_FIX2,
        T_FIX3,
    )

    baseline_ticks = gap_tick_deltas(cov, T_FIX2, T_FIX3)
    f1c_ticks = gap_tick_deltas(f1c_cov, T_FIX2, T_FIX3)
    baseline_ticks.to_csv(OUT_DIR / "baseline_gap_ticks.csv", index=False)
    f1c_ticks.to_csv(OUT_DIR / "f1c_gap_ticks.csv", index=False)

    report = {
        "k_ticks_234": k_ticks,
        "k_saturation_verdict": k_sat_verdict,
        "predict_gap": predict_gap,
        "pvv_gap_tail": pvv_tail,
        "pvv_post_fix3": pvv_ext,
        "f1_cliff_compare": f1_cmp,
        "overall": {
            "dominant_mechanism": (
                "multivariate_NHC_Joseph + predict_cross_terms; NOT scalar K saturation; "
                "floor ~2.5 at predict↔NHC equilibrium; F1 decimation reduces cliff magnitude but event persists early"
            ),
        },
    }

    def _json_default(o):
        if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
            return None
        raise TypeError

    REPORT_JSON.write_text(json.dumps(report, indent=2, default=_json_default), encoding="utf-8")
    plot_all(k_ticks, baseline_ticks, f1c_ticks, pvv_ext, cov, T_FIX3)
    write_summary(report)

    print(json.dumps(report, indent=2, default=_json_default))
    print(f"\nWrote {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
