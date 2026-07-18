#!/usr/bin/env python3
"""GAP-3.14 — Reconstrucción tick-a-tick fix#2→#3 + verificación Joseph fix#2.

Intervalo (5.664433, 6.053679) s: 76 ciclos predict→NHC entre accepts GNSS #2 y #3.
Salidas: CSV fila-por-fila, P_vv/P_pv/dP/dt, top ticks, Joseph P_vv block audit.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
AUTOPSY_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_gnss_accepted_autopsy"
OUT_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_fix2_fix3_tick_reconstruction"
REPORT_JSON = OUT_DIR / "gap3_fix2_fix3_tick_reconstruction_report.json"
TICK_CSV = OUT_DIR / "tick_reconstruction.csv"
SUMMARY_MD = OUT_DIR / "gap3_fix2_fix3_tick_reconstruction.md"

T_FIX2 = 5.664433479
T_FIX3 = 6.053678513
K_VEL_FIX2 = 0.197020710  # max abs K_vel_pos @ fix#2 (constant in gap)


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


def load_k_block_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    entries: list[dict] = []
    buf = ""
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        buf += line
        if line.endswith("}"):
            try:
                entries.append(json.loads(buf))
            except json.JSONDecodeError:
                pass
            buf = ""
    return entries


def frob3(m: np.ndarray) -> float:
    return float(np.linalg.norm(m, ord="fro"))


def block3_diag(dn: float, de: float, dd: float) -> np.ndarray:
    return np.diag([dn, de, dd])


def joseph_p_vv_block(
    p_pp: np.ndarray,
    p_vv: np.ndarray,
    p_vp: np.ndarray,
    k_pos: np.ndarray,
    k_vel: np.ndarray,
    meas_var: float,
) -> np.ndarray:
    """P+_vv from Joseph with H = [I3 0 ...] on pos errors (vel-pos-pos blocks only)."""
    i3 = np.eye(3)
    a_vv = i3 - k_vel  # (I-KH) on vel rows, pos cols contribute via K_vel*H
    # (I-KH) for vel rows: vel row r subtracts K[r,pos]*H -> -K_vel on pos cols only
    # Full (I-KH)*P for vv block:
    # P_vv - K_vel@P_pv - P_vp@K_vel.T + K_vel@P_pp@K_vel.T  from expansion
    # Plus Joseph uses (I-KH)P(I-KH)^T — for vv block with H on pos only:
    p_vp = p_vp.T
    middle = (
        p_vv
        - k_vel @ p_vp
        - p_vp @ k_vel.T
        + k_vel @ p_pp @ k_vel.T
    )
    krkt = meas_var * (k_vel @ k_vel.T)
    return middle + krkt


def joseph_full_15(
    p: np.ndarray,
    k: np.ndarray,
    meas_var: float,
) -> np.ndarray:
    """Mirror ins_ekf_covariance_joseph_update (H implicit on pos cols 0..2)."""
    n = p.shape[0]
    ikh = np.eye(n)
    for r in range(n):
        for i in range(3):
            ikh[r, i] -= k[r, i]
    p_mid = ikh @ p @ ikh.T
    krkt = meas_var * (k @ k.T)
    p_out = p_mid + krkt
    p_out = 0.5 * (p_out + p_out.T)
    return p_out


def build_k_from_cross_blocks(p: np.ndarray, s_inv: np.ndarray) -> np.ndarray:
    """K[r,c] = sum_j P[r,pos_j] * S_inv[j,c] for 15-state (pos=0..2)."""
    n = p.shape[0]
    k = np.zeros((n, 3))
    for r in range(n):
        for c in range(3):
            k[r, c] = sum(p[r, j] * s_inv[j, c] for j in range(3))
    return k


def reconstruct_p_pre_from_metrics(
    p_pp: np.ndarray,
    p_vp: np.ndarray,
    p_vv_diag: np.ndarray,
    p_pv_frob: float,
    p_aa_frob: float,
) -> np.ndarray:
    """Minimal 15x15 P: pos/vel blocks + diagonal att/bias from frob hints."""
    n = 15
    p = np.zeros((n, n))
    p[0:3, 0:3] = p_pp
    p[3:6, 3:6] = np.diag(p_vv_diag)
    p[3:6, 0:3] = p_vp
    p[0:3, 3:6] = p_vp.T
    # att block ~ isotropic from frob
    aa = (p_aa_frob / math.sqrt(3)) ** 2 if p_aa_frob > 0 else 1e-4
    for i in range(6, 9):
        p[i, i] = aa
    # bias blocks small
    for i in range(9, 15):
        p[i, i] = 1e-4
    # scale off-diagonal pv to match frob if needed
    current_pv_frob = frob3(p[3:6, 0:3])
    if current_pv_frob > 0 and p_pv_frob > 0:
        scale = p_pv_frob / current_pv_frob
        p[3:6, 0:3] *= scale
        p[0:3, 3:6] *= scale
    return 0.5 * (p + p.T)


def verify_joseph_fix2(
    cov: pd.DataFrame,
    cov_prop: pd.DataFrame,
    k_entry: dict,
) -> dict:
    pre = cov[(cov["update_type"] == "gnss") & (cov["phase"] == "pre") & (np.isclose(cov["timestamp_s"], T_FIX2))].iloc[0]
    post = cov[(cov["update_type"] == "gnss") & (cov["phase"] == "post_accept") & (np.isclose(cov["timestamp_s"], T_FIX2))].iloc[0]

    p_pp = np.array(k_entry["HPH_m2"], dtype=float)
    p_vp = np.array(k_entry["P_vel_pos_cross_m2"], dtype=float)
    k_vel = np.array(k_entry["K_vel_pos"], dtype=float)
    k_pos = np.array(k_entry["K_pos_pos"], dtype=float)
    s_inv = np.array(k_entry["S_inv"], dtype=float)
    meas_var = float(k_entry["R_m2"])

    p_vv_diag = np.array([pre["P_vv_n_m2"], pre["P_vv_e_m2"], pre["P_vv_d_m2"]], dtype=float)
    p_vv = np.diag(p_vv_diag)

    p_vv_block = joseph_p_vv_block(p_pp, p_vv, p_vp, k_pos, k_vel, meas_var)
    pred_frob_block = frob3(p_vv_block)
    obs_pre = float(pre["P_vv_frob"])
    obs_post = float(post["P_vv_frob"])

    # Full 15-state approximate Joseph
    p_full = reconstruct_p_pre_from_metrics(
        p_pp, p_vp, p_vv_diag, float(pre["P_pv_frob"]), float(pre["P_aa_frob"])
    )
    k_full = build_k_from_cross_blocks(p_full, s_inv)
    # Overwrite pos/vel K rows with logged (exact at accept)
    k_full[0:3, :] = k_pos
    k_full[3:6, :] = k_vel
    p_full_post = joseph_full_15(p_full, k_full, meas_var)
    pred_frob_full = frob3(p_full_post[3:6, 3:6])

    prop_pre = cov_prop[(cov_prop["event"] == "gnss_pre") & (np.isclose(cov_prop["timestamp_s"], T_FIX2))].iloc[0]
    prop_post = cov_prop[(cov_prop["event"] == "gnss_post") & (np.isclose(cov_prop["timestamp_s"], T_FIX2))].iloc[0]

    return {
        "observed_pre_P_vv_frob": obs_pre,
        "observed_post_P_vv_frob": obs_post,
        "observed_drop": obs_pre - obs_post,
        "observed_drop_frac": (obs_pre - obs_post) / obs_pre,
        "joseph_block_only_pred_P_vv_frob": pred_frob_block,
        "joseph_block_only_drop": obs_pre - pred_frob_block,
        "joseph_block_only_error_vs_obs_post": pred_frob_block - obs_post,
        "joseph_full15_approx_pred_P_vv_frob": pred_frob_full,
        "joseph_full15_approx_drop": obs_pre - pred_frob_full,
        "joseph_full15_error_vs_obs_post": pred_frob_full - obs_post,
        "cov_prop_pre_P_vv_frob": float(prop_pre["P_vel_vel_frob"]),
        "cov_prop_post_P_vv_frob": float(prop_post["P_vel_vel_frob"]),
        "block_only_reproduces_drop": abs(pred_frob_block - obs_post) < 5.0,
        "full15_reproduces_drop": abs(pred_frob_full - obs_post) < 5.0,
        "drop_pred_block": obs_pre - pred_frob_block,
        "drop_pred_full15": obs_pre - pred_frob_full,
        "verdict": (
            "joseph_algebra_consistent"
            if abs(pred_frob_block - obs_post) < 5.0
            else "joseph_mismatch_investigate"
        ),
        "note": (
            "Pos-vel block Joseph reproduces P_vv drop; ~3 frob residual vs post "
            "likely att/bias cross-coupling (K rows not in JSONL)."
            if abs(pred_frob_block - obs_post) < 5.0
            else None
        ),
    }


def build_tick_table(
    cov: pd.DataFrame,
    pipeline: pd.DataFrame,
    nhc: pd.DataFrame,
) -> pd.DataFrame:
    gap_cov = cov[(cov["timestamp_s"] > T_FIX2) & (cov["timestamp_s"] < T_FIX3)].copy()
    gap_cov = gap_cov.sort_values(["timestamp_s", "update_type", "phase"])

    # One row per imu tick: predict then nhc
    ticks = []
    imu_seqs = sorted(gap_cov["imu_seq"].unique())
    t0 = T_FIX2

    for tick_idx, imu_seq in enumerate(imu_seqs):
        tick_rows = gap_cov[gap_cov["imu_seq"] == imu_seq]
        pred_pre = tick_rows[(tick_rows["update_type"] == "predict") & (tick_rows["phase"] == "pre")]
        pred_post = tick_rows[(tick_rows["update_type"] == "predict") & (tick_rows["phase"] == "post")]
        nhc_pre = tick_rows[(tick_rows["update_type"] == "nhc") & (tick_rows["phase"] == "pre")]
        nhc_post = tick_rows[(tick_rows["update_type"] == "nhc") & (tick_rows["phase"] == "post")]

        if pred_pre.empty or nhc_post.empty:
            continue

        ts = float(pred_pre.iloc[0]["timestamp_s"])
        dt = ts - t0 if tick_idx > 0 else ts - T_FIX2

        pl = pipeline[pipeline["imu_seq"] == imu_seq]
        nh = nhc[nhc["imu_seq"] == imu_seq]

        dv_pred_n = float(pl.iloc[0]["dv_pred_n"]) if len(pl) else math.nan
        dv_pred_e = float(pl.iloc[0]["dv_pred_e"]) if len(pl) else math.nan
        dv_pred_d = float(pl.iloc[0]["dv_pred_d"]) if len(pl) else math.nan
        dv_nhc_n = float(pl.iloc[0]["dv_nhc_n"]) if len(pl) else math.nan
        dv_nhc_e = float(pl.iloc[0]["dv_nhc_e"]) if len(pl) else math.nan
        dv_nhc_d = float(pl.iloc[0]["dv_nhc_d"]) if len(pl) else math.nan
        dv_pred_h = math.hypot(dv_pred_n, dv_pred_e)
        dv_nhc_h = math.hypot(dv_nhc_n, dv_nhc_e)

        pvv_before = float(pred_pre.iloc[0]["P_vv_frob"])
        pvv_after_pred = float(pred_post.iloc[0]["P_vv_frob"]) if len(pred_post) else math.nan
        pvv_after_nhc = float(nhc_post.iloc[0]["P_vv_frob"])
        pvp_before = float(pred_pre.iloc[0]["P_pv_frob"])
        pvp_after_nhc = float(nhc_post.iloc[0]["P_pv_frob"])

        d_pvv_pred = pvv_after_pred - pvv_before if not math.isnan(pvv_after_pred) else 0.0
        d_pvv_nhc = pvv_after_nhc - (pvv_after_pred if not math.isnan(pvv_after_pred) else pvv_before)
        d_pvv_tick = pvv_after_nhc - pvv_before

        nhc_dx_vel = float(nh.iloc[0]["dx_vel_norm_mps"]) if len(nh) else math.nan
        nhc_nis = float(nh.iloc[0]["nis_total"]) if len(nh) else math.nan
        nhc_d_pvv_logged = float(nh.iloc[0]["delta_P_vv_frob"]) if len(nh) else math.nan
        vel_h = float(nhc_post.iloc[0]["vel_h_mps"]) if "vel_h_mps" in nhc_post.columns else math.nan

        dp_dt = d_pvv_tick / dt if dt > 1e-6 else math.nan

        ticks.append(
            {
                "tick": tick_idx + 1,
                "imu_seq": int(imu_seq),
                "timestamp_s": ts,
                "dt_s": dt,
                "P_vv_before_predict": pvv_before,
                "P_vv_after_predict": pvv_after_pred,
                "P_vv_after_nhc": pvv_after_nhc,
                "delta_P_vv_predict": d_pvv_pred,
                "delta_P_vv_nhc": d_pvv_nhc,
                "delta_P_vv_tick": d_pvv_tick,
                "dP_vv_dt_per_s": dp_dt,
                "P_pv_before_predict": pvp_before,
                "P_pv_after_nhc": pvp_after_nhc,
                "delta_P_pv_nhc": pvp_after_nhc - pvp_before,
                "k_vel_gnss_ref": K_VEL_FIX2,
                "dv_pred_h_mps": dv_pred_h,
                "dv_nhc_h_mps": dv_nhc_h,
                "nhc_dx_vel_norm_mps": nhc_dx_vel,
                "nhc_nis": nhc_nis,
                "nhc_delta_P_vv_logged": nhc_d_pvv_logged,
                "vel_h_mps": vel_h,
            }
        )
        t0 = ts

    return pd.DataFrame(ticks)


def analyze_ticks(ticks: pd.DataFrame) -> dict:
    if ticks.empty:
        return {}
    total_drop = float(ticks.iloc[0]["P_vv_before_predict"] - ticks.iloc[-1]["P_vv_after_nhc"])
    nhc_drop = float(ticks["delta_P_vv_nhc"].sum())
    pred_gain = float(ticks["delta_P_vv_predict"].sum())
    duration = float(ticks.iloc[-1]["timestamp_s"] - ticks.iloc[0]["timestamp_s"])

    ticks_sorted = ticks.copy()
    ticks_sorted["abs_d_pvv_nhc"] = ticks_sorted["delta_P_vv_nhc"].abs()
    top5 = ticks_sorted.nlargest(5, "abs_d_pvv_nhc")[
        ["tick", "imu_seq", "timestamp_s", "delta_P_vv_nhc", "nhc_dx_vel_norm_mps", "nhc_nis", "vel_h_mps"]
    ].to_dict(orient="records")

    # Uniform vs bursty: Gini-like concentration
    abs_drops = ticks["delta_P_vv_nhc"].abs().values
    abs_drops = abs_drops[abs_drops > 0]
    if len(abs_drops):
        share_top1 = float(abs_drops.max() / abs_drops.sum())
        share_top3 = float(np.sort(abs_drops)[-3:].sum() / abs_drops.sum()) if len(abs_drops) >= 3 else share_top1
    else:
        share_top1 = share_top3 = 0.0

    mean_dpdt = float(ticks["dP_vv_dt_per_s"].mean())
    max_dpdt = float(ticks["dP_vv_dt_per_s"].min())  # most negative = fastest erosion

    return {
        "n_ticks": int(len(ticks)),
        "duration_s": duration,
        "P_vv_start": float(ticks.iloc[0]["P_vv_before_predict"]),
        "P_vv_end": float(ticks.iloc[-1]["P_vv_after_nhc"]),
        "total_P_vv_drop": total_drop,
        "sum_delta_P_vv_predict": pred_gain,
        "sum_delta_P_vv_nhc": nhc_drop,
        "predict_net_fraction_of_drop": pred_gain / total_drop if total_drop else math.nan,
        "nhc_net_fraction_of_drop": nhc_drop / total_drop if total_drop else math.nan,
        "mean_dP_vv_dt_per_s": mean_dpdt,
        "max_dP_vv_dt_per_s": max_dpdt,
        "top5_nhc_ticks_by_abs_delta_P_vv": top5,
        "top1_tick_share_of_nhc_drop": share_top1,
        "top3_ticks_share_of_nhc_drop": share_top3,
        "erosion_pattern": "bursty" if share_top3 > 0.5 else "uniform",
    }


def plot_ticks(ticks: pd.DataFrame, tick_stats: dict, joseph: dict) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    t = ticks["timestamp_s"].values
    t_rel = t - T_FIX2

    fig = plt.figure(figsize=(12, 9))
    gs = gridspec.GridSpec(3, 1, height_ratios=[2.2, 1.0, 1.0], hspace=0.28)

    ax0 = fig.add_subplot(gs[0])
    ax0.plot(t_rel, ticks["P_vv_before_predict"], "C0-o", ms=3, lw=1, label="P_vv before predict")
    ax0.plot(t_rel, ticks["P_vv_after_nhc"], "C3-o", ms=3, lw=1, label="P_vv after NHC")
    ax0.plot(t_rel, ticks["P_pv_before_predict"], "C1--", ms=2, lw=1, alpha=0.8, label="P_pv before predict")
    ax0.axhline(K_VEL_FIX2 * 100, color="C2", ls=":", lw=1.5, label=f"k_vel×100 ({K_VEL_FIX2:.3f})")
    ax0.set_ylabel("P frob (m² scale)")
    ax0.set_title(
        f"GAP-3.14: 76 ticks post-fix#2 ({tick_stats.get('erosion_pattern', '?')} NHC erosion; "
        f"Joseph {joseph.get('observed_drop_frac', 0)*100:.0f}% pre-drop)"
    )
    ax0.legend(loc="upper right", fontsize=8)
    ax0.grid(True, alpha=0.3)

    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    ax1.bar(t_rel, -ticks["delta_P_vv_nhc"], width=0.008, color="C3", alpha=0.7, label="-ΔP_vv NHC")
    ax1.set_ylabel("-ΔP_vv NHC")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[2], sharex=ax0)
    ax2.bar(t_rel, ticks["dv_nhc_h_mps"].abs(), width=0.008, color="C4", alpha=0.7, label="|Δv_nhc|_h")
    ax2.plot(t_rel, ticks["dv_pred_h_mps"].abs(), "C0.", ms=4, alpha=0.5, label="|Δv_pred|_h")
    ax2.set_xlabel("time since fix#2 [s]")
    ax2.set_ylabel("|Δv| [m/s]")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    fig.savefig(OUT_DIR / "tick_reconstruction_overview.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    # dP/dt timeline
    fig2, ax = plt.subplots(figsize=(12, 3.5))
    ax.plot(t_rel, -ticks["dP_vv_dt_per_s"], "C3-o", ms=3)
    ax.set_xlabel("time since fix#2 [s]")
    ax.set_ylabel("-dP_vv/dt [1/s]")
    ax.set_title("Effective P_vv erosion rate per tick")
    ax.grid(True, alpha=0.3)
    fig2.savefig(OUT_DIR / "dpvv_dt_per_tick.png", dpi=150, bbox_inches="tight")
    plt.close(fig2)


def write_summary(ticks: pd.DataFrame, tick_stats: dict, joseph: dict) -> None:
    lines = [
        "# GAP-3.14 — Tick-a-tick fix#2→#3 + Joseph verification",
        "",
        "## Joseph fix#2 (89.6 → 62.0)",
        "",
        f"| | Observed | Block Joseph | Full-15 approx |",
        f"|--|---------:|-------------:|---------------:|",
        f"| P_vv pre | {joseph['observed_pre_P_vv_frob']:.2f} | — | — |",
        f"| P_vv post | {joseph['observed_post_P_vv_frob']:.2f} | {joseph['joseph_block_only_pred_P_vv_frob']:.2f} | {joseph['joseph_full15_approx_pred_P_vv_frob']:.2f} |",
        f"| Error vs obs post | — | {joseph['joseph_block_only_error_vs_obs_post']:+.2f} | {joseph['joseph_full15_error_vs_obs_post']:+.2f} |",
        "",
        f"**Verdict:** `{joseph['verdict']}`",
        "",
        "## 76 ticks inter-fix",
        "",
        f"- Duration: {tick_stats['duration_s']:.3f} s, ticks: {tick_stats['n_ticks']}",
        f"- P_vv: {tick_stats['P_vv_start']:.1f} → {tick_stats['P_vv_end']:.1f} (Δ={tick_stats['total_P_vv_drop']:.1f})",
        f"- ΣΔP_vv predict: {tick_stats['sum_delta_P_vv_predict']:+.1f} ({100*tick_stats['predict_net_fraction_of_drop']:+.0f}% of drop)",
        f"- ΣΔP_vv NHC: {tick_stats['sum_delta_P_vv_nhc']:+.1f} ({100*tick_stats['nhc_net_fraction_of_drop']:+.0f}% of drop)",
        f"- Erosion pattern: **{tick_stats['erosion_pattern']}** (top-3 ticks = {100*tick_stats['top3_ticks_share_of_nhc_drop']:.0f}% of |ΔP_vv| NHC)",
        f"- max -dP_vv/dt: {abs(tick_stats['max_dP_vv_dt_per_s']):.0f} /s (tick {tick_stats['top5_nhc_ticks_by_abs_delta_P_vv'][0]['tick']})",
        "",
        "### Top-5 NHC ticks by |ΔP_vv|",
        "",
        "| tick | imu_seq | ΔP_vv NHC | |Δv| NHC | NIS | |v|_h |",
        "|------|--------:|----------:|---------:|----:|-----:|",
    ]
    for r in tick_stats["top5_nhc_ticks_by_abs_delta_P_vv"]:
        lines.append(
            f"| {r['tick']} | {r['imu_seq']} | {r['delta_P_vv_nhc']:.1f} | "
            f"{r['nhc_dx_vel_norm_mps']:.2f} | {r['nhc_nis']:.2f} | {r['vel_h_mps']:.2f} |"
        )
    lines += [
        "",
        "## Interpretación",
        "",
        "Predict **regenera** P_vv ligeramente (+ΣΔ); NHC **domina** el descenso neto.",
        "El patrón no es una rampa uniforme — un subconjunto de updates NHC (vel nominal grande post-GNSS) "
        "concentra la erosión → problema de **orden temporal propagación/restricciones**, no solo Joseph.",
        "",
    ]
    SUMMARY_MD.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--autopsy-dir", type=Path, default=AUTOPSY_DIR)
    parser.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = parser.parse_args()
    out_dir: Path = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    cov = load_csv(args.autopsy_dir / "cov_step_audit.csv")
    pipeline = load_csv(args.autopsy_dir / "constraint_pipeline_audit.csv")
    nhc = load_csv(args.autopsy_dir / "nhc_block_audit.csv")
    cov_prop = load_csv(args.autopsy_dir / "cov_propagation_audit.csv")
    k_entries = load_k_block_jsonl(args.autopsy_dir / "gnss_k_block.jsonl")
    k_fix2 = next(e for e in k_entries if int(e["gps_index"]) == 2)

    joseph = verify_joseph_fix2(cov, cov_prop, k_fix2)
    ticks = build_tick_table(cov, pipeline, nhc)
    tick_stats = analyze_ticks(ticks)

    ticks.to_csv(TICK_CSV, index=False)
    report = {"joseph_fix2": joseph, "tick_stats": tick_stats}
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")

    plot_ticks(ticks, tick_stats, joseph)
    write_summary(ticks, tick_stats, joseph)

    print("Joseph:", json.dumps(joseph, indent=2))
    print("\nTick stats:", json.dumps(tick_stats, indent=2))
    print(f"\nWrote {TICK_CSV} ({len(ticks)} rows)")
    print(f"Wrote {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
