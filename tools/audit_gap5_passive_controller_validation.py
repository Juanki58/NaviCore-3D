#!/usr/bin/env python3
"""GAP-5 passive validation — controller implementation vs F1 offline Gamma."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
F1_REPORT = REPO_ROOT / "docs/benchmarks/gap3_f1_nhc_dose_response/gap3_f1_nhc_dose_response_report.json"


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, index_col=False)
    skip = {"update_type", "phase", "event", "constraint_policy", "source", "reason", "controller_mode"}
    for col in df.columns:
        if col in skip:
            continue
        converted = pd.to_numeric(df[col], errors="coerce")
        if converted.notna().any():
            df[col] = converted
    return df


def fix_timestamps(gnss: pd.DataFrame) -> dict | None:
    """Match run_gap3_f1_nhc_dose_response.fix_timestamps (gps_index 2/3)."""
    acc = gnss[gnss["accepted"] == 1].sort_values("gps_index")
    if not ((acc["gps_index"] == 2).any() and (acc["gps_index"] == 3).any()):
        return None
    t2 = float(acc[acc["gps_index"] == 2].iloc[0]["timestamp_s"])
    t3 = float(acc[acc["gps_index"] == 3].iloc[0]["timestamp_s"])
    return {"t_fix2": t2, "t_fix3": t3}


def f1_offline_gamma_gap(cov: pd.DataFrame, gnss: pd.DataFrame) -> dict:
    """Same definition as run_gap3_f1_nhc_dose_response.analyze_gap."""
    fixes = fix_timestamps(gnss)
    if fixes is None:
        return {"error": "insufficient accepts (need gps_index 2 and 3 accepted)"}
    t_fix2 = fixes["t_fix2"]
    t_fix3 = fixes["t_fix3"]
    gap = cov[(cov["timestamp_s"] > t_fix2) & (cov["timestamp_s"] < t_fix3)]
    imu_seqs = sorted(gap["imu_seq"].dropna().unique())
    sum_pred = 0.0
    sum_nhc_abs = 0.0
    for imu_seq in imu_seqs:
        tick = gap[gap["imu_seq"] == imu_seq]
        pred_pre = tick[(tick["update_type"] == "predict") & (tick["phase"] == "pre")]
        pred_post = tick[(tick["update_type"] == "predict") & (tick["phase"] == "post")]
        nhc_post = tick[(tick["update_type"] == "nhc") & (tick["phase"] == "post")]
        if pred_pre.empty:
            continue
        pvv_b = float(pred_pre.iloc[0]["P_vv_frob"])
        if not pred_post.empty:
            sum_pred += float(pred_post.iloc[0]["P_vv_frob"]) - pvv_b
        if not nhc_post.empty:
            pvv_ap = float(pred_post.iloc[0]["P_vv_frob"]) if not pred_post.empty else pvv_b
            sum_nhc_abs += abs(float(nhc_post.iloc[0]["P_vv_frob"]) - pvv_ap)
    gamma = sum_nhc_abs / sum_pred if sum_pred > 1e-9 else math.nan
    return {
        "t_fix2": t_fix2,
        "t_fix3": t_fix3,
        "gap_duration_s": t_fix3 - t_fix2,
        "gap_imu_ticks": len(imu_seqs),
        "sum_delta_P_vv_predict": sum_pred,
        "sum_abs_delta_P_vv_nhc": sum_nhc_abs,
        "gamma_offline_gap": gamma,
    }


def controller_hypothetical_table(ctrl: pd.DataFrame) -> list[dict]:
    """Rows where state or reason changes (hypothetical N sequence)."""
    rows = []
    prev_state = None
    prev_reason = None
    for _, r in ctrl.iterrows():
        state = int(r["controller_state"])
        reason = str(r["reason"])
        if state != prev_state or (reason != prev_reason and reason != "hold"):
            rows.append(
                {
                    "timestamp_s": float(r["timestamp_s"]),
                    "gamma_raw": float(r["gamma_raw"]),
                    "gamma_filtered": float(r["gamma_filtered"]),
                    "N_proposed": state,
                    "reason": reason,
                    "transition": int(r["transition"]),
                }
            )
            prev_state = state
            prev_reason = reason
    return rows


def gamma_regime_plateaus(ctrl: pd.DataFrame, min_duration_s: float = 2.0) -> list[dict]:
    """Plateaus where gamma_filtered stays within a band for >= min_duration_s."""
    gf = ctrl["gamma_filtered"].to_numpy()
    t = ctrl["timestamp_s"].to_numpy()
    if len(t) < 2:
        return []
    plateaus: list[dict] = []
    start = 0
    for i in range(1, len(t)):
        drift = abs(gf[i] - gf[start])
        if drift > 1.0 or (t[i] - t[start]) > 30.0:
            dur = t[i - 1] - t[start]
            if dur >= min_duration_s:
                seg = gf[start:i]
                plateaus.append(
                    {
                        "t_start": float(t[start]),
                        "t_end": float(t[i - 1]),
                        "duration_s": float(dur),
                        "gamma_filt_mean": float(np.nanmean(seg)),
                        "gamma_filt_std": float(np.nanstd(seg)),
                    }
                )
            start = i
    dur = t[-1] - t[start]
    if dur >= min_duration_s:
        seg = gf[start:]
        plateaus.append(
            {
                "t_start": float(t[start]),
                "t_end": float(t[-1]),
                "duration_s": float(dur),
                "gamma_filt_mean": float(np.nanmean(seg)),
                "gamma_filt_std": float(np.nanstd(seg)),
            }
        )
    return plateaus[:20]


def threshold_proximity(ctrl: pd.DataFrame) -> dict:
    gf = ctrl["gamma_filtered"].to_numpy()
    gr = ctrl["gamma_raw"].to_numpy()
    thresholds = [8.0, 12.0, 18.0, 22.0]
    near = {}
    for thr in thresholds:
        band = 1.0
        near[f"gamma_filt_within_{thr:g}_pm1s"] = float(((gf >= thr - band) & (gf <= thr + band)).mean())
        near[f"gamma_inst_within_{thr:g}_pm1s"] = float(((gr >= thr - band) & (gr <= thr + band)).mean())
    return near


def gamma_regime_stats(ctrl: pd.DataFrame) -> dict:
    gf = ctrl["gamma_filtered"].to_numpy()
    gr = ctrl["gamma_raw"].to_numpy()
    dt = np.diff(ctrl["timestamp_s"].to_numpy())
    dt = dt[dt > 0]
    return {
        "gamma_inst_max": float(np.nanmax(gr)),
        "gamma_inst_p95": float(np.nanpercentile(gr, 95)),
        "gamma_filtered_max": float(np.nanmax(gf)),
        "gamma_filtered_mean": float(np.nanmean(gf)),
        "gamma_filtered_std": float(np.nanstd(gf)),
        "median_tick_dt_s": float(np.median(dt)) if len(dt) else None,
        "fraction_gamma_filt_above_8": float((gf >= 8).mean()),
        "fraction_gamma_filt_above_12": float((gf >= 12).mean()),
        "fraction_gamma_inst_above_12": float((gr >= 12).mean()),
        "fraction_gamma_inst_above_22": float((gr >= 22).mean()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=REPO_ROOT / "docs/benchmarks/gap5_adaptive_nhc/p0_passive_validation",
    )
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    ctrl_path = args.run_dir / "controller_audit.csv"
    cov_path = args.run_dir / "cov_step_audit.csv"
    gnss_path = args.run_dir / "gnss_nis_audit.csv"
    if not ctrl_path.is_file():
        raise SystemExit(f"missing {ctrl_path} — run passive replay first")

    ctrl = load_csv(ctrl_path)
    cov = load_csv(cov_path) if cov_path.is_file() else pd.DataFrame()
    gnss = load_csv(gnss_path) if gnss_path.is_file() else pd.DataFrame()

    offline = f1_offline_gamma_gap(cov, gnss) if not cov.empty and not gnss.empty else {}
    f1_ref = {}
    if F1_REPORT.is_file():
        f1_ref = json.loads(F1_REPORT.read_text(encoding="utf-8"))
        baseline = next((c for c in f1_ref.get("cases", []) if c["label"] == "baseline"), {})
        f1_ref = {
            "gamma_f1_baseline_gap": baseline.get("gamma_nhc_over_predict"),
            "t_fix2_f1": baseline.get("t_fix2"),
            "t_fix3_f1": baseline.get("t_fix3"),
        }

    # Online gamma at fix#2-3 window (same run)
    online_window = {}
    if offline.get("t_fix2") is not None:
        w = ctrl[
            (ctrl["timestamp_s"] >= offline["t_fix2"] - 0.2)
            & (ctrl["timestamp_s"] <= offline["t_fix3"] + 0.5)
        ]
        if len(w):
            online_window = {
                "gamma_inst_max_in_fix2_3_window": float(w["gamma_raw"].max()),
                "gamma_filtered_max_in_fix2_3_window": float(w["gamma_filtered"].max()),
                "gamma_inst_at_fix2": float(
                    w.loc[(w["timestamp_s"] - offline["t_fix2"]).abs().idxmin(), "gamma_raw"]
                ),
            }

    hypo = controller_hypothetical_table(ctrl)
    plateaus = gamma_regime_plateaus(ctrl)
    thr_near = threshold_proximity(ctrl)
    stats = gamma_regime_stats(ctrl)
    state_frac = {
        f"time_frac_N{n}": float((ctrl["controller_state"] == n).mean())
        for n in (1, 5, 10)
    }
    n_trans = int((ctrl["transition"] == 1).sum())

    offline_this = offline.get("gamma_offline_gap")
    f1_hist = f1_ref.get("gamma_f1_baseline_gap")
    inst_peak_burst = online_window.get("gamma_inst_max_in_fix2_3_window")
    scale_bridge = {
        "interpretation": (
            "F1 offline Gamma = gap-integrated sum|ΔP_vv|_NHC / sum ΔP_predict over fix#2→#3 "
            "(strict open interval, gps_index 2/3). Online gamma_inst uses rolling 1 s decay "
            "with predict/NHC split approximation; gamma_filtered is EWMA τ=1 s. "
            "Init bootstrap can spike gamma before any burst. "
            "F1 historical used pos-only GNSS; GAP-5 PoC uses pos_vel + p_pv none — expect "
            "different offline gap Gamma even on same trajectory."
        ),
        "gamma_offline_this_run": offline_this,
        "gamma_f1_historical_baseline": f1_hist,
        "ratio_offline_this_to_f1_historical": (
            offline_this / f1_hist if offline_this and f1_hist and f1_hist > 0 else None
        ),
        "ratio_inst_peak_burst_to_offline_this": (
            inst_peak_burst / offline_this
            if inst_peak_burst and offline_this and offline_this > 0
            else None
        ),
        "ratio_inst_peak_burst_to_f1_historical": (
            inst_peak_burst / f1_hist if inst_peak_burst and f1_hist and f1_hist > 0 else None
        ),
    }

    report = {
        "phase": "GAP-5 passive validation (hypothesis 1 only — controller implementation)",
        "run_dir": str(args.run_dir),
        "gamma_regime_stats": stats,
        "controller_activity": {
            **state_frac,
            "n_transitions": n_trans,
            "duration_s": float(ctrl["timestamp_s"].iloc[-1] - ctrl["timestamp_s"].iloc[0]),
        },
        "offline_gamma_fix2_fix3": offline,
        "f1_reference": f1_ref,
        "online_gamma_fix2_fix3_window": online_window,
        "scale_bridge": scale_bridge,
        "threshold_separation": {
            "threshold_up_1_to_5": 12.0,
            "threshold_up_5_to_10": 22.0,
            "gamma_filtered_ever_reaches_12": bool(stats["gamma_filtered_max"] >= 12.0),
            "gamma_inst_ever_reaches_12": bool(stats["gamma_inst_max"] >= 12.0),
            "note": "If gamma_filtered never reaches 12, v1 controller instance is inactive — not hypothesis failure.",
        },
        "gamma_regime_plateaus_ge_2s": plateaus,
        "threshold_proximity": thr_near,
        "hypothetical_state_sequence": hypo[:50],
        "verdict_implementation": None,
    }

    # Implementation verdict heuristics (not PoC success)
    burst_inst = inst_peak_burst if inst_peak_burst is not None else stats["gamma_inst_max"]
    ok_burst_signal = burst_inst >= 5.0  # mechanistic burst visible
    ok_scale_vs_f1 = (
        f1_hist is not None
        and offline_this is not None
        and not math.isnan(offline_this)
        and f1_hist > 0
        and abs(offline_this - f1_hist) / f1_hist < 0.25
    )
    ok_scale_vs_offline = (
        offline_this is not None
        and offline_this > 0
        and burst_inst >= 0.3 * offline_this
    )
    ok_stable = stats["gamma_filtered_std"] < 5.0
    controller_would_act = stats["gamma_filtered_max"] >= 12.0
    report["verdict_implementation"] = {
        "burst_gamma_inst_visible": ok_burst_signal,
        "offline_gamma_matches_f1_historical": ok_scale_vs_f1,
        "inst_peak_consistent_with_offline_gap": ok_scale_vs_offline,
        "gamma_filtered_not_excessively_noisy": ok_stable,
        "gamma_inst_crosses_threshold_12": stats["gamma_inst_max"] >= 12.0,
        "gamma_filtered_crosses_threshold_12": controller_would_act,
        "controller_v1_would_transition_in_active": controller_would_act,
        "ready_for_active_poc": ok_burst_signal and ok_scale_vs_offline and controller_would_act,
        "caveat": (
            "If gamma_filtered never crosses 12, v1 controller is inactive — recalibrate "
            "thresholds (v2), not H5-PoC. Compare F1-bridge profile before blaming online estimator."
        ),
    }

    out_json = args.out_json or (args.run_dir / "passive_validation_report.json")
    out_json.write_text(json.dumps(report, indent=2), encoding="utf-8")

    if args.plot:
        fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        t = ctrl["timestamp_s"]
        axes[0].plot(t, ctrl["gamma_raw"], lw=0.6, alpha=0.7, label="gamma_inst")
        axes[0].plot(t, ctrl["gamma_filtered"], lw=1.2, color="#d62728", label="gamma_filtered")
        axes[0].axhline(12, color="#888", ls="--", label="thr 12")
        axes[0].axhline(22, color="#aaa", ls="--", label="thr 22")
        if offline.get("t_fix2"):
            axes[0].axvspan(offline["t_fix2"], offline["t_fix3"], alpha=0.15, color="green", label="fix2-3 gap")
        axes[0].set_ylabel("Gamma")
        axes[0].legend(loc="upper right", fontsize=8)
        axes[0].set_title("GAP-5 passive — online Gamma vs F1 gap window")
        axes[1].step(t, ctrl["controller_state"], where="post", color="#1f77b4")
        axes[1].set_ylabel("N proposed")
        axes[1].set_xlabel("time (s)")
        fig.tight_layout()
        plot_path = args.run_dir / "passive_gamma_regime.png"
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        print(f"Wrote {plot_path}")

    print(f"Wrote {out_json}")
    print(f"gamma_inst max={stats['gamma_inst_max']:.2f}  gamma_filt max={stats['gamma_filtered_max']:.2f}")
    print(f"offline gap gamma={offline.get('gamma_offline_gap', 'n/a')}  F1 ref={f1_ref.get('gamma_f1_baseline_gap', 'n/a')}")
    print(f"transitions={n_trans}  states %={state_frac}")
    print(f"ready_for_active_poc={report['verdict_implementation']['ready_for_active_poc']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
