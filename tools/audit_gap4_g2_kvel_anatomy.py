#!/usr/bin/env python3
"""GAP-4 G2 — Anatomía k_vel: K=P/(P+R), histograma, test discontinuidad vs GPS."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
G2_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap4_gnss_velocity" / "G2"
G0_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap4_gnss_velocity" / "G0"
REPORT_JSON = G2_DIR / "gap4_g2_kvel_anatomy_report.json"
R_VEL_M2 = 2.25  # (1.5 m/s)^2 preregistrado


def load_gnss(d: Path) -> pd.DataFrame:
    return pd.read_csv(d / "gnss_nis_audit.csv")


def load_cov(d: Path) -> pd.DataFrame:
    df = pd.read_csv(d / "cov_step_audit.csv")
    skip = {"update_type", "phase"}
    for col in df.columns:
        if col in skip:
            continue
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def pre_gnss_row(cov: pd.DataFrame, ts: float, tol: float = 1e-3) -> pd.Series | None:
    m = (
        (cov["update_type"] == "gnss")
        & (cov["phase"] == "pre")
        & (np.isclose(cov["timestamp_s"], ts, atol=tol))
    )
    if not m.any():
        return None
    return cov.loc[m].iloc[0]


def k_diagonal_from_p(p_nn: float, p_ee: float, r: float = R_VEL_M2) -> tuple[float, float, float]:
    kn = p_nn / (p_nn + r) if p_nn > 0 else 0.0
    ke = p_ee / (p_ee + r) if p_ee > 0 else 0.0
    return kn, ke, max(kn, ke)


def analyze_arm(gnss: pd.DataFrame, cov: pd.DataFrame, arm: str) -> dict:
    acc = gnss[gnss["accepted"] == 1].copy()
    out: dict = {"arm": arm, "n_accepts": int(len(acc))}
    if acc.empty:
        return out

    kv = acc["k_vel_max"].astype(float)
    out["k_vel_stats"] = {
        "min": float(kv.min()),
        "p25": float(kv.quantile(0.25)),
        "median": float(kv.median()),
        "p75": float(kv.quantile(0.75)),
        "p90": float(kv.quantile(0.90)),
        "p95": float(kv.quantile(0.95)),
        "max": float(kv.max()),
        "mean": float(kv.mean()),
    }
    bins = [0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 0.9, 1.01]
    hist = pd.cut(kv, bins=bins, include_lowest=True).value_counts().sort_index()
    out["k_vel_histogram"] = {str(k): int(v) for k, v in hist.items()}

    # truth-terrain on all accepts
    acc["vg_n"] = acc["gps_speed_mps"] * np.cos(np.radians(acc["gps_course_deg"]))
    acc["vg_e"] = acc["gps_speed_mps"] * np.sin(np.radians(acc["gps_course_deg"]))
    acc["err_vel_pre"] = np.hypot(acc["vel_pred_n_mps"] - acc["vg_n"], acc["vel_pred_e_mps"] - acc["vg_e"])
    acc["err_vel_post"] = np.hypot(acc["vel_after_n_mps"] - acc["vg_n"], acc["vel_after_e_mps"] - acc["vg_e"])
    acc["d_err_vel"] = acc["err_vel_post"] - acc["err_vel_pre"]
    acc["err_pos_pre"] = acc["innov_h_m"]
    acc["pos_after_n"] = acc["hx_n_m"] + acc["dx_pos_n_m"]
    acc["pos_after_e"] = acc["hx_e_m"] + acc["dx_pos_e_m"]
    acc["err_pos_post"] = np.hypot(acc["z_n_m"] - acc["pos_after_n"], acc["z_e_m"] - acc["pos_after_e"])

    out["truth_terrain_all_accepts"] = {
        "vel_error_improves": int((acc["d_err_vel"] < -0.01).sum()),
        "vel_error_worsens": int((acc["d_err_vel"] > 0.01).sum()),
        "vel_error_median_delta_mps": float(acc["d_err_vel"].median()),
        "pos_error_improves": int((acc["err_pos_post"] < acc["err_pos_pre"] - 0.01).sum()),
        "pos_error_worsens": int((acc["err_pos_post"] > acc["err_pos_pre"] + 0.01).sum()),
    }

    # top k_vel events with K=P/(P+R)
    top = acc.sort_values("k_vel_max", ascending=False).head(5)
    events = []
    for _, row in top.iterrows():
        ts = float(row["timestamp_s"])
        pre = pre_gnss_row(cov, ts)
        ev = {
            "gps_index": int(row["gps_index"]),
            "timestamp_s": ts,
            "k_vel_max": float(row["k_vel_max"]),
            "dt_since_prev_accept_s": float(row["dt_since_prev_accept_s"])
            if pd.notna(row["dt_since_prev_accept_s"])
            else None,
        }
        if pre is not None:
            pnn = float(pre["P_vv_n_m2"])
            pee = float(pre["P_vv_e_m2"])
            kn, ke, kmax = k_diagonal_from_p(pnn, pee)
            ev.update(
                {
                    "P_vv_frob_pre": float(pre["P_vv_frob"]),
                    "P_vv_n_m2_pre": pnn,
                    "P_vv_e_m2_pre": pee,
                    "vel_h_pre_mps": float(pre["vel_h_mps"]),
                    "K_nn_diagonal": kn,
                    "K_ee_diagonal": ke,
                    "K_max_diagonal": kmax,
                    "K_matches_k_vel": abs(kmax - float(row["k_vel_max"])) < 0.02,
                }
            )
        ev["vel_vs_gps"] = {
            "err_pre_mps": float(row["err_vel_pre"]),
            "err_post_mps": float(row["err_vel_post"]),
            "delta_err_mps": float(row["d_err_vel"]),
            "improves": bool(row["d_err_vel"] < -0.001),
        }
        ev["pos_vs_gps"] = {
            "err_pre_m": float(row["err_pos_pre"]),
            "err_post_m": float(row["err_pos_post"]),
            "delta_err_m": float(row["err_pos_post"] - row["err_pos_pre"]),
        }
        events.append(ev)
    out["top_k_vel_events"] = events

    # worst event detail
    worst = acc.loc[acc["k_vel_max"].idxmax()]
    out["worst_event_verdict"] = classify_event(worst, pre_gnss_row(cov, float(worst["timestamp_s"])))
    return out


def classify_event(row: pd.Series, pre: pd.Series | None) -> dict:
    """Bug-like vs legitimate high gain."""
    k = float(row["k_vel_max"])
    d_vel = float(row["err_vel_post"] - row["err_vel_pre"]) if "err_vel_post" in row else math.nan

    k_from_p = None
    if pre is not None:
        pnn = float(pre["P_vv_n_m2"])
        pee = float(pre["P_vv_e_m2"])
        _, _, kmax = k_diagonal_from_p(pnn, pee)
        k_from_p = kmax

    bayesian_match = k_from_p is not None and abs(k - k_from_p) < 0.02
    improves_truth = d_vel < -0.01 if not math.isnan(d_vel) else False
    worsens_truth = d_vel > 0.01 if not math.isnan(d_vel) else False

    if bayesian_match and improves_truth:
        label = "LEGITIMATE_HIGH_GAIN"
        note = "K≈P/(P+R); error vs GPS mejora — no firma bug ZUPT"
    elif bayesian_match and not worsens_truth:
        label = "LEGITIMATE_HIGH_GAIN_MARGINAL"
        note = "K≈P/(P+R); error vs GPS no empeora claramente"
    elif worsens_truth:
        label = "BUG_LIKE_DISCONTINUITY"
        note = "Corrección aleja estado de GPS — firma bug"
    else:
        label = "AMBIGUOUS"
        note = "Revisar manualmente"

    return {
        "gps_index": int(row["gps_index"]),
        "timestamp_s": float(row["timestamp_s"]),
        "k_vel_max": k,
        "k_diagonal_pred": k_from_p,
        "bayesian_K_match": bayesian_match,
        "vel_err_pre_mps": float(row.get("err_vel_pre", math.nan)),
        "vel_err_post_mps": float(row.get("err_vel_post", math.nan)),
        "delta_vel_err_mps": d_vel,
        "label": label,
        "note": note,
    }


def main() -> int:
    g2_gnss = load_gnss(G2_DIR)
    g2_cov = load_cov(G2_DIR)
    report = {"G2": analyze_arm(g2_gnss, g2_cov, "G2")}

    if (G0_DIR / "gnss_nis_audit.csv").is_file():
        report["G0_fix2_reference"] = {}
        g0_gnss = load_gnss(G0_DIR)
        g0_cov = load_cov(G0_DIR)
        r2 = g0_gnss[(g0_gnss["gps_index"] == 2) & (g0_gnss["accepted"] == 1)]
        if len(r2):
            row = r2.iloc[0]
            pre = pre_gnss_row(g0_cov, float(row["timestamp_s"]))
            report["G0_fix2_reference"] = {
                "timestamp_s": float(row["timestamp_s"]),
                "k_vel_max": float(row["k_vel_max"]),
                "P_vv_frob_pre": float(pre["P_vv_frob"]) if pre is not None else math.nan,
                "P_vv_n_m2_pre": float(pre["P_vv_n_m2"]) if pre is not None else math.nan,
                "note": "pos-only Joseph; k_vel cross-gain from position update",
            }

    # Confirmación explícita fix #2 y #3 (>0.5 k_vel en G2)
    g2_gnss = load_gnss(G2_DIR)
    g2_cov = load_cov(G2_DIR)
    report["G2_high_gain_confirmation"] = {}
    for fix_idx in (2, 3):
        rows = g2_gnss[(g2_gnss["gps_index"] == fix_idx) & (g2_gnss["accepted"] == 1)]
        if rows.empty:
            continue
        row = rows.iloc[0]
        pre = pre_gnss_row(g2_cov, float(row["timestamp_s"]))
        pnn = float(pre["P_vv_n_m2"]) if pre is not None else math.nan
        pee = float(pre["P_vv_e_m2"]) if pre is not None else math.nan
        kn, ke, kmax = k_diagonal_from_p(pnn, pee) if pre is not None else (math.nan, math.nan, math.nan)
        vg_n = float(row["gps_speed_mps"]) * math.cos(math.radians(float(row["gps_course_deg"])))
        vg_e = float(row["gps_speed_mps"]) * math.sin(math.radians(float(row["gps_course_deg"])))
        err_pre = math.hypot(float(row["vel_pred_n_mps"]) - vg_n, float(row["vel_pred_e_mps"]) - vg_e)
        err_post = math.hypot(float(row["vel_after_n_mps"]) - vg_n, float(row["vel_after_e_mps"]) - vg_e)
        report["G2_high_gain_confirmation"][f"fix_{fix_idx}"] = {
            "k_vel_max": float(row["k_vel_max"]),
            "P_vv_n_m2_pre": pnn,
            "P_vv_frob_pre": float(pre["P_vv_frob"]) if pre is not None else math.nan,
            "K_nn_diagonal": kn,
            "K_matches_k_vel": abs(kmax - float(row["k_vel_max"])) < 0.02 if pre is not None else False,
            "err_vel_pre_mps": err_pre,
            "err_vel_post_mps": err_post,
            "delta_err_vel_mps": err_post - err_pre,
            "verdict": "LEGITIMATE_HIGH_GAIN"
            if (pre is not None and abs(kmax - float(row["k_vel_max"])) < 0.02 and err_post < err_pre)
            else "REVIEW",
        }

    # interpret histogram pattern
    hist = report["G2"]["k_vel_histogram"]
    n_high = sum(v for k, v in hist.items() if "0.9" in k or "0.7" in k)
    n_low = sum(v for k, v in hist.items() if "0.1" in k or "0.2" in k or "0.3" in k)
    report["G2"]["interpretation"] = {
        "high_gain_events_ge_0.7": n_high,
        "moderate_gain_events_le_0.3": n_low,
        "pattern": "ISOLATED_SPIKES"
        if n_high <= 3 and report["G2"]["n_accepts"] > 10
        else "SYSTEMATIC_HIGH_GAIN",
        "abort_guardrail_0.5": "May over-flag legitimate P/(P+R) when P_vv large after long gap",
    }

    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"\nWrote {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
