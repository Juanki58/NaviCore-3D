#!/usr/bin/env python3
"""GAP-4 — cos(dv_pos, err_pre) over all accepts (counterfactual 5D pos+vel)."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
R_VEL = 2.25
ARMS = ("G0", "G1", "G2")


def load_all_k_blocks(path: Path) -> dict[int, dict]:
    if not path.is_file():
        return {}
    text = path.read_text(encoding="utf-8")
    objs = []
    depth = 0
    start = None
    for i, ch in enumerate(text):
        if ch == "{":
            if depth == 0:
                start = i
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0 and start is not None:
                objs.append(json.loads(text[start : i + 1]))
                start = None
    return {int(o["gps_index"]): o for o in objs}


def load_cov_pre(cov_path: Path, ts: float) -> pd.Series | None:
    cov = pd.read_csv(cov_path)
    for c in cov.columns:
        if c not in ("update_type", "phase"):
            cov[c] = pd.to_numeric(cov[c], errors="coerce")
    row = cov[
        (cov["update_type"] == "gnss")
        & (cov["phase"] == "pre")
        & (np.isclose(cov["timestamp_s"], ts, atol=1e-6))
    ]
    return row.iloc[0] if len(row) else None


def build_minimal_p(p_pp, p_vp, p_vv_diag, p_pv_frob, p_aa_frob):
    p = np.zeros((15, 15))
    p[0:3, 0:3] = p_pp
    p[3:6, 3:6] = np.diag(p_vv_diag)
    p[3:6, 0:3] = p_vp
    p[0:3, 3:6] = p_vp.T
    aa = (p_aa_frob / math.sqrt(3)) ** 2 if p_aa_frob > 0 else 1e-4
    for i in range(6, 9):
        p[i, i] = aa
    for i in range(9, 15):
        p[i, i] = 1e-4
    cur = np.linalg.norm(p[3:6, 0:3], "fro")
    if cur > 0 and p_pv_frob > 0:
        s = p_pv_frob / cur
        p[3:6, 0:3] *= s
        p[0:3, 3:6] *= s
    return 0.5 * (p + p.T)


def build_h5():
    h = np.zeros((5, 15))
    for i in range(3):
        h[i, i] = 1.0
    h[3, 3] = h[4, 4] = 1.0
    return h


def build_r5(r_pos):
    r = np.zeros((5, 5))
    for i in range(3):
        r[i, i] = r_pos
    r[3, 3] = r[4, 4] = R_VEL
    return r


def analyze_row(gnss_row: pd.Series, k: dict, pre) -> dict | None:
    if not gnss_row.has_gps_speed or gnss_row.gps_speed_mps <= 0:
        return None

    p_pp = np.array(k["HPH_m2"], dtype=float)
    p_vp = np.array(k["P_vel_pos_cross_m2"], dtype=float)
    p_vv_diag = np.array([pre["P_vv_n_m2"], pre["P_vv_e_m2"], pre["P_vv_d_m2"]])
    p = build_minimal_p(p_pp, p_vp, p_vv_diag, float(pre["P_pv_frob"]), float(pre["P_aa_frob"]))
    r_pos = float(k["R_m2"])
    h5 = build_h5()
    s5 = h5 @ p @ h5.T + build_r5(r_pos)
    k5 = p @ h5.T @ np.linalg.inv(s5)

    y_pos = np.array([gnss_row.innov_n_m, gnss_row.innov_e_m, gnss_row.innov_d_m])
    y_vel = np.array([gnss_row.pseudo_innov_v_n_mps, gnss_row.pseudo_innov_v_e_mps])
    dv_pos = k5[3:5, 0:3] @ y_pos
    dv_vel = k5[3:5, 3:5] @ y_vel
    dv_tot = dv_pos + dv_vel

    vg_n = gnss_row.gps_speed_mps * math.cos(math.radians(gnss_row.gps_course_deg))
    vg_e = gnss_row.gps_speed_mps * math.sin(math.radians(gnss_row.gps_course_deg))
    err = np.array([gnss_row.vel_pred_n_mps - vg_n, gnss_row.vel_pred_e_mps - vg_e])
    en = np.linalg.norm(err)
    dn = np.linalg.norm(dv_pos)
    if en < 1e-9 or dn < 1e-12:
        cos_err = float("nan")
        proj = float("nan")
    else:
        cos_err = float(np.dot(dv_pos, err) / (dn * en))
        proj = float(np.dot(dv_pos, -err / en))

    vn_post = gnss_row.vel_pred_n_mps + dv_tot[0]
    ve_post = gnss_row.vel_pred_e_mps + dv_tot[1]
    err_post = math.hypot(vn_post - vg_n, ve_post - vg_e)

    mag_pos = float(np.linalg.norm(dv_pos))
    mag_vel = float(np.linalg.norm(dv_vel))
    cross_frac = mag_pos / max(mag_pos + mag_vel, 1e-9)

    dt_acc = gnss_row.dt_since_prev_accept_s
    dt_gnss = gnss_row.dt_since_prev_gnss_s

    return {
        "gps_index": int(gnss_row.gps_index),
        "timestamp_s": float(gnss_row.timestamp_s),
        "dt_since_prev_accept_s": None if pd.isna(dt_acc) else float(dt_acc),
        "dt_since_prev_gnss_s": None if pd.isna(dt_gnss) else float(dt_gnss),
        "effective_gap_s": None,
        "innov_h_m": float(gnss_row.innov_h_m),
        "y_pos_norm_3d_m": float(np.linalg.norm(y_pos)),
        "P_pv_frob_pre": float(pre["P_pv_frob"]),
        "P_vv_frob_pre": float(pre["P_vv_frob"]),
        "cross_term_fraction": cross_frac,
        "cos_dv_pos_err_pre": cos_err,
        "proj_dv_pos_on_correction_mps": proj,
        "cross_aids_correction": bool(cos_err < 0) if not math.isnan(cos_err) else None,
        "counterfactual_delta_err_vel_mps": err_post - en,
        "counterfactual_vel_improves": bool(err_post < en),
        "actual_delta_err_vel_mps": None,
        "actual_vel_improves": None,
    }


def corr(x, y) -> float:
    x, y = np.array(x, float), np.array(y, float)
    m = np.isfinite(x) & np.isfinite(y)
    x, y = x[m], y[m]
    if len(x) < 3 or np.std(x) < 1e-12 or np.std(y) < 1e-12:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def summarize_pool(rows: list[dict]) -> dict:
    cos_v = [r["cos_dv_pos_err_pre"] for r in rows if np.isfinite(r["cos_dv_pos_err_pre"])]
    aids = [r for r in rows if r.get("cross_aids_correction") is True]
    hurts = [r for r in rows if r.get("cross_aids_correction") is False]
    d_acc = [r["dt_since_prev_accept_s"] for r in rows if r["dt_since_prev_accept_s"] is not None]
    return {
        "n": len(rows),
        "n_finite_cos": len(cos_v),
        "cos_mean": float(np.mean(cos_v)) if cos_v else None,
        "cos_median": float(np.median(cos_v)) if cos_v else None,
        "cos_std": float(np.std(cos_v)) if cos_v else None,
        "frac_cross_aids": len(aids) / len(rows) if rows else None,
        "frac_counterfactual_improves": sum(r["counterfactual_vel_improves"] for r in rows) / len(rows)
        if rows
        else None,
        "corr_cos_vs_dt_since_accept": corr(
            [r["cos_dv_pos_err_pre"] for r in rows],
            [r["dt_since_prev_accept_s"] if r["dt_since_prev_accept_s"] is not None else np.nan for r in rows],
        ),
        "corr_cos_vs_dt_since_gnss": corr(
            [r["cos_dv_pos_err_pre"] for r in rows],
            [r["dt_since_prev_gnss_s"] if r["dt_since_prev_gnss_s"] is not None else np.nan for r in rows],
        ),
        "corr_cos_vs_y_pos": corr([r["cos_dv_pos_err_pre"] for r in rows], [r["y_pos_norm_3d_m"] for r in rows]),
        "corr_cos_vs_P_pv": corr([r["cos_dv_pos_err_pre"] for r in rows], [r["P_pv_frob_pre"] for r in rows]),
        "corr_cos_vs_effective_gap": corr(
            [r["cos_dv_pos_err_pre"] for r in rows],
            [r["effective_gap_s"] if r.get("effective_gap_s") is not None else np.nan for r in rows],
        ),
        "corr_cos_vs_counterfactual_delta_err": corr(
            [r["cos_dv_pos_err_pre"] for r in rows],
            [r["counterfactual_delta_err_vel_mps"] for r in rows],
        ),
        "dt_since_accept_max_s": max(d_acc) if d_acc else None,
        "first_accept_n": sum(1 for r in rows if r["dt_since_prev_accept_s"] is None),
    }


def enrich_gaps(arm_rows: list[dict], gnss: pd.DataFrame) -> None:
    """Compute dt since previous accept / GNSS when CSV columns are NaN."""
    acc = gnss[gnss["accepted"] == 1].sort_values("timestamp_s")
    acc_ts = {int(r.gps_index): float(r.timestamp_s) for _, r in acc.iterrows()}
    prev_acc_ts = None
    for _, r in acc.iterrows():
        idx = int(r.gps_index)
        ts = float(r.timestamp_s)
        for row in arm_rows:
            if row["gps_index"] == idx and abs(row["timestamp_s"] - ts) < 1e-6:
                if row["dt_since_prev_accept_s"] is None and prev_acc_ts is not None:
                    row["dt_since_prev_accept_s"] = ts - prev_acc_ts
                break
        prev_acc_ts = ts

    gps = gnss.sort_values("timestamp_s")
    prev_gps_ts = None
    gps_dt = {}
    for _, r in gps.iterrows():
        ts = float(r.timestamp_s)
        if prev_gps_ts is not None:
            gps_dt[int(r.gps_index)] = ts - prev_gps_ts
        prev_gps_ts = ts
    for row in arm_rows:
        if row["dt_since_prev_gnss_s"] is None:
            row["dt_since_prev_gnss_s"] = gps_dt.get(row["gps_index"])
        # First accept: no prev accept → use GNSS observation gap as proxy
        row["effective_gap_s"] = row["dt_since_prev_accept_s"]
        if row["effective_gap_s"] is None and row["dt_since_prev_gnss_s"] is not None:
            row["effective_gap_s"] = row["dt_since_prev_gnss_s"]


def gap_bucket_analysis(rows: list[dict], thresholds: list[float], gap_key: str = "dt_since_prev_accept_s") -> dict:
    out = {}
    for th in thresholds:
        long_gap = [r for r in rows if r.get(gap_key) is not None and r[gap_key] > th]
        short_gap = [r for r in rows if r.get(gap_key) is not None and r[gap_key] <= th]
        first = [r for r in rows if r.get(gap_key) is None]
        out[f"gap_gt_{th}s"] = {
            "n": len(long_gap),
            "cos_mean": float(np.mean([r["cos_dv_pos_err_pre"] for r in long_gap])) if long_gap else None,
            "frac_aids": sum(r["cross_aids_correction"] for r in long_gap) / len(long_gap) if long_gap else None,
            "frac_cf_improves": sum(r["counterfactual_vel_improves"] for r in long_gap) / len(long_gap)
            if long_gap
            else None,
        }
        out[f"gap_le_{th}s"] = {
            "n": len(short_gap),
            "cos_mean": float(np.mean([r["cos_dv_pos_err_pre"] for r in short_gap])) if short_gap else None,
            "frac_aids": sum(r["cross_aids_correction"] for r in short_gap) / len(short_gap) if short_gap else None,
            "frac_cf_improves": sum(r["counterfactual_vel_improves"] for r in short_gap) / len(short_gap)
            if short_gap
            else None,
        }
        out["first_accept_no_dt"] = {
            "n": len(first),
            "cos_mean": float(np.mean([r["cos_dv_pos_err_pre"] for r in first])) if first else None,
        }
    return out


def contingency_cos_vs_outcome(rows: list[dict]) -> dict:
    pos = [r for r in rows if r["cos_dv_pos_err_pre"] > 0]
    neg = [r for r in rows if r["cos_dv_pos_err_pre"] < 0]
    zero = [r for r in rows if r["cos_dv_pos_err_pre"] == 0]
    return {
        "cos_positive_n": len(pos),
        "cos_negative_n": len(neg),
        "cos_positive_frac_cf_improves": sum(r["counterfactual_vel_improves"] for r in pos) / len(pos) if pos else None,
        "cos_negative_frac_cf_improves": sum(r["counterfactual_vel_improves"] for r in neg) / len(neg) if neg else None,
        "hypothesis_cos_neg_helps": (
            len(neg) >= 5
            and len(pos) >= 5
            and sum(r["counterfactual_vel_improves"] for r in neg) / len(neg)
            > sum(r["counterfactual_vel_improves"] for r in pos) / len(pos)
        ),
    }


def main() -> int:
    all_rows: list[dict] = []
    by_arm: dict[str, list[dict]] = {}

    for arm in ARMS:
        d = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity" / arm
        gnss = pd.read_csv(d / "gnss_nis_audit.csv")
        for c in gnss.columns:
            if c not in ("reject_reason",):
                gnss[c] = pd.to_numeric(gnss[c], errors="coerce")
        acc = gnss[gnss["accepted"] == 1]
        kblocks = load_all_k_blocks(d / "gnss_k_block.jsonl")
        arm_rows = []

        for _, row in acc.iterrows():
            idx = int(row.gps_index)
            if idx not in kblocks:
                continue
            pre = load_cov_pre(d / "cov_step_audit.csv", float(row.timestamp_s))
            if pre is None:
                continue
            rec = analyze_row(row, kblocks[idx], pre)
            if rec is None:
                continue
            rec["arm"] = arm
            if arm == "G1":
                vg_n = row.gps_speed_mps * math.cos(math.radians(row.gps_course_deg))
                vg_e = row.gps_speed_mps * math.sin(math.radians(row.gps_course_deg))
                err_pre = math.hypot(row.vel_pred_n_mps - vg_n, row.vel_pred_e_mps - vg_e)
                err_post = math.hypot(row.vel_after_n_mps - vg_n, row.vel_after_e_mps - vg_e)
                rec["actual_delta_err_vel_mps"] = err_post - err_pre
                rec["actual_vel_improves"] = bool(err_post < err_pre)
            arm_rows.append(rec)
            all_rows.append(rec)

        by_arm[arm] = arm_rows

    for arm in ARMS:
        enrich_gaps(by_arm[arm], pd.read_csv(REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity" / arm / "gnss_nis_audit.csv"))

    all_rows = [r for rows in by_arm.values() for r in rows]

    pool = summarize_pool(all_rows)
    g2_rows = by_arm.get("G2", [])
    g2_only = summarize_pool(g2_rows)
    gap = gap_bucket_analysis(all_rows, [1.0, 2.0, 4.0, 5.0, 10.0])
    g2_gap = gap_bucket_analysis(g2_rows, [1.0, 2.0, 4.0, 5.0, 10.0])
    g2_gap_effective = gap_bucket_analysis(g2_rows, [1.0, 2.0, 4.0, 5.0, 10.0], "effective_gap_s")

    # Decision heuristic for preregistration branch (G2 primary n=33)
    g2 = g2_only
    r_gap = g2.get("corr_cos_vs_effective_gap") or g2.get("corr_cos_vs_dt_since_accept")
    r_y = g2.get("corr_cos_vs_y_pos")
    cos_mean = g2.get("cos_mean") or 0
    frac_aids = g2.get("frac_cross_aids") or 0.5
    cont = contingency_cos_vs_outcome(g2_rows)

    if g2.get("n", 0) >= 10:
        gap_buckets = g2_gap_effective.get("gap_gt_4.0s") or {}
        short_buckets = g2_gap_effective.get("gap_le_1.0s") or {}
        long_n = gap_buckets.get("n") or 0
        if cont.get("hypothesis_cos_neg_helps"):
            if long_n >= 3 and gap_buckets.get("cos_mean") is not None and short_buckets.get("cos_mean") is not None:
                if gap_buckets["cos_mean"] < short_buckets["cos_mean"] - 0.15:
                    branch = (
                        "SHORT_GAP_CONDITIONAL_RESET — long effective_gap → cos<0 (cross aids); "
                        "short gap → cos>0 (cross hurts). **Invert** naive post-long-gap reset."
                    )
                elif gap_buckets["cos_mean"] > short_buckets["cos_mean"] + 0.15:
                    branch = "GAP_CONDITIONAL_RESET — long-gap accepts have more cos>0 (cross hurts)"
                elif abs(cos_mean) < 0.15 and 0.35 < frac_aids < 0.65:
                    branch = "UNCONDITIONAL_DOWNWEIGHT — cos ~ symmetric, ~50/50 aids/hurts"
                else:
                    branch = "MIXED — alignment real (cos<0 helps) but gap buckets weak/inverted vs naive reset"
            elif abs(cos_mean) < 0.15 and 0.35 < frac_aids < 0.65:
                branch = "UNCONDITIONAL_DOWNWEIGHT — cos ~ symmetric, ~50/50 aids/hurts"
            else:
                branch = "MIXED — cos<0→helps confirmed; gap signal weak (r_eff=%s, r_y=%.2f)" % (
                    "nan" if r_gap is None or math.isnan(r_gap) else f"{r_gap:.2f}",
                    r_y or float("nan"),
                )
        elif abs(cos_mean) < 0.15 and 0.35 < frac_aids < 0.65:
            branch = "UNCONDITIONAL_DOWNWEIGHT — cos ~ symmetric, ~50/50 aids/hurts"
        else:
            branch = "MIXED — alignment/outcome link weak at population level"
    else:
        branch = "INSUFFICIENT_N"

    report = {
        "experiment": "GAP-4 alignment sweep — cos(dv_pos, err_pre) all k_block accepts",
        "coverage_note": (
            "k_block accepts with speed; counterfactual 5D pos+vel. "
            "Pool 45 includes same physical fixes across G0/G1/G2 replays — **G2-only (n=33) is primary**."
        ),
        "by_arm_counts": {a: len(by_arm.get(a, [])) for a in ARMS},
        "pool_summary_all_arms": pool,
        "G2_summary_primary": g2_only,
        "G2_gap_buckets_accept_dt": g2_gap,
        "G2_gap_buckets_effective_gap": g2_gap_effective,
        "G2_contingency_cos_vs_cf_outcome": contingency_cos_vs_outcome(g2_rows),
        "gap_bucket_analysis_all_arms": gap,
        "preregistration_branch_suggestion": branch,
        "rows": sorted(all_rows, key=lambda r: r["timestamp_s"]),
        "highlight_fix2_fix7_fix56": [r for r in all_rows if r["gps_index"] in (2, 7, 56)],
    }

    out = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G1/gap4_alignment_sweep_report.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "rows"}, indent=2))
    print(f"\nWrote {out} ({len(all_rows)} rows)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
