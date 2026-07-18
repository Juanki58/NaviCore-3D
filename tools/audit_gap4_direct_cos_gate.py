#!/usr/bin/env python3
"""Simulate H1a/H1b/H1d gates on G2 n=33 — direct cos vs proxy rules."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
R_VEL = 2.25
G2 = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G2"


def load_all_k_blocks(path: Path) -> dict[int, dict]:
    text = path.read_text(encoding="utf-8")
    objs, depth, start = [], 0, None
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


def cos_u(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-12 or nb < 1e-12:
        return float("nan")
    return float(np.dot(a, b) / (na * nb))


def enrich_effective_gap(rows: list[dict], gnss: pd.DataFrame) -> None:
    acc = gnss[gnss["accepted"] == 1].sort_values("timestamp_s")
    prev = None
    gps_prev = None
    gps_dt = {}
    for _, r in gnss.sort_values("timestamp_s").iterrows():
        ts = float(r.timestamp_s)
        if gps_prev is not None:
            gps_dt[int(r.gps_index)] = ts - gps_prev
        gps_prev = ts
    for _, r in acc.iterrows():
        idx, ts = int(r.gps_index), float(r.timestamp_s)
        for row in rows:
            if row["gps_index"] == idx:
                if prev is not None:
                    row["effective_gap_s"] = ts - prev
                elif idx in gps_dt:
                    row["effective_gap_s"] = gps_dt[idx]
                break
        prev = ts


def load_g2_enriched() -> list[dict]:
    gnss = pd.read_csv(G2 / "gnss_nis_audit.csv")
    for c in gnss.columns:
        if c not in ("reject_reason",):
            gnss[c] = pd.to_numeric(gnss[c], errors="coerce")
    kblocks = load_all_k_blocks(G2 / "gnss_k_block.jsonl")
    rows = []
    for _, row in gnss[gnss["accepted"] == 1].iterrows():
        idx = int(row.gps_index)
        if idx not in kblocks:
            continue
        pre = load_cov_pre(G2 / "cov_step_audit.csv", float(row.timestamp_s))
        if pre is None or not row.has_gps_speed or row.gps_speed_mps <= 0:
            continue
        k = kblocks[idx]
        p_pp = np.array(k["HPH_m2"], dtype=float)
        p_vp = np.array(k["P_vel_pos_cross_m2"], dtype=float)
        p_vv_diag = np.array([pre["P_vv_n_m2"], pre["P_vv_e_m2"], pre["P_vv_d_m2"]])
        p = build_minimal_p(p_pp, p_vp, p_vv_diag, float(pre["P_pv_frob"]), float(pre["P_aa_frob"]))
        h5 = build_h5()
        k5 = p @ h5.T @ np.linalg.inv(h5 @ p @ h5.T + build_r5(float(k["R_m2"])))
        y_pos = np.array([row.innov_n_m, row.innov_e_m, row.innov_d_m])
        y_vel = np.array([row.pseudo_innov_v_n_mps, row.pseudo_innov_v_e_mps])
        dv_pos = k5[3:5, 0:3] @ y_pos
        dv_vel = k5[3:5, 3:5] @ y_vel
        dv_tot = dv_pos + dv_vel
        vg_n = row.gps_speed_mps * math.cos(math.radians(row.gps_course_deg))
        vg_e = row.gps_speed_mps * math.sin(math.radians(row.gps_course_deg))
        err = np.array([row.vel_pred_n_mps - vg_n, row.vel_pred_e_mps - vg_e])
        err_pre = float(np.linalg.norm(err))

        def err_after(dv):
            vn = row.vel_pred_n_mps + dv[0]
            ve = row.vel_pred_e_mps + dv[1]
            return float(math.hypot(vn - vg_n, ve - vg_e))

        err_full = err_after(dv_tot)
        err_vel_only = err_after(dv_vel)
        err_no_cross = err_vel_only

        rows.append(
            {
                "gps_index": idx,
                "timestamp_s": float(row.timestamp_s),
                "effective_gap_s": None,
                "err_pre_mps": err_pre,
                "err_full_5d_mps": err_full,
                "err_vel_only_mps": err_vel_only,
                "delta_full_mps": err_full - err_pre,
                "delta_vel_only_mps": err_vel_only - err_pre,
                "zero_cross_helps": bool(err_vel_only < err_full),
                "cos_dv_pos_err_pre": cos_u(dv_pos, err),
                "cos_dv_tot_err_pre": cos_u(dv_tot, err),
                "cross_term_fraction": float(np.linalg.norm(dv_pos) / max(np.linalg.norm(dv_pos) + np.linalg.norm(dv_vel), 1e-9)),
            }
        )
    enrich_effective_gap(rows, gnss)
    return rows


def apply_gate(rows: list[dict], intervene_fn) -> dict:
    """Zero cross (use vel-only) when intervene_fn(row) is True."""
    deltas = []
    tp = fp = tn = fn = 0
    for r in rows:
        intervene = intervene_fn(r)
        err_post = r["err_vel_only_mps"] if intervene else r["err_full_5d_mps"]
        deltas.append(err_post - r["err_pre_mps"])
        need_zero = r["zero_cross_helps"]
        if intervene and need_zero:
            tp += 1
        elif intervene and not need_zero:
            fp += 1
        elif not intervene and not need_zero:
            tn += 1
        else:
            fn += 1
    n_need = sum(r["zero_cross_helps"] for r in rows)
    n_keep = len(rows) - n_need
    return {
        "n_intervene": sum(1 for r in rows if intervene_fn(r)),
        "sensitivity_zero_cross_helps": tp / n_need if n_need else None,
        "specificity_keep_cross": tn / n_keep if n_keep else None,
        "fp_good_cross_killed": fp,
        "fn_harmful_cross_missed": fn,
        "mean_delta_err_mps": float(np.mean(deltas)),
        "sum_delta_err_mps": float(np.sum(deltas)),
        "frac_improves": sum(d < 0 for d in deltas) / len(deltas),
    }


def gate_classification_vs_cos_sign(rows: list[dict], intervene_fn, label: str) -> dict:
    cos_pos = [r for r in rows if r["cos_dv_pos_err_pre"] > 0]
    cos_neg = [r for r in rows if r["cos_dv_pos_err_pre"] < 0]
    return {
        "rule": label,
        **apply_gate(rows, intervene_fn),
        "frac_cos_pos_intervened": sum(intervene_fn(r) for r in cos_pos) / len(cos_pos) if cos_pos else None,
        "frac_cos_neg_spared": sum(not intervene_fn(r) for r in cos_neg) / len(cos_neg) if cos_neg else None,
    }


def main() -> int:
    rows = load_g2_enriched()
    sweep = json.loads((REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G1/gap4_alignment_sweep_report.json").read_text())
    y_by_idx = {r["gps_index"]: r["y_pos_norm_3d_m"] for r in sweep["rows"] if r["arm"] == "G2"}
    for r in rows:
        r["y_pos_norm_3d_m"] = y_by_idx.get(r["gps_index"], float("nan"))

    n_need = sum(r["zero_cross_helps"] for r in rows)

    rules = [
        gate_classification_vs_cos_sign(rows, lambda r: False, "baseline_5d_no_gate"),
        gate_classification_vs_cos_sign(rows, lambda r: True, "H1b_unconditional_zero_cross"),
        gate_classification_vs_cos_sign(
            rows, lambda r: r.get("effective_gap_s") is not None and r["effective_gap_s"] <= 1.0, "H1a_gap_le_1s"
        ),
        gate_classification_vs_cos_sign(rows, lambda r: r["cos_dv_pos_err_pre"] > 0, "H1d_cos_pos_gt_0"),
        gate_classification_vs_cos_sign(rows, lambda r: r["cos_dv_tot_err_pre"] > 0, "H1d_prime_cos_tot_gt_0"),
        gate_classification_vs_cos_sign(
            rows,
            lambda r: (r.get("effective_gap_s") is not None and r["effective_gap_s"] <= 1.0)
            and r.get("y_pos_norm_3d_m", float("inf")) <= 197,
            "H1c_gap_le_1s_AND_y_le_197",
        ),
    ]

    g1_fix7 = next((r for r in sweep["rows"] if r["arm"] == "G1" and r["gps_index"] == 7), None)

    report = {
        "experiment": "GAP-4 direct cos gate simulation (G2 n=33, counterfactual 5D algebra)",
        "n": len(rows),
        "outcome_definition": "zero_cross_helps := err_vel_only < err_full_5d (counterfactual)",
        "n_zero_cross_helps": n_need,
        "n_keep_cross_better": len(rows) - n_need,
        "rules": rules,
        "H1d_vs_H1a": {
            "H1d_sensitivity": rules[3]["sensitivity_zero_cross_helps"],
            "H1a_sensitivity": rules[2]["sensitivity_zero_cross_helps"],
            "H1d_specificity": rules[3]["specificity_keep_cross"],
            "H1a_specificity": rules[2]["specificity_keep_cross"],
            "H1d_mean_delta_err": rules[3]["mean_delta_err_mps"],
            "H1a_mean_delta_err": rules[2]["mean_delta_err_mps"],
            "H1d_better_mean_delta": rules[3]["mean_delta_err_mps"] < rules[2]["mean_delta_err_mps"],
        },
        "fix7_G1_note": g1_fix7,
        "fix7_warning": (
            "G1 fix#7: actual vel improves despite cos(dv_pos,err)>0; "
            "H1d_pos would zero cross but full 5d counterfactual on G2 fix#7 also listed in sweep"
        ),
        "causal_gate_note": (
            "H1d uses cos(dv_pos, err_pre) with err_pre=v_pred-v_GPS at update time — no look-ahead. "
            "Classification vs zero_cross_helps tests whether gate targets cases where dropping cross improves vel."
        ),
        "preregistration_recommendation": (
            "§11: 1a+1b+1d+1d_prime on G1 replay (out-of-sample). "
            "Table below is EXPLORATORY ONLY — same cos variable used in ALIGNMENT_PRIMARY on this G2 set."
        ),
    }

    out = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G1/gap4_direct_cos_gate_report.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "fix7_G1_note"}, indent=2))
    print(f"\nWrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
