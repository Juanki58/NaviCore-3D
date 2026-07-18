#!/usr/bin/env python3
"""Compare gap vs ||y_pos|| as predictors of cos(dv_pos, err_pre) sign (G2 n=33)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SWEEP = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G1/gap4_alignment_sweep_report.json"


def load_g2_rows() -> list[dict]:
    data = json.loads(SWEEP.read_text(encoding="utf-8"))
    return [r for r in data["rows"] if r["arm"] == "G2" and np.isfinite(r["cos_dv_pos_err_pre"])]


def spearman(x: np.ndarray, y: np.ndarray) -> float:
    x, y = np.asarray(x, float), np.asarray(y, float)
    m = np.isfinite(x) & np.isfinite(y)
    x, y = x[m], y[m]
    if len(x) < 3:
        return float("nan")
    rx = np.argsort(np.argsort(x))
    ry = np.argsort(np.argsort(y))
    return float(np.corrcoef(rx, ry)[0, 1])


def threshold_sweep(rows: list[dict], key: str, direction: str) -> list[dict]:
    """Intervene (predict cos>0 bad) when value is on 'low' or 'high' side of threshold."""
    vals = np.array([r[key] for r in rows], float)
    cos = np.array([r["cos_dv_pos_err_pre"] for r in rows], float)
    bad = cos > 0
    good = cos < 0
    uniq = sorted(set(vals[np.isfinite(vals)]))
    if len(uniq) < 2:
        return []
    candidates = []
    for i in range(len(uniq) - 1):
        candidates.append(0.5 * (uniq[i] + uniq[i + 1]))
    candidates.extend(uniq)

    out = []
    for th in candidates:
        if direction == "low":
            pred_bad = vals <= th
        else:
            pred_bad = vals >= th
        tp = int(np.sum(pred_bad & bad))
        fn = int(np.sum(~pred_bad & bad))
        fp = int(np.sum(pred_bad & good))
        tn = int(np.sum(~pred_bad & good))
        n_bad, n_good = int(np.sum(bad)), int(np.sum(good))
        sens = tp / n_bad if n_bad else float("nan")
        spec = tn / n_good if n_good else float("nan")
        youden = sens + spec - 1 if np.isfinite(sens) and np.isfinite(spec) else float("nan")
        out.append(
            {
                "threshold": float(th),
                "direction": direction,
                "n_intervene": int(np.sum(pred_bad)),
                "tp_bad_caught": tp,
                "fn_bad_missed": fn,
                "fp_good_killed": fp,
                "tn_good_spared": tn,
                "sensitivity_bad": sens,
                "specificity_good": spec,
                "youden_j": youden,
            }
        )
    return out


def best_by_youden(sweep: list[dict]) -> dict | None:
    finite = [s for s in sweep if np.isfinite(s["youden_j"])]
    return max(finite, key=lambda s: s["youden_j"]) if finite else None


def overlap_gap(rows: list[dict], key: str) -> dict:
    pos = [r[key] for r in rows if r["cos_dv_pos_err_pre"] > 0]
    neg = [r[key] for r in rows if r["cos_dv_pos_err_pre"] < 0]
    if not pos or not neg:
        return {"separable": False}
    max_pos = max(pos)
    min_neg = min(neg)
    max_neg = max(neg)
    min_pos = min(pos)
    return {
        "separable_strict": max_pos < min_neg or min_pos > max_neg,
        "max_among_cos_pos": max_pos,
        "min_among_cos_pos": min_pos,
        "max_among_cos_neg": max_neg,
        "min_among_cos_neg": min_neg,
        "overlap_range": max(0.0, min(max_pos, max_neg) - max(min_pos, min_neg)),
    }


def short_gap_y_pos_analysis(rows: list[dict]) -> dict:
    short = [r for r in rows if r.get("effective_gap_s") is not None and r["effective_gap_s"] <= 1.0]
    if len(short) < 3:
        return {"n": len(short)}
    y = np.array([r["y_pos_norm_3d_m"] for r in short])
    c = np.array([r["cos_dv_pos_err_pre"] for r in short])
    med_y = float(np.median(y))
    low_y = [r for r in short if r["y_pos_norm_3d_m"] <= med_y]
    high_y = [r for r in short if r["y_pos_norm_3d_m"] > med_y]
    return {
        "n": len(short),
        "cos_mean": float(np.mean(c)),
        "cos_median": float(np.median(c)),
        "cos_std": float(np.std(c)),
        "corr_cos_vs_y_pos_pearson": float(np.corrcoef(y, c)[0, 1]) if np.std(y) > 0 else None,
        "corr_cos_vs_y_pos_spearman": spearman(y, c),
        "y_pos_median_split_m": med_y,
        "low_y_pos_n": len(low_y),
        "low_y_pos_frac_cos_pos_bad": sum(r["cos_dv_pos_err_pre"] > 0 for r in low_y) / len(low_y),
        "high_y_pos_n": len(high_y),
        "high_y_pos_frac_cos_pos_bad": sum(r["cos_dv_pos_err_pre"] > 0 for r in high_y) / len(high_y),
        "high_y_pos_frac_cos_neg_good": sum(r["cos_dv_pos_err_pre"] < 0 for r in high_y) / len(high_y),
        "note": "Within gap<=1s: if high-y_pos still has cos<0, gap-only reset would wrongly kill helpful cross",
    }


def combined_rules(rows: list[dict]) -> list[dict]:
    gap = np.array([r["effective_gap_s"] for r in rows], float)
    y = np.array([r["y_pos_norm_3d_m"] for r in rows], float)
    cos = np.array([r["cos_dv_pos_err_pre"] for r in rows], float)
    bad, good = cos > 0, cos < 0
    rules = [
        ("gap_le_1s", gap <= 1.0),
        ("y_pos_le_50m", y <= 50),
        ("y_pos_le_100m", y <= 100),
        ("y_pos_le_211m", y <= 211),
        ("gap_le_1s_AND_y_le_100", (gap <= 1.0) & (y <= 100)),
        ("gap_le_1s_AND_y_le_197", (gap <= 1.0) & (y <= 197)),
        ("gap_le_1s_AND_y_le_50", (gap <= 1.0) & (y <= 50)),
    ]
    out = []
    for name, mask in rules:
        tp = int(np.sum(mask & bad))
        fn = int(np.sum(~mask & bad))
        fp = int(np.sum(mask & good))
        tn = int(np.sum(~mask & good))
        nb, ng = int(np.sum(bad)), int(np.sum(good))
        out.append(
            {
                "rule": name,
                "n_intervene": int(np.sum(mask)),
                "sensitivity_bad": tp / nb if nb else None,
                "specificity_good": tn / ng if ng else None,
                "fp_good_killed": fp,
                "fn_bad_missed": fn,
                "youden_j": (tp / nb + tn / ng - 1) if nb and ng else None,
            }
        )
    return out


def main() -> int:
    rows = load_g2_rows()
    y = np.array([r["y_pos_norm_3d_m"] for r in rows])
    g = np.array([r["effective_gap_s"] for r in rows])
    c = np.array([r["cos_dv_pos_err_pre"] for r in rows])

    y_sweep = threshold_sweep(rows, "y_pos_norm_3d_m", "low")
    g_sweep = threshold_sweep(rows, "effective_gap_s", "low")
    y_best = best_by_youden(y_sweep)
    g_best = best_by_youden(g_sweep)

    # Also try high-side for y (large y -> good cos) — intervene when NOT large y
    # equivalent to low threshold on y

    winner = "y_pos" if (y_best and g_best and y_best["youden_j"] >= g_best["youden_j"]) else "gap"
    if y_best and g_best and abs(y_best["youden_j"] - g_best["youden_j"]) < 0.05:
        winner = "TIE"

    scatter = sorted(
        [
            {
                "gps_index": r["gps_index"],
                "y_pos_norm_3d_m": r["y_pos_norm_3d_m"],
                "effective_gap_s": r["effective_gap_s"],
                "cos_dv_pos_err_pre": r["cos_dv_pos_err_pre"],
                "cos_sign": "bad_pos" if r["cos_dv_pos_err_pre"] > 0 else "good_neg",
            }
            for r in rows
        ],
        key=lambda x: x["y_pos_norm_3d_m"],
    )

    report = {
        "experiment": "GAP-4 threshold discrimination — gap vs ||y_pos|| for cos sign (G2 n=33)",
        "n": len(rows),
        "correlations": {
            "pearson_cos_y_pos": float(np.corrcoef(y, c)[0, 1]),
            "pearson_cos_gap": float(np.corrcoef(g, c)[0, 1]),
            "spearman_cos_y_pos": spearman(y, c),
            "spearman_cos_gap": spearman(g, c),
        },
        "overlap": {
            "y_pos_m": overlap_gap(rows, "y_pos_norm_3d_m"),
            "effective_gap_s": overlap_gap(rows, "effective_gap_s"),
        },
        "best_threshold_y_pos_low": y_best,
        "best_threshold_gap_low": g_best,
        "winner_by_youden_j": winner,
        "preregistration_hint": (
            "downweight cross when y_pos <= T_y"
            if winner == "y_pos"
            else "downweight cross when effective_gap <= T"
            if winner == "gap"
            else "compare both arms in preregistration"
        ),
        "short_gap_le_1s_y_pos_stratification": short_gap_y_pos_analysis(rows),
        "combined_intervention_rules": combined_rules(rows),
        "short_gap_high_y_good_cos_neg_indices": [
            r["gps_index"]
            for r in rows
            if r.get("effective_gap_s") is not None
            and r["effective_gap_s"] <= 1.0
            and r["y_pos_norm_3d_m"] > float(np.median([x["y_pos_norm_3d_m"] for x in rows if x.get("effective_gap_s") is not None and x["effective_gap_s"] <= 1.0]))
            and r["cos_dv_pos_err_pre"] < 0
        ],
        "y_pos_threshold_sweep": y_sweep,
        "gap_threshold_sweep": g_sweep,
        "scatter_sorted_by_y_pos": scatter,
    }

    out = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G1/gap4_threshold_discrimination_report.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")

    summary = {k: v for k, v in report.items() if k not in ("y_pos_threshold_sweep", "gap_threshold_sweep", "scatter_sorted_by_y_pos")}
    print(json.dumps(summary, indent=2))
    print(f"\nWrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
