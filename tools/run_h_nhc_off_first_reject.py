#!/usr/bin/env python3
"""NHC-off only: locate the first GNSS that ends the early-accept regime; decompose 5-DoF NIS."""

from __future__ import annotations

import json
from pathlib import Path

import pandas as pd

REPO = Path(__file__).resolve().parents[1]
ARM = REPO / "docs" / "benchmarks" / "h_nhc_policy_ab" / "B_nhc_disabled"
OUT = REPO / "docs" / "benchmarks" / "h_nhc_off_first_reject"
CONTRIB = ["nis_contrib_n", "nis_contrib_e", "nis_contrib_d", "nis_contrib_vn", "nis_contrib_ve"]


def main() -> int:
    path = ARM / "gnss_nis_audit.csv"
    df = pd.read_csv(path)
    OUT.mkdir(parents=True, exist_ok=True)

    # Early regime: consecutive accepts from start until first reject
    first_rej_idx = None
    for i, a in enumerate(df["accepted"].tolist()):
        if int(a) == 0:
            first_rej_idx = i
            break

    if first_rej_idx is None:
        verdict = {"error": "no rejects in arm B"}
        (OUT / "verdict.json").write_text(json.dumps(verdict, indent=2), encoding="utf-8")
        print(json.dumps(verdict, indent=2))
        return 1

    # Prefer first reject that ends a streak of accepts (regime change),
    # not seed-time oddities: find last accept before first contiguous reject streak
    # that is followed by ≥3 rejects (onset of closed gate).
    regime_idx = None
    for i in range(len(df)):
        if int(df.iloc[i]["accepted"]) != 0:
            continue
        # look ahead: is this start of sustained reject?
        window = df.iloc[i : i + 5]["accepted"]
        if len(window) >= 3 and (window == 0).sum() >= 3:
            # and previous was accept (regime change)
            if i > 0 and int(df.iloc[i - 1]["accepted"]) == 1:
                regime_idx = i
                break
    if regime_idx is None:
        regime_idx = first_rej_idx

    prev = df.iloc[regime_idx - 1] if regime_idx > 0 else None
    cur = df.iloc[regime_idx]

    def row_pack(r: pd.Series | None) -> dict | None:
        if r is None:
            return None
        contrib = {c: float(r[c]) for c in CONTRIB}
        gate = float(r["gnss_nis_gate"])
        thr = float(r["nis_threshold"])
        total_c = sum(max(v, 0.0) for v in contrib.values())  # signed contribs exist
        abs_sum = sum(abs(v) for v in contrib.values())
        shares = {
            c.replace("nis_contrib_", ""): (abs(contrib[c]) / abs_sum if abs_sum > 0 else None)
            for c in CONTRIB
        }
        dominant = max(CONTRIB, key=lambda c: abs(contrib[c])).replace("nis_contrib_", "")
        return {
            "timestamp_s": float(r["timestamp_s"]),
            "gps_index": int(r["gps_index"]),
            "accepted": int(r["accepted"]),
            "reject_reason": int(r["reject_reason"]),
            "n_meas": int(r["n_meas"]),
            "gnss_nis_gate": gate,
            "nis_threshold": thr,
            "gate_margin": thr - gate,
            "crossed": gate > thr,
            "contrib": contrib,
            "contrib_abs_share": shares,
            "dominant_abs_component": dominant,
            "innov_h_m": float(r["innov_h_m"]),
            "innov_vn_mps": float(r["innov_vn_mps"]),
            "innov_ve_mps": float(r["innov_ve_mps"]),
            "vel_pred_h_mps": float(r["vel_pred_h_mps"]),
            "gps_speed_mps": float(r["gps_speed_mps"]),
            "dt_since_prev_accept_s": float(r["dt_since_prev_accept_s"]),
        }

    last_early_accept_idx = None
    for i in range(regime_idx - 1, -1, -1):
        if int(df.iloc[i]["accepted"]) == 1:
            last_early_accept_idx = i
            break

    # Also: first absolute reject (if different)
    first_abs = row_pack(df.iloc[first_rej_idx])

    # Context: all events from last_early_accept-2 through regime+3
    lo = max(0, (last_early_accept_idx or regime_idx) - 2)
    hi = min(len(df), regime_idx + 4)
    context = [row_pack(df.iloc[i]) for i in range(lo, hi)]

    verdict = {
        "arm": "B_nhc_disabled",
        "question": (
            "In NHC-off G1 shell, which 5-DoF NIS component crosses the gate "
            "at the first GNSS that ends the early-accept regime?"
        ),
        "exit_rule": (
            "Days 1–4: produce a concrete causal mechanism for this transition, "
            "or freeze knowledge and start EKF rewrite. No parallel branches."
        ),
        "first_absolute_reject": first_abs,
        "regime_change_reject": row_pack(cur),
        "last_accept_before_regime_change": row_pack(prev),
        "context_rows": context,
        "answer": {
            "t_regime_change_s": float(cur["timestamp_s"]),
            "gps_index": int(cur["gps_index"]),
            "nis_gate": float(cur["gnss_nis_gate"]),
            "threshold": float(cur["nis_threshold"]),
            "dominant_component": row_pack(cur)["dominant_abs_component"],
            "contrib": row_pack(cur)["contrib"],
            "shares_abs": row_pack(cur)["contrib_abs_share"],
        },
    }

    out = OUT / "verdict.json"
    out.write_text(json.dumps(verdict, indent=2), encoding="utf-8")
    print(json.dumps(verdict["answer"], indent=2))
    print("regime_change_t=", verdict["answer"]["t_regime_change_s"])
    print("dominant=", verdict["answer"]["dominant_component"])
    print("->", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
