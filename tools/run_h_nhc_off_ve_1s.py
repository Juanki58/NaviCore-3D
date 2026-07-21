#!/usr/bin/env python3
"""Single-second v_E chronology: Accept #17 → Reject #18 (NHC-off).

No general audits. One table: who wrote Δv_E, how much, cumulative v_E.
"""

from __future__ import annotations

import json
from pathlib import Path

import pandas as pd

REPO = Path(__file__).resolve().parents[1]
ARM = REPO / "docs" / "benchmarks" / "h_nhc_policy_ab" / "B_nhc_disabled"
OUT = REPO / "docs" / "benchmarks" / "h_nhc_off_ve_1s"
T0, T1 = 19.301353455, 20.301353455


def main() -> int:
    pipe = pd.read_csv(ARM / "constraint_pipeline_audit.csv")
    gnss = pd.read_csv(ARM / "gnss_nis_audit.csv")
    OUT.mkdir(parents=True, exist_ok=True)

    a17 = gnss[gnss["gps_index"] == 17].iloc[0]
    r18 = gnss[gnss["gps_index"] == 18].iloc[0]

    # IMU ticks strictly after accept time and up to reject time
    w = pipe[(pipe["timestamp_s"] > T0) & (pipe["timestamp_s"] <= T1)].copy()
    w = w.sort_values("timestamp_s")

    rows: list[dict] = []

    # State immediately after Accept #17 (GNSS wrote velocity)
    v_e = float(a17["vel_after_e_mps"])
    rows.append(
        {
            "tick": "Accept #17 post-GNSS",
            "t_s": float(a17["timestamp_s"]),
            "source": "update_gnss",
            "dx_vel_e": float(a17["dx_vel_e_mps"]),
            "dv_e": float(a17["dx_vel_e_mps"]),
            "v_e_after": v_e,
            "why": "GNSS vel correction at accept (dx_vel_e from Kalman update)",
        }
    )

    sum_dv_pred = 0.0
    sum_dv_nhc = 0.0
    sum_dv_zupt = 0.0
    for i, r in enumerate(w.itertuples(index=False), start=1):
        dv_pred = float(r.dv_pred_e)
        dv_nhc = float(r.dv_nhc_e)
        dv_zupt = float(r.dv_zupt_e)
        sum_dv_pred += dv_pred
        sum_dv_nhc += dv_nhc
        sum_dv_zupt += dv_zupt
        # Prefer after-zupt as end-of-tick state (NHC/ZUPT off → same as after pred)
        v_e = float(r.vel_after_zupt_e)
        # Only emit every tick but also a condensed summary later; full table for ~100 Hz = ~100 rows
        rows.append(
            {
                "tick": f"IMU #{i}",
                "t_s": float(r.timestamp_s),
                "source": "predict",
                "dv_pred_e": dv_pred,
                "dv_nhc_e": dv_nhc,
                "dv_zupt_e": dv_zupt,
                "dv_e": dv_pred + dv_nhc + dv_zupt,
                "v_e_after": v_e,
                "nhc_applied": int(r.nhc_applied),
                "zupt_applied": int(r.zupt_applied),
                "why": "IMU predict integrates specific force → Δv_NED (NHC/ZUPT off ⇒ Δ=dv_pred)",
            }
        )

    rows.append(
        {
            "tick": "Reject #18 pre-GNSS (vel_pred)",
            "t_s": float(r18["timestamp_s"]),
            "source": "state_at_gate",
            "dv_e": None,
            "v_e_after": float(r18["vel_pred_e_mps"]),
            "innov_ve": float(r18["innov_ve_mps"]),
            "nis_contrib_ve": float(r18["nis_contrib_ve"]),
            "gnss_nis_gate": float(r18["gnss_nis_gate"]),
            "why": "Filter v_E presented to GNSS gate; reject because NIS dominated by ve",
        }
    )

    v_e_start = float(a17["vel_after_e_mps"])
    v_e_end = float(r18["vel_pred_e_mps"])
    delta_total = v_e_end - v_e_start

    # Dominance: who explains the net change
    writers = {
        "update_gnss_at_17": float(a17["dx_vel_e_mps"]),  # already applied before interval
        "sum_dv_pred_e_in_interval": sum_dv_pred,
        "sum_dv_nhc_e_in_interval": sum_dv_nhc,
        "sum_dv_zupt_e_in_interval": sum_dv_zupt,
    }
    # Net change in interval should ≈ sum of predict/nhc/zupt
    explained = sum_dv_pred + sum_dv_nhc + sum_dv_zupt
    residual_unexplained = delta_total - explained

    abs_writers = {
        "predict": abs(sum_dv_pred),
        "nhc": abs(sum_dv_nhc),
        "zupt": abs(sum_dv_zupt),
    }
    total_abs = sum(abs_writers.values()) or 1.0
    shares = {k: v / total_abs for k, v in abs_writers.items()}
    dominant = max(abs_writers, key=abs_writers.get)

    # Compact table: endpoints + top |dv| ticks + aggregate
    w_sorted = w.copy()
    w_sorted["abs_dv"] = w_sorted["dv_pred_e"].abs()
    top = w_sorted.nlargest(5, "abs_dv")[
        ["timestamp_s", "imu_seq", "dv_pred_e", "vel_after_zupt_e"]
    ]

    verdict = {
        "question": "Who writes v_E between Accept #17 and Reject #18 (NHC-off)?",
        "interval_s": [T0, T1],
        "endpoints": {
            "v_E_after_accept_17": v_e_start,
            "v_E_at_reject_18_pred": v_e_end,
            "delta_v_E": delta_total,
            "innov_ve_at_18": float(r18["innov_ve_mps"]),
            "contrib_ve_at_18": float(r18["nis_contrib_ve"]),
        },
        "writers_in_interval": writers,
        "closure": {
            "sum_pipeline_dv_e": explained,
            "delta_v_E": delta_total,
            "unexplained_m_s": residual_unexplained,
            "note": "Interval starts AFTER accept #17; GNSS dx at #17 is outside the Δ",
        },
        "dominance": {
            "by_abs_sum": shares,
            "dominant_source": dominant,
            "dominant_share": shares[dominant],
            "PASS_single_function_ge_90pct": bool(shares[dominant] >= 0.90),
        },
        "n_imu_ticks": int(len(w)),
        "top5_abs_dv_pred_e_ticks": top.to_dict(orient="records"),
        "full_table_rows": len(rows),
    }

    # Write CSV chronology (full)
    pd.DataFrame(rows).to_csv(OUT / "ve_chronology.csv", index=False)
    # Human-readable compact markdown table
    lines = [
        "# v_E chronology — Accept #17 → Reject #18 (NHC-off)",
        "",
        f"| Tick | t (s) | Fuente | Δv_E (m/s) | v_E acumulada |",
        f"|------|-------|--------|------------|---------------|",
        f"| Accept #17 post-GNSS | {T0:.3f} | update_gnss | {float(a17['dx_vel_e_mps']):+.4f}* | {v_e_start:.4f} |",
    ]
    # subsample: first, every 20th, last, plus any |dv|>0.05
    for i, r in enumerate(w.itertuples(index=False), start=1):
        dv = float(r.dv_pred_e)
        show = i == 1 or i == len(w) or (i % 20 == 0) or abs(dv) >= 0.05
        if show:
            lines.append(
                f"| IMU #{i} | {float(r.timestamp_s):.3f} | predict | {dv:+.4f} | {float(r.vel_after_zupt_e):.4f} |"
            )
    lines += [
        f"| Reject #18 pre-GNSS | {T1:.3f} | state_at_gate | — | {v_e_end:.4f} |",
        "",
        f"\\* dx_vel_e del accept ya está aplicado al inicio del intervalo.",
        "",
        f"**Σ Δv_E predict en el segundo:** {sum_dv_pred:+.4f} m/s",
        f"**Σ Δv_E NHC / ZUPT:** {sum_dv_nhc:+.4f} / {sum_dv_zupt:+.4f}",
        f"**Δv_E neto (accept→reject pred):** {delta_total:+.4f} m/s",
        f"**Dominante:** `{dominant}` ({shares[dominant]*100:.1f}% del |Σ|)",
        f"**PASS ≥90% una sola función:** {shares[dominant] >= 0.90}",
        "",
        "Tabla completa: `ve_chronology.csv`",
    ]
    (OUT / "TABLE.md").write_text("\n".join(lines), encoding="utf-8")
    (OUT / "verdict.json").write_text(json.dumps(verdict, indent=2), encoding="utf-8")

    print(json.dumps(verdict, indent=2))
    print("->", OUT / "TABLE.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
