#!/usr/bin/env python3
"""Tick autopsy for super_tunnel_bd_rerun (protocol §3–§4)."""

from __future__ import annotations

import json
from pathlib import Path

import pandas as pd

OUT = Path(__file__).resolve().parents[1] / "docs" / "benchmarks" / "super_tunnel_bd_rerun"
REPORT = OUT / "autopsy.md"


def arm_autopsy(arm: str) -> str:
    df = pd.read_csv(OUT / f"{arm}_anatomy.csv")
    df["d_drift"] = df["drift_h_m"].diff()
    df["d_pvv"] = df["P_vv_frob"].diff()
    df["d_pvv_rel"] = df["d_pvv"] / df["P_vv_frob"].shift(1).clip(lower=1e-9)

    # Ignore GPS-reacquire cliff at outage end (expected, not NHC gain spike).
    during = df[(df["t_ms"] >= 10000) & (df["t_ms"] < 55000)].copy()
    jumps = during[(during["d_drift"].abs() > 5) | (during["d_pvv_rel"].abs() > 0.5)]

    lines = [f"## {arm}", ""]
    exit_row = df.loc[df["t_ms"] == 55000]
    exit_drift = float(exit_row["drift_h_m"].iloc[0]) if len(exit_row) else float("nan")
    lines.append(
        f"- rows={len(df)} nhc_ticks={int(df['nhc_applied'].sum())} "
        f"drift_exit@55s={exit_drift:.2f} m final={df['drift_h_m'].iloc[-1]:.2f} m"
    )

    t0 = df.loc[df["t_ms"] == 0]
    t10 = df.loc[df["t_ms"] == 10000]
    if len(t0) and len(t10):
        lines.append(
            f"- P_vv_frob 0s→10s: {t0['P_vv_frob'].iloc[0]:.4e} → {t10['P_vv_frob'].iloc[0]:.4e}"
        )
        lines.append(
            f"- P_pv_frob 0s→10s: {t0['P_pv_frob'].iloc[0]:.4e} → {t10['P_pv_frob'].iloc[0]:.4e}"
        )
        lines.append(
            f"- drift_h 0s→10s: {t0['drift_h_m'].iloc[0]:.3f} → {t10['drift_h_m'].iloc[0]:.3f} m"
        )

    lines.append(
        f"- outage single-tick jumps (|Δdrift|>5 or |ΔP_vv_rel|>0.5): **{len(jumps)}**"
    )
    if len(jumps):
        cols = [
            "t_ms",
            "nhc_applied",
            "drift_h_m",
            "d_drift",
            "P_vv_frob",
            "d_pvv_rel",
            "dx_pos_norm",
            "k_max",
            "innov_norm",
        ]
        lines.append("```")
        lines.append(jumps[cols].head(12).to_string(index=False))
        lines.append("```")

    nhc = during[during["nhc_applied"] == 1]
    if len(nhc):
        i = nhc["dx_pos_norm"].idxmax()
        r = df.loc[i]
        lines.append(
            f"- max dx_pos_norm in outage NHC: {r['dx_pos_norm']:.4f} m "
            f"@t={int(r['t_ms'])} ms k_max={r['k_max']:.3f} "
            f"innov={r['innov_norm']:.3f} drift={r['drift_h_m']:.2f}"
        )
        top = nhc.reindex(nhc["d_drift"].abs().sort_values(ascending=False).index).head(5)
        lines.append("- top |Δdrift_h| among outage NHC ticks:")
        lines.append("```")
        lines.append(
            top[
                [
                    "t_ms",
                    "d_drift",
                    "drift_h_m",
                    "dx_pos_norm",
                    "k_max",
                    "innov_norm",
                    "P_vv_frob",
                ]
            ].to_string(index=False)
        )
        lines.append("```")

        # Early cascade in first 2 s of outage
        first2 = nhc[nhc["t_ms"] <= 12000]
        if len(first2):
            lines.append(
                f"- first 2 s outage: drift {first2['drift_h_m'].iloc[0]:.2f}→"
                f"{first2['drift_h_m'].iloc[-1]:.2f} m; "
                f"max|d_drift|={first2['d_drift'].abs().max():.3f}; "
                f"max k_max={first2['k_max'].max():.3f}; "
                f"max innov={first2['innov_norm'].max():.3f}"
            )

    # Pre-outage damage (N_always)
    pre = df[(df["t_ms"] < 10000) & (df["nhc_applied"] == 1)]
    if len(pre):
        lines.append(
            f"- pre-outage NHC ticks={len(pre)} drift@10s={t10['drift_h_m'].iloc[0]:.3f} m "
            f"max innov_pre={pre['innov_norm'].max():.3f} max|d_drift|_pre={pre['d_drift'].abs().max():.3f}"
        )

    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parts = [
        "# Autopsia tick-a-tick — super_tunnel_bd_rerun",
        "",
        "Protocolo: `docs/diagnostics/16-super-tunnel-ieee952-rerun-protocol.md` §3–§4.",
        "Salto a t≈55010 ms = reaparición GNSS (esperado); excluido del conteo de jumps en outage.",
        "",
    ]
    for arm in ["A", "A_dirty", "B", "B_dirty", "N_always", "N_always_dirty"]:
        parts.append(arm_autopsy(arm))

    verdict = json.loads((OUT / "verdict.json").read_text(encoding="utf-8"))
    parts.append("## Lectura causal (post-autopsia)")
    parts.append("")
    parts.append(
        f"- Overall preregistrado: **{verdict['overall_verdict']}** "
        f"(panel_B={verdict['panel_B_constant_vel']['verdict']}, "
        f"panel_N={verdict['panel_N_always']['verdict']})."
    )
    parts.append(
        "- NHC empeora con IMU **ideal** (D1): no atribuir a sesgo IEEE-952; "
        "el daño limpio es del propio update NHC / acoplamiento."
    )
    parts.append(
        "- Panel N_always: Δ_dirty < Δ_clean (dirty no empeora más que ideal) — "
        "opuesto a la hipótesis IEEE-952."
    )
    parts.append(
        "- Baseline A actual ~299 m ≠ histórico ~481 m: binario distinto (protocolo §0); "
        "comparar deltas, no anclar al 481 absoluto."
    )
    parts.append("")

    REPORT.write_text("\n".join(parts), encoding="utf-8")
    print(REPORT.read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
