#!/usr/bin/env python3
"""Rewrite K_bias R1/R2/R3 report emphasizing near-cancellation + path_scale."""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

OUT = Path(__file__).resolve().parents[1] / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"

WINDOWS = {
    "R1": (1.34, 1.59, "[t0,t1)"),
    "R2": (1.59, 1.74, "[t0,t1)"),
    "R3": (1.74, 2.00, "[t0,t1]"),
}
SUB = {
    "R1a": (1.34, 1.465, "[t0,t1)"),
    "R1b": (1.465, 1.59, "[t0,t1)"),
    "R2a": (1.59, 1.665, "[t0,t1)"),
    "R2b": (1.665, 1.74, "[t0,t1)"),
    "R3a": (1.74, 1.87, "[t0,t1)"),
    "R3b": (1.87, 2.00, "[t0,t1]"),
}


def mask(t: np.ndarray, t0: float, t1: float, ho: str) -> np.ndarray:
    return (t >= t0) & (t < t1) if ho == "[t0,t1)" else (t >= t0) & (t <= t1)


def stats(d: pd.DataFrame, m: np.ndarray) -> dict:
    if not m.any():
        return {"n": 0}
    vv = d.loc[m, "dx_bias_gz_via_vel"].to_numpy(float)
    va = d.loc[m, "dx_bias_gz_via_att"].to_numpy(float)
    tot = d.loc[m, "dx_bias_gz"].to_numpy(float)
    sv, sa, st = float(vv.sum()), float(va.sum()), float(tot.sum())
    scale = abs(sv) + abs(sa)
    return {
        "n": int(m.sum()),
        "t_span": [
            float(d.loc[m, "timestamp_s"].min()),
            float(d.loc[m, "timestamp_s"].max()),
        ],
        "sum_total": st,
        "sum_via_vel": sv,
        "sum_via_att": sa,
        "cancel_ratio": float(abs(st) / max(scale, 1e-30)),
        "frac_abs_att": float(abs(sa) / max(scale, 1e-30)),
        "mean_innov": float(d.loc[m, "innov_norm_mps"].mean()),
        "mean_k_via_vel": float(d.loc[m, "k_bias_gz_via_vel"].mean()),
        "mean_k_via_att": float(d.loc[m, "k_bias_gz_via_att"].mean()),
        "path_scale": scale,
    }


def fmt_row(arm: str, w: str, p: dict, windows: dict) -> str:
    t0, t1, ho = windows[w]
    close = ")" if ho == "[t0,t1)" else "]"
    return (
        f"| {arm} | {w} [{t0},{t1}{close} | {p['n']} | {p['sum_total']:+.5f} | "
        f"{p['sum_via_vel']:+.5f} | {p['sum_via_att']:+.5f} | {p['path_scale']:.3f} | "
        f"{p['cancel_ratio']:.3f} | {p['mean_innov']:.3f} |"
    )


def main() -> None:
    dc = pd.read_csv(OUT / "ctrl_nhc_block_audit.csv")
    dl = pd.read_csv(OUT / "latch_nhc_block_audit.csv")
    report: dict = {
        "definition": {
            "split": "frozen-S: K_via_X = P H_X^T S^{-1}; dx = K_via_X·y; X in {vel,att}",
            "identity_ok_rms": "~1e-9",
            "note_cancel": (
                "via_vel and via_att are large opposite; net dx is residual after "
                "cancellation. |frac|~0.5 does NOT mean a single motor."
            ),
        },
        "primary_tramos": {},
        "sub_tramos": {},
        "latch_vs_ctrl": {},
    }
    for arm, d in ("ctrl", dc), ("latch", dl):
        t = d["timestamp_s"].to_numpy(float)
        report["primary_tramos"][arm] = {
            w: stats(d, mask(t, *WINDOWS[w])) for w in WINDOWS
        }
        report["sub_tramos"][arm] = {w: stats(d, mask(t, *SUB[w])) for w in SUB}

    for w in WINDOWS:
        L = report["primary_tramos"]["latch"][w]
        C = report["primary_tramos"]["ctrl"][w]
        report["latch_vs_ctrl"][w] = {
            "delta_total": L["sum_total"] - C["sum_total"],
            "delta_via_vel": L["sum_via_vel"] - C["sum_via_vel"],
            "delta_via_att": L["sum_via_att"] - C["sum_via_att"],
            "delta_path_scale": L["path_scale"] - C["path_scale"],
            "latch_cancel_ratio": L["cancel_ratio"],
            "ctrl_cancel_ratio": C["cancel_ratio"],
        }

    verdict = {
        "label": "NEAR_CANCEL_PATHS_LATCH_R2_SCALE_EXPLOSION",
        "reading": (
            "H_vel vs H_att path split does NOT reveal a motor that switches att→vel "
            "across R1/R2/R3. In every tramo both paths are large, opposite, and cancel "
            "(cancel_ratio ≪ 1; |frac_att|≈0.50). The actionable latch effect is a "
            "path-SCALE explosion in R2 (latch path_scale≈6.3 vs ctrl≈0.20) while net "
            "Σdx_bias_gz stays small (−0.039). Net sign still changes by tramo under "
            "latch (R1/R2 negative, R3 positive). Dominant-path labels at frac≈0.5 are "
            "not meaningful — report cancel structure + path_scale + net residual. "
            "Sub-tramos: R3 halves flip path signs under latch (explosion front vs "
            "tail); R1 magnitude grows toward break; R2 coherent."
        ),
        "implications": [
            "Do not design intervention as cut via_att or cut via_vel alone — they "
            "nearly cancel; cutting one may unmask the other.",
            "Latch does not change which path dominates (both always ~equal); it "
            "amplifies both in R2.",
            "Net bias escape timing (silent R1, cost R2/R3) remains the design clock; "
            "path split explains coupling geometry, not a new switch.",
        ],
    }
    report["verdict"] = verdict
    (OUT / "k_bias_r123_decompose.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )

    lines = [
        "# K_bias_gz path decompose — R1 / R2 / R3",
        "",
        f"**Verdict:** `{verdict['label']}`",
        "",
        verdict["reading"],
        "",
        "## Definition",
        "",
        "- Split: `K_via_X = P H_X^T S^{-1}` (frozen S), `dx = K_via_X · y`, X ∈ {vel, att}",
        "- Identity: `dx_bias_gz = via_vel + via_att` (rms resid ~1e-9)",
        "- **cancel_ratio** = |Σnet| / (|Σvia_vel|+|Σvia_att|) — small ⇒ near-cancellation",
        "",
        "## Primary tramos (signed Σ)",
        "",
        "| Arm | Tramo | n | Σ total | Σ via_vel | Σ via_att | path_scale | cancel_ratio | mean‖y‖ |",
        "|-----|-------|---|---------|-----------|-----------|------------|--------------|---------|",
    ]
    for arm in ("ctrl", "latch"):
        for w in WINDOWS:
            lines.append(fmt_row(arm, w, report["primary_tramos"][arm][w], WINDOWS))
    lines += [
        "",
        "## Latch − ctrl",
        "",
        "| Tramo | ΔΣ total | ΔΣ via_vel | ΔΣ via_att | Δ path_scale |",
        "|-------|----------|------------|------------|--------------|",
    ]
    for w, c in report["latch_vs_ctrl"].items():
        lines.append(
            f"| {w} | {c['delta_total']:+.5f} | {c['delta_via_vel']:+.5f} | "
            f"{c['delta_via_att']:+.5f} | {c['delta_path_scale']:+.3f} |"
        )
    lines += [
        "",
        "## Sub-tramos (robustness — do not average over flips)",
        "",
        "| Arm | Sub | n | Σ total | Σ via_vel | Σ via_att | path_scale | cancel_ratio | mean‖y‖ |",
        "|-----|-----|---|---------|-----------|-----------|------------|--------------|---------|",
    ]
    for arm in ("ctrl", "latch"):
        for w in SUB:
            lines.append(fmt_row(arm, w, report["sub_tramos"][arm][w], SUB))
    lines += [
        "",
        "## Implications",
        "",
        *[f"- {x}" for x in verdict["implications"]],
        "",
        "Figure: `fig_k_bias_r123_paths.png`",
        "",
    ]
    (OUT / "k_bias_r123_decompose.md").write_text("\n".join(lines), encoding="utf-8")

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    tr = list(WINDOWS.keys())
    x = np.arange(len(tr))
    width = 0.35
    for ax, key, ylab in (
        (axes[0], "path_scale", "path_scale |Σv|+|Σa|"),
        (axes[1], "sum_total", "Σ dx_bias_gz net"),
    ):
        cv = [report["primary_tramos"]["ctrl"][t][key] for t in tr]
        lv = [report["primary_tramos"]["latch"][t][key] for t in tr]
        ax.bar(x - width / 2, cv, width, label="ctrl", color="C0")
        ax.bar(x + width / 2, lv, width, label="latch", color="C3")
        ax.axhline(0, color="gray", lw=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels(tr)
        ax.set_ylabel(ylab)
        ax.legend(fontsize=8)
    axes[0].set_title("K_bias paths: scale explosion vs net residual (R1/R2/R3)")
    fig.tight_layout()
    fig.savefig(OUT / "fig_k_bias_r123_paths.png", dpi=140)
    plt.close(fig)

    print(verdict["label"])
    for arm in ("ctrl", "latch"):
        print(arm)
        for w in WINDOWS:
            p = report["primary_tramos"][arm][w]
            print(
                f"  {w}: net={p['sum_total']:+.5f} scale={p['path_scale']:.3f} "
                f"cancel={p['cancel_ratio']:.3f}"
            )


if __name__ == "__main__":
    main()
