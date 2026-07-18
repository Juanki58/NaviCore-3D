#!/usr/bin/env python3
"""GAP-4 diagnostic synthesis — three closure figures (no new replays)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TREE = (
    REPO_ROOT
    / "docs/benchmarks/gap4_gnss_velocity/G1_intervention/ppv_divergence_tree.json"
)
DEFAULT_OUT = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G1_intervention"


def load_tree(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def short_node(n: dict) -> str:
    fix = n["gps_index"]
    acc = "A" if n["accepted"] else "R"
    pv = n["P_pv_frob_pre"]
    cp = n["cos_pos"]
    ct = n["cos_tot"]
    trig = n.get("ppv_triggered")
    trig_s = "—" if trig is None else str(trig)
    return (
        f"fix#{fix} [{acc}]\n"
        f"|Ppv|={pv:.1f}  trig={trig_s}\n"
        f"cos_pos={cp:+.2f}  cos_tot={ct:+.2f}"
    )


def fig_divergence_tree(tree: dict, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 12)
    ax.axis("off")
    ax.set_title("Fig 1 — Divergence tree (fix#4 bifurcation)", fontsize=14, fontweight="bold", pad=12)

    def box(x, y, text, w=2.6, h=1.05, fc="#eef2ff", ec="#4338ca", fontsize=8):
        p = FancyBboxPatch(
            (x - w / 2, y - h / 2),
            w,
            h,
            boxstyle="round,pad=0.04",
            linewidth=1.2,
            edgecolor=ec,
            facecolor=fc,
        )
        ax.add_patch(p)
        ax.text(x, y, text, ha="center", va="center", fontsize=fontsize, family="monospace")

    def arrow(x1, y1, x2, y2):
        ax.add_patch(
            FancyArrowPatch(
                (x1, y1),
                (x2, y2),
                arrowstyle="-|>",
                mutation_scale=12,
                linewidth=1.2,
                color="#64748b",
            )
        )

    box(5, 11, "shared trunk\n(1d == 1d' fix#2-3)", fc="#f0fdf4", ec="#15803d")
    trunk = tree["trunk_shared"][:2]
    y = 9.2
    prev_y = 11
    for n in trunk:
        box(5, y, short_node(n))
        arrow(5, prev_y - 0.55, 5, y + 0.55)
        prev_y = y
        y -= 2.0

    pre = tree["fix4_split"]["pre"]
    box(5, y, "fix#4 PRE (shared)\n" + short_node(pre).split("\n", 1)[1], fc="#fef9c3", ec="#ca8a04", w=3.0)
    arrow(5, prev_y - 0.55, 5, y + 0.55)
    split_y = y

    d4 = tree["fix4_split"]["1d_post"]
    dp4 = tree["fix4_split"]["1d_prime_post"]
    box(2.2, split_y - 2.2, "1d POST\nPpv->0\n" + short_node(d4).split("\n", 1)[1], fc="#dbeafe", ec="#2563eb")
    box(7.8, split_y - 2.2, "1d' POST\nPpv retained\n" + short_node(dp4).split("\n", 1)[1], fc="#fce7f3", ec="#db2777")
    arrow(5, split_y - 0.55, 2.2, split_y - 2.2 + 0.55)
    arrow(5, split_y - 0.55, 7.8, split_y - 2.2 + 0.55)

    y_d = split_y - 4.4
    y_p = split_y - 4.4
    for n in tree["branch_1d"]:
        box(2.2, y_d, short_node(n), w=2.8, fontsize=7)
        if n == tree["branch_1d"][0]:
            arrow(2.2, split_y - 2.2 - 0.55, 2.2, y_d + 0.55)
        else:
            arrow(2.2, y_d + 1.6, 2.2, y_d + 0.55)
        y_d -= 1.7

    for n in tree["branch_1d_prime"]:
        box(7.8, y_p, short_node(n), w=2.8, fontsize=7)
        if n == tree["branch_1d_prime"][0]:
            arrow(7.8, split_y - 2.2 - 0.55, 7.8, y_p + 0.55)
        else:
            arrow(7.8, y_p + 1.6, 7.8, y_p + 0.55)
        y_p -= 1.7

    ax.text(
        5,
        0.4,
        "After fix#4: two EKF instances — cos comparisons are trajectory comparisons, not policy-on-same-state.",
        ha="center",
        fontsize=9,
        color="#475569",
    )
    fig.tight_layout()
    fig.savefig(out, dpi=160, bbox_inches="tight")
    plt.close(fig)


def fig_causal_chain(out: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 11))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 14)
    ax.axis("off")
    ax.set_title("Fig 2 — Mechanistic causal chain (GAP-4 diagnostic model)", fontsize=14, fontweight="bold", pad=12)

    steps = [
        ("predict()", "#f8fafc", "#334155"),
        ("P (covariance)", "#eef2ff", "#4338ca"),
        ("P_pv block", "#fef9c3", "#ca8a04"),
        ("Δv_pos = K_vel,pos · y_pos", "#ffedd5", "#c2410c"),
        ("cos_pos / cos_tot\n(projection observables)", "#fce7f3", "#be185d"),
        ("gate 1d / 1d'\n(cos_pos>0 vs cos_tot>0)", "#dbeafe", "#1d4ed8"),
        ("P_pv post-update\n(zero or retain)", "#dcfce7", "#15803d"),
        ("future EKF trajectory\n(NHC, GNSS, Joseph, …)", "#f1f5f9", "#475569"),
    ]

    y = 13.0
    prev_cy = None
    for i, (label, fc, ec) in enumerate(steps):
        h = 1.15 if "\n" in label else 0.85
        p = FancyBboxPatch(
            (2.5, y - h / 2),
            5.0,
            h,
            boxstyle="round,pad=0.05",
            linewidth=1.4,
            edgecolor=ec,
            facecolor=fc,
        )
        ax.add_patch(p)
        ax.text(5, y, label, ha="center", va="center", fontsize=10)
        if prev_cy is not None:
            ax.add_patch(
                FancyArrowPatch(
                    (5, prev_cy - 0.45),
                    (5, y + h / 2 + 0.05),
                    arrowstyle="-|>",
                    mutation_scale=14,
                    linewidth=1.3,
                    color="#64748b",
                )
            )
        if i == 2:
            ax.add_patch(
                FancyArrowPatch(
                    (7.6, y),
                    (8.8, y - 2.5),
                    arrowstyle="-|>",
                    mutation_scale=12,
                    linewidth=1.0,
                    color="#94a3b8",
                    linestyle="dashed",
                )
            )
            ax.text(9.0, y - 2.5, "NHC compresses\nP_vv faster\nthan predict()", fontsize=8, va="center", color="#64748b")
        prev_cy = y
        y -= 1.55

    ax.text(
        5,
        0.6,
        "cos is a decision observable at fix#4; after bifurcation it co-evolves with |P_pv|/P_vv and nominal state.",
        ha="center",
        fontsize=9,
        color="#475569",
    )
    fig.tight_layout()
    fig.savefig(out, dpi=160, bbox_inches="tight")
    plt.close(fig)


def fig_scatter(tree: dict, out: Path) -> None:
    nodes = []
    for n in tree["branch_1d"]:
        nodes.append({**n, "series": "1d (post fix#4)"})
    for n in tree["branch_1d_prime"]:
        nodes.append({**n, "series": "1d' (post fix#4)"})
    nodes.append({**tree["fix4_split"]["pre"], "series": "fix#4 PRE (shared)"})

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    fig.suptitle("Fig 3 — |P_pv|/P_vv vs cos (filter-logged, post-bifurcation)", fontsize=13, fontweight="bold")

    colors = {"1d (post fix#4)": "#2563eb", "1d' (post fix#4)": "#db2777", "fix#4 PRE (shared)": "#ca8a04"}
    markers = {"1d (post fix#4)": "o", "1d' (post fix#4)": "s", "fix#4 PRE (shared)": "*"}

    for ax, cos_key, title in [
        (axes[0], "cos_pos", "cos_pos"),
        (axes[1], "cos_tot", "cos_tot"),
    ]:
        for series in colors:
            pts = [n for n in nodes if n["series"] == series and n.get("P_pv_over_P_vv") is not None]
            if not pts:
                continue
            xs = [p["P_pv_over_P_vv"] for p in pts]
            ys = [p[cos_key] for p in pts]
            ax.scatter(
                xs,
                ys,
                c=colors[series],
                marker=markers[series],
                s=80 if series.startswith("fix#4") else 55,
                label=series,
                edgecolors="white",
                linewidths=0.6,
                zorder=3,
            )
            for p in pts:
                ax.annotate(
                    f"#{p['gps_index']}",
                    (p["P_pv_over_P_vv"], p[cos_key]),
                    textcoords="offset points",
                    xytext=(4, 4),
                    fontsize=7,
                    color="#334155",
                )
        ax.axhline(0, color="#94a3b8", linewidth=0.8, linestyle="--")
        ax.set_xlabel("|P_pv| / P_vv (Frobenius, pre-update)")
        ax.set_ylabel(cos_key)
        ax.set_title(title)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)

    fig.text(
        0.5,
        0.02,
        "Not for discovery — synthesis only: cos and cross-covariance ratio co-evolve after fix#4 bifurcation.",
        ha="center",
        fontsize=9,
        color="#475569",
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.96))
    fig.savefig(out, dpi=160, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tree-json", type=Path, default=DEFAULT_TREE)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    tree = load_tree(args.tree_json)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    f1 = args.out_dir / "fig1_divergence_tree.png"
    f2 = args.out_dir / "fig2_causal_chain.png"
    f3 = args.out_dir / "fig3_ppv_ratio_vs_cos_scatter.png"

    fig_divergence_tree(tree, f1)
    fig_causal_chain(f2)
    fig_scatter(tree, f3)

    manifest = {
        "figures": {
            "fig1_divergence_tree": str(f1),
            "fig2_causal_chain": str(f2),
            "fig3_ppv_ratio_vs_cos_scatter": str(f3),
        },
        "source_tree": str(args.tree_json),
        "tag": "gap4-diagnostic-complete",
    }
    (args.out_dir / "diagnostic_synthesis_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    print(f"Wrote {f1}")
    print(f"Wrote {f2}")
    print(f"Wrote {f3}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
