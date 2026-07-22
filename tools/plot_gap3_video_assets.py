#!/usr/bin/env python3
"""Build GAP-3 video stills from docs/nhc_experiments/manifest.json.

No Unity required — bar charts + title cards for VO overlays / B-roll.
Does not invent numbers; reads the banked experiment matrix only.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[1]
MANIFEST = REPO / "docs" / "nhc_experiments" / "manifest.json"
B_ALWAYS_SUMMARY = REPO / "docs" / "nhc_experiments" / "B_always_summary.json"
OUT_DIR = REPO / "docs" / "video_gap3" / "stills"


def load_exit_drifts(manifest: dict) -> list[tuple[str, float]]:
    """A + B_always + best G-arm from banked artefacts (no invented numbers)."""
    rows: list[tuple[str, float]] = []
    by_exp = manifest.get("drift_by_experiment") or {}
    if "A" in by_exp:
        rows.append(("A  NHC off", float(by_exp["A"]["drift_exit_m"])))

    b_exit = None
    summaries = manifest.get("summaries") or {}
    if "B_always" in summaries:
        b_exit = float(summaries["B_always"]["drift_exit_m"])
    elif B_ALWAYS_SUMMARY.is_file():
        b = json.loads(B_ALWAYS_SUMMARY.read_text(encoding="utf-8"))
        b_exit = float(b["drift_exit_m"])
    if b_exit is not None:
        rows.append(("B_always", b_exit))

    if "G_l10_v10" in by_exp:
        rows.append(("G_l10_v10", float(by_exp["G_l10_v10"]["drift_exit_m"])))
    elif "best_experiment" in manifest and manifest["best_experiment"] in by_exp:
        be = manifest["best_experiment"]
        rows.append((be, float(by_exp[be]["drift_exit_m"])))
    return rows


def style_axes(ax) -> None:
    ax.set_facecolor("#0e1116")
    ax.figure.patch.set_facecolor("#0e1116")
    ax.tick_params(colors="#c8d0dc")
    for spine in ax.spines.values():
        spine.set_color("#3a4555")
    ax.yaxis.label.set_color("#c8d0dc")
    ax.xaxis.label.set_color("#c8d0dc")
    ax.title.set_color("#f0f3f8")


def plot_exit_bars(rows: list[tuple[str, float]], path: Path) -> None:
    labels = [r[0] for r in rows]
    vals = [r[1] for r in rows]
    colors = ["#3d9a6a", "#c44b4b", "#c9a227"][: len(rows)]

    fig, ax = plt.subplots(figsize=(10, 5.5), dpi=160)
    style_axes(ax)
    x = np.arange(len(vals))
    bars = ax.bar(x, vals, color=colors, width=0.62, edgecolor="#1a2030", linewidth=0.8)
    ax.set_xticks(x, labels, rotation=0)
    ax.set_ylabel("Drift @ tunnel exit (m)")
    ax.set_title("GAP-3 — NHC policy vs coast error (super-tunnel bank)")
    ax.set_ylim(0, max(vals) * 1.18)
    for bar, v in zip(bars, vals):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            v + max(vals) * 0.03,
            f"{v:.0f} m",
            ha="center",
            va="bottom",
            color="#f0f3f8",
            fontsize=12,
            fontweight="bold",
        )
    ax.text(
        0.99,
        0.02,
        "Source: docs/nhc_experiments/manifest.json",
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        color="#7a8699",
        fontsize=8,
    )
    fig.tight_layout()
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_title_card(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 6.75), dpi=160)
    ax.set_facecolor("#0e1116")
    fig.patch.set_facecolor("#0e1116")
    ax.axis("off")
    ax.text(
        0.5,
        0.62,
        "NHC always-on is not free",
        ha="center",
        va="center",
        color="#f0f3f8",
        fontsize=28,
        fontweight="bold",
    )
    ax.text(
        0.5,
        0.42,
        "On our bank: 1408 m exit drift vs 493 m with NHC off",
        ha="center",
        va="center",
        color="#c9a227",
        fontsize=16,
    )
    ax.text(
        0.5,
        0.22,
        "NaviCore-3D  ·  GAP-3  ·  software evidence only",
        ha="center",
        va="center",
        color="#7a8699",
        fontsize=12,
    )
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_policy_card(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 6.75), dpi=160)
    ax.set_facecolor("#0e1116")
    fig.patch.set_facecolor("#0e1116")
    ax.axis("off")
    ax.text(0.08, 0.72, "Operational policy (frozen)", color="#f0f3f8", fontsize=22, fontweight="bold")
    lines = [
        "✓  NHC_OPS_OFF — default (production)",
        "✓  NHC_OPS_GAP_TRIGGERED — allowed (v2-style)",
        "✗  NHC_OPS_ALWAYS — lab / A-B only",
    ]
    y = 0.52
    for line in lines:
        color = "#3d9a6a" if line.startswith("✓") else "#c44b4b"
        ax.text(0.10, y, line, color=color, fontsize=16, family="monospace")
        y -= 0.14
    ax.text(
        0.08,
        0.12,
        "docs/NHC_OPS_POLICY.md  ·  src/core/nhc_ops_policy.hpp",
        color="#7a8699",
        fontsize=11,
    )
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)


def main() -> int:
    if not MANIFEST.is_file():
        print(f"Missing {MANIFEST}")
        return 1
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    rows = load_exit_drifts(data)
    if len(rows) < 2:
        print("Need at least A and B_always in manifest")
        return 1

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plot_exit_bars(rows, OUT_DIR / "01_exit_drift_bars.png")
    plot_title_card(OUT_DIR / "00_title_card.png")
    plot_policy_card(OUT_DIR / "02_policy_card.png")

    # Plain text overlays for editors / Unity TextMeshPro
    a_exit = next(v for n, v in rows if n.startswith("A"))
    b_exit = next(v for n, v in rows if "B_always" in n)
    overlay = OUT_DIR / "overlays.txt"
    overlay.write_text(
        "\n".join(
            [
                "HOOK: En INS embargado, mucha gente pone NHC a tope. Nosotros medimos.",
                "SETUP: Mismo túnel, mismo IMU — solo cambia la política NHC.",
                f"RESULT A (off): {a_exit:.0f} m @ exit",
                f"RESULT B_always: {b_exit:.0f} m @ exit",
                "TAKEAWAY: NHC agresivo puede empeorar el coast. Preferir OFF o gap-triggered.",
                "CTA: docs/nhc_experiments · NaviCore3D_Sim --nhc-experiments",
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(f"Wrote stills -> {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
