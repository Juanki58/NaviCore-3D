#!/usr/bin/env python3
"""GAP-3.17 / F1.1 — Anatomía del gate NIS: r, S, Λ=r/√S por componente.

Separa mecanismo observabilidad (P, K) vs innovación/estado nominal (r, S).
Compara baseline N=1, F1c N=10, F1d N=20, OFF.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
F1_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_f1_nhc_dose_response"
OUT_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_f1_nis_gate_anatomy"
REPORT_JSON = OUT_DIR / "gap3_f1_nis_gate_anatomy_report.json"
SUMMARY_MD = OUT_DIR / "gap3_f1_nis_gate_anatomy.md"
REJECT_CSV = OUT_DIR / "rejected_fixes_nis_anatomy.csv"

POLICIES = [
    {"label": "N=1", "subdir": "baseline", "nhc_n": 1},
    {"label": "N=10", "subdir": "F1c", "nhc_n": 10},
    {"label": "N=20", "subdir": "F1d", "nhc_n": 20},
    {"label": "OFF", "subdir": "OFF", "nhc_n": None},
]


def load_gnss(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, index_col=False)
    skip = {"reject_reason"}
    for c in df.columns:
        if c in skip or pd.api.types.is_numeric_dtype(df[c]):
            continue
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def enrich_gnss(df: pd.DataFrame) -> pd.DataFrame:
    out = df.copy()
    vn = out["vel_pred_n_mps"].astype(float)
    ve = out["vel_pred_e_mps"].astype(float)
    heading = np.arctan2(ve, vn)
    inn_n = out["innov_n_m"].astype(float)
    inn_e = out["innov_e_m"].astype(float)
    out["innov_long_m"] = inn_n * np.cos(heading) + inn_e * np.sin(heading)
    out["innov_lat_m"] = -inn_n * np.sin(heading) + inn_e * np.cos(heading)
    out["innov_vert_m"] = out["innov_d_m"].astype(float)
    out["innov_h_m"] = out["innov_h_m"].astype(float)

    for axis, innov_col, s_col in [
        ("n", "innov_n_m", "s_nn"),
        ("e", "innov_e_m", "s_ee"),
        ("d", "innov_d_m", "s_dd"),
    ]:
        s = out[s_col].astype(float).clip(lower=1e-9)
        r = out[innov_col].astype(float)
        out[f"Lambda_{axis}"] = r / np.sqrt(s)
        out[f"Lambda_{axis}_abs"] = np.abs(r) / np.sqrt(s)
        out[f"nis_margin_{axis}"] = out["nis_threshold"].astype(float) - out[f"nis_contrib_{axis}"].astype(float)

    out["nis_margin_total"] = out["nis_threshold"].astype(float) - out["nis_full"].astype(float)
    out["dominant_nis_axis"] = out[["nis_contrib_n", "nis_contrib_e", "nis_contrib_d"]].astype(float).idxmax(axis=1)
    out["dominant_nis_axis"] = out["dominant_nis_axis"].str.replace("nis_contrib_", "")
    return out


def first_reject_index(df: pd.DataFrame) -> int | None:
    rej = df[df["accepted"] == 0]
    return int(rej.iloc[0]["gps_index"]) if len(rej) else None


def analyze_policy(label: str, df: pd.DataFrame) -> dict:
    acc = df[df["accepted"] == 1]
    rej = df[df["accepted"] == 0]
    first_rej = first_reject_index(df)

    def _stats(sub: pd.DataFrame, prefix: str) -> dict:
        if sub.empty:
            return {}
        return {
            f"{prefix}_count": int(len(sub)),
            f"{prefix}_innov_h_mean": float(sub["innov_h_m"].mean()),
            f"{prefix}_Lambda_n_abs_mean": float(sub["Lambda_n_abs"].mean()),
            f"{prefix}_Lambda_e_abs_mean": float(sub["Lambda_e_abs"].mean()),
            f"{prefix}_Lambda_d_abs_mean": float(sub["Lambda_d_abs"].mean()),
            f"{prefix}_s_nn_mean": float(sub["s_nn"].mean()),
            f"{prefix}_s_ee_mean": float(sub["s_ee"].mean()),
            f"{prefix}_s_dd_mean": float(sub["s_dd"].mean()),
            f"{prefix}_nis_full_mean": float(sub["nis_full"].mean()),
            f"{prefix}_k_vel_mean": float(sub["k_vel_max"].mean()),
        }

    # Last accepted before gate closes
    last_acc_idx = int(acc.iloc[-1]["gps_index"]) if len(acc) else None

    # Transition: last accept vs first reject
    transition = {}
    if last_acc_idx and first_rej:
        la = df[df["gps_index"] == last_acc_idx].iloc[0]
        fr = df[df["gps_index"] == first_rej].iloc[0]
        transition = {
            "last_accept_gps_index": last_acc_idx,
            "first_reject_gps_index": first_rej,
            "last_accept_nis": float(la["nis_full"]),
            "first_reject_nis": float(fr["nis_full"]),
            "last_accept_innov_h": float(la["innov_h_m"]),
            "first_reject_innov_h": float(fr["innov_h_m"]),
            "last_accept_Lambda_n_abs": float(la["Lambda_n_abs"]),
            "first_reject_Lambda_n_abs": float(fr["Lambda_n_abs"]),
            "last_accept_s_nn": float(la["s_nn"]),
            "first_reject_s_nn": float(fr["s_nn"]),
            "last_accept_k_vel": float(la["k_vel_max"]),
            "first_reject_k_vel": float(fr["k_vel_max"]),
            "first_reject_dominant_axis": str(fr["dominant_nis_axis"]),
            "first_reject_contrib_n": float(fr["nis_contrib_n"]),
            "first_reject_contrib_e": float(fr["nis_contrib_e"]),
            "first_reject_contrib_d": float(fr["nis_contrib_d"]),
        }

    return {
        "label": label,
        "gnss_accept_count": int(len(acc)),
        "gnss_reject_count": int(len(rej)),
        "first_reject_gps_index": first_rej,
        "last_accept_gps_index": last_acc_idx,
        **_stats(acc, "accepted"),
        **_stats(rej, "rejected"),
        "transition": transition,
    }


def rejected_table(df: pd.DataFrame, policy: str) -> pd.DataFrame:
    rej = df[df["accepted"] == 0].copy()
    cols = [
        "policy",
        "gps_index",
        "timestamp_s",
        "nis_full",
        "nis_threshold",
        "nis_margin_total",
        "innov_h_m",
        "innov_long_m",
        "innov_lat_m",
        "innov_vert_m",
        "innov_n_m",
        "innov_e_m",
        "innov_d_m",
        "s_nn",
        "s_ee",
        "s_dd",
        "Lambda_n",
        "Lambda_e",
        "Lambda_d",
        "Lambda_n_abs",
        "Lambda_e_abs",
        "Lambda_d_abs",
        "nis_contrib_n",
        "nis_contrib_e",
        "nis_contrib_d",
        "dominant_nis_axis",
        "k_vel_max",
        "k_pos_max",
        "pred_error_3d_m",
        "vel_pred_h_mps",
    ]
    rej.insert(0, "policy", policy)
    return rej[[c for c in cols if c in rej.columns]]


def plot_comparison(enriched: dict[str, pd.DataFrame], stats: list[dict]) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    colors = {"N=1": "C0", "N=10": "C1", "N=20": "C2", "OFF": "C3"}

    for label, df in enriched.items():
        c = colors.get(label, "gray")
        idx = df["gps_index"].astype(float)
        axes[0, 0].plot(idx, df["Lambda_n_abs"], "o-", ms=3, lw=1, color=c, label=label, alpha=0.8)
        axes[0, 1].plot(idx, df["innov_h_m"], "o-", ms=3, lw=1, color=c, label=label, alpha=0.8)
        axes[0, 2].plot(idx, df["s_nn"], "o-", ms=3, lw=1, color=c, label=label, alpha=0.8)
        axes[1, 0].plot(idx, df["nis_full"], "o-", ms=3, lw=1, color=c, label=label, alpha=0.8)
        axes[1, 1].plot(idx, df["k_vel_max"], "o-", ms=3, lw=1, color=c, label=label, alpha=0.8)
        acc_mask = df["accepted"] == 1
        axes[1, 2].scatter(
            df.loc[acc_mask, "gps_index"],
            df.loc[acc_mask, "Lambda_n_abs"],
            s=20,
            c=c,
            marker="o",
            alpha=0.8,
        )
        axes[1, 2].scatter(
            df.loc[~acc_mask, "gps_index"],
            df.loc[~acc_mask, "Lambda_n_abs"],
            s=30,
            c=c,
            marker="x",
            alpha=0.9,
        )

    for ax, title, ylabel in [
        (axes[0, 0], "|Λ_N| = |r_N|/√S_NN", "|Λ|"),
        (axes[0, 1], "innov_h [m]", "m"),
        (axes[0, 2], "S_NN [m²]", "m²"),
        (axes[1, 0], "NIS total", "χ²"),
        (axes[1, 1], "k_vel_max", "gain"),
        (axes[1, 2], "|Λ_N| accepts(o) rejects(x)", "|Λ|"),
    ]:
        ax.axhline(11.345, color="gray", ls=":", lw=1, alpha=0.5) if "NIS" in title else None
        ax.set_title(title, fontsize=9)
        ax.grid(True, alpha=0.3)
        if ax is axes[0, 0]:
            ax.legend(fontsize=7, loc="upper left")

    fig.suptitle("F1.1 — Gate anatomy: r, S, Λ across NHC policies", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "f1_nis_gate_anatomy.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    # N=20 paradox panel: first 15 rejects contrib
    fig2, ax = plt.subplots(figsize=(10, 4))
    base = enriched["N=1"]
    n10 = enriched["N=10"]
    n20 = enriched["N=20"]
    for label, df, color in [("N=1", base, "C0"), ("N=10", n10, "C1"), ("N=20", n20, "C2")]:
        sub = df[(df["gps_index"] >= 6) & (df["gps_index"] <= 15)]
        ax.plot(sub["gps_index"], sub["nis_contrib_n"], "o-", ms=4, color=color, label=f"{label} contrib_N")
    ax.axhline(11.345, color="gray", ls="--", label="threshold")
    ax.set_xlabel("gps_index")
    ax.set_ylabel("NIS contrib / NIS total")
    ax.set_title("First rejects: N-axis NIS contribution (gate bottleneck)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fig2.savefig(OUT_DIR / "f1_first_rejects_contrib_n.png", dpi=150)
    plt.close(fig2)


def write_summary(stats: list[dict], enriched: dict[str, pd.DataFrame]) -> None:
    lines = [
        "# GAP-3.17 — F1.1 Anatomía del gate NIS",
        "",
        "## Cadena tras F1",
        "",
        "| Eslabón | Estado |",
        "|---------|--------|",
        "| NHC → P_vv | ✅ confirmado |",
        "| P_vv → k_vel | ✅ confirmado |",
        "| k_vel → accepts | ❌ **no confirmado** (N=10: k_vel×12, accepts=7) |",
        "",
        "**Cuello de botella:** innovación / estado nominal (r), no K solo.",
        "",
        "## Transición último accept → primer reject",
        "",
        "| Policy | acc# | rej# | NIS_acc | NIS_rej | innov_h_acc | innov_h_rej | |Λ_N|_rej | S_NN_rej | k_vel_rej | dom axis |",
        "|--------|-----:|-----:|--------:|--------:|------------:|------------:|---------:|---------:|----------:|---------|",
    ]
    for s in stats:
        t = s.get("transition") or {}
        if not t:
            continue
        lines.append(
            f"| {s['label']} | {t.get('last_accept_gps_index','')} | {t.get('first_reject_gps_index','')} | "
            f"{t.get('last_accept_nis',0):.1f} | {t.get('first_reject_nis',0):.1f} | "
            f"{t.get('last_accept_innov_h',0):.1f} | {t.get('first_reject_innov_h',0):.1f} | "
            f"{t.get('first_reject_Lambda_n_abs',0):.2f} | {t.get('first_reject_s_nn',0):.0f} | "
            f"{t.get('first_reject_k_vel',0):.3f} | {t.get('first_reject_dominant_axis','')} |"
        )

    lines += [
        "",
        "## Paradoja N=10 vs N=20",
        "",
        "N=20 tiene **más** P_vv pre#3 (78 vs 22) y **más** k_vel (0.24 vs 0.09) pero **menos** accepts (5 vs 7).",
        "Estado nominal peor (innovación mayor) compite con mejor S — el gate ve rᵀS⁻¹r.",
        "",
        "## Rejects gps_index 8–14 (N=1) — descomposición NIS",
        "",
        "| fix | NIS | contrib_N | contrib_E | contrib_D | dom | |Λ_N| | innov_h |",
        "|-----|----:|----------:|----------:|----------:|-----|------:|--------:|",
    ]
    rej = enriched["N=1"]
    for _, r in rej[(rej["gps_index"] >= 8) & (rej["gps_index"] <= 14)].iterrows():
        lines.append(
            f"| {int(r['gps_index'])} | {r['nis_full']:.0f} | {r['nis_contrib_n']:.0f} | "
            f"{r['nis_contrib_e']:.1f} | {r['nis_contrib_d']:.1f} | {r['dominant_nis_axis']} | "
            f"{r['Lambda_n_abs']:.2f} | {r['innov_h_m']:.1f} |"
        )

    lines += [
        "",
        "**Patrón:** contrib_N domina en rejects tempranos (fix 8–14); eje N crece monotónicamente.",
        "No es crecimiento homogéneo — es **error nominal en N** mientras S_NN también cae.",
        "",
    ]
    SUMMARY_MD.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--f1-dir", type=Path, default=F1_DIR)
    args = parser.parse_args()
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    enriched: dict[str, pd.DataFrame] = {}
    stats: list[dict] = []
    reject_frames = []

    for pol in POLICIES:
        path = args.f1_dir / pol["subdir"] / "gnss_nis_audit.csv"
        if not path.is_file():
            print(f"WARN: missing {path}")
            continue
        df = enrich_gnss(load_gnss(path))
        enriched[pol["label"]] = df
        stats.append(analyze_policy(pol["label"], df))
        reject_frames.append(rejected_table(df, pol["label"]))

    if reject_frames:
        pd.concat(reject_frames, ignore_index=True).to_csv(REJECT_CSV, index=False)

    # N=20 paradox detail
    paradox = {}
    if "N=10" in enriched and "N=20" in enriched:
        for label in ("N=10", "N=20"):
            df = enriched[label]
            acc = df[df["accepted"] == 1]
            rej = df[df["accepted"] == 0]
            paradox[label] = {
                "accepts": int(len(acc)),
                "first_reject": first_reject_index(df),
                "first_reject_nis": float(rej.iloc[0]["nis_full"]) if len(rej) else None,
                "first_reject_innov_n": float(rej.iloc[0]["innov_n_m"]) if len(rej) else None,
                "first_reject_s_nn": float(rej.iloc[0]["s_nn"]) if len(rej) else None,
                "first_reject_Lambda_n_abs": float(rej.iloc[0]["Lambda_n_abs"]) if len(rej) else None,
            }

    report = {
        "experiment": "GAP-3.17 F1.1 NIS gate anatomy",
        "metric_Lambda": "r / sqrt(S_ii) per axis; gate uses r^T S^-1 r",
        "policies": stats,
        "n20_paradox": paradox,
        "verdict": (
            "GATE_LIMITED_BY_INNOVATION_NOT_K_ALONE: restoring k_vel via NHC decimation "
            "does not reopen accepts; first rejects dominated by N-axis NIS contrib and growing |Lambda_n| "
            "while S_nn also shrinks — coupled r/S effect, not monotonic P→accepts."
        ),
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    plot_comparison(enriched, stats)
    write_summary(stats, enriched)

    print(json.dumps(report, indent=2))
    print(f"\nWrote {REPORT_JSON}")
    print(f"Wrote {REJECT_CSV}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
