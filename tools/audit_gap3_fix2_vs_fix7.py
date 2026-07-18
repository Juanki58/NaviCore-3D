#!/usr/bin/env python3
"""GAP-3.11 — Autopsia comparativa GNSS fix #2 vs #7 (exp B).

Dos estados del mismo sistema: innov ~ mismo orden, cond(S) bueno,
k_vel cae ~10×. Reconstruye P(t), ΔP/P, |K·innov| vs |innov|, k_vel/P_vv.
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
AUTOPSY_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_gnss_accepted_autopsy"
OUT_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_fix2_vs_fix7"
REPORT_JSON = OUT_DIR / "gap3_fix2_vs_fix7_report.json"

PRE_S = 2.0
POST_S = 0.5

FIXES = {
    "fix2": {"label": "Fix #2", "gps_index": 2, "color": "#1f77b4"},
    "fix7": {"label": "Fix #7", "gps_index": 7, "color": "#ff7f0e"},
}


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, index_col=False)
    skip = {"update_type", "phase", "event", "constraint_policy", "source"}
    for col in df.columns:
        if col in skip:
            continue
        converted = pd.to_numeric(df[col], errors="coerce")
        if converted.notna().any():
            df[col] = converted
    return df


def enrich_nhc(nhc: pd.DataFrame) -> pd.DataFrame:
    df = nhc.copy()
    df["dx_total_norm"] = np.sqrt(
        df["dx_vel_norm_mps"].fillna(0) ** 2
        + df["dx_pos_norm_m"].fillna(0) ** 2
        + df["dx_att_norm_rad"].fillna(0) ** 2
        + df["dx_bias_norm"].fillna(0) ** 2
    )
    df["k_innov_energy"] = df["dx_total_norm"]
    pre_vv = df["P_pre_vv_frob"].replace(0, np.nan)
    pre_pv = df["P_pre_pv_frob"].replace(0, np.nan)
    pre_aa = df["P_pre_aa_frob"].replace(0, np.nan)
    df["delta_P_vv_rel"] = df["delta_P_vv_frob"].abs() / pre_vv
    df["delta_P_pv_rel"] = df["delta_P_pv_frob"].abs() / pre_pv
    df["delta_P_aa_rel"] = df["delta_P_aa_frob"].abs() / pre_aa
    df["t_rel_fix"] = np.nan
    return df


def window(df: pd.DataFrame, t_fix: float) -> pd.DataFrame:
    return df[(df["timestamp_s"] >= t_fix - PRE_S) & (df["timestamp_s"] <= t_fix + POST_S)].copy()


def gnss_row(gnss: pd.DataFrame, gps_index: int) -> pd.Series:
    rows = gnss[(gnss["accepted"] == 1) & (gnss["gps_index"] == gps_index)]
    if rows.empty:
        raise ValueError(f"accepted fix gps_index={gps_index} not found")
    return rows.iloc[0]


def cov_at_gnss(cov_step: pd.DataFrame, t_fix: float) -> dict:
    gnss = cov_step[cov_step["update_type"] == "gnss"]
    pre = gnss[(gnss["phase"] == "pre") & (np.isclose(gnss["timestamp_s"], t_fix, atol=0.05))]
    post = gnss[(gnss["phase"] == "post_accept") & (np.isclose(gnss["timestamp_s"], t_fix, atol=0.05))]
    out = {}
    if len(pre):
        r = pre.iloc[-1]
        out["pre"] = {
            "P_pp_frob": float(r["P_pp_frob"]),
            "P_vv_frob": float(r["P_vv_frob"]),
            "P_pv_frob": float(r["P_pv_frob"]),
            "P_aa_frob": float(r["P_aa_frob"]),
        }
    if len(post):
        r = post.iloc[-1]
        out["post"] = {
            "P_pp_frob": float(r["P_pp_frob"]),
            "P_vv_frob": float(r["P_vv_frob"]),
            "P_pv_frob": float(r["P_pv_frob"]),
            "P_aa_frob": float(r["P_aa_frob"]),
        }
    return out


def build_staircase(nhc_win: pd.DataFrame) -> pd.DataFrame:
    """P pre/post por update NHC — escalera predict→NHC."""
    rows = []
    prev_post_vv = np.nan
    for _, r in nhc_win.iterrows():
        pre_vv = float(r["P_pre_vv_frob"])
        post_vv = float(r["P_post_vv_frob"])
        rows.append(
            {
                "timestamp_s": float(r["timestamp_s"]),
                "predict_jump_vv": pre_vv - prev_post_vv if not math.isnan(prev_post_vv) else np.nan,
                "P_pre_vv": pre_vv,
                "P_post_vv": post_vv,
                "P_pre_pv": float(r["P_pre_pv_frob"]),
                "P_post_pv": float(r["P_post_pv_frob"]),
                "P_pre_aa": float(r["P_pre_aa_frob"]),
                "P_post_aa": float(r["P_post_aa_frob"]),
                "delta_P_vv_rel": float(r["delta_P_vv_rel"]),
                "innov_norm": float(r["innov_norm_mps"]),
                "k_innov_energy": float(r["k_innov_energy"]),
                "k_vel_max": float(r["k_vel_max"]),
            }
        )
        prev_post_vv = post_vv
    return pd.DataFrame(rows)


def summarize_fix(
    key: str,
    gnss: pd.DataFrame,
    nhc: pd.DataFrame,
    cov_step: pd.DataFrame,
    k_blocks: list[dict],
) -> dict:
    meta = FIXES[key]
    row = gnss_row(gnss, meta["gps_index"])
    t_fix = float(row["timestamp_s"])
    nhc_all = enrich_nhc(nhc)
    nhc_win = window(nhc_all, t_fix)
    nhc_win["t_rel_fix"] = nhc_win["timestamp_s"] - t_fix
    stairs = build_staircase(nhc_win)

    cov = cov_at_gnss(cov_step, t_fix)
    p_vv_pre = cov.get("pre", {}).get("P_vv_frob", np.nan)
    k_vel = float(row["k_vel_max"])
    k_block = next((k for k in k_blocks if abs(k.get("timestamp_s", -1) - t_fix) < 0.05), None)

    return {
        "key": key,
        "label": meta["label"],
        "gps_index": meta["gps_index"],
        "timestamp_s": t_fix,
        "gnss_at_fix": {
            "innov_h_m": float(row["innov_h_m"]),
            "nis_horizontal_2d": float(row["nis_horizontal_2d"]),
            "s_cond": float(row.get("s_cond", np.nan)),
            "k_vel_max": k_vel,
            "k_pos_max": float(row["k_pos_max"]),
            "P_vv_pre_gnss": p_vv_pre,
            "k_vel_over_P_vv": (k_vel / p_vv_pre) if p_vv_pre and p_vv_pre > 0 else None,
            "K_vel_pos_frob": float(np.linalg.norm(k_block["K_vel_pos"], ord="fro"))
            if k_block and "K_vel_pos" in k_block
            else None,
        },
        "cov_gnss": cov,
        "pre_window_nhc": {
            "n_updates": int(len(nhc_win)),
            "sum_innov_norm": float(nhc_win["innov_norm_mps"].sum()),
            "sum_k_innov_energy": float(nhc_win["k_innov_energy"].sum()),
            "sum_abs_delta_P_vv_frob": float(nhc_win["delta_P_vv_frob"].abs().sum()),
            "mean_delta_P_vv_rel": float(nhc_win["delta_P_vv_rel"].mean()),
            "median_delta_P_vv_rel": float(nhc_win["delta_P_vv_rel"].median()),
            "P_vv_last_nhc_pre": float(nhc_win.iloc[-1]["P_pre_vv_frob"]) if len(nhc_win) else None,
            "P_vv_last_nhc_post": float(nhc_win.iloc[-1]["P_post_vv_frob"]) if len(nhc_win) else None,
            "predict_jump_mean_vv": float(stairs["predict_jump_vv"].mean(skipna=True))
            if len(stairs)
            else None,
        },
        "staircase": stairs.to_dict(orient="records"),
    }


def load_k_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    entries = []
    buf = ""
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        buf += line
        if line.endswith("}"):
            try:
                entries.append(json.loads(buf))
            except json.JSONDecodeError:
                pass
            buf = ""
    return entries


def plot_P_staircase(fix2: dict, fix7: dict, out_png: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    for fix, color in ((fix2, FIXES["fix2"]["color"]), (fix7, FIXES["fix7"]["color"])):
        stairs = pd.DataFrame(fix["staircase"])
        if stairs.empty:
            continue
        t = stairs["timestamp_s"] - fix["timestamp_s"]
        label = fix["label"]
        axes[0].plot(t, stairs["P_pre_vv"], color=color, lw=0.6, alpha=0.5)
        axes[0].plot(t, stairs["P_post_vv"], color=color, lw=0.9, label=f"{label} P_vv")
        axes[1].plot(t, stairs["P_pre_pv"], color=color, lw=0.6, alpha=0.5)
        axes[1].plot(t, stairs["P_post_pv"], color=color, lw=0.9, label=f"{label} P_pv")
        axes[2].plot(t, stairs["P_pre_aa"], color=color, lw=0.6, alpha=0.5)
        axes[2].plot(t, stairs["P_post_aa"], color=color, lw=0.9, label=f"{label} P_aa")
    for ax in axes:
        ax.axvline(0, color="k", ls=":", lw=1.0)
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8)
    axes[0].set_ylabel("P_vv frob")
    axes[1].set_ylabel("P_pv frob")
    axes[2].set_ylabel("P_aa frob")
    axes[2].set_xlabel("t − t_fix [s]")
    fig.suptitle("GAP-3.11 — Trayectoria P (pre/post NHC): escalera predict→NHC")
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def plot_deltaP_rel(fix2: dict, fix7: dict, out_png: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 4))
    for fix, color in ((fix2, FIXES["fix2"]["color"]), (fix7, FIXES["fix7"]["color"])):
        stairs = pd.DataFrame(fix["staircase"])
        if stairs.empty:
            continue
        t = stairs["timestamp_s"] - fix["timestamp_s"]
        ax.plot(t, stairs["delta_P_vv_rel"] * 100, color=color, lw=0.7, alpha=0.85, label=fix["label"])
    ax.axvline(0, color="k", ls=":", lw=1.0)
    ax.set_xlabel("t − t_fix [s]")
    ax.set_ylabel("|ΔP_vv| / P_vv_pre [%]")
    ax.set_title("Reducción relativa de P_vv por update NHC")
    ax.legend()
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def plot_k_innov_vs_innov(fix2: dict, fix7: dict, out_png: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for fix, color, ax in (
        (fix2, FIXES["fix2"]["color"], axes[0]),
        (fix7, FIXES["fix7"]["color"], axes[1]),
    ):
        stairs = pd.DataFrame(fix["staircase"])
        ax.scatter(
            stairs["innov_norm"],
            stairs["k_innov_energy"],
            c=stairs["k_vel_max"],
            cmap="viridis",
            s=8,
            alpha=0.6,
        )
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlabel("|innov| [m/s]")
        ax.set_ylabel("|K·innov| ≈ ||Δx||")
        ax.set_title(fix["label"])
        ax.grid(True, alpha=0.25, which="both")
    fig.suptitle("Energía de corrección NHC: innov vs ||Δx|| (color=k_vel_max)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def plot_summary_bars(fix2: dict, fix7: dict, out_png: Path) -> None:
    labels = ["innov_h [m]", "k_vel", "P_vv@GNSS", "k_vel/P_vv", "Σ|K·innov|", "Σ|innov|", "cond(S)"]
    f2 = fix2["gnss_at_fix"]
    f7 = fix7["gnss_at_fix"]
    w2 = fix2["pre_window_nhc"]
    w7 = fix7["pre_window_nhc"]
    v2 = [
        f2["innov_h_m"],
        f2["k_vel_max"],
        f2["P_vv_pre_gnss"],
        f2["k_vel_over_P_vv"] or 0,
        w2["sum_k_innov_energy"],
        w2["sum_innov_norm"],
        f2["s_cond"],
    ]
    v7 = [
        f7["innov_h_m"],
        f7["k_vel_max"],
        f7["P_vv_pre_gnss"],
        f7["k_vel_over_P_vv"] or 0,
        w7["sum_k_innov_energy"],
        w7["sum_innov_norm"],
        f7["s_cond"],
    ]
    x = np.arange(len(labels))
    width = 0.35
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.bar(x - width / 2, v2, width, label="Fix #2", color=FIXES["fix2"]["color"])
    ax.bar(x + width / 2, v7, width, label="Fix #7", color=FIXES["fix7"]["color"])
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=25, ha="right")
    ax.set_title("Comparación #2 vs #7 — magnitudes clave")
    ax.legend()
    ax.grid(True, alpha=0.25, axis="y")
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def comparison_verdict(fix2: dict, fix7: dict) -> dict:
    f2, f7 = fix2["gnss_at_fix"], fix7["gnss_at_fix"]
    ratio_k = f2["k_vel_max"] / max(f7["k_vel_max"], 1e-9)
    ratio_p = f2["P_vv_pre_gnss"] / max(f7["P_vv_pre_gnss"], 1e-9)
    r2 = f2.get("k_vel_over_P_vv")
    r7 = f7.get("k_vel_over_P_vv")
    ratio_k_over_p = (r2 / r7) if r2 and r7 and r7 > 0 else None

    return {
        "innov_h_ratio_fix2_over_fix7": f2["innov_h_m"] / f7["innov_h_m"],
        "k_vel_ratio_fix2_over_fix7": ratio_k,
        "P_vv_pre_gnss_ratio_fix2_over_fix7": ratio_p,
        "k_vel_over_P_vv_fix2": r2,
        "k_vel_over_P_vv_fix7": r7,
        "k_vel_over_P_vv_ratio_fix2_over_fix7": ratio_k_over_p,
        "interpretation_strict": (
            "Entre fix #2 y #7 se observa reducción marcada de P_vv y k_vel "
            "con innov_h del mismo orden y cond(S)≈1. "
            "La trayectoria P_vv en -2s muestra compresión NHC repetida (escalera). "
            "Cadena ΔP→K aún en verificación."
        ),
        "k_over_P_similar": ratio_k_over_p is not None and 0.5 < ratio_k_over_p < 2.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3.11 fix #2 vs #7 comparative autopsy")
    parser.add_argument("--data-dir", type=Path, default=AUTOPSY_DIR)
    args = parser.parse_args()

    data_dir = args.data_dir
    gnss = load_csv(data_dir / "gnss_nis_audit.csv")
    nhc = load_csv(data_dir / "nhc_block_audit.csv")
    cov_step = load_csv(data_dir / "cov_step_audit.csv")
    k_blocks = load_k_jsonl(data_dir / "gnss_k_block.jsonl")

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    fix2 = summarize_fix("fix2", gnss, nhc, cov_step, k_blocks)
    fix7 = summarize_fix("fix7", gnss, nhc, cov_step, k_blocks)
    comparison = comparison_verdict(fix2, fix7)

    plot_P_staircase(fix2, fix7, OUT_DIR / "fix2_vs_fix7_P_staircase.png")
    plot_deltaP_rel(fix2, fix7, OUT_DIR / "fix2_vs_fix7_deltaP_rel.png")
    plot_k_innov_vs_innov(fix2, fix7, OUT_DIR / "fix2_vs_fix7_k_innov_vs_innov.png")
    plot_summary_bars(fix2, fix7, OUT_DIR / "fix2_vs_fix7_summary.png")

    report = {
        "experiment": "GAP-3.11 comparative autopsy fix #2 vs #7",
        "config": "ZUPT OFF, NHC ON (exp B)",
        "window_s": {"pre": PRE_S, "post": POST_S},
        "fix2": fix2,
        "fix7": fix7,
        "comparison": comparison,
        "artifacts": {
            "report_json": str(REPORT_JSON),
            "P_staircase_png": str(OUT_DIR / "fix2_vs_fix7_P_staircase.png"),
            "deltaP_rel_png": str(OUT_DIR / "fix2_vs_fix7_deltaP_rel.png"),
            "k_innov_png": str(OUT_DIR / "fix2_vs_fix7_k_innov_vs_innov.png"),
            "summary_png": str(OUT_DIR / "fix2_vs_fix7_summary.png"),
        },
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k not in ("fix2", "fix7")}, indent=2))
    print(f"Wrote {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
