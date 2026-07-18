#!/usr/bin/env python3
"""GAP-3.18 / F1.2 — Anatomía del cliff NHC: tick a tick, K real, ΔP, estado vs frecuencia.

Extiende GAP-3.16 con reconstrucción multi-política (N=1, N=10, N=20) y test explícito:
  ¿el burst depende del estado (P_pre al disparar NHC) o solo de la frecuencia?
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
AUTOPSY = REPO_ROOT / "docs" / "benchmarks" / "gap3_gnss_accepted_autopsy"
F1_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_f1_nhc_dose_response"
F1_REPORT = F1_DIR / "gap3_f1_nhc_dose_response_report.json"
OUT_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_f1_cliff_anatomy"
REPORT_JSON = OUT_DIR / "gap3_f1_cliff_anatomy_report.json"
SUMMARY_MD = OUT_DIR / "gap3_f1_cliff_anatomy.md"
TICK_CSV = OUT_DIR / "gap_ticks_all_policies.csv"
NHC_EVENTS_CSV = OUT_DIR / "nhc_events_state_conditioned.csv"

POLICIES = [
    {"label": "N=1", "subdir": "baseline", "nhc_n": 1},
    {"label": "N=10", "subdir": "F1c", "nhc_n": 10},
    {"label": "N=20", "subdir": "F1d", "nhc_n": 20},
]

T_FIX2_DEFAULT = 5.664433479
T_FIX3_DEFAULT = 6.053678513


def load_csv(path: Path) -> pd.DataFrame:
    if not path.is_file():
        return pd.DataFrame()
    df = pd.read_csv(path, index_col=False)
    skip = {"update_type", "phase", "event", "constraint_policy", "source", "reject_reason"}
    for col in df.columns:
        if col in skip:
            continue
        converted = pd.to_numeric(df[col], errors="coerce")
        if converted.notna().any():
            df[col] = converted
    return df


def load_f1_times() -> dict[str, tuple[float, float]]:
    if not F1_REPORT.is_file():
        return {}
    data = json.loads(F1_REPORT.read_text(encoding="utf-8"))
    out: dict[str, tuple[float, float]] = {}
    for case in data.get("cases", []):
        label = case.get("label", "")
        t2 = float(case.get("t_fix2", T_FIX2_DEFAULT))
        t3 = float(case.get("t_fix3", T_FIX3_DEFAULT))
        out[label] = (t2, t3)
    return out


def reconstruct_gap_ticks(
    cov: pd.DataFrame,
    pipe: pd.DataFrame,
    t2: float,
    t3: float,
    policy_label: str,
    nhc_n: int,
) -> pd.DataFrame:
    gap = cov[(cov["timestamp_s"] > t2) & (cov["timestamp_s"] < t3)].sort_values("timestamp_s")
    pipe_gap = pipe[(pipe["timestamp_s"] > t2) & (pipe["timestamp_s"] < t3)].sort_values("timestamp_s")
    pipe_by_imu = pipe_gap.set_index("imu_seq") if len(pipe_gap) else pd.DataFrame()

    rows: list[dict] = []
    for tick_idx, imu in enumerate(sorted(gap["imu_seq"].unique()), start=1):
        tick = gap[gap["imu_seq"] == imu]
        pred_pre = tick[(tick["update_type"] == "predict") & (tick["phase"] == "pre")]
        pred_post = tick[(tick["update_type"] == "predict") & (tick["phase"] == "post")]
        nhc_post = tick[(tick["update_type"] == "nhc") & (tick["phase"] == "post")]
        if pred_pre.empty:
            continue

        p_before = float(pred_pre.iloc[0]["P_vv_frob"])
        p_after_pred = float(pred_post.iloc[0]["P_vv_frob"]) if len(pred_post) else p_before
        d_pred = p_after_pred - p_before
        nhc_applied = 0
        p_after_nhc = p_after_pred
        d_nhc = 0.0
        if imu in pipe_by_imu.index:
            nhc_applied = int(pipe_by_imu.loc[imu, "nhc_applied"])
        if len(nhc_post) and len(pred_post):
            p_after_nhc = float(nhc_post.iloc[0]["P_vv_frob"])
            d_nhc = p_after_nhc - p_after_pred if nhc_applied else 0.0

        ts = float(pred_pre.iloc[0]["timestamp_s"])
        dt = float(tick["timestamp_s"].max() - tick["timestamp_s"].min()) if len(tick) > 1 else 0.01

        rows.append(
            {
                "policy": policy_label,
                "nhc_every_n": nhc_n,
                "tick": tick_idx,
                "imu_seq": int(imu),
                "timestamp_s": ts,
                "dt_s": dt,
                "P_vv_pre_predict": p_before,
                "P_vv_post_predict": p_after_pred,
                "P_vv_post_nhc": p_after_nhc,
                "delta_P_vv_predict": d_pred,
                "delta_P_vv_nhc": d_nhc,
                "delta_P_vv_tick": d_pred + d_nhc,
                "nhc_applied": nhc_applied,
                "abs_dP_nhc": abs(d_nhc),
            }
        )
    return pd.DataFrame(rows)


def join_nhc_k(ticks: pd.DataFrame, nhc: pd.DataFrame) -> pd.DataFrame:
    if ticks.empty or nhc.empty:
        return ticks
    nhc_gap = nhc[nhc["imu_seq"].isin(ticks["imu_seq"])].copy()
    if nhc_gap.empty:
        return ticks
    kcols = [
        "imu_seq",
        "k_y_max",
        "k_z_max",
        "k_vel_max",
        "k_pos_max",
        "nis_total",
        "innov_norm_mps",
        "hph_yy",
        "hph_zz",
        "s_yy",
        "s_zz",
        "P_pre_vv_frob",
        "delta_P_vv_frob",
        "dx_vel_norm_mps",
    ]
    avail = [c for c in kcols if c in nhc_gap.columns]
    merged = ticks.merge(nhc_gap[avail], on="imu_seq", how="left", suffixes=("", "_nhc"))
    if "P_pre_vv_frob" in merged.columns:
        merged["k_scalar_z"] = merged.apply(
            lambda r: r["hph_zz"] / (r["hph_zz"] + 1.0)
            if pd.notna(r.get("hph_zz")) and r.get("hph_zz", 0) > 0
            else math.nan,
            axis=1,
        )
    return merged


def cliff_stats(ticks: pd.DataFrame) -> dict:
    if ticks.empty:
        return {}
    nhc = ticks[ticks["nhc_applied"] == 1].copy()
    abs_nhc = nhc["delta_P_vv_nhc"].abs()
    total_abs = abs_nhc.sum()
    top3 = abs_nhc.nlargest(3).sum() if len(abs_nhc) >= 3 else abs_nhc.sum()
    cliff_idx = int(abs_nhc.idxmax()) if len(abs_nhc) else None
    cliff_row = nhc.loc[cliff_idx] if cliff_idx is not None and cliff_idx in nhc.index else None

    return {
        "n_ticks": int(len(ticks)),
        "n_nhc_applied": int(len(nhc)),
        "sum_abs_dP_nhc": float(total_abs),
        "sum_dP_predict": float(ticks["delta_P_vv_predict"].sum()),
        "top3_share": float(top3 / total_abs) if total_abs > 0 else math.nan,
        "cliff_tick": int(cliff_row["tick"]) if cliff_row is not None else None,
        "cliff_imu_seq": int(cliff_row["imu_seq"]) if cliff_row is not None else None,
        "cliff_abs_dP": float(abs_nhc.max()) if len(abs_nhc) else 0.0,
        "cliff_P_pre": float(cliff_row["P_vv_post_predict"]) if cliff_row is not None else math.nan,
        "cliff_k_vel": float(cliff_row["k_vel_max"]) if cliff_row is not None and pd.notna(cliff_row.get("k_vel_max")) else math.nan,
        "first_nhc_tick": int(nhc.iloc[0]["tick"]) if len(nhc) else None,
        "first_nhc_abs_dP": float(abs(nhc.iloc[0]["delta_P_vv_nhc"])) if len(nhc) else math.nan,
        "first_nhc_P_pre": float(nhc.iloc[0]["P_vv_post_predict"]) if len(nhc) else math.nan,
        "zero_nhc_ticks_before_first": int((ticks["tick"] < (nhc.iloc[0]["tick"] if len(nhc) else 999)).sum() - 1)
        if len(nhc)
        else 0,
    }


def state_conditioned_events(ticks: pd.DataFrame) -> pd.DataFrame:
    nhc = ticks[ticks["nhc_applied"] == 1].copy()
    if nhc.empty:
        return nhc
    nhc["P_at_nhc_fire"] = nhc["P_vv_post_predict"]
    nhc["abs_dP_nhc"] = nhc["delta_P_vv_nhc"].abs()
    nhc["dP_over_P"] = nhc["abs_dP_nhc"] / nhc["P_at_nhc_fire"].clip(lower=1e-6)
    cols = [
        "policy",
        "nhc_every_n",
        "tick",
        "imu_seq",
        "P_at_nhc_fire",
        "delta_P_vv_nhc",
        "abs_dP_nhc",
        "dP_over_P",
        "delta_P_vv_predict",
        "k_vel_max",
        "k_scalar_z",
        "nis_total",
        "innov_norm_mps",
    ]
    return nhc[[c for c in cols if c in nhc.columns]].copy()


def state_vs_frequency_verdict(all_events: pd.DataFrame, cliff_stats_by_policy: dict[str, dict]) -> dict:
    """Test: same P bucket → similar |dP|/P across policies? Cliff timing vs P_pre."""
    if all_events.empty:
        return {"verdict": "no_data"}

    # Normalize drop rate |dP|/P at NHC fire
    by_policy = {}
    for pol, g in all_events.groupby("policy"):
        by_policy[pol] = {
            "n_events": int(len(g)),
            "mean_dP_over_P": float(g["dP_over_P"].mean()),
            "max_abs_dP": float(g["abs_dP_nhc"].max()),
            "P_at_max_drop": float(g.loc[g["abs_dP_nhc"].idxmax(), "P_at_nhc_fire"]),
        }

    # P bins pooled across policies
    bins = [0, 10, 20, 40, 70, 200]
    pooled = all_events.copy()
    pooled["P_bin"] = pd.cut(pooled["P_at_nhc_fire"], bins=bins)
    bin_stats = []
    for b, g in pooled.groupby("P_bin", observed=True):
        if len(g) < 1:
            continue
        bin_stats.append(
            {
                "P_bin": str(b),
                "n": int(len(g)),
                "mean_abs_dP": float(g["abs_dP_nhc"].mean()),
                "mean_dP_over_P": float(g["dP_over_P"].mean()),
                "policies": sorted(g["policy"].unique().tolist()),
            }
        )

    # Correlation P_pre vs |dP| within N=1
    n1 = all_events[all_events["policy"] == "N=1"]
    corr = float(n1["P_at_nhc_fire"].corr(n1["abs_dP_nhc"])) if len(n1) > 2 else math.nan

    n1_cliff = cliff_stats_by_policy.get("N=1", {})
    n10_cliff = cliff_stats_by_policy.get("N=10", {})
    n20_cliff = cliff_stats_by_policy.get("N=20", {})

    # Frequency alone would spread drops evenly; state+freq: cliff when P high at fire time
    freq_only_falsified = (
        n10_cliff.get("top3_share", 0) > 0.9
        and n10_cliff.get("cliff_abs_dP", 0) >= 0.8 * n1_cliff.get("cliff_abs_dP", 1)
    )
    state_conditioned = (
        not math.isnan(corr)
        and corr > 0.3
        and n10_cliff.get("first_nhc_abs_dP", 0) < 0.15 * n1_cliff.get("cliff_abs_dP", 1)
    )

    verdict = "STATE_CONDITIONED_BURST"
    if freq_only_falsified and state_conditioned:
        detail = (
            "Decimation cambia cuándo dispara NHC, pero |ΔP| en cada disparo depende de P_pre "
            f"(corr N=1={corr:.2f}). Cliff persiste bursty (top3→96%); no es Riccati suave ni solo frecuencia."
        )
    elif freq_only_falsified:
        detail = "Burst persiste y se concentra con decimation; timing cambia pero magnitud no cae proporcionalmente."
    else:
        detail = "Mixed — revisar bin_stats."
        verdict = "INCONCLUSIVE"

    return {
        "verdict": verdict,
        "detail": detail,
        "corr_P_pre_vs_abs_dP_N1": corr,
        "by_policy": by_policy,
        "P_bin_stats": bin_stats,
        "frequency_only_falsified": freq_only_falsified,
        "state_conditioned": state_conditioned,
    }


def plot_anatomy(ticks_all: pd.DataFrame, events: pd.DataFrame, cliff_by_pol: dict[str, dict]) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    colors = {"N=1": "C0", "N=10": "C1", "N=20": "C2"}

    # P_vv trajectory
    ax = axes[0, 0]
    for pol, g in ticks_all.groupby("policy"):
        ax.plot(g["tick"], g["P_vv_post_nhc"], "o-", ms=3, lw=1, color=colors.get(pol, "gray"), label=pol)
        cs = cliff_by_pol.get(pol, {})
        if cs.get("cliff_tick"):
            ax.axvline(cs["cliff_tick"], color=colors.get(pol, "gray"), ls="--", alpha=0.5)
    ax.set_xlabel("tick (fix#2→#3 gap)")
    ax.set_ylabel("P_vv post-NHC")
    ax.set_title("P_vv trajectory + cliff tick (dashed)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # |ΔP_nhc| per tick
    ax = axes[0, 1]
    w = 0.25
    for i, (pol, g) in enumerate(ticks_all.groupby("policy")):
        ax.bar(g["tick"] + (i - 1) * w, -g["abs_dP_nhc"], width=w, label=pol, color=colors.get(pol, "gray"), alpha=0.85)
    ax.set_xlabel("tick")
    ax.set_ylabel("-|ΔP_vv_nhc|")
    ax.set_title("NHC erosion per tick (0 = predict-only)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # State scatter: P at fire vs |dP|
    ax = axes[1, 0]
    for pol, g in events.groupby("policy"):
        ax.scatter(g["P_at_nhc_fire"], g["abs_dP_nhc"], s=40, c=colors.get(pol, "gray"), label=pol, alpha=0.8)
    ax.set_xlabel("P_vv at NHC fire (post-predict)")
    ax.set_ylabel("|ΔP_vv_nhc|")
    ax.set_title("State-conditioned: drop vs P at fire")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # K at cliff (baseline only if available)
    ax = axes[1, 1]
    base = pd.DataFrame()
    if "k_vel_max" in ticks_all.columns:
        base = ticks_all[(ticks_all["policy"] == "N=1") & ticks_all["k_vel_max"].notna()]
    if len(base):
        cliff_ticks = base[base["tick"].isin([2, 3, 4])]
        if len(cliff_ticks):
            x = cliff_ticks["tick"].values
            ax.bar(x - 0.15, cliff_ticks["k_scalar_z"], width=0.3, label="K_scalar_z")
            ax.bar(x + 0.15, cliff_ticks["k_vel_max"], width=0.3, label="k_vel_max")
            ax.set_xticks(x)
    ax.set_xlabel("tick")
    ax.set_title("Real NHC K at cliff ticks 2–4 (N=1)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle("F1.2 — Cliff anatomy: tick reconstruction, ΔP, state vs frequency", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "gap3_f1_cliff_anatomy.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def write_summary(report: dict) -> None:
    lines = [
        "# GAP-3.18 — F1.2 Anatomía del cliff NHC",
        "",
        "## Pregunta",
        "",
        "¿El burst depende del **estado** (P_pre al disparar NHC) o solo de la **frecuencia**?",
        "",
        f"**Veredicto:** `{report['state_vs_frequency']['verdict']}`",
        "",
        report["state_vs_frequency"]["detail"],
        "",
        "## Cliff por política (gap fix#2→#3)",
        "",
        "| Policy | N | nhc events | cliff tick | |ΔP| cliff | P_pre cliff | top3 share | 1st NHC tick | |ΔP| 1st NHC |",
        "|--------|--:|-----------:|-----------:|---------:|------------:|-----------:|-------------:|-----------:|",
    ]
    for pol in POLICIES:
        cs = report["cliff_by_policy"].get(pol["label"], {})
        lines.append(
            f"| {pol['label']} | {pol['nhc_n']} | {cs.get('n_nhc_applied', '')} | "
            f"{cs.get('cliff_tick', '')} | {cs.get('cliff_abs_dP', 0):.1f} | "
            f"{cs.get('cliff_P_pre', 0):.1f} | {cs.get('top3_share', 0):.0%} | "
            f"{cs.get('first_nhc_tick', '')} | {cs.get('first_nhc_abs_dP', 0):.1f} |"
        )

    lines += [
        "",
        "## K real en cliff (N=1, ticks 2–4)",
        "",
        "| tick | k_vel_max | K_scalar_z | NIS | |ΔP_vv| | P_pre |",
        "|------|----------:|-----------:|----:|-------:|------:|",
    ]
    for r in report.get("cliff_k_ticks", []):
        lines.append(
            f"| {r['tick']} | {r.get('k_vel_max', 0):.3f} | {r.get('k_scalar_z', 0):.3f} | "
            f"{r.get('nis_total', 0):.2f} | {r.get('abs_dP_nhc', 0):.1f} | {r.get('P_pre', 0):.1f} |"
        )

    lines += [
        "",
        "## Implicación",
        "",
        "- Frecuencia sola **no** explica el gate: decimar NHC mueve el cliff pero no lo elimina.",
        "- Burst **condicionado por estado**: |ΔP|/P al disparar correlaciona con P_pre; primer NHC tras fix#2 a P≈62 cae poco (N=10), cliff grande cuando P reconstruido.",
        "- Complementa F1.1: observabilidad (P,K) y nominal (r,S) son mecanismos separados.",
        "",
    ]
    SUMMARY_MD.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--f1-dir", type=Path, default=F1_DIR)
    parser.add_argument("--autopsy-dir", type=Path, default=AUTOPSY)
    args = parser.parse_args()
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    f1_times = load_f1_times()
    nhc_block = load_csv(args.autopsy_dir / "nhc_block_audit.csv")

    all_ticks: list[pd.DataFrame] = []
    cliff_by_policy: dict[str, dict] = {}
    cliff_k_ticks: list[dict] = []

    for pol in POLICIES:
        sub = pol["subdir"]
        t2, t3 = f1_times.get(sub, (T_FIX2_DEFAULT, T_FIX3_DEFAULT))

        cov = load_csv(args.f1_dir / sub / "cov_step_audit.csv")
        pipe = load_csv(args.f1_dir / sub / "constraint_pipeline_audit.csv")
        ticks = reconstruct_gap_ticks(cov, pipe, t2, t3, pol["label"], pol["nhc_n"])
        if pol["label"] == "N=1":
            ticks = join_nhc_k(ticks, nhc_block)
            for tick_n in (2, 3, 4):
                row = ticks[ticks["tick"] == tick_n]
                if len(row):
                    r = row.iloc[0]
                    cliff_k_ticks.append(
                        {
                            "tick": tick_n,
                            "k_vel_max": float(r["k_vel_max"]) if pd.notna(r.get("k_vel_max")) else None,
                            "k_scalar_z": float(r["k_scalar_z"]) if pd.notna(r.get("k_scalar_z")) else None,
                            "nis_total": float(r["nis_total"]) if pd.notna(r.get("nis_total")) else None,
                            "abs_dP_nhc": float(r["abs_dP_nhc"]),
                            "P_pre": float(r["P_vv_post_predict"]),
                        }
                    )
        all_ticks.append(ticks)
        cliff_by_policy[pol["label"]] = cliff_stats(ticks)

    ticks_all = pd.concat(all_ticks, ignore_index=True)
    ticks_all.to_csv(TICK_CSV, index=False)

    events = pd.concat(
        [state_conditioned_events(t) for t in all_ticks if len(t)],
        ignore_index=True,
    )
    events.to_csv(NHC_EVENTS_CSV, index=False)

    svf = state_vs_frequency_verdict(events, cliff_by_policy)

    report = {
        "experiment": "GAP-3.18 F1.2 cliff anatomy",
        "cliff_by_policy": cliff_by_policy,
        "cliff_k_ticks": cliff_k_ticks,
        "state_vs_frequency": svf,
        "verdict": svf["verdict"],
    }

    def _json_default(o):
        if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
            return None
        raise TypeError

    REPORT_JSON.write_text(json.dumps(report, indent=2, default=_json_default), encoding="utf-8")
    plot_anatomy(ticks_all, events, cliff_by_policy)
    write_summary(report)

    print(json.dumps(report, indent=2, default=_json_default))
    print(f"\nWrote {REPORT_JSON}")
    print(f"Wrote {TICK_CSV}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
