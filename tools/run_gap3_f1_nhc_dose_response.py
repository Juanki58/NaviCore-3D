#!/usr/bin/env python3
"""GAP-3.15 / F1 — NHC dose-response (falsificación sobre-observación por frecuencia).

Políticas: N=1 (baseline), 2, 5, 10, 20, OFF (∞).
Métricas mecanísticas: P_vv pre#3, k_vel, accepts, innov_h, Σ|ΔP_vv| NHC, top-3 share, Γ.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_f1_nhc_dose_response"
REPORT_JSON = OUT_DIR / "gap3_f1_nhc_dose_response_report.json"
SUMMARY_MD = OUT_DIR / "gap3_f1_nhc_dose_response.md"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"

CASES = [
    {"label": "baseline", "nhc_every_n_ticks": 1, "nhc_policy": "enabled"},
    {"label": "F1a", "nhc_every_n_ticks": 2, "nhc_policy": "enabled"},
    {"label": "F1b", "nhc_every_n_ticks": 5, "nhc_policy": "enabled"},
    {"label": "F1c", "nhc_every_n_ticks": 10, "nhc_policy": "enabled"},
    {"label": "F1d", "nhc_every_n_ticks": 20, "nhc_policy": "enabled"},
    {"label": "OFF", "nhc_every_n_ticks": None, "nhc_policy": "disabled"},
]

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


def load_csv(path: Path) -> pd.DataFrame:
    if not path.is_file():
        return pd.DataFrame()
    df = pd.read_csv(path, index_col=False)
    skip = {"update_type", "phase", "event", "constraint_policy", "source"}
    for col in df.columns:
        if col in skip:
            continue
        converted = pd.to_numeric(df[col], errors="coerce")
        if converted.notna().any():
            df[col] = converted
    return df


def run_case(
    replay_exe: Path,
    replay_csv: Path,
    calibration: Path,
    case: dict,
) -> Path:
    label = case["label"]
    n = case["nhc_every_n_ticks"]
    out_dir = OUT_DIR / label
    out_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--h9a-gravity-tilt-init",
        "--constraint-policy",
        "disabled",
        "--nhc-policy",
        case["nhc_policy"],
        "--output",
        str(out_dir / "replay_output.csv"),
        "--gap3-gnss-nis-audit-csv",
        str(out_dir / "gnss_nis_audit.csv"),
        "--gap3-cov-step-audit-csv",
        str(out_dir / "cov_step_audit.csv"),
        "--gap3-constraint-pipeline-audit-csv",
        str(out_dir / "constraint_pipeline_audit.csv"),
    ]
    if n is not None:
        cmd.extend(["--nhc-every-n-ticks", str(n)])

    print(f"\n=== {label} (NHC policy={case['nhc_policy']}, N={n}) ===")
    print("RUN:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)
    return out_dir


def fix_timestamps(gnss: pd.DataFrame) -> dict | None:
    acc = gnss[gnss["accepted"] == 1].sort_values("gps_index")
    if not ((acc["gps_index"] == 2).any() and (acc["gps_index"] == 3).any()):
        return None
    t2 = float(acc[acc["gps_index"] == 2].iloc[0]["timestamp_s"])
    t3 = float(acc[acc["gps_index"] == 3].iloc[0]["timestamp_s"])
    return {"t_fix2": t2, "t_fix3": t3}


def analyze_gap(cov: pd.DataFrame, t_fix2: float, t_fix3: float, nhc_enabled: bool) -> dict:
    gap = cov[(cov["timestamp_s"] > t_fix2) & (cov["timestamp_s"] < t_fix3)].copy()
    imu_seqs = sorted(gap["imu_seq"].unique()) if not gap.empty else []

    sum_pred = 0.0
    sum_nhc_abs = 0.0
    nhc_deltas: list[float] = []
    n_ticks = 0
    pvv_start = math.nan
    pvv_end = math.nan

    for imu_seq in imu_seqs:
        tick = gap[gap["imu_seq"] == imu_seq]
        pred_pre = tick[(tick["update_type"] == "predict") & (tick["phase"] == "pre")]
        pred_post = tick[(tick["update_type"] == "predict") & (tick["phase"] == "post")]
        nhc_post = tick[(tick["update_type"] == "nhc") & (tick["phase"] == "post")]
        if pred_pre.empty:
            continue
        pvv_b = float(pred_pre.iloc[0]["P_vv_frob"])
        if math.isnan(pvv_start):
            pvv_start = pvv_b
        if not pred_post.empty:
            sum_pred += float(pred_post.iloc[0]["P_vv_frob"]) - pvv_b
        if not nhc_post.empty:
            pvv_ap = float(pred_post.iloc[0]["P_vv_frob"]) if not pred_post.empty else pvv_b
            d_nhc = float(nhc_post.iloc[0]["P_vv_frob"]) - pvv_ap
            sum_nhc_abs += abs(d_nhc)
            nhc_deltas.append(d_nhc)
            pvv_end = float(nhc_post.iloc[0]["P_vv_frob"])
        elif not pred_post.empty:
            pvv_end = float(pred_post.iloc[0]["P_vv_frob"])
        n_ticks += 1

    if not nhc_enabled:
        sum_nhc_abs = 0.0
        nhc_deltas = []

    abs_drops = np.array([abs(d) for d in nhc_deltas if abs(d) > 0])
    if len(abs_drops):
        sorted_d = np.sort(abs_drops)[::-1]
        top3_share = float(sorted_d[:3].sum() / abs_drops.sum())
        top1_share = float(sorted_d[0] / abs_drops.sum())
    else:
        top3_share = top1_share = math.nan

    gamma = sum_nhc_abs / sum_pred if sum_pred > 1e-9 and sum_nhc_abs > 0 else math.nan

    f3_pre_rows = cov[
        (cov["update_type"] == "gnss")
        & (cov["phase"] == "pre")
        & (np.isclose(cov["timestamp_s"], t_fix3, atol=1e-3))
    ]
    f2_post_rows = cov[
        (cov["update_type"] == "gnss")
        & (cov["phase"] == "post_accept")
        & (np.isclose(cov["timestamp_s"], t_fix2, atol=1e-3))
    ]
    f2_pre_rows = cov[
        (cov["update_type"] == "gnss")
        & (cov["phase"] == "pre")
        & (np.isclose(cov["timestamp_s"], t_fix2, atol=1e-3))
    ]

    return {
        "gap_duration_s": t_fix3 - t_fix2,
        "gap_imu_ticks": n_ticks,
        "P_vv_post_fix2": float(f2_post_rows.iloc[0]["P_vv_frob"]) if len(f2_post_rows) else math.nan,
        "P_vv_pre_fix2": float(f2_pre_rows.iloc[0]["P_vv_frob"]) if len(f2_pre_rows) else math.nan,
        "P_vv_pre_fix3": float(f3_pre_rows.iloc[0]["P_vv_frob"]) if len(f3_pre_rows) else math.nan,
        "P_vv_gap_start": pvv_start,
        "P_vv_gap_end": pvv_end,
        "sum_delta_P_vv_predict": sum_pred,
        "sum_abs_delta_P_vv_nhc": sum_nhc_abs,
        "gamma_nhc_over_predict": gamma,
        "nhc_updates_in_gap": len(nhc_deltas),
        "top1_nhc_share_of_abs_drop": top1_share,
        "top3_nhc_share_of_abs_drop": top3_share,
        "erosion_pattern": "bursty" if (not math.isnan(top3_share) and top3_share > 0.5) else ("uniform" if len(nhc_deltas) else "no_nhc"),
    }


def analyze_case_dir(case: dict, out_dir: Path) -> dict:
    gnss = load_csv(out_dir / "gnss_nis_audit.csv")
    cov = load_csv(out_dir / "cov_step_audit.csv")
    nhc_on = case["nhc_policy"] == "enabled"

    accepted = gnss[gnss["accepted"] == 1]
    fixes = fix_timestamps(gnss)

    row: dict = {
        "label": case["label"],
        "nhc_policy": case["nhc_policy"],
        "nhc_every_n_ticks": case["nhc_every_n_ticks"],
        "gnss_accept_count": int(len(accepted)),
        "gnss_reject_count": int(len(gnss) - len(accepted)),
        "innov_h_mean_accepted": float(accepted["innov_h_m"].mean()) if len(accepted) else math.nan,
        "innov_h_by_gps_index": {
            int(r["gps_index"]): float(r["innov_h_m"])
            for _, r in accepted.iterrows()
        },
        "k_vel_max_by_gps_index": {
            int(r["gps_index"]): float(r["k_vel_max"])
            for _, r in accepted.iterrows()
        },
        "k_vel_mean_accepted": float(accepted["k_vel_max"].mean()) if len(accepted) else math.nan,
        "k_vel_fix2": float(accepted[accepted["gps_index"] == 2]["k_vel_max"].iloc[0])
        if (accepted["gps_index"] == 2).any()
        else math.nan,
        "k_vel_fix3": float(accepted[accepted["gps_index"] == 3]["k_vel_max"].iloc[0])
        if (accepted["gps_index"] == 3).any()
        else math.nan,
    }

    if fixes:
        gap = analyze_gap(cov, fixes["t_fix2"], fixes["t_fix3"], nhc_on)
        row.update(gap)
        row["t_fix2"] = fixes["t_fix2"]
        row["t_fix3"] = fixes["t_fix3"]
    else:
        row["gap_analysis"] = "missing_fix2_or_fix3"

    return row


def evaluate_stopping_criterion(results: list[dict]) -> dict:
    base = next((r for r in results if r["label"] == "baseline"), None)
    f1c = next((r for r in results if r["label"] == "F1c"), None)
    if not base or not f1c:
        return {"evaluated": False}

    def delta(a, b, key):
        av, bv = a.get(key), b.get(key)
        if av is None or bv is None or (isinstance(av, float) and math.isnan(av)):
            return math.nan
        return bv - av

    d_pvv = delta(base, f1c, "P_vv_pre_fix3")
    d_kvel = delta(base, f1c, "k_vel_mean_accepted")
    d_acc = f1c["gnss_accept_count"] - base["gnss_accept_count"]
    d_innov = delta(base, f1c, "innov_h_mean_accepted")
    d_gamma = delta(base, f1c, "gamma_nhc_over_predict")
    if not math.isnan(d_gamma):
        d_gamma = f1c["gamma_nhc_over_predict"] - base["gamma_nhc_over_predict"]

    pvv_up = d_pvv > 5.0
    kvel_up = d_kvel > 0.02
    acc_up = d_acc >= 2
    innov_down = d_innov < -2.0
    gamma_down = (
        not math.isnan(d_gamma)
        and base.get("gamma_nhc_over_predict", 0) > 0
        and f1c.get("gamma_nhc_over_predict", 0) < base["gamma_nhc_over_predict"] * 0.6
    )

    dominant_frequency = sum([pvv_up, kvel_up, acc_up, innov_down, gamma_down]) >= 3

    if pvv_up and kvel_up and gamma_down and not acc_up:
        verdict = "FREQUENCY_MECHANISM_CONFIRMED_GATE_UNCHANGED"
        interpretation = (
            "N=10 confirma mecanismo (P_vv pre#3, k_vel, Γ) pero accepts siguen en 7 — "
            "la compresión P_vv ya no es el único gate; estudiar política NHC antes de GNSS_MAX_GAIN."
        )
    elif dominant_frequency:
        verdict = "FREQUENCY_HYPOTHESIS_SUPPORTED"
        interpretation = (
            "N=10 mueve métricas mecanísticas en dirección predicha → estudiar política NHC antes de GNSS_MAX_GAIN."
        )
    else:
        verdict = "FREQUENCY_HYPOTHESIS_WEAK_OR_REJECTED"
        interpretation = (
            "N=1→10 no mueve indicadores clave → investigar geometría/update del NHC."
        )

    return {
        "evaluated": True,
        "baseline_N": 1,
        "comparison_N": 10,
        "delta_P_vv_pre_fix3": d_pvv,
        "delta_k_vel_mean": d_kvel,
        "delta_k_vel_fix3": (
            f1c.get("k_vel_fix3", math.nan) - base.get("k_vel_fix3", math.nan)
            if not math.isnan(f1c.get("k_vel_fix3", math.nan))
            else math.nan
        ),
        "delta_accepts": d_acc,
        "delta_innov_h_mean": d_innov,
        "delta_gamma": d_gamma if not math.isnan(d_gamma) else None,
        "signals": {
            "P_vv_pre_fix3_up": pvv_up,
            "k_vel_up": kvel_up,
            "accepts_up": acc_up,
            "innov_h_down": innov_down,
            "gamma_down": gamma_down,
        },
        "verdict": verdict,
        "interpretation": interpretation,
    }


def plot_dose_response(results: list[dict]) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    on_cases = [r for r in results if r["nhc_policy"] == "enabled"]
    on_cases = sorted(on_cases, key=lambda r: r["nhc_every_n_ticks"] or 0)
    off = next((r for r in results if r["label"] == "OFF"), None)

    x = [r["nhc_every_n_ticks"] for r in on_cases]
    labels = [f"N={n}" for n in x]

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    metrics = [
        ("P_vv_pre_fix3", "P_vv pre fix#3", "C0"),
        ("k_vel_mean_accepted", "k_vel mean (accepts)", "C1"),
        ("gnss_accept_count", "GNSS accepts", "C2"),
        ("innov_h_mean_accepted", "innov_h mean accepts [m]", "C3"),
        ("gamma_nhc_over_predict", "Γ = Σ|ΔP_vv|_NHC / ΣΔP_vv_pred", "C4"),
        ("top3_nhc_share_of_abs_drop", "top-3 NHC share |ΔP_vv|", "C5"),
    ]
    for ax, (key, title, color) in zip(axes.flat, metrics):
        y = [r.get(key, math.nan) for r in on_cases]
        ax.plot(x, y, "o-", color=color, lw=2, ms=8)
        if off and not math.isnan(off.get(key, math.nan)):
            ax.axhline(off[key], color="gray", ls="--", lw=1, label=f"OFF={off[key]:.2g}")
        ax.set_xscale("log", base=2)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=30, ha="right")
        ax.set_title(title, fontsize=10)
        ax.grid(True, alpha=0.3)
        if off and not math.isnan(off.get(key, math.nan)):
            ax.legend(fontsize=7)

    fig.suptitle("GAP-3.15 F1: NHC dose-response (ZUPT OFF)", fontsize=12)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "f1_dose_response.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def write_summary(results: list[dict], stopping: dict) -> None:
    lines = [
        "# GAP-3.15 — F1 NHC dose-response",
        "",
        "| Policy | N | accepts | P_vv pre#3 | k_vel mean | k_vel#2 | k_vel#3 | innov_h | Σ|ΔP| NHC | Γ | top3 share |",
        "|--------|--:|--------:|-----------:|-----------:|--------:|--------:|--------:|----------:|--:|-----------:|",
    ]
    for r in sorted(results, key=lambda x: (x["nhc_every_n_ticks"] is None, x["nhc_every_n_ticks"] or 999)):
        n = "∞" if r["nhc_every_n_ticks"] is None else str(r["nhc_every_n_ticks"])
        g = r.get("gamma_nhc_over_predict", math.nan)
        g_s = f"{g:.1f}" if not math.isnan(g) else "—"
        t3 = r.get("top3_nhc_share_of_abs_drop", math.nan)
        t3_s = f"{100*t3:.0f}%" if not math.isnan(t3) else "—"
        lines.append(
            f"| {r['label']} | {n} | {r['gnss_accept_count']} | "
            f"{r.get('P_vv_pre_fix3', float('nan')):.1f} | {r.get('k_vel_mean_accepted', float('nan')):.4f} | "
            f"{r.get('k_vel_fix2', float('nan')):.4f} | {r.get('k_vel_fix3', float('nan')):.4f} | "
            f"{r.get('innov_h_mean_accepted', float('nan')):.1f} | "
            f"{r.get('sum_abs_delta_P_vv_nhc', float('nan')):.1f} | {g_s} | {t3_s} |"
        )
    lines += ["", "## Criterio de parada (N=1 vs N=10)", ""]
    if stopping.get("evaluated"):
        lines.append(f"**Verdict:** `{stopping['verdict']}`")
        lines.append("")
        lines.append(stopping["interpretation"])
        lines.append("")
        lines.append("Señales:")
        for k, v in stopping["signals"].items():
            lines.append(f"- {k}: {v}")
    lines.append("")
    SUMMARY_MD.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3.15 F1 NHC dose-response")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-run", action="store_true")
    parser.add_argument("--labels", type=str, nargs="*", default=None, help="subset e.g. baseline F1c OFF")
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    ensure_calibration(args.calibration)
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    cases = CASES
    if args.labels:
        labels = set(args.labels)
        cases = [c for c in CASES if c["label"] in labels]

    if not args.skip_run:
        for case in cases:
            run_case(args.replay_exe, replay_csv, args.calibration, case)

    results = []
    for case in cases:
        out_dir = OUT_DIR / case["label"]
        if not (out_dir / "gnss_nis_audit.csv").is_file():
            print(f"WARN: missing output for {case['label']}")
            continue
        results.append(analyze_case_dir(case, out_dir))

    stopping = evaluate_stopping_criterion(results)
    report = {
        "experiment": "GAP-3.15 F1 NHC dose-response",
        "config": "ZUPT OFF, sweep nhc_every_n_ticks + OFF",
        "cases": results,
        "stopping_criterion_n1_vs_n10": stopping,
    }

    def _json_default(o):
        if isinstance(o, float) and math.isnan(o):
            return None
        raise TypeError

    REPORT_JSON.write_text(json.dumps(report, indent=2, default=_json_default), encoding="utf-8")
    plot_dose_response(results)
    write_summary(results, stopping)

    print(json.dumps(report, indent=2))
    print(f"\nWrote {REPORT_JSON}")
    print(f"Wrote {SUMMARY_MD}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
