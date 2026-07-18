#!/usr/bin/env python3
"""GAP-3.10 — Autopsia de los 7 fixes GNSS aceptados (exp B: ZUPT OFF, NHC ON).

Por cada fix aceptado construye una ficha con ventana [-2s, +0.5s]:
  innov/S/NIS/gate, P pre/post, K, dx, v, acumulados NHC/predict/ZUPT.

No añade escenarios ni matrices — sólo reconstrucción causal de eventos críticos.
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
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap3_gnss_accepted_autopsy"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
REPORT_JSON = BENCH_DIR / "gap3_gnss_accepted_autopsy_report.json"
SUMMARY_MD = BENCH_DIR / "gap3_gnss_accepted_autopsy.md"

PRE_WINDOW_S = 2.0
POST_WINDOW_S = 0.5

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


def vec_norm3(row: pd.Series, prefix: str) -> float:
    return float(math.sqrt(row[f"{prefix}_n"] ** 2 + row[f"{prefix}_e"] ** 2 + row[f"{prefix}_d"] ** 2))


def run_replay(replay_exe: Path, replay_csv: Path, calibration: Path, out_dir: Path) -> None:
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
        "enabled",
        "--output",
        str(out_dir / "replay_output.csv"),
        "--gap3-gnss-nis-audit-csv",
        str(out_dir / "gnss_nis_audit.csv"),
        "--gap3-nhc-block-audit-csv",
        str(out_dir / "nhc_block_audit.csv"),
        "--gap3-cov-step-audit-csv",
        str(out_dir / "cov_step_audit.csv"),
        "--gap3-cov-propagation-audit-csv",
        str(out_dir / "cov_propagation_audit.csv"),
        "--gap3-constraint-pipeline-audit-csv",
        str(out_dir / "constraint_pipeline_audit.csv"),
        "--gap3-gnss-k-block-audit-json",
        str(out_dir / "gnss_k_block.jsonl"),
    ]
    print("RUN:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)


def load_k_block_jsonl(path: Path) -> list[dict]:
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


def window_slice(df: pd.DataFrame, t0: float, pre: float, post: float) -> pd.DataFrame:
    if df.empty or "timestamp_s" not in df.columns:
        return df.iloc[0:0]
    return df[(df["timestamp_s"] >= t0 - pre) & (df["timestamp_s"] <= t0 + post)].copy()


def sum_pipeline_dv(pipeline: pd.DataFrame) -> dict:
    if pipeline.empty:
        return {}
    def _sum(col: str) -> float:
        if col not in pipeline.columns:
            return 0.0
        return float(np.sqrt((pipeline[col] ** 2).sum()))

    return {
        "n_imu_ticks": int(len(pipeline)),
        "sum_dv_predict_norm": _sum("dv_pred_n") if "dv_pred_n" in pipeline.columns else None,
        "sum_dv_nhc_norm": float(
            np.sqrt(
                pipeline.get("dv_nhc_n", 0) ** 2
                + pipeline.get("dv_nhc_e", 0) ** 2
                + pipeline.get("dv_nhc_d", 0) ** 2
            ).sum()
        )
        if "dv_nhc_n" in pipeline.columns
        else None,
        "sum_dv_zupt_norm": float(
            np.sqrt(
                pipeline.get("dv_zupt_n", 0) ** 2
                + pipeline.get("dv_zupt_e", 0) ** 2
                + pipeline.get("dv_zupt_d", 0) ** 2
            ).sum()
        )
        if "dv_zupt_n" in pipeline.columns
        else None,
        "nhc_applied_ticks": int(pipeline["nhc_applied"].sum()) if "nhc_applied" in pipeline.columns else None,
        "zupt_applied_ticks": int(pipeline["zupt_applied"].sum()) if "zupt_applied" in pipeline.columns else None,
    }


def sum_nhc_window(nhc: pd.DataFrame) -> dict:
    if nhc.empty:
        return {}
    return {
        "n_nhc_updates": int(len(nhc)),
        "sum_abs_delta_P_vv_frob": float(nhc["delta_P_vv_frob"].abs().sum())
        if "delta_P_vv_frob" in nhc.columns
        else None,
        "sum_abs_delta_P_pv_frob": float(nhc["delta_P_pv_frob"].abs().sum())
        if "delta_P_pv_frob" in nhc.columns
        else None,
        "sum_abs_delta_P_aa_frob": float(nhc["delta_P_aa_frob"].abs().sum())
        if "delta_P_aa_frob" in nhc.columns
        else None,
        "sum_dx_vel_norm": float(nhc["dx_vel_norm_mps"].sum()) if "dx_vel_norm_mps" in nhc.columns else None,
        "P_vv_frob_at_last_nhc_pre": float(nhc.iloc[-1]["P_pre_vv_frob"])
        if "P_pre_vv_frob" in nhc.columns
        else None,
        "P_pv_frob_at_last_nhc_pre": float(nhc.iloc[-1]["P_pre_pv_frob"])
        if "P_pre_pv_frob" in nhc.columns
        else None,
        "P_vv_frob_at_last_nhc_post": float(nhc.iloc[-1]["P_post_vv_frob"])
        if "P_post_vv_frob" in nhc.columns
        else None,
    }


def gnss_cov_at_fix(cov_step: pd.DataFrame, cov_prop: pd.DataFrame, t: float) -> dict:
    out: dict = {}
    if not cov_step.empty:
        gnss = cov_step[cov_step["update_type"] == "gnss"]
        pre = gnss[(gnss["phase"] == "pre") & (np.isclose(gnss["timestamp_s"], t, atol=0.05))]
        post = gnss[(gnss["phase"] == "post_accept") & (np.isclose(gnss["timestamp_s"], t, atol=0.05))]
        if len(pre):
            r = pre.iloc[-1]
            out["P_pre_gnss"] = {
                "P_pp_frob": float(r.get("P_pp_frob", np.nan)),
                "P_vv_frob": float(r.get("P_vv_frob", np.nan)),
                "P_pv_frob": float(r.get("P_pv_frob", np.nan)),
                "P_aa_frob": float(r.get("P_aa_frob", np.nan)),
            }
        if len(post):
            r = post.iloc[-1]
            out["P_post_gnss"] = {
                "P_pp_frob": float(r.get("P_pp_frob", np.nan)),
                "P_vv_frob": float(r.get("P_vv_frob", np.nan)),
                "P_pv_frob": float(r.get("P_pv_frob", np.nan)),
                "P_aa_frob": float(r.get("P_aa_frob", np.nan)),
            }
        if "P_pre_gnss" in out and "P_post_gnss" in out:
            out["delta_P_gnss"] = {
                k: out["P_post_gnss"][k] - out["P_pre_gnss"][k]
                for k in out["P_pre_gnss"]
            }
    if not cov_prop.empty:
        pre = cov_prop[(cov_prop["event"] == "gnss_pre") & (np.isclose(cov_prop["timestamp_s"], t, atol=0.05))]
        post = cov_prop[
            (cov_prop["event"] == "gnss_post") & (np.isclose(cov_prop["timestamp_s"], t, atol=0.05))
        ]
        if len(pre):
            r = pre.iloc[-1]
            out["P_vel_pos_max_pre"] = float(r.get("P_vel_pos_max", np.nan))
            out["K_vel_pos_max_pre"] = float(r.get("K_vel_pos_max", np.nan))
        if len(post):
            r = post.iloc[-1]
            out["K_vel_pos_max_post"] = float(r.get("K_vel_pos_max", np.nan))
    return out


def build_fix_dossier(
    fix_idx: int,
    row: pd.Series,
    nhc: pd.DataFrame,
    pipeline: pd.DataFrame,
    cov_step: pd.DataFrame,
    cov_prop: pd.DataFrame,
    k_blocks: list[dict],
) -> dict:
    t = float(row["timestamp_s"])
    gps_index = int(row.get("gps_index", fix_idx))

    win_nhc = window_slice(nhc, t, PRE_WINDOW_S, POST_WINDOW_S)
    win_pipe = window_slice(pipeline, t, PRE_WINDOW_S, POST_WINDOW_S)

    k_match = next(
        (k for k in k_blocks if abs(k.get("timestamp_s", -1) - t) < 0.05),
        None,
    )

    vel_before = {
        "n_mps": float(row.get("vel_pred_n_mps", np.nan)),
        "e_mps": float(row.get("vel_pred_e_mps", np.nan)),
        "d_mps": float(row.get("vel_pred_d_mps", np.nan)),
        "h_mps": float(row.get("vel_pred_h_mps", np.nan)),
    }
    vel_after = {
        "n_mps": float(row.get("vel_after_n_mps", np.nan)),
        "e_mps": float(row.get("vel_after_e_mps", np.nan)),
        "h_mps": float(row.get("vel_after_h_mps", np.nan)),
    }

    dossier = {
        "fix_number": fix_idx + 1,
        "gps_index": gps_index,
        "timestamp_s": t,
        "window_s": {"pre": PRE_WINDOW_S, "post": POST_WINDOW_S},
        "innovation": {
            "innov_n_m": float(row.get("innov_n_m", np.nan)),
            "innov_e_m": float(row.get("innov_e_m", np.nan)),
            "innov_d_m": float(row.get("innov_d_m", np.nan)),
            "innov_h_m": float(row.get("innov_h_m", np.nan)),
            "pred_error_3d_m": float(row.get("pred_error_3d_m", np.nan)),
        },
        "S_and_gate": {
            "s_nn": float(row.get("s_nn", np.nan)),
            "s_ee": float(row.get("s_ee", np.nan)),
            "s_dd": float(row.get("s_dd", np.nan)),
            "s_ne": float(row.get("s_ne", np.nan)),
            "s_eigmin": float(row.get("s_eigmin", np.nan)),
            "s_eigmax": float(row.get("s_eigmax", np.nan)),
            "s_cond": float(row.get("s_cond", np.nan)),
            "nis_full": float(row.get("nis_full", np.nan)),
            "nis_horizontal_2d": float(row.get("nis_horizontal_2d", np.nan)),
            "nis_contrib_n": float(row.get("nis_contrib_n", np.nan)),
            "nis_contrib_e": float(row.get("nis_contrib_e", np.nan)),
            "nis_contrib_d": float(row.get("nis_contrib_d", np.nan)),
            "nis_threshold": float(row.get("nis_threshold", np.nan)),
            "accepted": int(row.get("accepted", 0)),
        },
        "K_at_fix": {
            "k_pos_max": float(row.get("k_pos_max", np.nan)),
            "k_vel_max": float(row.get("k_vel_max", np.nan)),
            "k_att_max": float(row.get("k_att_max", np.nan)),
        },
        "delta_x_at_fix": {
            "dx_pos_n_m": float(row.get("dx_pos_n_m", np.nan)),
            "dx_pos_e_m": float(row.get("dx_pos_e_m", np.nan)),
            "dx_pos_d_m": float(row.get("dx_pos_d_m", np.nan)),
            "dx_vel_n_mps": float(row.get("dx_vel_n_mps", np.nan)),
            "dx_vel_e_mps": float(row.get("dx_vel_e_mps", np.nan)),
            "dx_vel_d_mps": float(row.get("dx_vel_d_mps", np.nan)),
            "corr_pos_h_m": float(row.get("corr_pos_h_m", np.nan)),
            "corr_vel_h_mps": float(row.get("corr_vel_h_mps", np.nan)),
        },
        "velocity": {"before": vel_before, "after": vel_after},
        "gps_speed_mps": float(row.get("gps_speed_mps", np.nan)),
        "pre_window_2s": {
            "nhc_accum": sum_nhc_window(win_nhc),
            "pipeline_accum": sum_pipeline_dv(win_pipe),
        },
        "cov_at_fix": gnss_cov_at_fix(cov_step, cov_prop, t),
    }

    if k_match:
        dossier["k_block"] = {
            "P_vel_pos_frob": float(
                np.linalg.norm(k_match.get("P_vel_pos_cross_m2", [[0]]), ord="fro")
            )
            if "P_vel_pos_cross_m2" in k_match
            else None,
            "K_vel_pos_frob": float(
                np.linalg.norm(k_match.get("K_vel_pos", [[0]]), ord="fro")
            )
            if "K_vel_pos" in k_match
            else None,
            "dx_bias_accel_norm": k_match.get("delta_x", {}).get("bias_accel_norm"),
            "dx_bias_gyro_norm": k_match.get("delta_x", {}).get("bias_gyro_norm"),
        }

    dossier["causal_reading"] = classify_fix(dossier)
    return dossier


def classify_fix(d: dict) -> str:
    """Hipótesis tentativa — no afirma ΔP→K sin cadena completa."""
    nhc = d.get("pre_window_2s", {}).get("nhc_accum", {})
    k_vel = d.get("K_at_fix", {}).get("k_vel_max")
    innov_h = d.get("innovation", {}).get("innov_h_m")
    nis = d.get("S_and_gate", {}).get("nis_horizontal_2d")
    s_cond = d.get("S_and_gate", {}).get("s_cond")

    parts = []
    if nhc.get("n_nhc_updates", 0) > 100:
        parts.append(f"{nhc['n_nhc_updates']} NHC updates in -2s window")
    if nhc.get("sum_abs_delta_P_vv_frob") is not None:
        parts.append(f"Σ|ΔP_vv|_NHC={nhc['sum_abs_delta_P_vv_frob']:.3f}")
    if k_vel is not None and not math.isnan(k_vel):
        parts.append(f"k_vel_max={k_vel:.4f}")
    if innov_h is not None and not math.isnan(innov_h):
        parts.append(f"innov_h={innov_h:.1f}m")
    if nis is not None and not math.isnan(nis):
        parts.append(f"NIS_2d={nis:.2f}")
    if s_cond is not None and not math.isnan(s_cond):
        parts.append(f"cond(S)={s_cond:.1f}")

    if not parts:
        return "INSUFFICIENT_DATA"
    return "; ".join(parts)


def plot_fix_autopsy(dossier: dict, nhc: pd.DataFrame, out_png: Path) -> None:
    t0 = dossier["timestamp_s"]
    win = window_slice(nhc, t0, PRE_WINDOW_S, POST_WINDOW_S)
    if win.empty:
        return

    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    t = win["timestamp_s"].values

    ax = axes[0]
    ax.plot(t, win["P_pre_vv_frob"], label="P_vv pre", lw=0.8)
    ax.plot(t, win["P_post_vv_frob"], label="P_vv post", lw=0.8)
    ax.plot(t, win["P_pre_pv_frob"], label="P_pv pre", lw=0.8, ls="--", alpha=0.7)
    ax.axvline(t0, color="k", ls=":", lw=1.0, label="GNSS fix")
    ax.set_ylabel("||P||")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.25)

    ax = axes[1]
    ax.plot(t, win.get("delta_P_vv_frob", 0).abs(), label="|ΔP_vv|", color="#d62728", lw=0.7)
    ax.plot(t, win.get("dx_vel_norm_mps", 0), label="|dx_vel|", color="#9467bd", lw=0.7)
    ax.axvline(t0, color="k", ls=":", lw=1.0)
    ax.set_ylabel("ΔP / dx")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.25)

    ax = axes[2]
    ax.plot(t, win.get("k_vel_max", 0), label="NHC k_vel_max", lw=0.7)
    ax.axhline(dossier["K_at_fix"]["k_vel_max"], color="g", ls="--", lw=0.9, label="GNSS k_vel@fix")
    ax.axvline(t0, color="k", ls=":", lw=1.0)
    ax.set_ylabel("K")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.25)

    ax = axes[3]
    ax.plot(t, win.get("v_body_x_before_mps", 0), label="vx before", lw=0.7)
    ax.plot(t, win.get("v_body_x_after_mps", 0), label="vx after", lw=0.7, alpha=0.8)
    ax.axvline(t0, color="k", ls=":", lw=1.0)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("v_body [m/s]")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.25)

    fig.suptitle(
        f"Fix #{dossier['fix_number']} @ t={t0:.2f}s | innov_h={dossier['innovation']['innov_h_m']:.1f}m"
    )
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def write_summary_md(dossiers: list[dict], path: Path) -> None:
    lines = [
        "# GAP-3.10 — Autopsia fixes GNSS aceptados (exp B)",
        "",
        "Config: ZUPT OFF, NHC ON. Ventana pre-fix: -2 s.",
        "",
        "**Nota:** correlaciones reportadas; cadena ΔP→K no afirmada globalmente.",
        "",
    ]
    for d in dossiers:
        lines.append(f"## Fix #{d['fix_number']} — t={d['timestamp_s']:.3f} s (gps_index={d['gps_index']})")
        lines.append("")
        inv = d["innovation"]
        sg = d["S_and_gate"]
        lines.append(f"- innov_h={inv['innov_h_m']:.2f} m | NIS_2d={sg['nis_horizontal_2d']:.3f} | cond(S)={sg.get('s_cond', float('nan')):.1f}")
        lines.append(f"- k_vel_max={d['K_at_fix']['k_vel_max']:.5f} | k_pos_max={d['K_at_fix']['k_pos_max']:.5f}")
        nhc = d["pre_window_2s"]["nhc_accum"]
        lines.append(
            f"- Pre -2s: {nhc.get('n_nhc_updates', 0)} NHC updates, "
            f"Σ|ΔP_vv|={nhc.get('sum_abs_delta_P_vv_frob', float('nan')):.3f}, "
            f"P_vv_pre@last_NHC={nhc.get('P_vv_frob_at_last_nhc_pre', float('nan')):.3f}"
        )
        lines.append(f"- Lectura: {d['causal_reading']}")
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3.10 GNSS accepted-fix autopsy")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-run", action="store_true")
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    ensure_calibration(args.calibration)
    BENCH_DIR.mkdir(parents=True, exist_ok=True)

    if not args.skip_run:
        run_replay(args.replay_exe, replay_csv, args.calibration, BENCH_DIR)

    gnss = load_csv(BENCH_DIR / "gnss_nis_audit.csv")
    nhc = load_csv(BENCH_DIR / "nhc_block_audit.csv")
    pipeline = load_csv(BENCH_DIR / "constraint_pipeline_audit.csv")
    cov_step = load_csv(BENCH_DIR / "cov_step_audit.csv")
    cov_prop = load_csv(BENCH_DIR / "cov_propagation_audit.csv")
    k_blocks = load_k_block_jsonl(BENCH_DIR / "gnss_k_block.jsonl")

    accepted = gnss[gnss["accepted"] == 1].sort_values("timestamp_s").reset_index(drop=True)
    print(f"Accepted GNSS fixes: {len(accepted)}")

    dossiers = []
    for i, (_, row) in enumerate(accepted.iterrows()):
        d = build_fix_dossier(i, row, nhc, pipeline, cov_step, cov_prop, k_blocks)
        dossiers.append(d)
        plot_fix_autopsy(d, nhc, BENCH_DIR / f"fix_{i + 1:02d}_autopsy.png")

    report = {
        "experiment": "GAP-3.10 GNSS accepted-fix autopsy",
        "config": "ZUPT OFF (disabled), NHC ON (enabled)",
        "accepted_fix_count": len(dossiers),
        "pre_window_s": PRE_WINDOW_S,
        "post_window_s": POST_WINDOW_S,
        "fixes": dossiers,
        "methodology_note": (
            "Reports per-fix state and -2s NHC accumulation. "
            "Does NOT assert global ΔP→K causality without ruling out innov/S/linearization alternatives."
        ),
        "verdict": "PENDING_PER_FIX_CHAIN" if dossiers else "NO_ACCEPTED_FIXES",
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")
    write_summary_md(dossiers, SUMMARY_MD)
    print(json.dumps(report, indent=2))
    print(f"Wrote {REPORT_JSON}")
    print(f"Wrote {SUMMARY_MD}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
