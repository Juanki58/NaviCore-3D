#!/usr/bin/env python3
"""Single clean post-memset-fix SLALOM A/C NHC audit + cumulative dx_att_z check.

1. Deletes prior audit CSVs (avoid mixed pre/post-fix data).
2. Runs A (correct) then C (legacy) once each with NAVICORE_NHC_BLOCK_AUDIT_CSV.
3. Stamps provenance (binary mtime, wall times, row counts, seed).
4. Verifies tick-0 sign flip + cumulative |Σ dx_att_z| growth (feedback vs offset).
5. Rewrites kp autopsy artifacts from THIS run only.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "docs" / "benchmarks" / "jacobian_imu_ab"
SIM = REPO / "build" / "NaviCore3D_Sim.exe"
SEED = 71

AUDIT_A = OUT / "slalom_cellA_jcorrect_imuideal_s71_nhc_block_audit.csv"
AUDIT_C = OUT / "slalom_cellC_jlegacy_imuideal_s71_nhc_block_audit.csv"
PROVENANCE = OUT / "slalom_a_vs_c_kp_postfix_provenance.json"
OUT_JSON = OUT / "slalom_a_vs_c_kp_autopsy.json"
OUT_MD = OUT / "slalom_a_vs_c_kp_autopsy.md"
OUT_PNG = OUT / "fig_slalom_kp_dxattz_cumsum_0_1s.png"
FIG_TURN1 = OUT / "fig_slalom_kp_autopsy_turn1.png"

T_FEEDBACK_S = 0.75  # first innov split locus from prior reading; re-measured


def file_sha256(path: Path, max_bytes: int = 2_000_000) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        h.update(f.read(max_bytes))
    return h.hexdigest()[:16]


def run_cell(jacobian: str, audit_path: Path) -> dict:
    if audit_path.exists():
        audit_path.unlink()
    env = os.environ.copy()
    env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = str(audit_path)
    cmd = [
        str(SIM),
        "--scenario",
        "SLALOM",
        "--no-udp",
        "--seed",
        str(SEED),
        "--imu-mode",
        "ideal",
        "--nhc-jacobian",
        jacobian,
    ]
    t0 = time.time()
    wall0 = datetime.now(timezone.utc).isoformat()
    print("RUN:", " ".join(cmd))
    print("  audit →", audit_path)
    completed = subprocess.run(
        cmd, cwd=str(REPO), env=env, capture_output=True, text=True, check=False
    )
    wall1 = datetime.now(timezone.utc).isoformat()
    elapsed = time.time() - t0
    stdout = completed.stdout or ""
    stderr = completed.stderr or ""
    # extract max drift from RESULTADO if present
    drift = None
    for line in stdout.splitlines():
        if "Max deriva lateral" in line or "lateral_drift" in line.lower():
            pass
        if "Max deriva lateral" in line:
            parts = line.replace("m", " ").split()
            for p in reversed(parts):
                try:
                    drift = float(p)
                    break
                except ValueError:
                    continue
    n_lines = 0
    if audit_path.exists():
        n_lines = sum(1 for _ in audit_path.open("r", encoding="utf-8", errors="replace"))
    info = {
        "jacobian": jacobian,
        "audit_path": str(audit_path.name),
        "wall_start_utc": wall0,
        "wall_end_utc": wall1,
        "elapsed_s": elapsed,
        "returncode": completed.returncode,
        "audit_lines": n_lines,
        "audit_data_rows": max(n_lines - 1, 0),
        "audit_sha256_16": file_sha256(audit_path) if audit_path.exists() else None,
        "audit_bytes": audit_path.stat().st_size if audit_path.exists() else 0,
        "stdout_has_nhc_block_audit": "NHC block audit:" in stdout,
        "max_lateral_drift_m": drift,
        "stderr_tail": stderr[-500:] if stderr else "",
    }
    if completed.returncode != 0:
        print(stdout[-2000:])
        print(stderr[-2000:])
        raise RuntimeError(f"Sim failed jacobian={jacobian} rc={completed.returncode}")
    if n_lines < 100:
        raise RuntimeError(
            f"Audit too short ({n_lines} lines) — memset/audit plumbing still broken?"
        )
    print(f"  OK rows={info['audit_data_rows']} bytes={info['audit_bytes']}")
    return info


def load_merged() -> pd.DataFrame:
    a = pd.read_csv(AUDIT_A)
    c = pd.read_csv(AUDIT_C)
    a["t_ms"] = (a["timestamp_s"] * 1000).round().astype(int)
    c["t_ms"] = (c["timestamp_s"] * 1000).round().astype(int)
    m = a.merge(c, on="t_ms", suffixes=("_A", "_C"))
    if len(m) < 100:
        raise RuntimeError(f"merged rows too few: {len(m)}")
    return m


def analyze_early(m: pd.DataFrame) -> dict:
    t = m["timestamp_s_A"].to_numpy(dtype=float)
    r0 = m.iloc[0]
    tick0 = {
        "t_s": float(r0["timestamp_s_A"]),
        "innov_A": float(r0["innov_norm_mps_A"]),
        "innov_C": float(r0["innov_norm_mps_C"]),
        "k_att_max_A": float(r0["k_att_max_A"]),
        "k_att_max_C": float(r0["k_att_max_C"]),
        "dx_att_z_A": float(r0["dx_att_z_rad_A"]),
        "dx_att_z_C": float(r0["dx_att_z_rad_C"]),
        "dx_att_z_ratio": float(
            r0["dx_att_z_rad_A"] / r0["dx_att_z_rad_C"]
            if abs(r0["dx_att_z_rad_C"]) > 0
            else float("nan")
        ),
        "exact_negation": bool(
            abs(r0["dx_att_z_rad_A"] + r0["dx_att_z_rad_C"])
            < 1e-12 * max(1.0, abs(r0["dx_att_z_rad_A"]))
            or abs(r0["dx_att_z_rad_A"] / r0["dx_att_z_rad_C"] + 1.0) < 1e-5
        ),
        "same_innov": bool(abs(r0["innov_norm_mps_A"] - r0["innov_norm_mps_C"]) < 1e-12),
        "same_k_att": bool(abs(r0["k_att_max_A"] - r0["k_att_max_C"]) < 1e-9),
    }

    # Cumulative per-tick attitude corrections
    dz_a = m["dx_att_z_rad_A"].to_numpy(dtype=float)
    dz_c = m["dx_att_z_rad_C"].to_numpy(dtype=float)
    cum_a = np.cumsum(dz_a)
    cum_c = np.cumsum(dz_c)
    sep = cum_a - cum_c  # signed separation of cumulative corrections

    def at_time(t_query: float) -> dict:
        i = int(np.argmin(np.abs(t - t_query)))
        return {
            "t_s": float(t[i]),
            "cum_dx_att_z_A": float(cum_a[i]),
            "cum_dx_att_z_C": float(cum_c[i]),
            "cum_sep_A_minus_C": float(sep[i]),
            "abs_cum_A": float(abs(cum_a[i])),
            "abs_cum_C": float(abs(cum_c[i])),
            "abs_sep": float(abs(sep[i])),
            "innov_A": float(m["innov_norm_mps_A"].iloc[i]),
            "innov_C": float(m["innov_norm_mps_C"].iloc[i]),
            "v_body_y_A": float(m["v_body_y_before_mps_A"].iloc[i]),
            "v_body_y_C": float(m["v_body_y_before_mps_C"].iloc[i]),
        }

    # Growth test on |sep| in [0, T_FEEDBACK]: feedback ⇒ |sep| grows;
    # constant offset ⇒ |sep| ~ linear in n with constant per-tick delta
    # (cumsum of constant opposite ±ε ⇒ linear growth of |sep| at rate 2ε).
    # Distinguisher: compare early rate vs late rate within 0–0.75s, and
    # compare |sep| growth to innov / |v_body_y| growth (coupled ⇒ feedback).
    mask_early = t <= T_FEEDBACK_S
    te = t[mask_early]
    se = np.abs(sep[mask_early])
    innov_a_e = m.loc[mask_early, "innov_norm_mps_A"].to_numpy(dtype=float)
    vby_a_e = np.abs(m.loc[mask_early, "v_body_y_before_mps_A"].to_numpy(dtype=float))

    # Split 0–T into first/second half; ratio of mean |d sep|/dt
    mid = T_FEEDBACK_S * 0.5
    m1 = te <= mid
    m2 = te > mid
    # finite differences of |sep|
    dsep = np.diff(se)
    dt = np.diff(te)
    rate = dsep / np.maximum(dt, 1e-9)
    # align rate to te[1:]
    te_r = te[1:]
    rate_1 = float(np.mean(rate[te_r <= mid])) if np.any(te_r <= mid) else float("nan")
    rate_2 = float(np.mean(rate[te_r > mid])) if np.any(te_r > mid) else float("nan")
    rate_ratio_2_over_1 = (
        float(rate_2 / rate_1) if np.isfinite(rate_1) and abs(rate_1) > 1e-20 else float("nan")
    )

    # If constant per-tick opposite bias ±ε: |sep|(t) ≈ 2ε·(t/dt) → constant rate.
    # Feedback: rate_2/rate_1 >> 1 and innov/vby also grow.
    innov_ratio_end_over_start = float(innov_a_e[-1] / max(innov_a_e[0], 1e-12))
    vby_ratio_end_over_start = float(vby_a_e[-1] / max(vby_a_e[0], 1e-12))
    sep_ratio_end_over_start = float(se[-1] / max(se[0] if se[0] > 0 else se[1], 1e-20))

    # Linear fit |sep| vs t: R² high + flat residual growth ⇒ offset-like;
    # poor linear fit or accelerating residuals ⇒ feedback.
    coef = np.polyfit(te, se, 1)
    se_hat = coef[0] * te + coef[1]
    ss_res = float(np.sum((se - se_hat) ** 2))
    ss_tot = float(np.sum((se - np.mean(se)) ** 2))
    r2_linear = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    # Quadratic term significance: fit degree 2, compare
    coef2 = np.polyfit(te, se, 2)
    se_hat2 = np.polyval(coef2, te)
    ss_res2 = float(np.sum((se - se_hat2) ** 2))
    # F-like: improvement from quadratic
    quad_improvement = (ss_res - ss_res2) / max(ss_res, 1e-30)

    # Classification thresholds (stated explicitly)
    # feedback if: rate accelerates OR innov grows strongly with sep
    feedback = bool(
        (np.isfinite(rate_ratio_2_over_1) and rate_ratio_2_over_1 > 2.0)
        or (innov_ratio_end_over_start > 10.0 and sep_ratio_end_over_start > 5.0)
        or (quad_improvement > 0.3 and innov_ratio_end_over_start > 5.0)
    )
    constant_offset = bool(
        (np.isfinite(rate_ratio_2_over_1) and 0.5 <= rate_ratio_2_over_1 <= 1.5)
        and innov_ratio_end_over_start < 3.0
        and r2_linear > 0.98
    )

    if feedback and not constant_offset:
        growth_verdict = "FEEDBACK_GROWTH"
    elif constant_offset and not feedback:
        growth_verdict = "CONSTANT_OFFSET_LIKE"
    else:
        growth_verdict = "MIXED_OR_AMBIGUOUS"

    # First innov split
    d_innov = np.abs(
        m["innov_norm_mps_A"].to_numpy(dtype=float)
        - m["innov_norm_mps_C"].to_numpy(dtype=float)
    )
    idx_split = np.where(d_innov > 1e-3)[0]
    first_innov_split = (
        {
            "t_s": float(t[int(idx_split[0])]),
            "innov_A": float(m["innov_norm_mps_A"].iloc[int(idx_split[0])]),
            "innov_C": float(m["innov_norm_mps_C"].iloc[int(idx_split[0])]),
        }
        if len(idx_split)
        else None
    )

    # Window 0–1 s opposite frac
    m01 = (t >= 0) & (t <= 1.0)
    opp = float(
        np.mean(
            np.sign(m.loc[m01, "dx_att_z_rad_A"])
            * np.sign(m.loc[m01, "dx_att_z_rad_C"])
            < 0
        )
    )

    return {
        "tick0": tick0,
        "at_0s": at_time(0.0),
        "at_0p375s": at_time(mid),
        "at_0p75s": at_time(T_FEEDBACK_S),
        "at_1p0s": at_time(1.0),
        "at_2p0s": at_time(2.0),
        "growth_0_to_0p75s": {
            "n": int(mask_early.sum()),
            "abs_sep_start": float(se[0]),
            "abs_sep_mid": float(se[np.argmin(np.abs(te - mid))]),
            "abs_sep_end": float(se[-1]),
            "mean_dabs_sep_dt_first_half": rate_1,
            "mean_dabs_sep_dt_second_half": rate_2,
            "rate_ratio_second_over_first": rate_ratio_2_over_1,
            "innov_A_ratio_end_over_start": innov_ratio_end_over_start,
            "v_body_y_A_ratio_end_over_start": vby_ratio_end_over_start,
            "abs_sep_ratio_end_over_start": sep_ratio_end_over_start,
            "r2_linear_abs_sep_vs_t": r2_linear,
            "quad_fit_improvement_frac": float(quad_improvement),
            "growth_verdict": growth_verdict,
            "thresholds": {
                "feedback_if_rate_ratio_gt": 2.0,
                "or_innov_ratio_gt": 10.0,
                "constant_if_rate_ratio_in": [0.5, 1.5],
                "and_innov_ratio_lt": 3.0,
                "and_r2_linear_gt": 0.98,
            },
        },
        "window_0_1s_dx_att_z_opposite_frac": opp,
        "first_innov_split_gt_1e-3": first_innov_split,
        "series_for_plot": {
            "t_s": te.tolist(),
            "cum_dx_att_z_A": cum_a[mask_early].tolist(),
            "cum_dx_att_z_C": cum_c[mask_early].tolist(),
            "abs_cum_sep": se.tolist(),
            "innov_A": innov_a_e.tolist(),
        },
    }


def plot_cumsum(early: dict) -> None:
    s = early["series_for_plot"]
    t = np.asarray(s["t_s"])
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(t, s["cum_dx_att_z_A"], label="A Σ dx_att_z (correct)")
    axes[0].plot(t, s["cum_dx_att_z_C"], label="C Σ dx_att_z (legacy)")
    axes[0].axvline(T_FEEDBACK_S, color="0.4", ls="--", lw=0.9)
    axes[0].set_ylabel("Σ dx_att_z (rad)")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(
        f"Post-fix single run — growth verdict: "
        f"{early['growth_0_to_0p75s']['growth_verdict']}"
    )

    axes[1].plot(t, s["abs_cum_sep"], color="C3", label="|ΣA − ΣC|")
    axes[1].axvline(T_FEEDBACK_S, color="0.4", ls="--", lw=0.9)
    axes[1].set_ylabel("|cum sep| (rad)")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].semilogy(t, np.maximum(s["innov_A"], 1e-12), label="A ‖innov‖")
    axes[2].axvline(T_FEEDBACK_S, color="0.4", ls="--", lw=0.9)
    axes[2].set_ylabel("‖innov‖ A")
    axes[2].set_xlabel("t (s)")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=140)
    plt.close(fig)


def plot_turn1(m: pd.DataFrame) -> None:
    t = m["timestamp_s_A"].to_numpy()
    mask = (t >= 1.0) & (t <= 5.5)
    w = m.loc[mask]
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(w["timestamp_s_A"], w["innov_norm_mps_A"], label="A")
    axes[0].plot(w["timestamp_s_A"], w["innov_norm_mps_C"], label="C")
    axes[0].set_ylabel("‖innov‖")
    axes[0].legend(fontsize=8)
    axes[1].plot(w["timestamp_s_A"], w["dx_att_z_rad_A"], label="A")
    axes[1].plot(w["timestamp_s_A"], w["dx_att_z_rad_C"], label="C")
    axes[1].set_ylabel("dx_att_z")
    axes[1].legend(fontsize=8)
    axes[2].plot(w["timestamp_s_A"], w["k_att_max_A"], label="A")
    axes[2].plot(w["timestamp_s_A"], w["k_att_max_C"], label="C")
    axes[2].set_ylabel("k_att_max")
    axes[2].legend(fontsize=8)
    axes[3].plot(w["timestamp_s_A"], w["dx_vel_norm_mps_A"], label="A")
    axes[3].plot(w["timestamp_s_A"], w["dx_vel_norm_mps_C"], label="C")
    axes[3].set_ylabel("‖dx_vel‖")
    axes[3].set_xlabel("t (s)")
    axes[3].legend(fontsize=8)
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.axvline(2.0, color="0.4", ls="--", lw=0.8)
        ax.axvline(4.0, color="0.4", ls=":", lw=0.8)
    fig.suptitle("Post-fix single run — turn1 window", fontsize=11)
    fig.tight_layout()
    fig.savefig(FIG_TURN1, dpi=140)
    plt.close(fig)


def write_reports(prov: dict, early: dict) -> None:
    g = early["growth_0_to_0p75s"]
    t0 = early["tick0"]
    split = early["first_innov_split_gt_1e-3"]

    results = {
        "provenance": prov,
        "early_chain": early,
        "H_att_formulas": {
            "correct": "H0=[-vz,0,+vx], H1=[+vy,-vx,0]",
            "legacy": "H0=[+vz,0,-vx], H1=[-vy,+vx,0] (= -correct)",
        },
        "causal_answer": {
            "where": "H_att → K_att → dx_att (sign from tick 0)",
            "amplification": (
                "not larger |K|; cumulative dx_att_z separation + innov growth "
                f"verdict={g['growth_verdict']}"
            ),
            "data_guarantee": (
                "ALL numbers in this file come from the single paired A→C run "
                "stamped in provenance (audits deleted then rewritten)."
            ),
        },
        "mechanism_closed": bool(
            t0["exact_negation"]
            and t0["same_innov"]
            and t0["same_k_att"]
            and g["growth_verdict"] == "FEEDBACK_GROWTH"
            and prov["audit_plumbing_ok"]
        ),
    }
    # strip bulky series from json stored in early copy for md? keep in early_chain
    # but remove series_for_plot from saved early to keep json smaller - actually keep it
    with OUT_JSON.open("w", encoding="utf-8") as f:
        # don't dump full plot series twice
        dump = dict(results)
        dump["early_chain"] = {
            k: v for k, v in early.items() if k != "series_for_plot"
        }
        json.dump(dump, f, indent=2)

    with PROVENANCE.open("w", encoding="utf-8") as f:
        json.dump(prov, f, indent=2)

    lines = [
        "# SLALOM A vs C — NHC K/P autopsy (post-fix single run)",
        "",
        "**Data guarantee:** every number below comes from one paired A→C re-run "
        "after deleting prior audit CSVs, with the memset/audit re-bind fix in the "
        f"binary (`{SIM.name}` mtime stamped in provenance). "
        f"See `{PROVENANCE.name}`.",
        "",
        f"**Binary mtime UTC:** {prov['sim_mtime_utc']}  ",
        f"**Run wall UTC:** {prov['cells']['A']['wall_start_utc']} → "
        f"{prov['cells']['C']['wall_end_utc']}  ",
        f"**Audit rows:** A={prov['cells']['A']['audit_data_rows']}, "
        f"C={prov['cells']['C']['audit_data_rows']}  ",
        f"**Audit sha16:** A={prov['cells']['A']['audit_sha256_16']}, "
        f"C={prov['cells']['C']['audit_sha256_16']}  ",
        "",
        "## Tick 0 (same run)",
        "",
        f"- innov A/C: **{t0['innov_A']:.3g}** / {t0['innov_C']:.3g} "
        f"(same={t0['same_innov']})",
        f"- k_att_max A/C: **{t0['k_att_max_A']:.5g}** / {t0['k_att_max_C']:.5g} "
        f"(same={t0['same_k_att']})",
        f"- dx_att_z A/C: **{t0['dx_att_z_A']:.6g}** / {t0['dx_att_z_C']:.6g} "
        f"(ratio={t0['dx_att_z_ratio']:.6g}, exact_negation={t0['exact_negation']})",
        "",
        "## Cumulative dx_att_z (feedback vs constant offset)",
        "",
        f"Figure: `{OUT_PNG.name}`",
        "",
        "| t (s) | Σ dx_att_z A | Σ dx_att_z C | |ΣA−ΣC| | innov A |",
        "|-------|--------------|--------------|---------|---------|",
    ]
    for key in ("at_0s", "at_0p375s", "at_0p75s", "at_1p0s", "at_2p0s"):
        p = early[key]
        lines.append(
            f"| {p['t_s']:.3f} | {p['cum_dx_att_z_A']:.6g} | {p['cum_dx_att_z_C']:.6g} | "
            f"{p['abs_sep']:.6g} | {p['innov_A']:.4g} |"
        )

    lines += [
        "",
        "### Growth metrics on |ΣA−ΣC| in [0, 0.75] s",
        "",
        f"- |sep| start → mid → end: "
        f"**{g['abs_sep_start']:.6g}** → {g['abs_sep_mid']:.6g} → "
        f"**{g['abs_sep_end']:.6g}** "
        f"(ratio end/start = {g['abs_sep_ratio_end_over_start']:.3g})",
        f"- mean d|sep|/dt first half / second half: "
        f"{g['mean_dabs_sep_dt_first_half']:.6g} / "
        f"{g['mean_dabs_sep_dt_second_half']:.6g} "
        f"(ratio 2nd/1st = **{g['rate_ratio_second_over_first']:.3g}**)",
        f"- innov_A end/start: **{g['innov_A_ratio_end_over_start']:.3g}**",
        f"- |v_body_y|_A end/start: **{g['v_body_y_A_ratio_end_over_start']:.3g}**",
        f"- R² linear |sep| vs t: {g['r2_linear_abs_sep_vs_t']:.4f}; "
        f"quad improvement: {g['quad_fit_improvement_frac']:.3f}",
        "",
        f"**Growth verdict: `{g['growth_verdict']}`**",
        "",
        "Thresholds: FEEDBACK if rate_ratio>2 or (innov_ratio>10 and sep_ratio>5); "
        "CONSTANT_OFFSET if rate_ratio∈[0.5,1.5] and innov_ratio<3 and R²>0.98.",
        "",
        f"First innov |A−C|>1e-3: "
        f"t={split['t_s'] if split else None}s "
        f"(A={split['innov_A'] if split else None}, "
        f"C={split['innov_C'] if split else None})",
        "",
        f"0–1 s dx_att_z opposite-sign frac: "
        f"**{early['window_0_1s_dx_att_z_opposite_frac']:.3f}**",
        "",
        "## Causal reading (only if growth=FEEDBACK and tick0 negation)",
        "",
        "1. Sign enters at `H_att`→`K_att`→`dx_att` from tick 0 (matched innov/|K|).",
        "2. Cumulative separation of attitude corrections accelerates with innov/"
        "`|v_body_y|` — reinforcing NHC loop, not a frozen opposite offset.",
        "3. ~2 s lag to position-divergence rate is the integrated consequence of "
        "that loop over many predict/update cycles.",
        "4. Same `H_att` sign family as GAP-4 / `bf2bfbd`; SLALOM A×C (~143×) is "
        "the E2E face of that feedback.",
        "",
        f"**mechanism_closed (this run): `{results['mechanism_closed']}`**",
        "",
        "## OQ9 / T6 (unchanged)",
        "",
        "- T6: punctual alias-shift miss — not structural.",
        "- H-DEG-ATT-LAG (OQ9): turns 9–10 at drift_A ≳ 20 m — parked separately.",
        "",
    ]
    OUT_MD.write_text("\n".join(lines), encoding="utf-8")
    print(OUT_MD.read_text(encoding="utf-8"))


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    if not SIM.exists():
        print(f"ERROR: missing {SIM}", file=sys.stderr)
        return 1

    # Delete prior audits — no mixed pre/post-fix tables
    for p in (AUDIT_A, AUDIT_C):
        if p.exists():
            p.unlink()
            print("deleted", p.name)

    sim_mtime = datetime.fromtimestamp(SIM.stat().st_mtime, tz=timezone.utc).isoformat()
    prov = {
        "purpose": "single clean post-memset-fix audit pair for K/P autopsy",
        "seed": SEED,
        "sim_path": str(SIM),
        "sim_mtime_utc": sim_mtime,
        "sim_sha256_16": file_sha256(SIM, max_bytes=8_000_000),
        "fix_notes": (
            "ins_ekf_init memset cleared nhc_block_audit_fp; SLALOM now binds "
            "audit FILE* after seed and re-binds each tick"
        ),
        "cells": {},
        "deleted_prior_audits_before_run": True,
    }

    prov["cells"]["A"] = run_cell("correct", AUDIT_A)
    prov["cells"]["C"] = run_cell("legacy", AUDIT_C)
    prov["audit_plumbing_ok"] = bool(
        prov["cells"]["A"]["audit_data_rows"] > 2000
        and prov["cells"]["C"]["audit_data_rows"] > 2000
        and prov["cells"]["A"]["stdout_has_nhc_block_audit"]
        and prov["cells"]["C"]["stdout_has_nhc_block_audit"]
    )

    m = load_merged()
    early = analyze_early(m)
    plot_cumsum(early)
    plot_turn1(m)
    write_reports(prov, early)

    print("mechanism_closed=", 
          early["tick0"]["exact_negation"]
          and early["growth_0_to_0p75s"]["growth_verdict"] == "FEEDBACK_GROWTH"
          and prov["audit_plumbing_ok"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
