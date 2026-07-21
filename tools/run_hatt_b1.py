#!/usr/bin/env python3
"""H-ATT-b1 sweep — preregistered in docs/diagnostics/18-jacobian-imu-ab-protocol.md §11.

λ ∈ {0, 0.3, 0.5, 0.7}; ATT_Z only; ρ=1.20 per scenario; no b2/a.
Do not retune thresholds after seeing results.
"""

from __future__ import annotations

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "docs" / "benchmarks" / "jacobian_imu_ab" / "hatt_b1"
SEED = 71
LAMBDAS = (0.0, 0.3, 0.5, 0.7)
CELLS = (
    ("A", "correct", "ideal"),
    ("B", "correct", "dirty"),
    ("C", "legacy", "ideal"),
    ("D", "legacy", "dirty"),
)
RHO = 1.20
P1_ABS_M = 2.0
P1_IMPROVE = 10.0
sys.path.insert(0, str(REPO))
from run_all_benchmarks import run_benchmark  # noqa: E402


def metric_val(result, name: str):
    for metric in result.metrics:
        if metric.name == name:
            return metric.measured
    return None


def run_cell(
    cell_id: str,
    jac: str,
    imu: str,
    lam: float,
    *,
    scenario: str,
    audit_path: Path | None = None,
) -> dict:
    suffix = f"hatt_b1_l{lam:g}_cell{cell_id}_j{jac}_imu{imu}_s{SEED}"
    env = os.environ.copy()
    if audit_path is not None:
        audit_path.parent.mkdir(parents=True, exist_ok=True)
        if audit_path.exists():
            audit_path.unlink()
        env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = str(audit_path)
    else:
        env.pop("NAVICORE_NHC_BLOCK_AUDIT_CSV", None)

    r = run_benchmark(
        f"HATT-b1 lam={lam:g} {cell_id} {scenario}",
        scenario,
        seed=SEED,
        imu_mode=imu,
        nhc_jacobian=jac,
        nhc_att_z_forget=lam,
        archive_suffix=suffix,
        env=env,
    )
    out = {
        "cell": cell_id,
        "nhc_jacobian": jac,
        "imu_mode": imu,
        "lambda": lam,
        "scenario": scenario,
        "passed": bool(r.passed),
        "error": r.error or None,
        "sim_exit_code": r.sim_exit_code,
    }
    if scenario == "SLALOM":
        out["max_lateral_m"] = metric_val(r, "max_lateral_drift")
    else:
        out["tunnel_exit_m"] = metric_val(r, "tunnel_exit_drift")
        out["zupt_residual_speed"] = metric_val(r, "zupt_residual_speed")
        out["gps_recovery_time"] = metric_val(r, "gps_recovery_time")
    return out


def load_merged_audits(path_a: Path, path_c: Path) -> pd.DataFrame:
    a = pd.read_csv(path_a)
    c = pd.read_csv(path_c)
    a["t_ms"] = (a["timestamp_s"] * 1000).round().astype(int)
    c["t_ms"] = (c["timestamp_s"] * 1000).round().astype(int)
    return a.merge(c, on="t_ms", suffixes=("_A", "_C"))


def mechanism_checks(m: pd.DataFrame) -> dict:
    """P3 + D6 on SLALOM A×C audits (protocol §11)."""
    w02 = m[(m["timestamp_s_A"] >= 0.0) & (m["timestamp_s_A"] <= 2.0)].copy()
    w075 = m[(m["timestamp_s_A"] >= 0.0) & (m["timestamp_s_A"] <= 0.75)].copy()

    za = w02["dx_att_z_rad_A"].to_numpy(float)
    zc = w02["dx_att_z_rad_C"].to_numpy(float)
    ka = w02["k_att_max_A"].to_numpy(float)
    kc = w02["k_att_max_C"].to_numpy(float)

    opp = float(np.mean(np.sign(za) * np.sign(zc) < 0)) if len(za) else float("nan")
    rms_a = float(np.sqrt(np.mean(za**2))) if len(za) else float("nan")
    rms_c = float(np.sqrt(np.mean(zc**2))) if len(zc) else float("nan")
    rms_ratio = rms_a / rms_c if rms_c > 0 else float("inf")

    # FEEDBACK_GROWTH on |ΣA−ΣC| in [0, 0.75] (same thresholds as K/P autopsy)
    t = w075["timestamp_s_A"].to_numpy(float)
    sep = np.abs(
        np.cumsum(w075["dx_att_z_rad_A"].to_numpy(float))
        - np.cumsum(w075["dx_att_z_rad_C"].to_numpy(float))
    )
    innov_a = w075["innov_norm_mps_A"].to_numpy(float) if "innov_norm_mps_A" in w075 else None
    if innov_a is None and "innov_y_mps_A" in w075:
        innov_a = np.sqrt(
            w075["innov_y_mps_A"].to_numpy(float) ** 2
            + w075["innov_z_mps_A"].to_numpy(float) ** 2
        )

    growth = "INSUFFICIENT_SAMPLES"
    if len(t) >= 4:
        mid = len(t) // 2
        dsep = np.diff(sep) / np.maximum(np.diff(t), 1e-9)
        r1 = float(np.mean(dsep[:mid])) if mid else float("nan")
        r2 = float(np.mean(dsep[mid:])) if mid < len(dsep) else float("nan")
        rate_ratio = r2 / r1 if r1 > 0 else float("inf")
        sep_ratio = float(sep[-1] / sep[0]) if sep[0] > 0 else float("inf")
        if innov_a is not None and innov_a[0] > 0:
            innov_ratio = float(innov_a[-1] / innov_a[0])
        else:
            innov_ratio = float("nan")
        if rate_ratio > 2.0 or (innov_ratio > 10.0 and sep_ratio > 5.0):
            growth = "FEEDBACK_GROWTH"
        elif 0.5 <= rate_ratio <= 1.5 and innov_ratio < 3.0:
            growth = "CONSTANT_OFFSET"
        else:
            growth = "OTHER"
        growth_detail = {
            "rate_ratio_2nd_1st": rate_ratio,
            "sep_ratio_end_start": sep_ratio,
            "innov_ratio_end_start": innov_ratio,
        }
    else:
        growth_detail = {}

    k_mean_ratio = float(ka.mean() / kc.mean()) if kc.mean() > 0 else float("nan")
    k_corr = float(np.corrcoef(ka, kc)[0, 1]) if len(ka) > 2 else float("nan")

    p3_opp_or_rms = (opp <= 0.60) or (rms_ratio <= 100.0)
    p3_no_feedback = growth != "FEEDBACK_GROWTH"
    p3_pass = bool(p3_opp_or_rms and p3_no_feedback)

    d6_pass = bool(0.9 <= k_mean_ratio <= 1.1 and k_corr >= 0.9)

    return {
        "window_0_2s": {
            "n": int(len(za)),
            "opp_sign_frac": opp,
            "rms_dx_att_z_A": rms_a,
            "rms_dx_att_z_C": rms_c,
            "rms_ratio_A_C": rms_ratio,
            "k_att_mean_A": float(ka.mean()) if len(ka) else float("nan"),
            "k_att_mean_C": float(kc.mean()) if len(kc) else float("nan"),
            "k_att_mean_ratio_A_C": k_mean_ratio,
            "k_att_corr": k_corr,
        },
        "window_0_0.75s_growth": {"verdict": growth, **growth_detail},
        "P3_pass": p3_pass,
        "D6_k_att_consistency_pass": d6_pass,
        "D6_note": "pipeline check, not success gate",
    }


def score_lambda(control: dict, interv: dict, mech: dict | None) -> dict:
    """Score one λ against control (λ=0)."""
    a0 = control["slalom"]["A"]["max_lateral_m"]
    a1 = interv["slalom"]["A"]["max_lateral_m"]
    p1_abs = a1 is not None and a1 <= P1_ABS_M
    p1_rel = a0 is not None and a1 is not None and a0 > 0 and (a0 / a1) >= P1_IMPROVE
    p1 = bool(p1_abs and p1_rel)

    p2_slalom = {}
    p2_slalom_ok = True
    for cell in ("C", "D"):
        c0 = control["slalom"][cell]["max_lateral_m"]
        c1 = interv["slalom"][cell]["max_lateral_m"]
        ok = c0 is not None and c1 is not None and c1 <= RHO * c0
        p2_slalom[cell] = {
            "control_m": c0,
            "interv_m": c1,
            "ratio": (c1 / c0) if c0 and c0 > 0 else None,
            "pass": ok,
        }
        p2_slalom_ok = p2_slalom_ok and ok

    p2_tunnel = {}
    p2_tunnel_ok = True
    for cell in ("A", "B", "C", "D"):
        t0 = control["tunnel"][cell]["tunnel_exit_m"]
        t1 = interv["tunnel"][cell]["tunnel_exit_m"]
        ok = t0 is not None and t1 is not None and t1 <= RHO * t0
        p2_tunnel[cell] = {
            "control_m": t0,
            "interv_m": t1,
            "ratio": (t1 / t0) if t0 and t0 > 0 else None,
            "pass": ok,
        }
        p2_tunnel_ok = p2_tunnel_ok and ok

    p3 = mech["P3_pass"] if mech else False
    d6 = mech["D6_k_att_consistency_pass"] if mech else False
    hatt_pass = bool(p1 and p2_slalom_ok and p2_tunnel_ok and p3)

    return {
        "P1": {
            "pass": p1,
            "slalom_A_control_m": a0,
            "slalom_A_interv_m": a1,
            "abs_ok": p1_abs,
            "improve_ok": p1_rel,
            "improve_factor": (a0 / a1) if a0 and a1 and a1 > 0 else None,
        },
        "P2_slalom": {"pass": p2_slalom_ok, "rho": RHO, "cells": p2_slalom},
        "P2_tunnel": {"pass": p2_tunnel_ok, "rho": RHO, "cells": p2_tunnel},
        "P3": {"pass": p3, "detail": mech},
        "D6": {"pass": d6, "detail": mech},
        "HATT_b1_PASS": hatt_pass,
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    by_lam: dict[str, dict] = {}

    for lam in LAMBDAS:
        print(f"\n########## lambda = {lam:g} ##########")
        pack = {"lambda": lam, "slalom": {}, "tunnel": {}, "audits": {}}
        for cell_id, jac, imu in CELLS:
            audit = None
            if cell_id in ("A", "C") and imu == "ideal":
                audit = OUT / f"slalom_l{lam:g}_cell{cell_id}_nhc_block_audit.csv"
            pack["slalom"][cell_id] = run_cell(
                cell_id, jac, imu, lam, scenario="SLALOM", audit_path=audit
            )
            if audit is not None:
                pack["audits"][cell_id] = str(audit)
            pack["tunnel"][cell_id] = run_cell(
                cell_id, jac, imu, lam, scenario="TUNNEL_STRESS", audit_path=None
            )
        by_lam[f"{lam:g}"] = pack

    control = by_lam["0"]
    scores = {}
    for lam in LAMBDAS:
        key = f"{lam:g}"
        pack = by_lam[key]
        mech = None
        pa = Path(pack["audits"].get("A", ""))
        pc = Path(pack["audits"].get("C", ""))
        if pa.is_file() and pc.is_file():
            mech = mechanism_checks(load_merged_audits(pa, pc))
        scores[key] = score_lambda(control, pack, mech)

    report = {
        "protocol": "docs/diagnostics/18-jacobian-imu-ab-protocol.md §11",
        "intervention": "H-ATT-b1",
        "timestamp": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "seed": SEED,
        "lambdas": list(LAMBDAS),
        "rho": RHO,
        "p1_abs_m": P1_ABS_M,
        "p1_improve_factor": P1_IMPROVE,
        "scope": "ATT_Z only",
        "by_lambda": by_lam,
        "scores": scores,
        "discipline": "Do not retune thresholds after seeing this report",
    }
    out_path = OUT / "hatt_b1_report.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("\n======== H-ATT-b1 SCORECARD ========")
    for lam in LAMBDAS:
        key = f"{lam:g}"
        s = scores[key]
        a = s["P1"]["slalom_A_interv_m"]
        print(
            f"lam={lam:g}: HATT={'PASS' if s['HATT_b1_PASS'] else 'FAIL'} | "
            f"P1={s['P1']['pass']} A={a} | "
            f"P2s={s['P2_slalom']['pass']} P2t={s['P2_tunnel']['pass']} | "
            f"P3={s['P3']['pass']} D6={s['D6']['pass']}"
        )
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
