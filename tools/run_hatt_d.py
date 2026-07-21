#!/usr/bin/env python3
"""H-ATT-d sweep — protocol §13.21.

Z unobserved in H[*][ATT_Z] after cand1 (same T2 gate as H-ATT-c).
Arms: ctrl, c-L-l1 (negativo λ=1 post-hoc), d-E (unobs @ T2).
P1–P4 + P3-C as frozen.
"""

from __future__ import annotations

import json
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "docs" / "benchmarks" / "jacobian_imu_ab" / "hatt_d"
SEED = 71
T2 = 3.736646e-6
TMAX = 0.65
RHO = 1.20
P1_ABS_M = 2.0
P1_IMPROVE = 10.0
T_ON0, T_ON1 = 0.40, 1.10
T_LATE0, T_LATE1 = 1.10, 1.54
P3_MAX_GROWTH = 5.0  # max|ΔP| late ≤ 5× onset (same arm) or ≤5× ctrl late
P4_DEFICIT_FRAC = 0.25  # Σ|deficit|_d ≤ 0.25 × Σ|deficit|_c-L-l1

CELLS = (
    ("A", "correct", "ideal"),
    ("B", "correct", "dirty"),
    ("C", "legacy", "ideal"),
    ("D", "legacy", "dirty"),
)

sys.path.insert(0, str(REPO))
from run_all_benchmarks import run_benchmark  # noqa: E402

FIRE_C_RE = re.compile(
    r"HATT_C_FIRE t_s=([0-9.eE+-]+) sumabs=([0-9.eE+-]+) thr=([0-9.eE+-]+) lambda=([0-9.eE+-]+)"
)
FIRE_D_RE = re.compile(
    r"HATT_D_FIRE t_s=([0-9.eE+-]+) sumabs=([0-9.eE+-]+) thr=([0-9.eE+-]+) unobs=1"
)


def metric_val(result, name: str):
    for metric in result.metrics:
        if metric.name == name:
            return metric.measured
    return None


def parse_fire(stdout: str) -> dict | None:
    m = FIRE_D_RE.search(stdout or "")
    if m:
        return {
            "t_s": float(m.group(1)),
            "sumabs": float(m.group(2)),
            "thr": float(m.group(3)),
            "mode": "unobs",
        }
    m = FIRE_C_RE.search(stdout or "")
    if m:
        return {
            "t_s": float(m.group(1)),
            "sumabs": float(m.group(2)),
            "thr": float(m.group(3)),
            "lambda": float(m.group(4)),
            "mode": "forget",
        }
    return None


def run_cell(
    cell_id: str,
    jac: str,
    imu: str,
    *,
    mode: str,
    scenario: str,
    audit_path: Path | None = None,
) -> dict:
    """mode: ctrl | c_l1 | d_unobs"""
    if mode == "ctrl":
        tag = "ctrl"
        kwargs = dict(
            nhc_att_z_forget=0.0,
            nhc_att_z_forget_gate=0.0,
            nhc_att_z_forget_tmax=None,
            nhc_att_z_unobs=False,
        )
    elif mode == "c_l1":
        tag = "c-L-l1"
        kwargs = dict(
            nhc_att_z_forget=1.0,
            nhc_att_z_forget_gate=T2,
            nhc_att_z_forget_tmax=TMAX,
            nhc_att_z_unobs=False,
        )
    elif mode == "d_unobs":
        tag = "d-E"
        kwargs = dict(
            nhc_att_z_forget=0.0,
            nhc_att_z_forget_gate=T2,
            nhc_att_z_forget_tmax=TMAX,
            nhc_att_z_unobs=True,
        )
    else:
        raise ValueError(mode)

    suffix = f"hatt_d_{tag}_cell{cell_id}_j{jac}_imu{imu}_s{SEED}"
    env = os.environ.copy()
    if audit_path is not None:
        audit_path.parent.mkdir(parents=True, exist_ok=True)
        if audit_path.exists():
            audit_path.unlink()
        env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = str(audit_path)
    else:
        env.pop("NAVICORE_NHC_BLOCK_AUDIT_CSV", None)

    r = run_benchmark(
        f"HATT-d {tag} {cell_id} {scenario}",
        scenario,
        seed=SEED,
        imu_mode=imu,
        nhc_jacobian=jac,
        archive_suffix=suffix,
        env=env,
        **kwargs,
    )
    fire = parse_fire(r.stdout)
    out = {
        "cell": cell_id,
        "mode": mode,
        "tag": tag,
        "nhc_jacobian": jac,
        "imu_mode": imu,
        "scenario": scenario,
        "passed": bool(r.passed),
        "error": r.error or None,
        "fire": fire,
        "fired": fire is not None,
    }
    if scenario == "SLALOM":
        out["max_lateral_m"] = metric_val(r, "max_lateral_drift")
    else:
        out["tunnel_exit_m"] = metric_val(r, "tunnel_exit_drift")
    return out


def align_p(ctrl: pd.DataFrame, arm: pd.DataFrame) -> pd.DataFrame:
    c = ctrl.copy()
    a = arm.copy()
    c["t_r"] = c["timestamp_s"].round(6)
    a["t_r"] = a["timestamp_s"].round(6)
    return a.merge(c, on="t_r", suffixes=("_a", "_c"))


def joseph_deficit_sum(ctrl: pd.DataFrame, arm: pd.DataFrame) -> dict:
    m = align_p(ctrl, arm)
    m = m[(m["t_r"] >= T_LATE0) & (m["t_r"] <= T_LATE1)]
    if "P_pre_att_y_vn_a" not in m.columns:
        return {"error": "missing P_pre_att_y_vn — rebuild sim"}
    pre_a = m["P_pre_att_y_vn_a"].to_numpy(float)
    post_a = m["P_post_att_y_vn_a"].to_numpy(float)
    pre_c = m["P_pre_att_y_vn_c"].to_numpy(float)
    post_c = m["P_post_att_y_vn_c"].to_numpy(float)
    d_jos_a = post_a - pre_a
    d_jos_c = post_c - pre_c
    deficit = d_jos_a - d_jos_c
    return {
        "n": int(len(m)),
        "sum_joseph_deficit": float(np.sum(deficit)),
        "sum_abs_joseph_deficit": float(np.sum(np.abs(deficit))),
        "dP_end": float(pre_a[-1] - pre_c[-1]) if len(m) else float("nan"),
        "dP_start": float(pre_a[0] - pre_c[0]) if len(m) else float("nan"),
    }


def p3_pattvel(ctrl: pd.DataFrame, arm: pd.DataFrame) -> dict:
    m = align_p(ctrl, arm)
    if "P_pre_att_y_vn_a" not in m.columns:
        return {"error": "missing P_pre_att_y_vn", "pass": False}

    def max_abs_d(t0, t1):
        w = m[(m["t_r"] >= t0) & (m["t_r"] <= t1)]
        d = w["P_pre_att_y_vn_a"].to_numpy(float) - w["P_pre_att_y_vn_c"].to_numpy(float)
        return float(np.max(np.abs(d))) if len(d) else float("nan")

    max_on = max_abs_d(T_ON0, T_ON1)
    max_late = max_abs_d(T_LATE0, T_LATE1)
    # also vs ctrl late self-spread: use max|Δ| late vs 5× onset
    thr_onset = P3_MAX_GROWTH * max_on if np.isfinite(max_on) else float("inf")
    # ctrl late max|P| not used; protocol: max|ΔP| ≤ 5× onset OR ≤5× ctrl late max|Δ|
    # "5× max|ΔP| ctrl in late" — ctrl vs itself is 0; interpret as 5× max|ΔP| of
    # c-L-l1 late as alternate ceiling? Stick to onset rule + absolute vs c-L-l1.
    ok = bool(np.isfinite(max_late) and np.isfinite(max_on) and max_late <= thr_onset)
    return {
        "max_abs_dP_onset": max_on,
        "max_abs_dP_late": max_late,
        "ratio_late_over_onset": float(max_late / max_on) if max_on and max_on > 0 else float("inf"),
        "threshold_5x_onset": thr_onset,
        "pass": ok,
        "mean_abs_dP_late": float(
            np.mean(
                np.abs(
                    m[(m["t_r"] >= T_LATE0) & (m["t_r"] <= T_LATE1)]["P_pre_att_y_vn_a"].to_numpy(
                        float
                    )
                    - m[(m["t_r"] >= T_LATE0) & (m["t_r"] <= T_LATE1)]["P_pre_att_y_vn_c"].to_numpy(
                        float
                    )
                )
            )
        )
        if len(m) else float("nan"),
    }


def p2_block(control: dict, arm: dict, scenario: str, cells: tuple[str, ...], key: str) -> dict:
    cells_out = {}
    ok = True
    for cell in cells:
        c0 = control[scenario][cell][key]
        c1 = arm[scenario][cell][key]
        cell_ok = c0 is not None and c1 is not None and c1 <= RHO * c0
        cells_out[cell] = {
            "control": c0,
            "interv": c1,
            "ratio": (c1 / c0) if c0 and c0 > 0 else None,
            "pass": cell_ok,
        }
        ok = ok and cell_ok
    return {"pass": ok, "rho": RHO, "cells": cells_out}


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)

    packs = {}
    for mode in ("ctrl", "c_l1", "d_unobs"):
        print(f"\n########## {mode} ##########")
        pack = {"mode": mode, "slalom": {}, "tunnel": {}, "audits": {}}
        for cell_id, jac, imu in CELLS:
            audit = None
            if cell_id in ("A", "C") and imu == "ideal" and mode in ("ctrl", "c_l1", "d_unobs"):
                audit = OUT / f"{mode}_cell{cell_id}_nhc_block_audit.csv"
            pack["slalom"][cell_id] = run_cell(
                cell_id, jac, imu, mode=mode, scenario="SLALOM", audit_path=audit
            )
            if audit:
                pack["audits"][cell_id] = str(audit)
            pack["tunnel"][cell_id] = run_cell(
                cell_id, jac, imu, mode=mode, scenario="TUNNEL_STRESS"
            )
        packs[mode] = pack

    ctrl = packs["ctrl"]
    neg = packs["c_l1"]
    darm = packs["d_unobs"]

    # Load audits for P3/P4
    def load(mode, cell):
        p = Path(packs[mode]["audits"].get(cell, ""))
        return pd.read_csv(p) if p.is_file() else None

    ctrl_a = load("ctrl", "A")
    neg_a = load("c_l1", "A")
    d_a = load("d_unobs", "A")

    p3 = p3_pattvel(ctrl_a, d_a) if ctrl_a is not None and d_a is not None else {"pass": False, "error": "no audit"}
    p3_neg = (
        p3_pattvel(ctrl_a, neg_a) if ctrl_a is not None and neg_a is not None else {"error": "no audit"}
    )
    p4_d = (
        joseph_deficit_sum(ctrl_a, d_a) if ctrl_a is not None and d_a is not None else {"error": "no audit"}
    )
    p4_neg = (
        joseph_deficit_sum(ctrl_a, neg_a)
        if ctrl_a is not None and neg_a is not None
        else {"error": "no audit"}
    )

    a0 = ctrl["slalom"]["A"]["max_lateral_m"]
    a_d = darm["slalom"]["A"]["max_lateral_m"]
    a_neg = neg["slalom"]["A"]["max_lateral_m"]
    p1 = bool(
        a_d is not None and a0 is not None and a_d <= P1_ABS_M and a0 > 0 and (a0 / a_d) >= P1_IMPROVE
    )

    p2s = p2_block(ctrl, darm, "slalom", ("C", "D"), "max_lateral_m")
    p2t = p2_block(ctrl, darm, "tunnel", ("A", "B", "C", "D"), "tunnel_exit_m")

    fire_a = darm["slalom"]["A"].get("fire")
    fire_c = darm["slalom"]["C"].get("fire")
    p3c = fire_c is None
    fire_ok = bool(fire_a is not None and 0.0 < fire_a["t_s"] <= TMAX)

    neg_abs = p4_neg.get("sum_abs_joseph_deficit", float("nan"))
    d_abs = p4_d.get("sum_abs_joseph_deficit", float("nan"))
    p4_pass = bool(
        np.isfinite(neg_abs)
        and np.isfinite(d_abs)
        and neg_abs > 0
        and d_abs <= P4_DEFICIT_FRAC * neg_abs
    )

    hatt_pass = bool(p1 and p2s["pass"] and p2t["pass"] and p3.get("pass") and p4_pass and p3c)

    scores = {
        "P1": {
            "pass": p1,
            "A_ctrl": a0,
            "A_d": a_d,
            "A_c_l1": a_neg,
            "improve_vs_ctrl": (a0 / a_d) if a0 and a_d and a_d > 0 else None,
            "threshold": f"<= {P1_ABS_M} m AND >= {P1_IMPROVE}x vs ctrl",
        },
        "P2_slalom": p2s,
        "P2_tunnel": p2t,
        "P3_pattvel": p3,
        "P3_pattvel_neg_ref": p3_neg,
        "P3_C_nofire": {"pass": p3c, "fire_C": fire_c},
        "P4_joseph_deficit": {
            "pass": p4_pass,
            "d_unobs": p4_d,
            "c_l1": p4_neg,
            "ratio_d_over_c_l1": (d_abs / neg_abs) if neg_abs and neg_abs > 0 else None,
            "threshold": f"sum_abs_deficit_d <= {P4_DEFICIT_FRAC} * c_l1",
        },
        "fire_A": fire_a,
        "fire_ok": fire_ok,
        "HATT_d_PASS": hatt_pass,
    }

    report = {
        "protocol": "docs/diagnostics/18-jacobian-imu-ab-protocol.md §13.21",
        "intervention": "H-ATT-d att_z unobs columns",
        "timestamp": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "seed": SEED,
        "T2": T2,
        "tmax": TMAX,
        "packs": packs,
        "scores": scores,
    }
    out_path = OUT / "hatt_d_report.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    # markdown scorecard
    lines = [
        "# H-ATT-d scorecard (§13.21)",
        "",
        f"**HATT-d:** `{'PASS' if hatt_pass else 'FAIL'}`",
        "",
        "| Criterion | Result | Detail |",
        "|-----------|--------|--------|",
        f"| P1 SLALOM A | {'PASS' if p1 else 'FAIL'} | ctrl={a0} d={a_d} c-L-l1={a_neg} |",
        f"| P2-slalom C/D | {'PASS' if p2s['pass'] else 'FAIL'} | {p2s['cells']} |",
        f"| P2-tunnel | {'PASS' if p2t['pass'] else 'FAIL'} | see JSON |",
        f"| P3 P[ATT_Y,VN] | {'PASS' if p3.get('pass') else 'FAIL'} | "
        f"late/onset={p3.get('ratio_late_over_onset')} "
        f"max_late={p3.get('max_abs_dP_late')} (neg ref late={p3_neg.get('max_abs_dP_late')}) |",
        f"| P4 Joseph deficit | {'PASS' if p4_pass else 'FAIL'} | "
        f"d={d_abs:.3e} c-L-l1={neg_abs:.3e} ratio={scores['P4_joseph_deficit']['ratio_d_over_c_l1']} |",
        f"| P3-C no fire | {'PASS' if p3c else 'FAIL'} | fire_C={fire_c} |",
        "",
        f"Fire A: {fire_a}",
        "",
        f"Full JSON: `{out_path.name}`",
        "",
    ]
    (OUT / "hatt_d_report.md").write_text("\n".join(lines), encoding="utf-8")

    print("\n======== H-ATT-d SCORECARD ========")
    print(f"HATT-d: {'PASS' if hatt_pass else 'FAIL'}")
    print(f"P1: {p1}  A ctrl={a0} d={a_d} c-L-l1={a_neg}")
    print(f"P2s: {p2s['pass']}  P2t: {p2t['pass']}")
    print(f"P3: {p3}")
    print(f"P4: pass={p4_pass} d_abs={d_abs} neg_abs={neg_abs}")
    print(f"P3-C: {p3c}  fire_A={fire_a}")
    print(f"Wrote {out_path}")
    return 0 if hatt_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
