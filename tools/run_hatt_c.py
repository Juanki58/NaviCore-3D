#!/usr/bin/env python3
"""H-ATT-c sweep — protocol §12.

Detector cand1 (Σ|dx_att_z| >= T, t<=0.65s) + gated b1 λ.
Arms: T in {T2, T5} × λ in {0.3, 0.5, 0.7} + control.
Do not retune thresholds after seeing results.
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
OUT = REPO / "docs" / "benchmarks" / "jacobian_imu_ab" / "hatt_c"
SEED = 71
T2 = 3.736646e-6
T5 = 1.224574e-5
TMAX = 0.65
LAMBDAS = (0.3, 0.5, 0.7)
THRESHOLDS = (("E", T2), ("L", T5))  # early / late
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

FIRE_RE = re.compile(
    r"HATT_C_FIRE t_s=([0-9.eE+-]+) sumabs=([0-9.eE+-]+) thr=([0-9.eE+-]+) lambda=([0-9.eE+-]+)"
)


def metric_val(result, name: str):
    for metric in result.metrics:
        if metric.name == name:
            return metric.measured
    return None


def parse_fire(stdout: str) -> dict | None:
    m = FIRE_RE.search(stdout or "")
    if not m:
        return None
    return {
        "t_s": float(m.group(1)),
        "sumabs": float(m.group(2)),
        "thr": float(m.group(3)),
        "lambda": float(m.group(4)),
    }


def run_cell(
    cell_id: str,
    jac: str,
    imu: str,
    *,
    lam: float,
    gate: float | None,
    scenario: str,
    audit_path: Path | None = None,
) -> dict:
    tag = "ctrl" if gate is None or gate <= 0 else f"T{gate:.3e}_l{lam:g}"
    suffix = f"hatt_c_{tag}_cell{cell_id}_j{jac}_imu{imu}_s{SEED}"
    env = os.environ.copy()
    if audit_path is not None:
        audit_path.parent.mkdir(parents=True, exist_ok=True)
        if audit_path.exists():
            audit_path.unlink()
        env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = str(audit_path)
    else:
        env.pop("NAVICORE_NHC_BLOCK_AUDIT_CSV", None)

    r = run_benchmark(
        f"HATT-c {tag} {cell_id} {scenario}",
        scenario,
        seed=SEED,
        imu_mode=imu,
        nhc_jacobian=jac,
        nhc_att_z_forget=lam if gate and gate > 0 else 0.0,
        nhc_att_z_forget_gate=gate if gate and gate > 0 else 0.0,
        nhc_att_z_forget_tmax=TMAX if gate and gate > 0 else None,
        archive_suffix=suffix,
        env=env,
    )
    fire = parse_fire(r.stdout)
    out = {
        "cell": cell_id,
        "nhc_jacobian": jac,
        "imu_mode": imu,
        "lambda": lam if gate and gate > 0 else 0.0,
        "gate_thr": gate if gate else 0.0,
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


def load_merged(path_a: Path, path_c: Path) -> pd.DataFrame:
    a = pd.read_csv(path_a)
    c = pd.read_csv(path_c)
    a["t_ms"] = (a["timestamp_s"] * 1000).round().astype(int)
    c["t_ms"] = (c["timestamp_s"] * 1000).round().astype(int)
    return a.merge(c, on="t_ms", suffixes=("_A", "_C"))


def mechanism_p3(m: pd.DataFrame) -> dict:
    w075 = m[(m["timestamp_s_A"] >= 0.0) & (m["timestamp_s_A"] <= 0.75)]
    w02 = m[(m["timestamp_s_A"] >= 0.0) & (m["timestamp_s_A"] <= 2.0)]
    za = w02["dx_att_z_rad_A"].to_numpy(float)
    zc = w02["dx_att_z_rad_C"].to_numpy(float)
    rms_a = float(np.sqrt(np.mean(za**2))) if len(za) else float("nan")
    rms_c = float(np.sqrt(np.mean(zc**2))) if len(zc) else float("nan")
    rms_ratio = rms_a / rms_c if rms_c > 0 else float("inf")
    opp = float(np.mean(np.sign(za) * np.sign(zc) < 0)) if len(za) else float("nan")

    t = w075["timestamp_s_A"].to_numpy(float)
    sep = np.abs(
        np.cumsum(w075["dx_att_z_rad_A"].to_numpy(float))
        - np.cumsum(w075["dx_att_z_rad_C"].to_numpy(float))
    )
    innov = w075["innov_norm_mps_A"].to_numpy(float)
    growth = "INSUFFICIENT"
    if len(t) >= 4:
        mid = len(t) // 2
        dsep = np.diff(sep) / np.maximum(np.diff(t), 1e-9)
        r1 = float(np.mean(dsep[:mid]))
        r2 = float(np.mean(dsep[mid:]))
        rate_ratio = r2 / r1 if r1 > 0 else float("inf")
        sep_ratio = float(sep[-1] / sep[0]) if sep[0] > 0 else float("inf")
        innov_ratio = float(innov[-1] / innov[0]) if innov[0] > 0 else float("nan")
        if rate_ratio > 2.0 or (innov_ratio > 10.0 and sep_ratio > 5.0):
            growth = "FEEDBACK_GROWTH"
        else:
            growth = "OTHER"
    p3_mech = (opp <= 0.60) or (rms_ratio <= 100.0)
    p3_nofb = growth != "FEEDBACK_GROWTH"
    return {
        "opp_sign_frac": opp,
        "rms_ratio_A_C": rms_ratio,
        "growth": growth,
        "P3_mech_pass": bool(p3_mech and p3_nofb),
    }


def score_arm(control: dict, arm: dict, mech: dict | None) -> dict:
    a0 = control["slalom"]["A"]["max_lateral_m"]
    a1 = arm["slalom"]["A"]["max_lateral_m"]
    p1 = bool(
        a1 is not None
        and a0 is not None
        and a1 <= P1_ABS_M
        and a0 > 0
        and (a0 / a1) >= P1_IMPROVE
    )

    def p2_block(scenario: str, cells: tuple[str, ...], key: str) -> dict:
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

    p2s = p2_block("slalom", ("C", "D"), "max_lateral_m")
    p2t = p2_block("tunnel", ("A", "B", "C", "D"), "tunnel_exit_m")

    fire_a = arm["slalom"]["A"].get("fire")
    fire_c = arm["slalom"]["C"].get("fire")
    p3a_fire = bool(fire_a is not None and 0.0 < fire_a["t_s"] <= TMAX)
    p3c_nofire = fire_c is None
    p3_mech = bool(mech and mech.get("P3_mech_pass"))
    p3a = p3a_fire and p3_mech
    p3c = p3c_nofire

    hatt = bool(p1 and p2s["pass"] and p2t["pass"] and p3a and p3c)
    return {
        "P1": {
            "pass": p1,
            "A_control": a0,
            "A_interv": a1,
            "improve": (a0 / a1) if a0 and a1 and a1 > 0 else None,
        },
        "P2_slalom": p2s,
        "P2_tunnel": p2t,
        "P3_A": {
            "pass": p3a,
            "fired": fire_a is not None,
            "fire": fire_a,
            "mech": mech,
        },
        "P3_C": {"pass": p3c, "fired": fire_c is not None, "fire": fire_c},
        "HATT_c_PASS": hatt,
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)

    print("\n########## CONTROL ##########")
    control = {"slalom": {}, "tunnel": {}, "audits": {}}
    for cell_id, jac, imu in CELLS:
        audit = None
        if cell_id in ("A", "C") and imu == "ideal":
            audit = OUT / f"ctrl_cell{cell_id}_nhc_block_audit.csv"
        control["slalom"][cell_id] = run_cell(
            cell_id, jac, imu, lam=0.0, gate=None, scenario="SLALOM", audit_path=audit
        )
        if audit:
            control["audits"][cell_id] = str(audit)
        control["tunnel"][cell_id] = run_cell(
            cell_id, jac, imu, lam=0.0, gate=None, scenario="TUNNEL_STRESS"
        )

    arms = {}
    scores = {}
    for arm_name, thr in THRESHOLDS:
        for lam in LAMBDAS:
            key = f"c-{arm_name}-l{lam:g}"
            print(f"\n########## {key} T={thr:.6e} lambda={lam:g} ##########")
            pack = {
                "arm": key,
                "T": thr,
                "lambda": lam,
                "tmax": TMAX,
                "slalom": {},
                "tunnel": {},
                "audits": {},
            }
            for cell_id, jac, imu in CELLS:
                audit = None
                if cell_id in ("A", "C") and imu == "ideal":
                    audit = OUT / f"{key}_cell{cell_id}_nhc_block_audit.csv"
                pack["slalom"][cell_id] = run_cell(
                    cell_id,
                    jac,
                    imu,
                    lam=lam,
                    gate=thr,
                    scenario="SLALOM",
                    audit_path=audit,
                )
                if audit:
                    pack["audits"][cell_id] = str(audit)
                pack["tunnel"][cell_id] = run_cell(
                    cell_id, jac, imu, lam=lam, gate=thr, scenario="TUNNEL_STRESS"
                )
            mech = None
            pa = Path(pack["audits"].get("A", ""))
            pc = Path(pack["audits"].get("C", ""))
            if pa.is_file() and pc.is_file():
                mech = mechanism_p3(load_merged(pa, pc))
            arms[key] = pack
            scores[key] = score_arm(control, pack, mech)

    report = {
        "protocol": "docs/diagnostics/18-jacobian-imu-ab-protocol.md §12",
        "intervention": "H-ATT-c",
        "timestamp": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "seed": SEED,
        "T2": T2,
        "T5": T5,
        "tmax": TMAX,
        "lambdas": list(LAMBDAS),
        "rho": RHO,
        "control": control,
        "arms": arms,
        "scores": scores,
        "discipline": "Do not retune T/lambda/tmax after seeing this report",
    }
    out_path = OUT / "hatt_c_report.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("\n======== H-ATT-c SCORECARD ========")
    for key, s in scores.items():
        print(
            f"{key}: HATT={'PASS' if s['HATT_c_PASS'] else 'FAIL'} | "
            f"P1={s['P1']['pass']} A={s['P1']['A_interv']} | "
            f"P2s={s['P2_slalom']['pass']} P2t={s['P2_tunnel']['pass']} | "
            f"P3A={s['P3_A']['pass']} (fire={s['P3_A']['fire']}) | "
            f"P3C={s['P3_C']['pass']}"
        )
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
