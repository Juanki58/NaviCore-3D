#!/usr/bin/env python3
"""Ctrl-only audit: cand1 sumabs scale slalom vs tunnel (protocol §13.22).

Reconstructs cumulative |dx_att_z| from NHC block audits (gate off).
Chooses grace N and reports sumabs / P_att_zz comparability.
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
OUT = REPO / "docs" / "benchmarks" / "jacobian_imu_ab" / "cand1_gate_e12"
SEED = 71
T2 = 3.736646e-6
CELLS = (
    ("A", "correct", "ideal"),
    ("B", "correct", "dirty"),
    ("C", "legacy", "ideal"),
    ("D", "legacy", "dirty"),
)

sys.path.insert(0, str(REPO))
from run_all_benchmarks import run_benchmark  # noqa: E402


def run_ctrl_audit(cell_id: str, jac: str, imu: str, scenario: str) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"ctrl_{scenario.lower()}_cell{cell_id}_nhc_block_audit.csv"
    if path.is_file() and path.stat().st_size > 100:
        print(f"reuse {path.name}")
        return path
    env = os.environ.copy()
    env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = str(path.resolve())
    run_benchmark(
        f"cand1-scale ctrl {cell_id} {scenario}",
        scenario,
        seed=SEED,
        imu_mode=imu,
        nhc_jacobian=jac,
        archive_suffix=f"cand1_scale_ctrl_{scenario.lower()}_cell{cell_id}",
        env=env,
    )
    if not path.is_file():
        raise FileNotFoundError(f"NHC audit not written: {path}")
    return path


def series_from_audit(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    dx = df["dx_att_z_rad"].to_numpy(float)
    pzz = df["P_pre_att_zz"].to_numpy(float)
    t = df["timestamp_s"].to_numpy(float)
    sumabs = np.cumsum(np.abs(dx))
    # avoid /0
    scale = np.maximum(pzz, 1e-30)
    return pd.DataFrame(
        {
            "t_s": t,
            "tick": np.arange(1, len(df) + 1),
            "dx_att_z": dx,
            "abs_dx": np.abs(dx),
            "sumabs": sumabs,
            "P_att_zz": pzz,
            "sumabs_over_Pzz": sumabs / scale,
            "abs_dx_over_Pzz": np.abs(dx) / scale,
        }
    )


def when_crosses_t2(s: pd.DataFrame) -> dict:
    hit = s[s["sumabs"] >= T2]
    if hit.empty:
        return {"crosses": False}
    row = hit.iloc[0]
    return {
        "crosses": True,
        "t_s": float(row["t_s"]),
        "tick": int(row["tick"]),
        "sumabs": float(row["sumabs"]),
        "P_att_zz": float(row["P_att_zz"]),
        "sumabs_over_Pzz": float(row["sumabs_over_Pzz"]),
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    summaries = []
    for scenario in ("SLALOM", "TUNNEL_STRESS"):
        for cell_id, jac, imu in CELLS:
            path = run_ctrl_audit(cell_id, jac, imu, scenario)
            s = series_from_audit(path)
            # persist trimmed early window for inspection
            early = s.head(80)
            early.to_csv(
                OUT / f"scale_early_{scenario.lower()}_cell{cell_id}.csv", index=False
            )
            cross = when_crosses_t2(s)
            first = s.iloc[0]
            # ticks until |dx| median of ticks 40..80 (post-transient proxy)
            post = s.iloc[39:80] if len(s) >= 80 else s.iloc[len(s) // 2 :]
            post_med = float(np.median(post["abs_dx"])) if len(post) else float("nan")
            # first tick where abs_dx <= 3× post median (transient end heuristic)
            thr = 3.0 * post_med if post_med > 0 else float("inf")
            calm = s[s["abs_dx"] <= thr]
            # require lasting calm: first index after which next 5 ticks also calm
            grace_guess = None
            for i in range(len(s) - 5):
                if all(s.iloc[i + k]["abs_dx"] <= thr for k in range(5)):
                    grace_guess = int(s.iloc[i]["tick"])
                    break
            summaries.append(
                {
                    "scenario": scenario,
                    "cell": cell_id,
                    "jac": jac,
                    "imu": imu,
                    "n_rows": int(len(s)),
                    "t0_abs_dx": float(first["abs_dx"]),
                    "t0_sumabs": float(first["sumabs"]),
                    "t0_Pzz": float(first["P_att_zz"]),
                    "t0_sumabs_over_Pzz": float(first["sumabs_over_Pzz"]),
                    "t0_sumabs_over_T2": float(first["sumabs"] / T2),
                    "cross_T2": cross,
                    "post_med_abs_dx": post_med,
                    "grace_guess_tick": grace_guess,
                    "audit": str(path),
                }
            )
            print(
                f"{scenario} {cell_id}: t0|dx|={first['abs_dx']:.3e} "
                f"sumabs/T2={first['sumabs']/T2:.1f} Pzz={first['P_att_zz']:.3e} "
                f"sumabs/Pzz={first['sumabs_over_Pzz']:.3e} "
                f"cross={cross} grace_guess={grace_guess}"
            )

    # Compare scales: tunnel t0 vs slalom A at fire-ish (tick ~39 = 0.39s @100Hz)
    by = {(x["scenario"], x["cell"]): x for x in summaries}
    sl_a = by[("SLALOM", "A")]
    # slalom series for A at tick 39
    s_sla = series_from_audit(Path(sl_a["audit"]))
    row39 = s_sla[s_sla["tick"] == 39].iloc[0] if (s_sla["tick"] == 39).any() else s_sla.iloc[-1]

    report = {
        "protocol": "docs/diagnostics/18-jacobian-imu-ab-protocol.md §13.22",
        "timestamp": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "T2": T2,
        "cells": summaries,
        "slalom_A_tick39": {
            "t_s": float(row39["t_s"]),
            "sumabs": float(row39["sumabs"]),
            "P_att_zz": float(row39["P_att_zz"]),
            "sumabs_over_Pzz": float(row39["sumabs_over_Pzz"]),
            "sumabs_over_T2": float(row39["sumabs"] / T2),
        },
        "scale_comparison": {},
    }
    for cell in "ABCD":
        tn = by[("TUNNEL_STRESS", cell)]
        report["scale_comparison"][cell] = {
            "tunnel_t0_sumabs_over_T2": tn["t0_sumabs_over_T2"],
            "tunnel_t0_sumabs_over_Pzz": tn["t0_sumabs_over_Pzz"],
            "slalom_A_t0_sumabs_over_Pzz": sl_a["t0_sumabs_over_Pzz"],
            "slalom_A_tick39_sumabs_over_Pzz": float(row39["sumabs_over_Pzz"]),
            "ratio_tunnel_t0_Pnorm_vs_slalom_onset_Pnorm": (
                tn["t0_sumabs_over_Pzz"] / float(row39["sumabs_over_Pzz"])
                if row39["sumabs_over_Pzz"]
                else None
            ),
            "grace_guess_tick": tn["grace_guess_tick"],
        }

    guesses = [x["grace_guess_tick"] for x in summaries if x["scenario"] == "TUNNEL_STRESS"]
    guesses = [g for g in guesses if g is not None]
    report["recommended_grace_N_ticks"] = int(max(guesses)) if guesses else None
    report["note"] = (
        "E1: skip accumulate+evaluate for first N NHC ticks. "
        "E2 only if P-normalized ratios tunnel-t0 vs slalom-onset are O(1)."
    )

    out = OUT / "cand1_scale_audit.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\nWrote {out}")
    print(f"recommended_grace_N_ticks={report['recommended_grace_N_ticks']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
