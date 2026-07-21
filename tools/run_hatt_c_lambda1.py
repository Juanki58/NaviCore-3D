#!/usr/bin/env python3
"""H-ATT-c extension §12.6 — λ=1 with same gate (T2/T5). Reuses run_hatt_c helpers."""

from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO))

import run_hatt_c as hc  # noqa: E402

LAMBDAS = (1.0,)


def main() -> int:
    hc.OUT.mkdir(parents=True, exist_ok=True)
    print("\n########## CONTROL ##########")
    control = {"slalom": {}, "tunnel": {}, "audits": {}}
    for cell_id, jac, imu in hc.CELLS:
        audit = None
        if cell_id in ("A", "C") and imu == "ideal":
            audit = hc.OUT / f"ctrl_l1_cell{cell_id}_nhc_block_audit.csv"
        control["slalom"][cell_id] = hc.run_cell(
            cell_id, jac, imu, lam=0.0, gate=None, scenario="SLALOM", audit_path=audit
        )
        if audit:
            control["audits"][cell_id] = str(audit)
        control["tunnel"][cell_id] = hc.run_cell(
            cell_id, jac, imu, lam=0.0, gate=None, scenario="TUNNEL_STRESS"
        )

    arms = {}
    scores = {}
    for arm_name, thr in hc.THRESHOLDS:
        for lam in LAMBDAS:
            key = f"c-{arm_name}-l{lam:g}"
            print(f"\n########## {key} T={thr:.6e} lambda={lam:g} ##########")
            pack = {
                "arm": key,
                "T": thr,
                "lambda": lam,
                "tmax": hc.TMAX,
                "slalom": {},
                "tunnel": {},
                "audits": {},
            }
            for cell_id, jac, imu in hc.CELLS:
                audit = None
                if cell_id in ("A", "C") and imu == "ideal":
                    audit = hc.OUT / f"{key}_cell{cell_id}_nhc_block_audit.csv"
                pack["slalom"][cell_id] = hc.run_cell(
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
                pack["tunnel"][cell_id] = hc.run_cell(
                    cell_id, jac, imu, lam=lam, gate=thr, scenario="TUNNEL_STRESS"
                )
            mech = None
            pa = Path(pack["audits"].get("A", ""))
            pc = Path(pack["audits"].get("C", ""))
            if pa.is_file() and pc.is_file():
                mech = hc.mechanism_p3(hc.load_merged(pa, pc))
            arms[key] = pack
            scores[key] = hc.score_arm(control, pack, mech)

    report = {
        "protocol": "docs/diagnostics/18-jacobian-imu-ab-protocol.md §12.6",
        "intervention": "H-ATT-c lambda=1 extension",
        "timestamp": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "seed": hc.SEED,
        "T2": hc.T2,
        "T5": hc.T5,
        "tmax": hc.TMAX,
        "lambdas": list(LAMBDAS),
        "control": control,
        "arms": arms,
        "scores": scores,
    }
    out_path = hc.OUT / "hatt_c_lambda1_report.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("\n======== H-ATT-c lambda=1 SCORECARD ========")
    for key, s in scores.items():
        print(
            f"{key}: HATT={'PASS' if s['HATT_c_PASS'] else 'FAIL'} | "
            f"P1={s['P1']['pass']} A={s['P1']['A_interv']} | "
            f"P2s={s['P2_slalom']['pass']} P2t={s['P2_tunnel']['pass']} | "
            f"P3A={s['P3_A']['pass']} (fire={s['P3_A']['fire']}) | "
            f"P3C={s['P3_C']['pass']}"
        )
        # C/D detail
        for cell in ("C", "D"):
            print(f"  slalom {cell}: {s['P2_slalom']['cells'][cell]}")
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
