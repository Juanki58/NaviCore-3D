#!/usr/bin/env python3
"""§13.22 cand1 gate sanitize — E1 grace vs E2 Pzz-normalized.

G1: no tunnel fire at NHC-onset (t_s<=0.05).
G2: slalom A fire @ 0.39±0.02 s.
G3: slalom C no fire.
"""

from __future__ import annotations

import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "docs" / "benchmarks" / "jacobian_imu_ab" / "cand1_gate_e12"
SEED = 71
T2 = 3.736646e-6
TMAX = 0.65
# Freeze-Pzz E2: κ = T2 / Pzz_slalom_A(t0) so slalom reproduces absolute-T2 fire time.
# (Using fire-time sumabs/Pzz would break G2 under freeze-at-start.)
PZZ_SLALOM_A_T0 = 3.04627791e-2
KAPPA = T2 / PZZ_SLALOM_A_T0
CELLS = (
    ("A", "correct", "ideal"),
    ("B", "correct", "dirty"),
    ("C", "legacy", "ideal"),
    ("D", "legacy", "dirty"),
)

sys.path.insert(0, str(REPO))
from run_all_benchmarks import run_benchmark  # noqa: E402

FIRE_RE = re.compile(
    r"HATT_D_FIRE t_s=([0-9.eE+-]+) sumabs=([0-9.eE+-]+)"
)


def parse_fire(stdout: str) -> dict | None:
    m = FIRE_RE.search(stdout or "")
    if not m:
        return None
    return {"t_s": float(m.group(1)), "sumabs": float(m.group(2))}


def run_arm(
    tag: str,
    *,
    grace: int = 0,
    gate_norm: bool = False,
    gate_thr: float = T2,
) -> dict:
    pack = {"tag": tag, "grace": grace, "gate_norm": gate_norm, "gate_thr": gate_thr,
            "slalom": {}, "tunnel": {}}
    for cell_id, jac, imu in CELLS:
        kwargs = dict(
            nhc_att_z_forget=0.0,
            nhc_att_z_forget_gate=gate_thr,
            nhc_att_z_forget_tmax=TMAX,
            nhc_att_z_forget_grace=grace,
            nhc_att_z_forget_gate_norm=gate_norm,
            nhc_att_z_unobs=True,
        )
        for scenario, key in (("SLALOM", "slalom"), ("TUNNEL_STRESS", "tunnel")):
            r = run_benchmark(
                f"cand1 {tag} {cell_id} {scenario}",
                scenario,
                seed=SEED,
                imu_mode=imu,
                nhc_jacobian=jac,
                archive_suffix=f"cand1_{tag}_cell{cell_id}_{scenario.lower()}",
                **kwargs,
            )
            fire = parse_fire(r.stdout)
            pack[key][cell_id] = {
                "fired": fire is not None,
                "fire": fire,
                "error": r.error,
            }
            print(
                f"  {tag} {scenario} {cell_id}: "
                f"{'FIRE@'+str(fire['t_s']) if fire else 'nofire'}"
            )
    return pack


def score(pack: dict) -> dict:
    # G1 (prereg): no tunnel fire with t_s <= 0.05 (NHC-epoch)
    g1_cells = {}
    g1_ok = True
    g1_tmax_ok = True  # secondary: no fire within tmax (stronger)
    for cell in "ABCD":
        f = pack["tunnel"][cell].get("fire")
        early = f is not None and f["t_s"] <= 0.05
        in_tmax = f is not None and f["t_s"] <= TMAX
        g1_cells[cell] = {"fire": f, "early_fail": early, "fire_within_tmax": in_tmax}
        if early:
            g1_ok = False
        if in_tmax:
            g1_tmax_ok = False
    # G2 slalom A
    fa = pack["slalom"]["A"].get("fire")
    g2 = bool(fa is not None and abs(fa["t_s"] - 0.39) <= 0.02)
    # G3 slalom C
    g3 = pack["slalom"]["C"].get("fire") is None
    return {
        "G1_tunnel_no_early_fire": {"pass": g1_ok, "cells": g1_cells},
        "G1b_tunnel_nofire_within_tmax": {"pass": g1_tmax_ok, "cells": g1_cells},
        "G2_slalom_A_039": {"pass": g2, "fire": fa},
        "G3_slalom_C_nofire": {"pass": g3, "fire": pack["slalom"]["C"].get("fire")},
        "PASS": bool(g1_ok and g2 and g3),
        "PASS_strict_G1b": bool(g1_tmax_ok and g2 and g3),
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)

    # Load grace N from scale audit if present
    scale_path = OUT / "cand1_scale_audit.json"
    # Scale audit: dirty B needs ~32 for |dx| calm; C spike is 1-tick.
    # Primary E1 arm uses max heuristic; also report N=1 smoke in JSON via E1n1.
    grace_n = 32
    if scale_path.is_file():
        scale = json.loads(scale_path.read_text(encoding="utf-8"))
        if scale.get("recommended_grace_N_ticks"):
            grace_n = max(int(scale["recommended_grace_N_ticks"]), 1)

    print(f"Using grace_N={grace_n}, kappa={KAPPA:.6e} (T2/Pzz_slalom_t0)")

    arms = {}
    print("\n===== E0 baseline (no grace, absolute T2) =====")
    arms["E0"] = run_arm("E0", grace=0, gate_norm=False, gate_thr=T2)
    print("\n===== E1 grace N=1 (discard single onset spike) =====")
    arms["E1n1"] = run_arm("E1n1", grace=1, gate_norm=False, gate_thr=T2)
    print(f"\n===== E1 grace N={grace_n} =====")
    arms["E1"] = run_arm("E1", grace=grace_n, gate_norm=False, gate_thr=T2)
    print("\n===== E2 freeze-Pzz norm =====")
    arms["E2"] = run_arm("E2", grace=0, gate_norm=True, gate_thr=KAPPA)
    print(f"\n===== E1n1+E2 (grace1 + freeze norm) =====")
    arms["E1n1_E2"] = run_arm("E1n1_E2", grace=1, gate_norm=True, gate_thr=KAPPA)

    scores = {k: score(v) for k, v in arms.items()}
    report = {
        "protocol": "docs/diagnostics/18-jacobian-imu-ab-protocol.md §13.22",
        "timestamp": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "grace_N": grace_n,
        "kappa": KAPPA,
        "T2": T2,
        "arms": arms,
        "scores": scores,
    }
    out = OUT / "cand1_gate_e12_report.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")

    lines = [
        "# cand1 gate E1/E2 scorecard (§13.22)",
        "",
        f"grace_N={grace_n}  kappa={KAPPA:.3e}",
        "",
        "| Arm | G1 early | G1b tmax | G2 A@0.39 | G3 C | PASS | PASS_strict |",
        "|-----|----------|----------|-----------|------|------|-------------|",
    ]
    for k in ("E0", "E1n1", "E1", "E2", "E1n1_E2"):
        s = scores[k]
        lines.append(
            f"| {k} | {'PASS' if s['G1_tunnel_no_early_fire']['pass'] else 'FAIL'} | "
            f"{'PASS' if s['G1b_tunnel_nofire_within_tmax']['pass'] else 'FAIL'} | "
            f"{'PASS' if s['G2_slalom_A_039']['pass'] else 'FAIL'} | "
            f"{'PASS' if s['G3_slalom_C_nofire']['pass'] else 'FAIL'} | "
            f"{'PASS' if s['PASS'] else 'FAIL'} | "
            f"{'PASS' if s['PASS_strict_G1b'] else 'FAIL'} |"
        )
    lines.append("")
    lines.append(f"JSON: `{out.name}`")
    (OUT / "cand1_gate_e12_report.md").write_text("\n".join(lines), encoding="utf-8")

    print("\n======== SCORECARD ========")
    for k, s in scores.items():
        print(f"{k}: PASS={s['PASS']}  G1={s['G1_tunnel_no_early_fire']['pass']} "
              f"G2={s['G2_slalom_A_039']['pass']} G3={s['G3_slalom_C_nofire']['pass']}")
        print(f"  tunnel fires: "
              + ", ".join(
                  f"{c}:{(arms[k]['tunnel'][c]['fire'] or {}).get('t_s', '—')}"
                  for c in "ABCD"
              ))
        print(f"  slalom A={arms[k]['slalom']['A']['fire']} C={arms[k]['slalom']['C']['fire']}")
    print(f"Wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
