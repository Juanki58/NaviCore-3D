#!/usr/bin/env python3
"""Evaluate preregistered super_tunnel B/B_dirty + N_always isolation.

Thresholds and verdict rules are frozen in:
  docs/diagnostics/16-super-tunnel-ieee952-rerun-protocol.md

Do not change thresholds here after seeing results — edit the protocol first.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "docs" / "benchmarks" / "super_tunnel_bd_rerun"
RESULTS_CSV = OUT_DIR / "results.csv"
VERDICT_JSON = OUT_DIR / "verdict.json"
PROTOCOL = "docs/diagnostics/16-super-tunnel-ieee952-rerun-protocol.md"

# --- preregistered (must match protocol §2) ---
C1_MAX_CLEAN_PENALTY_M = 50.0
C2_MIN_DIRTY_PENALTY_M = 400.0
C3_MIN_DIRTY_MINUS_CLEAN_M = 300.0
D1_MIN_CLEAN_PENALTY_M = 400.0
D2_MAX_ABS_DIFF_M = 150.0

JUMP_DRIFT_H_M = 5.0
JUMP_P_VV_REL = 0.5


def load_results() -> dict[str, dict]:
    if not RESULTS_CSV.is_file():
        raise FileNotFoundError(f"Missing {RESULTS_CSV}; run NaviCore3D_Sim --nhc-bd-rerun")
    rows: dict[str, dict] = {}
    with RESULTS_CSV.open(newline="", encoding="utf-8") as fp:
        for row in csv.DictReader(fp):
            rows[row["experiment_id"]] = {
                "drift_exit_m": float(row["drift_exit_m"]),
                "drift_final_m": float(row["drift_final_m"]),
                "nhc_updates": int(row["nhc_updates"]),
                "innov_max_norm_mps": float(row["innov_max_norm_mps"]),
                "anatomy_csv": row["anatomy_csv"],
                "trace_csv": row.get("trace_csv", ""),
                "nhc_policy": row["nhc_policy"],
                "imu_mode": row["imu_mode"],
            }
    required = ["A", "A_dirty", "B", "B_dirty", "N_always", "N_always_dirty"]
    missing = [k for k in required if k not in rows]
    if missing:
        raise KeyError(f"results.csv missing arms: {missing}")
    return rows


def scan_anatomy_jumps(path: Path) -> dict:
    if not path.is_file():
        return {
            "path": str(path),
            "rows": 0,
            "single_tick_jump": False,
            "jump_events": [],
            "error": "missing_anatomy",
        }

    jump_events: list[dict] = []
    prev_drift = None
    prev_pvv = None
    n = 0
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            n += 1
            drift = float(row["drift_h_m"])
            pvv = float(row["P_vv_frob"])
            t_ms = int(row["t_ms"])
            reasons: list[str] = []
            if prev_drift is not None and abs(drift - prev_drift) > JUMP_DRIFT_H_M:
                reasons.append(f"d_drift_h={drift - prev_drift:+.3f}")
            if prev_pvv is not None:
                denom = max(prev_pvv, 1e-9)
                rel = abs(pvv - prev_pvv) / denom
                if rel > JUMP_P_VV_REL:
                    reasons.append(f"d_P_vv_rel={rel:.3f}")
            if reasons:
                jump_events.append(
                    {
                        "t_ms": t_ms,
                        "reasons": reasons,
                        "drift_h_m": drift,
                        "P_vv_frob": pvv,
                        "nhc_applied": int(row["nhc_applied"]),
                    }
                )
            prev_drift = drift
            prev_pvv = pvv

    return {
        "path": str(path),
        "rows": n,
        "single_tick_jump": len(jump_events) > 0,
        "jump_event_count": len(jump_events),
        "jump_events_head": jump_events[:10],
    }


def panel_verdict(delta_clean: float, delta_dirty: float) -> dict:
    c1 = delta_clean <= C1_MAX_CLEAN_PENALTY_M
    c2 = delta_dirty >= C2_MIN_DIRTY_PENALTY_M
    c3 = (delta_dirty - delta_clean) >= C3_MIN_DIRTY_MINUS_CLEAN_M
    d1 = delta_clean >= D1_MIN_CLEAN_PENALTY_M
    d2 = abs(delta_dirty - delta_clean) < D2_MAX_ABS_DIFF_M

    if c1 and c2 and c3:
        label = "IEEE952_BIAS_CONFIRMED"
    elif d1 or d2:
        label = "IEEE952_BIAS_REJECTED"
    else:
        label = "INCONCLUSIVE"

    return {
        "delta_clean_m": delta_clean,
        "delta_dirty_m": delta_dirty,
        "delta_dirty_minus_clean_m": delta_dirty - delta_clean,
        "C1": c1,
        "C2": c2,
        "C3": c3,
        "D1": d1,
        "D2": d2,
        "verdict": label,
    }


def autopsy_required(panel: dict, anatomy: dict, clean_worsens: bool) -> bool:
    if anatomy.get("single_tick_jump"):
        return True
    if clean_worsens:
        return True
    if panel["verdict"] != "IEEE952_BIAS_CONFIRMED" and panel["delta_clean_m"] > C1_MAX_CLEAN_PENALTY_M:
        # protocol §3: NHC worsens with clean IMU → autopsy before IEEE attribution
        return True
    return False


def main() -> int:
    rows = load_results()

    delta_b = rows["B"]["drift_exit_m"] - rows["A"]["drift_exit_m"]
    delta_bdirty = rows["B_dirty"]["drift_exit_m"] - rows["A_dirty"]["drift_exit_m"]
    delta_n = rows["N_always"]["drift_exit_m"] - rows["A"]["drift_exit_m"]
    delta_ndirty = rows["N_always_dirty"]["drift_exit_m"] - rows["A_dirty"]["drift_exit_m"]

    panel_b = panel_verdict(delta_b, delta_bdirty)
    panel_n = panel_verdict(delta_n, delta_ndirty)

    anatomy = {}
    for arm_id, meta in rows.items():
        anatomy[arm_id] = scan_anatomy_jumps(REPO_ROOT / meta["anatomy_csv"])

    autopsy_b = autopsy_required(
        panel_b, anatomy["B"], clean_worsens=delta_b > C1_MAX_CLEAN_PENALTY_M
    )
    autopsy_n = autopsy_required(
        panel_n, anatomy["N_always"], clean_worsens=delta_n > C1_MAX_CLEAN_PENALTY_M
    )

    # Aggregate claim: IEEE confirmed only if BOTH panels confirm AND no autopsy gate
    if (
        panel_b["verdict"] == "IEEE952_BIAS_CONFIRMED"
        and panel_n["verdict"] == "IEEE952_BIAS_CONFIRMED"
        and not autopsy_b
        and not autopsy_n
    ):
        overall = "IEEE952_BIAS_CONFIRMED"
    elif (
        panel_b["verdict"] == "IEEE952_BIAS_REJECTED"
        or panel_n["verdict"] == "IEEE952_BIAS_REJECTED"
    ):
        overall = "IEEE952_BIAS_REJECTED"
    else:
        overall = "INCONCLUSIVE"

    if autopsy_b or autopsy_n:
        # Protocol: do not accept aggregate as IEEE confirmation when autopsy required
        if overall == "IEEE952_BIAS_CONFIRMED":
            overall = "INCONCLUSIVE_PENDING_AUTOPSY"
        note_autopsy = (
            "Autopsy required before attributing to IEEE-952 "
            "(clean-IMU worsening and/or single-tick jump)."
        )
    else:
        note_autopsy = ""

    out = {
        "protocol": PROTOCOL,
        "thresholds": {
            "C1_max_clean_penalty_m": C1_MAX_CLEAN_PENALTY_M,
            "C2_min_dirty_penalty_m": C2_MIN_DIRTY_PENALTY_M,
            "C3_min_dirty_minus_clean_m": C3_MIN_DIRTY_MINUS_CLEAN_M,
            "D1_min_clean_penalty_m": D1_MIN_CLEAN_PENALTY_M,
            "D2_max_abs_diff_m": D2_MAX_ABS_DIFF_M,
            "jump_drift_h_m": JUMP_DRIFT_H_M,
            "jump_p_vv_rel": JUMP_P_VV_REL,
        },
        "drifts_exit_m": {k: v["drift_exit_m"] for k, v in rows.items()},
        "panel_B_constant_vel": panel_b,
        "panel_N_always": panel_n,
        "anatomy": anatomy,
        "autopsy_required_panel_B": autopsy_b,
        "autopsy_required_panel_N_always": autopsy_n,
        "overall_verdict": overall,
        "note": note_autopsy,
        "provenance_note": (
            "Current binary != original 481/1416 run; see protocol §0. "
            "Do not treat original figures as today's measurement."
        ),
    }

    VERDICT_JSON.write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("=== super_tunnel B/B_dirty preregistered verdict ===")
    print(f"protocol: {PROTOCOL}")
    print("drifts_exit_m:")
    for k, v in out["drifts_exit_m"].items():
        print(f"  {k:16s} {v:10.2f} m")
    print(
        f"panel_B:  Δ_clean={panel_b['delta_clean_m']:+.2f}  "
        f"Δ_dirty={panel_b['delta_dirty_m']:+.2f}  -> {panel_b['verdict']}"
    )
    print(
        f"panel_N:  Δ_clean={panel_n['delta_clean_m']:+.2f}  "
        f"Δ_dirty={panel_n['delta_dirty_m']:+.2f}  -> {panel_n['verdict']}"
    )
    print(f"autopsy_B={autopsy_b}  autopsy_N={autopsy_n}")
    print(f"OVERALL: {overall}")
    if note_autopsy:
        print(note_autopsy)
    print(f"wrote {VERDICT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
