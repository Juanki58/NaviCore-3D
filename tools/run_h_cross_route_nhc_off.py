#!/usr/bin/env python3
"""Cross-route control: same EKF / same shell (NHC off), different trajectory.

REF  = 19082026 (B_nhc_disabled metrics, or re-run)
ALT  = 16072026 (docs/benchmarks/real_run_replay.csv) — only alternate in-repo

Max 2 new routes; we have 1. Classify A/B/C vs REF signature.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
REPLAY = REPO / "build" / "NaviCore3D_Replay.exe"
MOUNT = REPO / "calibration" / "imu_mount.json"
OUT = REPO / "docs" / "benchmarks" / "h_cross_route_nhc_off"

ROUTES = {
    "REF_19082026": REPO
    / "docs"
    / "benchmarks"
    / "real_run_19082026_baseline"
    / "real_run_replay.csv",
    "ALT_16072026": REPO / "docs" / "benchmarks" / "real_run_replay.csv",
}

# Structural signature from REF (NHC-off investigation)
REF_SIGNATURE = {
    "t_first_permanent_reject_s_approx": 20.3,
    "early_reject_window_s": (15.0, 30.0),
    "delta_v_E_regime_s_approx": -8.3,
    "delta_v_E_abs_min_for_same": 4.0,  # |Δv_E| in 1s before first regime reject
    "nis_ve_share_min": 0.70,
}


def run_route(name: str, csv: Path) -> Path:
    arm = OUT / name
    arm.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(REPLAY),
        "--input",
        str(csv),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(MOUNT),
        "--yaw-init",
        "zero",
        "--h9a-gravity-tilt-init",
        "--constraint-policy",
        "disabled",
        "--nhc-policy",
        "disabled",
        "--gnss-obs-mode",
        "pos_vel",
        "--p-pv-policy",
        "none",
        "--output",
        str(arm / "replay_output.csv"),
        "--gap3-gnss-nis-audit-csv",
        str(arm / "gnss_nis_audit.csv"),
        "--gap3-constraint-pipeline-audit-csv",
        str(arm / "constraint_pipeline_audit.csv"),
    ]
    print("RUN", name, flush=True)
    log = subprocess.run(cmd, cwd=str(REPO), check=True, capture_output=True, text=True)
    (arm / "replay.log").write_text(
        (log.stdout or "") + (log.stderr or ""), encoding="utf-8", errors="replace"
    )
    return arm


def t_first_permanent_reject(gnss: pd.DataFrame) -> float | None:
    accepts = gnss["accepted"].to_numpy()
    times = gnss["timestamp_s"].to_numpy()
    for i in range(len(accepts)):
        if int(accepts[i]) == 0 and (accepts[i:] == 0).all():
            return float(times[i])
    return None


def regime_change_idx(gnss: pd.DataFrame) -> int | None:
    for i in range(1, len(gnss)):
        if int(gnss.iloc[i]["accepted"]) != 0:
            continue
        window = gnss.iloc[i : i + 5]["accepted"]
        if len(window) >= 3 and int((window == 0).sum()) >= 3:
            if int(gnss.iloc[i - 1]["accepted"]) == 1:
                return i
    # fallback: first reject
    for i in range(len(gnss)):
        if int(gnss.iloc[i]["accepted"]) == 0:
            return i
    return None


def t_sep_residual(out: pd.DataFrame, thr: float = 30.0, hold_s: float = 10.0) -> float | None:
    """Horizontal residual vs GPS using GPS-labeled rows if present; else None."""
    if "residual_h_m" in out.columns:
        r = out["residual_h_m"].to_numpy()
        t = out["timestamp_s"].to_numpy()
    else:
        return None
    above = r >= thr
    if not above.any():
        return None
    for i in range(len(r)):
        if not above[i]:
            continue
        t0 = t[i]
        if np.all(r[(t >= t0) & (t <= t0 + hold_s)] >= thr):
            return float(t0)
    return None


def residual_from_input(arm: Path, input_csv: Path) -> dict:
    out = pd.read_csv(arm / "replay_output.csv")
    truth = pd.read_csv(input_csv)
    gps_t = truth[truth["type"] == "GPS"][["timestamp_s", "pos_n", "pos_e"]].dropna()
    filt = out[out["row_type"] == "GPS"][["timestamp_s", "pos_n_m", "pos_e_m"]].copy()
    if filt.empty or gps_t.empty:
        # log deriva
        log = (arm / "replay.log").read_text(encoding="utf-8", errors="replace")
        for line in log.splitlines():
            if "Deriva final H:" in line:
                try:
                    return {"deriva_final_h_m": float(line.split(":")[-1].strip().split()[0])}
                except ValueError:
                    pass
        return {"available": False}
    # match by nearest timestamp
    residuals = []
    times = []
    gt = gps_t.to_numpy()
    for _, row in filt.iterrows():
        ts = float(row["timestamp_s"])
        j = int(np.argmin(np.abs(gt[:, 0] - ts)))
        if abs(gt[j, 0] - ts) > 0.05:
            continue
        rh = math.hypot(float(row["pos_n_m"]) - gt[j, 1], float(row["pos_e_m"]) - gt[j, 2])
        residuals.append(rh)
        times.append(ts)
    if not residuals:
        return {"available": False}
    r = np.array(residuals)
    t = np.array(times)
    outd = {
        "available": True,
        "residual_h_final_m": float(r[-1]),
        "residual_h_max_m": float(r.max()),
        "residual_h_at_60s_m": float(r[t <= 60][-1]) if np.any(t <= 60) else None,
    }
    # t_sep
    thr, hold = 30.0, 10.0
    t_sep = None
    for i in range(len(r)):
        if r[i] < thr:
            continue
        t0 = t[i]
        mask = (t >= t0) & (t <= t0 + hold)
        if mask.any() and np.all(r[mask] >= thr):
            t_sep = float(t0)
            break
    outd["t_sep_s"] = t_sep
    return outd


def summarize(arm: Path, input_csv: Path) -> dict:
    gnss = pd.read_csv(arm / "gnss_nis_audit.csv")
    pipe = pd.read_csv(arm / "constraint_pipeline_audit.csv")
    n = len(gnss)
    n_acc = int((gnss["accepted"] == 1).sum())
    n_rej = int((gnss["accepted"] == 0).sum())
    t_perm = t_first_permanent_reject(gnss)
    ri = regime_change_idx(gnss)
    regime = None
    delta_ve = None
    nis_info = None
    if ri is not None:
        cur = gnss.iloc[ri]
        prev = gnss.iloc[ri - 1] if ri > 0 else None
        t_rej = float(cur["timestamp_s"])
        t_acc = float(prev["timestamp_s"]) if prev is not None else None
        contrib = {
            "n": float(cur["nis_contrib_n"]),
            "e": float(cur["nis_contrib_e"]),
            "d": float(cur["nis_contrib_d"]),
            "vn": float(cur["nis_contrib_vn"]),
            "ve": float(cur["nis_contrib_ve"]),
        }
        abs_sum = sum(abs(v) for v in contrib.values()) or 1.0
        shares = {k: abs(v) / abs_sum for k, v in contrib.items()}
        dom = max(shares, key=shares.get)
        nis_info = {
            "t_s": t_rej,
            "gps_index": int(cur["gps_index"]),
            "gnss_nis_gate": float(cur["gnss_nis_gate"]),
            "nis_threshold": float(cur["nis_threshold"]),
            "n_meas": int(cur["n_meas"]),
            "dominant": dom,
            "shares": shares,
            "contrib_ve": contrib["ve"],
            "innov_ve": float(cur["innov_ve_mps"]),
        }
        if t_acc is not None and prev is not None:
            v0 = float(prev["vel_after_e_mps"])
            v1 = float(cur["vel_pred_e_mps"])
            delta_ve = v1 - v0
            # also sum pipeline dv_pred_e
            w = pipe[(pipe["timestamp_s"] > t_acc) & (pipe["timestamp_s"] <= t_rej)]
            sum_dv = float(w["dv_pred_e"].sum()) if len(w) else None
            regime = {
                "t_last_accept_s": t_acc,
                "t_first_regime_reject_s": t_rej,
                "delta_v_E_vel_mps": delta_ve,
                "sum_dv_pred_e_mps": sum_dv,
                "v_E_after_accept": v0,
                "v_E_at_reject_pred": v1,
            }

    log = (arm / "replay.log").read_text(encoding="utf-8", errors="replace")
    deriva = None
    for line in log.splitlines():
        if "Deriva final H:" in line:
            try:
                deriva = float(line.split(":")[-1].strip().split()[0])
            except ValueError:
                pass

    res = residual_from_input(arm, input_csv)
    return {
        "input": str(input_csv.relative_to(REPO)),
        "duration_hint_s": float(gnss["timestamp_s"].iloc[-1]) if n else None,
        "gnss": {
            "n_events": n,
            "n_accept": n_acc,
            "n_reject": n_rej,
            "accept_rate": float(n_acc / n) if n else None,
            "t_first_permanent_reject_s": t_perm,
        },
        "regime_change": regime,
        "first_regime_reject_nis": nis_info,
        "residual": res,
        "deriva_final_h_m": deriva,
    }


def classify(ref: dict, alt: dict) -> dict:
    """User A/B/C vs REF structural signature."""
    alt_t = (alt.get("regime_change") or {}).get("t_first_regime_reject_s")
    alt_dv = (alt.get("regime_change") or {}).get("sum_dv_pred_e_mps")
    if alt_dv is None:
        alt_dv = (alt.get("regime_change") or {}).get("delta_v_E_vel_mps")
    alt_nis = alt.get("first_regime_reject_nis") or {}
    alt_ve_share = (alt_nis.get("shares") or {}).get("ve", 0.0)
    alt_dom = alt_nis.get("dominant")

    early = alt_t is not None and REF_SIGNATURE["early_reject_window_s"][0] <= alt_t <= REF_SIGNATURE["early_reject_window_s"][1]
    big_dv = alt_dv is not None and abs(alt_dv) >= REF_SIGNATURE["delta_v_E_abs_min_for_same"]
    ve_dom = alt_dom == "ve" and alt_ve_share >= REF_SIGNATURE["nis_ve_share_min"]

    same_structure = early and big_dv and ve_dom

    # "works perfectly": high accept rate, no early permanent reject, residual not km-scale early
    ar = (alt.get("gnss") or {}).get("accept_rate") or 0.0
    t_perm = (alt.get("gnss") or {}).get("t_first_permanent_reject_s")
    res_60 = (alt.get("residual") or {}).get("residual_h_at_60s_m")
    works = (
        ar >= 0.5
        and (t_perm is None or t_perm > 120.0)
        and (res_60 is None or res_60 < 100.0)
    )

    if same_structure:
        case = "A"
        meaning = (
            "Same structural signature as REF (early ~20s reject, large |Δv_E|, NIS ve-dominant). "
            "Not route-specific → structural bug pressure toward rewrite."
        )
    elif works:
        case = "B"
        meaning = (
            "ALT works under same shell → REF route has special property; "
            "search space shrinks to route-specific factors."
        )
    else:
        case = "C"
        meaning = (
            "Fails but different signature → general weakness, mechanism not identical to REF."
        )

    return {
        "case": case,
        "meaning": meaning,
        "flags": {
            "early_reject_15_30s": bool(early),
            "large_abs_delta_v_E": bool(big_dv),
            "nis_ve_dominant": bool(ve_dom),
            "same_structure_A": bool(same_structure),
            "works_well_B": bool(works),
        },
        "alt_key_numbers": {
            "t_regime_reject_s": alt_t,
            "delta_v_E_mps": alt_dv,
            "nis_dominant": alt_dom,
            "nis_ve_share": alt_ve_share,
            "accept_rate": ar,
            "t_permanent_reject_s": t_perm,
            "residual_h_at_60s_m": res_60,
            "deriva_final_h_m": alt.get("deriva_final_h_m"),
        },
    }


def main() -> int:
    skip = "--skip-replay" in sys.argv
    OUT.mkdir(parents=True, exist_ok=True)

    # Prefer existing B arm for REF if present
    ref_existing = (
        REPO / "docs" / "benchmarks" / "h_nhc_policy_ab" / "B_nhc_disabled"
    )
    results = {}
    for name, csv in ROUTES.items():
        if not csv.is_file():
            print("MISSING", csv, file=sys.stderr)
            return 1
        if name.startswith("REF") and ref_existing.is_dir() and (ref_existing / "gnss_nis_audit.csv").is_file() and skip:
            arm = ref_existing
        elif name.startswith("REF") and not skip:
            # still re-run into OUT for apples-to-apples logs, OR copy metrics from B
            if (ref_existing / "gnss_nis_audit.csv").is_file():
                arm = ref_existing
                print("REF using existing B_nhc_disabled", flush=True)
            else:
                arm = run_route(name, csv)
        else:
            if skip and (OUT / name / "gnss_nis_audit.csv").is_file():
                arm = OUT / name
            else:
                arm = run_route(name, csv)
        results[name] = summarize(arm, csv)

    disc = classify(results["REF_19082026"], results["ALT_16072026"])
    verdict = {
        "protocol": {
            "shell": "G1-like NHC-off: constraint disabled, nhc disabled, pos_vel, p_pv none, yaw zero + h9a",
            "single_variable": "trajectory CSV only",
            "max_new_routes": 2,
            "available_new_routes_in_repo": 1,
            "metrics": [
                "t_sep",
                "gnss accept/reject",
                "first regime reject",
                "Δv_E",
                "NIS",
                "residual",
            ],
            "note": "Does not replace Δv_E term investigation; control experiment only.",
        },
        "REF_19082026": results["REF_19082026"],
        "ALT_16072026": results["ALT_16072026"],
        "discrimination": disc,
    }
    out = OUT / "verdict.json"
    out.write_text(json.dumps(verdict, indent=2), encoding="utf-8")

    # compact table
    def row(label: str, d: dict) -> str:
        g = d["gnss"]
        r = d.get("regime_change") or {}
        n = d.get("first_regime_reject_nis") or {}
        res = d.get("residual") or {}
        return (
            f"| {label} | {g.get('n_accept')}/{g.get('n_reject')} | "
            f"{g.get('t_first_permanent_reject_s')} | "
            f"{r.get('t_first_regime_reject_s')} | "
            f"{r.get('sum_dv_pred_e_mps') or r.get('delta_v_E_vel_mps')} | "
            f"{n.get('dominant')} ({(n.get('shares') or {}).get('ve')}) | "
            f"{res.get('t_sep_s')} | "
            f"{res.get('residual_h_at_60s_m')} | "
            f"{d.get('deriva_final_h_m')} |"
        )

    md = [
        "# Cross-route control — NHC off",
        "",
        f"**Case {disc['case']}:** {disc['meaning']}",
        "",
        "| Route | accept/reject | t_perm_rej | t_regime_rej | Δv_E (Σdv_pred_e) | NIS dom (ve share) | t_sep | res@60s | deriva_final |",
        "|-------|---------------|------------|--------------|-------------------|--------------------|-------|---------|--------------|",
        row("REF 19082026", results["REF_19082026"]),
        row("ALT 16072026", results["ALT_16072026"]),
        "",
        "Sólo 1 trayecto alternativo en repo; límite usuario = 2.",
    ]
    (OUT / "TABLE.md").write_text("\n".join(md), encoding="utf-8")
    print(json.dumps({"case": disc["case"], "flags": disc["flags"], "alt": disc["alt_key_numbers"]}, indent=2))
    print("CASE", disc["case"], "->", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
