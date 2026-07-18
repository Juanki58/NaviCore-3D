#!/usr/bin/env python3
"""GAP-4 §11 — P_pv gate autopsy: trigger frequency, cos alignment, K/P at anchor ticks."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
R_VEL = 2.25


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    skip = {"update_type", "phase", "reject_reason"}
    for col in df.columns:
        if col not in skip:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def load_k_blocks(path: Path) -> dict[int, dict]:
    text = path.read_text(encoding="utf-8")
    out: dict[int, dict] = {}
    depth = 0
    start = None
    for i, ch in enumerate(text):
        if ch == "{":
            if depth == 0:
                start = i
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0 and start is not None:
                obj = json.loads(text[start : i + 1])
                out[int(obj["gps_index"])] = obj
                start = None
    return out


def frob(m: np.ndarray) -> float:
    return float(np.linalg.norm(m, "fro"))


def build_minimal_p(
    p_pp: np.ndarray,
    p_vp: np.ndarray,
    p_vv_diag: np.ndarray,
    p_pv_frob: float,
    p_aa_frob: float,
) -> np.ndarray:
    p = np.zeros((15, 15))
    p[0:3, 0:3] = p_pp
    p[3:6, 3:6] = np.diag(p_vv_diag)
    p[3:6, 0:3] = p_vp
    p[0:3, 3:6] = p_vp.T
    aa = (p_aa_frob / math.sqrt(3)) ** 2 if p_aa_frob > 0 else 1e-4
    for i in range(6, 9):
        p[i, i] = aa
    for i in range(9, 15):
        p[i, i] = 1e-4
    cur = frob(p[3:6, 0:3])
    if cur > 0 and p_pv_frob > 0:
        s = p_pv_frob / cur
        p[3:6, 0:3] *= s
        p[0:3, 3:6] *= s
    return 0.5 * (p + p.T)


def build_h(n_meas: int) -> np.ndarray:
    h = np.zeros((n_meas, 15))
    if n_meas >= 3:
        for i in range(3):
            h[i, i] = 1.0
    if n_meas >= 5:
        h[3, 3] = 1.0
        h[4, 4] = 1.0
    return h


def build_r(n_meas: int, r_pos: float) -> np.ndarray:
    r = np.zeros((n_meas, n_meas))
    if n_meas >= 3:
        for i in range(3):
            r[i, i] = r_pos
    if n_meas == 5:
        r[3, 3] = R_VEL
        r[4, 4] = R_VEL
    return r


def cos2(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-12 or nb < 1e-12:
        return float("nan")
    return float(np.dot(a, b) / (na * nb))


def gate_replay_row(
    arm_dir: Path,
    gps_index: int,
    *,
    policy: str,
) -> dict | None:
    gnss = load_csv(arm_dir / "gnss_nis_audit.csv")
    row = gnss[gnss["gps_index"] == gps_index]
    if row.empty:
        return None
    g = row.iloc[0]
    if int(g["accepted"]) != 1:
        return {"gps_index": gps_index, "accepted": False, "timestamp_s": float(g["timestamp_s"])}

    k_blocks = load_k_blocks(arm_dir / "gnss_k_block.jsonl")
    if gps_index not in k_blocks:
        return {"gps_index": gps_index, "accepted": True, "error": "missing k_block"}
    kb = k_blocks[gps_index]
    cov = load_csv(arm_dir / "cov_step_audit.csv")
    ts = float(g["timestamp_s"])
    pre = cov[
        (cov["update_type"] == "gnss")
        & (cov["phase"] == "pre")
        & (np.isclose(cov["timestamp_s"], ts, atol=1e-6))
    ]
    post = cov[
        (cov["update_type"] == "gnss")
        & (cov["phase"] == "post_accept")
        & (np.isclose(cov["timestamp_s"], ts, atol=1e-6))
    ]
    if pre.empty:
        return {"gps_index": gps_index, "accepted": True, "error": "missing cov pre"}
    pre = pre.iloc[0]
    post = post.iloc[0] if not post.empty else None

    p_pp = np.array(kb["HPH_m2"], dtype=float)
    p_vp = np.array(kb["P_vel_pos_cross_m2"], dtype=float)
    p_vv = np.array([pre["P_vv_n_m2"], pre["P_vv_e_m2"], pre["P_vv_d_m2"]], dtype=float)
    p = build_minimal_p(p_pp, p_vp, p_vv, float(pre["P_pv_frob"]), float(pre["P_aa_frob"]))
    h5 = build_h(5)
    r5 = build_r(5, float(kb["R_m2"]))
    k5 = p @ h5.T @ np.linalg.inv(h5 @ p @ h5.T + r5)

    y5 = np.array(
        [
            g["innov_n_m"],
            g["innov_e_m"],
            g["innov_d_m"],
            g["pseudo_innov_v_n_mps"],
            g["pseudo_innov_v_e_mps"],
        ]
    )
    vg_n = g["gps_speed_mps"] * math.cos(math.radians(g["gps_course_deg"]))
    vg_e = g["gps_speed_mps"] * math.sin(math.radians(g["gps_course_deg"]))
    err_pre = np.array([g["vel_pred_n_mps"] - vg_n, g["vel_pred_e_mps"] - vg_e])

    dv_pos = k5[3:5, 0:3] @ y5[0:3]
    dv_tot = k5[3:5, :] @ y5
    cos_pos = cos2(dv_pos, err_pre)
    cos_tot = cos2(dv_tot, err_pre)

    trig_pos = bool(cos_pos > 0.0)
    trig_tot = bool(cos_tot > 0.0)
    if policy == "cos_pos":
        trig_expected = trig_pos
    elif policy == "cos_tot":
        trig_expected = trig_tot
    else:
        trig_expected = None

    p_pv_post = float(post["P_pv_frob"]) if post is not None else float("nan")
    p_pv_zeroed = bool(post is not None and p_pv_post < 1e-3)

    k_logged = np.array(kb["K_vel_pos"], dtype=float)
    k_vel_pos_max = float(np.max(np.abs(k_logged)))

    return {
        "gps_index": gps_index,
        "timestamp_s": ts,
        "accepted": True,
        "policy": policy,
        "innov_h_m": float(g["innov_h_m"]),
        "nis_full": float(g["nis_full"]),
        "P_pv_frob_pre": float(pre["P_pv_frob"]),
        "P_pv_frob_post": p_pv_post,
        "P_vv_frob_pre": float(pre["P_vv_frob"]),
        "P_vv_frob_post": float(post["P_vv_frob"]) if post is not None else float("nan"),
        "k_vel_max": float(g["k_vel_max"]),
        "k_pos_max": float(g["k_pos_max"]),
        "cos_pos": cos_pos,
        "cos_tot": cos_tot,
        "trigger_pos": trig_pos,
        "trigger_tot": trig_tot,
        "trigger_expected": trig_expected,
        "P_pv_zeroed_post": p_pv_zeroed,
        "trigger_matches_P_pv_zero": (
            None if trig_expected is None else bool(trig_expected == p_pv_zeroed)
        ),
        "K_vel_pos_max_logged": k_vel_pos_max,
        "dv_pos_NE": dv_pos.tolist(),
        "dv_tot_NE": dv_tot.tolist(),
        "dx_vel_logged_NE": [float(g["dx_vel_n_mps"]), float(g["dx_vel_e_mps"])],
        "err_vel_pre_mps": float(np.linalg.norm(err_pre)),
        "err_vel_post_mps": float(
            np.linalg.norm(
                np.array([g["vel_after_n_mps"], g["vel_after_e_mps"]])
                - np.array([vg_n, vg_e])
            )
        ),
    }


def summarize_window(arm_dir: Path, policy: str, t_max: float) -> dict:
    gnss = load_csv(arm_dir / "gnss_nis_audit.csv")
    acc = gnss[(gnss["accepted"] == 1) & (gnss["timestamp_s"] <= t_max)]
    rows = []
    for idx in acc["gps_index"].astype(int):
        r = gate_replay_row(arm_dir, int(idx), policy=policy)
        if r:
            rows.append(r)
    trig_col = "trigger_pos" if policy == "cos_pos" else "trigger_tot"
    n_trig = sum(1 for r in rows if r.get(trig_col))
    return {
        "arm_dir": str(arm_dir),
        "policy": policy,
        "t_max_s": t_max,
        "accepts_in_window": len(rows),
        "gate_triggers_in_window": n_trig,
        "trigger_rate": n_trig / max(len(rows), 1),
        "accepts": rows,
    }


def autopsy_tick(arm_dir: Path, gps_index: int, policy: str, control_dir: Path) -> dict:
    main = gate_replay_row(arm_dir, gps_index, policy=policy)
    ctrl_g = load_csv(control_dir / "gnss_nis_audit.csv")
    ctrl = ctrl_g[ctrl_g["gps_index"] == gps_index]
    ctrl_row = None
    if not ctrl.empty:
        c = ctrl.iloc[0]
        ctrl_row = {
            "accepted": int(c["accepted"]),
            "innov_h_m": float(c["innov_h_m"]),
            "nis_full": float(c["nis_full"]),
            "k_vel_max": float(c["k_vel_max"]),
            "k_pos_max": float(c["k_pos_max"]),
            "reject_reason": int(c["reject_reason"]) if pd.notna(c["reject_reason"]) else None,
        }
    return {
        "tick": main,
        "control_none": ctrl_row,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, default=REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity")
    parser.add_argument("--control", type=str, default="G1_control_full_ppv_none")
    parser.add_argument("--t-window", type=float, default=34.0)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    base = args.base
    control = base / args.control
    arms = {
        "1d_cos_pos": (base / "G1_intervention/arm_1d_cos_pos", "cos_pos"),
        "1d_prime_cos_tot": (base / "G1_intervention/arm_1d_prime_cos_tot", "cos_tot"),
    }

    report = {
        "control_dir": str(control),
        "control_abort": json.loads((control / "gap4_g1_report.json").read_text())["abort_guardrail"],
        "gate_frequency_first_34s": {},
        "autopsy": {
            "gps_236_1d": autopsy_tick(
                arms["1d_cos_pos"][0], 236, "cos_pos", control
            ),
            "gps_32_1d_prime": autopsy_tick(
                arms["1d_prime_cos_tot"][0], 32, "cos_tot", control
            ),
        },
    }

    for name, (arm_dir, policy) in arms.items():
        report["gate_frequency_first_34s"][name] = summarize_window(arm_dir, policy, args.t_window)

    # Control accepts in same window for reference
    ctrl_g = load_csv(control / "gnss_nis_audit.csv")
    ctrl_acc = ctrl_g[(ctrl_g["accepted"] == 1) & (ctrl_g["timestamp_s"] <= args.t_window)]
    report["control_accepts_first_34s"] = int(len(ctrl_acc))

    text = json.dumps(report, indent=2)
    print(text)
    out = args.out or (base / "G1_intervention" / "ppv_gate_autopsy_report.json")
    out.write_text(text, encoding="utf-8")
    print(f"\nWrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
