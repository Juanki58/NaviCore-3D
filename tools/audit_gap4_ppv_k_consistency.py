#!/usr/bin/env python3
"""GAP-4 — Verify P_pv policy uses K recomputed after zeroing cross block."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
R_VEL = 2.25  # m²/s² (σ=1.5 m/s)


def load_k_block(path: Path, gps_index: int) -> dict:
    text = path.read_text(encoding="utf-8")
    objs = []
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
                objs.append(json.loads(text[start : i + 1]))
                start = None
    for o in objs:
        if int(o["gps_index"]) == gps_index:
            return o
    raise KeyError(f"no k_block gps_index={gps_index} in {path}")


def load_cov_row(cov_path: Path, ts: float, phase: str) -> pd.Series:
    cov = pd.read_csv(cov_path)
    for c in cov.columns:
        if c not in ("update_type", "phase"):
            cov[c] = pd.to_numeric(cov[c], errors="coerce")
    row = cov[
        (cov["update_type"] == "gnss")
        & (cov["phase"] == phase)
        & (np.isclose(cov["timestamp_s"], ts, atol=1e-6))
    ]
    if row.empty:
        raise KeyError(f"no cov {phase} at {ts}")
    return row.iloc[0]


def load_gnss_row(gnss_path: Path, gps_index: int) -> pd.Series:
    g = pd.read_csv(gnss_path)
    for c in g.columns:
        if c not in ("reject_reason",):
            g[c] = pd.to_numeric(g[c], errors="coerce")
    row = g[(g["gps_index"] == gps_index) & (g["accepted"] == 1)]
    if row.empty:
        raise KeyError(f"no accept gps_index={gps_index}")
    return row.iloc[0]


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


def zero_pv_cross(p: np.ndarray) -> np.ndarray:
    out = p.copy()
    out[3:6, 0:3] = 0.0
    out[0:3, 3:6] = 0.0
    return out


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


def compute_k(p: np.ndarray, h: np.ndarray, r: np.ndarray) -> np.ndarray:
    s = h @ p @ h.T + r
    return p @ h.T @ np.linalg.inv(s)


def extract_logged_k_vel_pos(k_block: dict) -> np.ndarray:
    rows = k_block.get("K_vel_pos", k_block.get("k_vel_pos"))
    if rows is None:
        raise KeyError("no K_vel_pos in k_block")
    return np.array(rows, dtype=float)


def audit_tick(arm_dir: Path, gps_index: int) -> dict:
    k_blk = load_k_block(arm_dir / "gnss_k_block.jsonl", gps_index)
    ts = float(k_blk["timestamp_s"])
    gnss = load_gnss_row(arm_dir / "gnss_nis_audit.csv", gps_index)
    pre = load_cov_row(arm_dir / "cov_step_audit.csv", ts, "pre")

    p_pp = np.array(k_blk["HPH_m2"], dtype=float)
    p_vp = np.array(k_blk["P_vel_pos_cross_m2"], dtype=float)
    p_vv_diag = np.array([pre["P_vv_n_m2"], pre["P_vv_e_m2"], pre["P_vv_d_m2"]], dtype=float)
    r_pos = float(k_blk["R_m2"])
    p_full = build_minimal_p(
        p_pp, p_vp, p_vv_diag, float(pre["P_pv_frob"]), float(pre["P_aa_frob"])
    )
    p_zero = zero_pv_cross(p_full)

    h5 = build_h(5)
    r5 = build_r(5, r_pos)
    k_full = compute_k(p_full, h5, r5)
    k_zero = compute_k(p_zero, h5, r5)
    k_logged = extract_logged_k_vel_pos(k_blk)

    ppv_triggered = bool(int(gnss.get("ppv_triggered", gnss.get("gnss_ppv_triggered", 0))))
    cos_pos = float(gnss.get("cos_dv_pos_err_pre", math.nan))

    # Expected K used in Joseph: full if gate off, zero-cross if gate on
    k_expected = k_zero if ppv_triggered else k_full

    diff_full_vs_logged = k_full[3:6, 0:3] - k_logged
    diff_zero_vs_logged = k_zero[3:6, 0:3] - k_logged
    diff_expected_vs_logged = k_expected[3:6, 0:3] - k_logged

    y5 = np.array(
        [
            gnss.innov_n_m,
            gnss.innov_e_m,
            gnss.innov_d_m,
            gnss.pseudo_innov_v_n_mps,
            gnss.pseudo_innov_v_e_mps,
        ]
    )

    dv_full = k_full[3:5, 0:3] @ y5[0:3] + k_full[3:5, 3:5] @ y5[3:5]
    dv_zero = k_zero[3:5, 0:3] @ y5[0:3] + k_zero[3:5, 3:5] @ y5[3:5]
    dv_logged = np.array([gnss.dx_vel_n_mps, gnss.dx_vel_e_mps])

    post = load_cov_row(arm_dir / "cov_step_audit.csv", ts, "post_accept")
    joseph_err = None
    if post is not None:
        k_j = k_expected
        p_post_j = 0.5 * (
            (np.eye(15) - k_j @ h5) @ (p_zero if ppv_triggered else p_full) @ (np.eye(15) - k_j @ h5).T
            + k_j @ r5 @ k_j.T
        )
        joseph_err = abs(frob(p_post_j[3:6, 3:6]) - float(post["P_vv_frob"])) / max(
            float(post["P_vv_frob"]), 1e-9
        )

    max_full_logged = float(np.max(np.abs(diff_full_vs_logged)))
    max_zero_logged = float(np.max(np.abs(diff_zero_vs_logged)))
    max_expected_logged = float(np.max(np.abs(diff_expected_vs_logged)))

    consistent = max_expected_logged < 1e-4
    old_bug_pattern = ppv_triggered and max_zero_logged < 1e-4 and max_full_logged > 1e-3

    return {
        "arm_dir": str(arm_dir),
        "gps_index": gps_index,
        "timestamp_s": ts,
        "ppv_policy": str(gnss.get("ppv_policy", "unknown")),
        "ppv_triggered": ppv_triggered,
        "cos_dv_pos_err_pre": cos_pos,
        "P_pv_frob_pre": float(pre["P_pv_frob"]),
        "K_vel_pos": {
            "K_full_NE_pos": k_full[3:5, 0:3].tolist(),
            "K_zero_NE_pos": k_zero[3:5, 0:3].tolist(),
            "K_logged_NE_pos": k_logged.tolist(),
            "K_expected_NE_pos": k_expected[3:5, 0:3].tolist(),
            "max_abs_K_full_minus_logged": max_full_logged,
            "max_abs_K_zero_minus_logged": max_zero_logged,
            "max_abs_K_expected_minus_logged": max_expected_logged,
        },
        "delta_v_NE": {
            "from_K_full_mps": dv_full.tolist(),
            "from_K_zero_mps": dv_zero.tolist(),
            "logged_mps": dv_logged.tolist(),
            "max_abs_dv_zero_minus_logged": float(np.max(np.abs(dv_zero - dv_logged))),
        },
        "joseph_P_vv_rel_error_with_expected_K": joseph_err,
        "verdict": {
            "K_P_consistent": consistent,
            "old_bug_K_full_used_after_zero_P_pv": old_bug_pattern,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm-dir", type=Path, required=True)
    parser.add_argument("--gps-index", type=int, default=2)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    report = audit_tick(args.arm_dir, args.gps_index)
    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        args.out.write_text(text, encoding="utf-8")
        print(f"\nWrote {args.out}")

    if not report["verdict"]["K_P_consistent"]:
        print("\nFAIL: logged K does not match expected K after P_pv policy")
        return 1
    if report["verdict"]["old_bug_K_full_used_after_zero_P_pv"]:
        print("\nFAIL: old bug pattern — K_full used after zero P_pv")
        return 1
    print("\nPASS: K recomputed consistently with P_pv policy")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
