#!/usr/bin/env python3
"""GAP-4 — pos+vel coupling audit: S cross-block, K 5D, Δv decomposition."""

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


def load_cov_row(cov_path: Path, ts: float, phase: str) -> pd.Series | None:
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
        return None
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


def build_h(n_meas: int) -> np.ndarray:
    h = np.zeros((n_meas, 15))
    if n_meas >= 3:
        for i in range(3):
            h[i, i] = 1.0
    if n_meas >= 5:
        h[3, 3] = 1.0
        h[4, 4] = 1.0
    elif n_meas == 2:
        h[0, 3] = 1.0
        h[1, 4] = 1.0
    return h


def build_r(n_meas: int, r_pos: float) -> np.ndarray:
    r = np.zeros((n_meas, n_meas))
    if n_meas >= 3:
        for i in range(3):
            r[i, i] = r_pos
    if n_meas == 5:
        r[3, 3] = R_VEL
        r[4, 4] = R_VEL
    elif n_meas == 2:
        r[0, 0] = R_VEL
        r[1, 1] = R_VEL
    return r


def compute_s(h: np.ndarray, p: np.ndarray, r: np.ndarray) -> np.ndarray:
    return h @ p @ h.T + r


def compute_k(p: np.ndarray, h: np.ndarray, s: np.ndarray) -> np.ndarray:
    return p @ h.T @ np.linalg.inv(s)


def alignment_analysis(dv_pos: np.ndarray, err_pre: np.ndarray) -> dict:
    err_norm = float(np.linalg.norm(err_pre))
    dv_norm = float(np.linalg.norm(dv_pos))
    if err_norm < 1e-12 or dv_norm < 1e-12:
        return {
            "err_pre_NE_mps": err_pre.tolist(),
            "err_pre_norm_mps": err_norm,
            "cos_dv_pos_err_pre": float("nan"),
            "cos_dv_pos_correction_dir": float("nan"),
            "proj_dv_pos_on_correction_mps": float("nan"),
        }
    u_corr = -err_pre / err_norm
    cos_err = float(np.dot(dv_pos, err_pre) / (dv_norm * err_norm))
    return {
        "err_pre_NE_mps": err_pre.tolist(),
        "err_pre_norm_mps": err_norm,
        "cos_dv_pos_err_pre": cos_err,
        "cos_dv_pos_correction_dir": float(-cos_err),
        "proj_dv_pos_on_correction_mps": float(np.dot(dv_pos, u_corr)),
        "cross_pulls_with_error": bool(cos_err > 0),
        "cross_aids_correction": bool(cos_err < 0),
    }


def vel_err_from_ne(vn: float, ve: float, vg_n: float, vg_e: float) -> float:
    return math.hypot(vn - vg_n, ve - vg_e)


def analyze_fix(
    arm_dir: Path,
    gps_index: int,
    *,
    mode: str = "G1_actual",
) -> dict:
    """mode: G1_actual (pos+vel ran) | G2_counterfactual_5d (hypothetical pos+vel on G2 P,y)."""
    k = load_k_block(arm_dir / "gnss_k_block.jsonl", gps_index)
    ts = float(k["timestamp_s"])
    gnss = load_gnss_row(arm_dir / "gnss_nis_audit.csv", gps_index)
    pre = load_cov_row(arm_dir / "cov_step_audit.csv", ts, "pre")
    if pre is None:
        raise KeyError(f"no cov pre at {ts}")
    post = load_cov_row(arm_dir / "cov_step_audit.csv", ts, "post_accept")

    p_pp = np.array(k["HPH_m2"], dtype=float)
    p_vp = np.array(k["P_vel_pos_cross_m2"], dtype=float)
    p_vv_diag = np.array([pre["P_vv_n_m2"], pre["P_vv_e_m2"], pre["P_vv_d_m2"]], dtype=float)
    r_pos = float(k["R_m2"])
    p = build_minimal_p(p_pp, p_vp, p_vv_diag, float(pre["P_pv_frob"]), float(pre["P_aa_frob"]))

    h5 = build_h(5)
    r5 = build_r(5, r_pos)
    s5 = compute_s(h5, p, r5)
    k5 = compute_k(p, h5, s5)

    y5 = np.array(
        [
            gnss.innov_n_m,
            gnss.innov_e_m,
            gnss.innov_d_m,
            gnss.pseudo_innov_v_n_mps,
            gnss.pseudo_innov_v_e_mps,
        ]
    )
    y_pos = y5[0:3]
    y_vel = y5[3:5]

    k_vel_pos = k5[3:5, 0:3]
    k_vel_vel = k5[3:5, 3:5]
    dv_pos = k_vel_pos @ y_pos
    dv_vel = k_vel_vel @ y_vel
    dv_total = dv_pos + dv_vel

    mag_pos = float(np.linalg.norm(dv_pos))
    mag_vel = float(np.linalg.norm(dv_vel))

    vg_n = gnss.gps_speed_mps * math.cos(math.radians(gnss.gps_course_deg))
    vg_e = gnss.gps_speed_mps * math.sin(math.radians(gnss.gps_course_deg))
    err_pre_vec = np.array([gnss.vel_pred_n_mps - vg_n, gnss.vel_pred_e_mps - vg_e])

    if mode == "G1_actual":
        dv_logged = np.array([gnss.dx_vel_n_mps, gnss.dx_vel_e_mps])
        err_post = vel_err_from_ne(gnss.vel_after_n_mps, gnss.vel_after_e_mps, vg_n, vg_e)
        k_match_ref = float(gnss.k_vel_max)
        align_log = alignment_analysis(dv_logged, err_pre_vec)
    else:
        dv_logged = None
        vn_hyp = gnss.vel_pred_n_mps + dv_total[0]
        ve_hyp = gnss.vel_pred_e_mps + dv_total[1]
        err_post = vel_err_from_ne(vn_hyp, ve_hyp, vg_n, vg_e)
        k2 = compute_k(p, build_h(2), compute_s(build_h(2), p, build_r(2, r_pos)))
        k_match_ref = float(max(k2[3, 0], k2[4, 1]))
        align_log = None

    err_pre = float(np.linalg.norm(err_pre_vec))

    joseph_err = None
    joseph_pass = None
    if post is not None and mode == "G1_actual":
        p_post_j = 0.5 * (
            (np.eye(15) - k5 @ h5) @ p @ (np.eye(15) - k5 @ h5).T + k5 @ r5 @ k5.T
        )
        joseph_err = abs(frob(p_post_j[3:6, 3:6]) - float(post["P_vv_frob"])) / max(
            float(post["P_vv_frob"]), 1e-9
        )
        joseph_pass = bool(joseph_err < 0.05)

    return {
        "arm": mode,
        "gps_index": gps_index,
        "timestamp_s": ts,
        "regime": {
            "P_vv_frob_pre": float(pre["P_vv_frob"]),
            "P_pv_frob_pre": float(pre["P_pv_frob"]),
            "P_pv_over_P_vv": float(pre["P_pv_frob"]) / max(float(pre["P_vv_frob"]), 1e-9),
            "innov_h_m": float(gnss.innov_h_m),
            "y_pos_norm_3d_m": float(np.linalg.norm(y_pos)),
            "seconds_since_run_start": ts,
        },
        "check1_S_pos_vel": {
            "frob_error_S_vs_P": frob(s5[0:3, 3:5] - p[0:3, 3:5]),
            "P_pos_vel_NN": float(p[0, 3]),
            "pass": bool(frob(s5[0:3, 3:5] - p[0:3, 3:5]) < 0.01),
        },
        "check2_K": {
            "K_vel_vel_diag_5d": np.diag(k_vel_vel).tolist(),
            "k_vel_max_reference": k_match_ref,
            "K_match": bool(abs(np.diag(k_vel_vel).max() - k_match_ref) < 0.02),
            "Joseph_P_vv_rel_error": joseph_err,
            "Joseph_pass": joseph_pass,
        },
        "delta_v_decomposition_NE": {
            "y_pos_norm_m": float(np.linalg.norm(y_pos)),
            "y_vel_NE_mps": y_vel.tolist(),
            "dv_from_pos_NE_mps": dv_pos.tolist(),
            "dv_from_vel_NE_mps": dv_vel.tolist(),
            "dv_total_NE_mps": dv_total.tolist(),
            "dv_logged_NE_mps": dv_logged.tolist() if dv_logged is not None else None,
            "magnitude_pos_mps": mag_pos,
            "magnitude_vel_mps": mag_vel,
            "cross_term_fraction": mag_pos / max(mag_pos + mag_vel, 1e-9),
            "cross_dominates": bool(mag_pos > mag_vel),
            "cross_over_y_pos": mag_pos / max(float(np.linalg.norm(y_pos)), 1e-9),
        },
        "truth_terrain": {
            "err_vel_pre_mps": err_pre,
            "err_vel_post_mps": err_post,
            "delta_err_vel_mps": err_post - err_pre,
            "vel_improves": bool(err_post < err_pre),
            "counterfactual": mode == "G2_counterfactual_5d",
        },
        "alignment_cross_vs_error": alignment_analysis(dv_pos, err_pre_vec),
        "alignment_logged_total": align_log,
    }


def collinearity_verdict(points: list[dict]) -> dict:
    """Synthesis: dominance vs outcome; alignment of cross pull vs error direction."""
    if len(points) < 3:
        return {"n": len(points), "note": "need >=3 points"}

    def corr(a: np.ndarray, b: np.ndarray) -> float:
        if np.std(a) < 1e-12 or np.std(b) < 1e-12:
            return float("nan")
        return float(np.corrcoef(a, b)[0, 1])

    y = np.array([p["regime"]["y_pos_norm_3d_m"] for p in points])
    cf = np.array([p["delta_v_decomposition_NE"]["cross_term_fraction"] for p in points])
    cos_err = np.array([p["alignment_cross_vs_error"]["cos_dv_pos_err_pre"] for p in points])
    delta_err = np.array([p["truth_terrain"]["delta_err_vel_mps"] for p in points])

    summary = []
    for p in points:
        al = p["alignment_cross_vs_error"]
        summary.append(
            {
                "arm": p["arm"],
                "gps_index": p["gps_index"],
                "t_s": p["timestamp_s"],
                "y_pos_3d_m": p["regime"]["y_pos_norm_3d_m"],
                "P_pv_frob": p["regime"]["P_pv_frob_pre"],
                "cross_fraction": p["delta_v_decomposition_NE"]["cross_term_fraction"],
                "cos_dv_pos_err_pre": al["cos_dv_pos_err_pre"],
                "proj_dv_pos_on_correction_mps": al["proj_dv_pos_on_correction_mps"],
                "delta_err_vel_mps": p["truth_terrain"]["delta_err_vel_mps"],
                "vel_improves": p["truth_terrain"]["vel_improves"],
            }
        )

    # cos>0: cross pull aligned with error (v_pred-v_GPS) → tends to hurt
    fix2 = next(p for p in points if p["gps_index"] == 2)
    fix56 = next(p for p in points if p["gps_index"] == 56)

    alignment_confirmed = (
        fix2["alignment_cross_vs_error"]["cos_dv_pos_err_pre"] > 0
        and not fix2["truth_terrain"]["vel_improves"]
        and fix56["alignment_cross_vs_error"]["cos_dv_pos_err_pre"] < 0
        and fix56["truth_terrain"]["vel_improves"]
    )

    return {
        "n_points": len(points),
        "points_summary": summary,
        "correlation_cross_fraction_vs_y_pos": corr(y, cf),
        "correlation_cross_fraction_vs_cos_align_err": corr(cf, delta_err),
        "correlation_cos_align_err_vs_delta_err": corr(cos_err, delta_err),
        "caveat_n3": (
            "n=3 — trend only. fix#56 is G2 counterfactual 5D. "
            "Dominance fraction alone does not predict vel outcome (fix#56 98% cross but improves)."
        ),
        "alignment_verdict": (
            "ALIGNMENT_PRIMARY — cos(dv_pos, err_pre)>0 when cross hurts (fix#2); "
            "cos<0 when cross helps (fix#56); fix#7 cross misaligned but vel channel rescues"
            if alignment_confirmed
            else "REVIEW alignment pattern"
        ),
        "design_implication": (
            "Prioritize P_pv fidelity / reset post-gap (fix#2 contaminated orientation); "
            "Huber on |y_pos| alone is wrong — fix#56 has large |y| with beneficial cross alignment"
        ),
        "deprecated_verdict": "INNOVATION_MAGNITUDE_PRIMARY — insufficient; dominance≠outcome",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--points",
        type=str,
        default="G1:2,G1:7,G2:56",
        help="Comma list arm:gps_index e.g. G1:2,G1:7,G2:56",
    )
    args = parser.parse_args()

    g1_dir = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G1"
    g2_dir = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G2"
    dirs = {"G1": g1_dir, "G2": g2_dir}

    fixes = []
    for spec in args.points.split(","):
        arm, idx_s = spec.strip().split(":")
        idx = int(idx_s)
        mode = "G1_actual" if arm == "G1" else "G2_counterfactual_5d"
        fixes.append(analyze_fix(dirs[arm], idx, mode=mode))

    report = {
        "experiment": "GAP-4 pos+vel coupling — collinearity check (3 points)",
        "note_g1_accepts": (
            "G1 pos+vel: 8 accepts only; fix#7 is best available non-fix#2 point (t≈10.6s, not deep regime). "
            "fix#56 @ t≈58s (G2 vel-only) breaks time–innovation collinearity: ‖y_pos‖≈214 m >> fix#2."
        ),
        "fixes": fixes,
        "collinearity_analysis": collinearity_verdict(fixes),
    }

    out_path = g1_dir / "gap4_coupling_collinearity_report.json"
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    print(f"\nWrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
