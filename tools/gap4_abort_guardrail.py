#!/usr/bin/env python3
"""GAP-4 — Guardarraíl de abort §3.3 (condicional verdad-terreno)."""

from __future__ import annotations

import math
from typing import Literal

import numpy as np
import pandas as pd

ObsMode = Literal["pos", "vel_only", "pos_vel"]

ABORT_K_GAIN = 0.5
ABORT_DP_OVER_P = 0.5
ABORT_DP_ABS = 30.0
VEL_ERR_EPS_MPS = 0.01
POS_ERR_EPS_M = 0.5


def _vel_err_from_cols(row: pd.Series, vn_col: str, ve_col: str) -> float:
    vg_n = float(row["gps_speed_mps"]) * math.cos(math.radians(float(row["gps_course_deg"])))
    vg_e = float(row["gps_speed_mps"]) * math.sin(math.radians(float(row["gps_course_deg"])))
    vn = float(row[vn_col])
    ve = float(row[ve_col])
    return math.hypot(vn - vg_n, ve - vg_e)


def _vel_err_vs_gps(row: pd.Series, prefix: str) -> float:
    if prefix == "pred":
        return _vel_err_from_cols(row, "vel_pred_n_mps", "vel_pred_e_mps")
    return _vel_err_from_cols(row, "vel_after_n_mps", "vel_after_e_mps")


def _pos_err_vs_gps(row: pd.Series, prefix: str) -> float:
    if prefix == "pred":
        pn, pe = float(row["hx_n_m"]), float(row["hx_e_m"])
    else:
        pn = float(row["hx_n_m"]) + float(row["dx_pos_n_m"])
        pe = float(row["hx_e_m"]) + float(row["dx_pos_e_m"])
    zn, ze = float(row["z_n_m"]), float(row["z_e_m"])
    return math.hypot(zn - pn, ze - pe)


def enrich_accept_truth_errors(acc: pd.DataFrame) -> pd.DataFrame:
    out = acc.copy()
    out["err_vel_pre_mps"] = out.apply(lambda r: _vel_err_vs_gps(r, "pred"), axis=1)
    out["err_vel_post_mps"] = out.apply(lambda r: _vel_err_vs_gps(r, "post"), axis=1)
    out["err_pos_pre_m"] = out["innov_h_m"].astype(float)
    out["err_pos_post_m"] = out.apply(lambda r: _pos_err_vs_gps(r, "post"), axis=1)
    out["delta_err_vel_mps"] = out["err_vel_post_mps"] - out["err_vel_pre_mps"]
    out["delta_err_pos_m"] = out["err_pos_post_m"] - out["err_pos_pre_m"]
    return out


def pathological_k_vel_rows(acc: pd.DataFrame, obs_mode: ObsMode) -> pd.DataFrame:
    """k > 0.5 AND error worsens on observed velocity component."""
    if obs_mode == "pos":
        return acc.iloc[0:0]
    df = enrich_accept_truth_errors(acc)
    return df[
        (df["k_vel_max"] > ABORT_K_GAIN)
        & (df["delta_err_vel_mps"] > VEL_ERR_EPS_MPS)
    ]


def pathological_k_pos_rows(acc: pd.DataFrame, obs_mode: ObsMode) -> pd.DataFrame:
    if obs_mode == "vel_only":
        return acc.iloc[0:0]
    df = enrich_accept_truth_errors(acc)
    return df[
        (df["k_pos_max"] > ABORT_K_GAIN)
        & (df["delta_err_pos_m"] > POS_ERR_EPS_M)
    ]


def pathological_dP_rows(
    dP_audit: list[dict],
    acc_by_ts: dict[float, pd.Series],
    obs_mode: ObsMode,
) -> list[dict]:
    hits = []
    for row in dP_audit:
        if row.get("phase") != "post_accept":
            continue
        if row.get("dP_over_P_pre", 0.0) <= ABORT_DP_OVER_P and abs(row.get("delta_P_vv", 0.0)) <= ABORT_DP_ABS:
            continue
        ts = float(row["timestamp_s"])
        acc_row = acc_by_ts.get(ts)
        if acc_row is None:
            continue
        df = enrich_accept_truth_errors(pd.DataFrame([acc_row]))
        r = df.iloc[0]
        vel_worse = float(r["delta_err_vel_mps"]) > VEL_ERR_EPS_MPS
        pos_worse = float(r["delta_err_pos_m"]) > POS_ERR_EPS_M
        if obs_mode == "vel_only" and vel_worse:
            hits.append({**row, "trigger": "dP_vel_worse"})
        elif obs_mode == "pos" and pos_worse:
            hits.append({**row, "trigger": "dP_pos_worse"})
        elif obs_mode == "pos_vel" and (vel_worse or pos_worse):
            hits.append({**row, "trigger": "dP_obs_worse"})
    return hits


def evaluate_abort(
    gnss: pd.DataFrame,
    dP_audit: list[dict],
    obs_mode: ObsMode,
) -> dict:
    acc = gnss[gnss["accepted"] == 1].copy()
    acc_by_ts = {float(r["timestamp_s"]): r for _, r in acc.iterrows()}

    bad_k_vel = pathological_k_vel_rows(acc, obs_mode)
    bad_k_pos = pathological_k_pos_rows(acc, obs_mode)
    bad_dP = pathological_dP_rows(dP_audit, acc_by_ts, obs_mode)

    flags = {
        "k_vel_pathological": len(bad_k_vel) > 0,
        "k_pos_pathological": len(bad_k_pos) > 0,
        "dP_pathological": len(bad_dP) > 0,
        "legacy_k_vel_max_gt_0.5": bool(acc["k_vel_max"].max() > ABORT_K_GAIN) if len(acc) else False,
        "legacy_dP_over_P_gt_0.5": any(
            r.get("dP_over_P_pre", 0) > ABORT_DP_OVER_P for r in dP_audit if r.get("phase") == "post_accept"
        ),
    }
    abort = flags["k_vel_pathological"] or flags["k_pos_pathological"] or flags["dP_pathological"]

    return {
        "obs_mode": obs_mode,
        "abort": abort,
        "abort_flags": flags,
        "pathological_k_vel_events": bad_k_vel[
            ["gps_index", "timestamp_s", "k_vel_max", "err_vel_pre_mps", "err_vel_post_mps", "delta_err_vel_mps"]
        ].to_dict(orient="records")
        if len(bad_k_vel)
        else [],
        "pathological_k_pos_events": bad_k_pos[
            ["gps_index", "timestamp_s", "k_pos_max", "err_pos_pre_m", "err_pos_post_m", "delta_err_pos_m"]
        ].to_dict(orient="records")
        if len(bad_k_pos)
        else [],
        "pathological_dP_events": bad_dP,
    }


def event_anatomy(row: pd.Series, pre_cov: pd.Series | None, r_vel_m2: float = 2.25) -> dict:
    vg_n = float(row["gps_speed_mps"]) * math.cos(math.radians(float(row["gps_course_deg"])))
    vg_e = float(row["gps_speed_mps"]) * math.sin(math.radians(float(row["gps_course_deg"])))
    err_vel_pre = math.hypot(float(row["vel_pred_n_mps"]) - vg_n, float(row["vel_pred_e_mps"]) - vg_e)
    err_vel_post = math.hypot(float(row["vel_after_n_mps"]) - vg_n, float(row["vel_after_e_mps"]) - vg_e)

    out = {
        "gps_index": int(row["gps_index"]),
        "timestamp_s": float(row["timestamp_s"]),
        "k_vel_max": float(row["k_vel_max"]),
        "k_pos_max": float(row.get("k_pos_max", 0.0)),
        "err_vel_pre_mps": err_vel_pre,
        "err_vel_post_mps": err_vel_post,
        "delta_err_vel_mps": err_vel_post - err_vel_pre,
        "err_pos_pre_m": float(row["innov_h_m"]),
    }
    pn = float(row["hx_n_m"]) + float(row["dx_pos_n_m"])
    pe = float(row["hx_e_m"]) + float(row["dx_pos_e_m"])
    out["err_pos_post_m"] = math.hypot(float(row["z_n_m"]) - pn, float(row["z_e_m"]) - pe)
    out["delta_err_pos_m"] = out["err_pos_post_m"] - out["err_pos_pre_m"]

    if pre_cov is not None:
        pnn = float(pre_cov["P_vv_n_m2"])
        pee = float(pre_cov["P_vv_e_m2"])
        kn = pnn / (pnn + r_vel_m2)
        ke = pee / (pee + r_vel_m2)
        k_pred = max(kn, ke)
        out.update(
            {
                "P_vv_frob_pre": float(pre_cov["P_vv_frob"]),
                "P_vv_n_m2_pre": pnn,
                "P_vv_e_m2_pre": pee,
                "K_nn_diagonal": kn,
                "K_max_diagonal": k_pred,
                "K_matches_k_vel": abs(k_pred - float(row["k_vel_max"])) < 0.02,
            }
        )

    vel_improves = out["delta_err_vel_mps"] < -VEL_ERR_EPS_MPS
    k_match = out.get("K_matches_k_vel", False)
    if k_match and vel_improves:
        out["verdict"] = "LEGITIMATE_HIGH_GAIN"
    elif out["delta_err_vel_mps"] > VEL_ERR_EPS_MPS:
        out["verdict"] = "BUG_LIKE_DISCONTINUITY"
    elif k_match:
        out["verdict"] = "LEGITIMATE_HIGH_GAIN_MARGINAL"
    else:
        out["verdict"] = "AMBIGUOUS"
    return out
