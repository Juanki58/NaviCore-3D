#!/usr/bin/env python3
"""Summarize kinematic-identity audit CSV (I1/I2/I3 + pure-predict intervals).

Source: real_run_replay --kinematic-identity-audit-csv <path>
Identities are measured inside predict() BEFORE NHC/ZUPT.

Offline FD on replay_output.csv is NOT sufficient: NHC runs after pos+=v·dt
and can move pos/vel before the CSV row is written.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--t-max", type=float, default=6.0, help="Focus window end [s]")
    ap.add_argument(
        "--kin-tol",
        type=float,
        default=1e-5,
        help="Max |residual| [m] for I1 pass (within-predict)",
    )
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    if df.empty:
        print("EMPTY")
        return 1

    w = df[df["timestamp_s"] <= args.t_max].copy()
    print(f"rows_total={len(df)}  rows_t<={args.t_max}={len(w)}")
    print(
        f"nhc_applied_frac={w['nhc_applied'].mean():.3f}  "
        f"zupt_applied_frac={w['zupt_applied'].mean():.3f}  "
        f"pure_predict_frac={w['pure_predict_interval'].mean():.3f}"
    )
    print(
        f"I1 kin_pos_residual_m: max={w['kin_pos_residual_m'].max():.3e}  "
        f"p99={w['kin_pos_residual_m'].quantile(0.99):.3e}"
    )
    print(
        f"I2 body_ned_roundtrip_err_mps: max={w['body_ned_roundtrip_err_mps'].max():.3e}"
    )
    print(f"I3 euler_dcm_frob: max={w['euler_dcm_frob'].max():.3e}")
    print(
        f"pos_jump_since_prev_predict_m: "
        f"max={w['pos_jump_since_prev_predict_m'].max():.3e}  "
        f"p95={w['pos_jump_since_prev_predict_m'].quantile(0.95):.3e}"
    )

    pure = w[w["pure_predict_interval"] == 1]
    print(f"\npure_predict_intervals={len(pure)}")
    if len(pure) == 0:
        print(
            "WARNING: no pure IMU intervals in window. "
            "Re-run with --nhc-policy disabled --constraint-policy disabled "
            "(and short --replay-end-s) before concluding dp/dt≠v."
        )
    else:
        print(
            f"cross_tick_kin_residual_m: max={pure['cross_tick_kin_residual_m'].max():.3e}  "
            f"mean={pure['cross_tick_kin_residual_m'].mean():.3e}"
        )
        bad = pure[pure["cross_tick_kin_residual_m"] > args.kin_tol]
        print(f"pure failures (>{args.kin_tol} m): {len(bad)}")
        if len(bad):
            r = bad.iloc[0]
            print(
                f"  first_fail t={r['timestamp_s']:.6f}  "
                f"cross={r['cross_tick_kin_residual_m']:.3e}"
            )

    i1_bad = w[w["kin_pos_residual_m"] > args.kin_tol]
    i2_bad = w[w["body_ned_roundtrip_err_mps"] > args.kin_tol]
    i3_bad = w[w["euler_dcm_frob"] > 1e-5]
    print(
        f"\nwithin-predict failures in window: "
        f"I1={len(i1_bad)} I2={len(i2_bad)} I3={len(i3_bad)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
