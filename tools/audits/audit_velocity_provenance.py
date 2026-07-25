#!/usr/bin/env python3
"""Velocity before/after predict and after NHC — why is |v| stuck near zero?

Reads constraint_pipeline_audit.csv from real_run_replay.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd


def h(n: pd.Series, e: pd.Series) -> pd.Series:
    return np.hypot(n.to_numpy(), e.to_numpy())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "csv",
        type=Path,
        nargs="?",
        default=Path("docs/benchmarks/real_run_19082026_baseline/constraint_pipeline_audit.csv"),
    )
    ap.add_argument("--t-max", type=float, default=6.0)
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    w = df[df["timestamp_s"] <= args.t_max].copy()
    print(f"file={args.csv}")
    print(f"rows t<={args.t_max}: {len(w)} / {len(df)}")

    w["vel_before_h"] = h(w["vel_before_n"], w["vel_before_e"])
    w["vel_after_pred_h"] = h(w["vel_after_pred_n"], w["vel_after_pred_e"])
    w["vel_after_nhc_h"] = h(w["vel_after_nhc_n"], w["vel_after_nhc_e"])
    w["dv_pred_h"] = h(w["dv_pred_n"], w["dv_pred_e"])
    w["dv_nhc_h"] = h(w["dv_nhc_n"], w["dv_nhc_e"])

    r0 = w.iloc[0]
    print("\n=== First IMU tick ===")
    print(
        f"t={r0['timestamp_s']:.6f}  vel_before_h={r0['vel_before_h']:.6f}  "
        f"after_pred={r0['vel_after_pred_h']:.6f}  after_nhc={r0['vel_after_nhc_h']:.6f}"
    )
    print(
        f"  |dv_pred|_h={r0['dv_pred_h']:.6f}  |dv_nhc|_h={r0['dv_nhc_h']:.6f}  "
        f"gps_speed={r0['gps_speed_mps']:.3f}  nhc_applied={int(r0['nhc_applied'])}"
    )
    print(
        "  NOTE: real_run seeds via seed_from_ned_pos → vel forced to 0 at init "
        "(not seed_from_gnss_sample)."
    )

    print(f"\n=== t<={args.t_max} summary ===")
    for col in ("vel_before_h", "vel_after_pred_h", "vel_after_nhc_h", "gps_speed_mps"):
        print(
            f"  {col}: max={w[col].max():.4g}  mean={w[col].mean():.4g}  "
            f"p95={w[col].quantile(0.95):.4g}"
        )
    print(
        f"  |dv_pred|_h mean={w['dv_pred_h'].mean():.4g}  "
        f"|dv_nhc|_h mean={w['dv_nhc_h'].mean():.4g}"
    )
    both = w[w["nhc_applied"] == 1]
    if len(both):
        frac = (both["dv_nhc_h"] > both["dv_pred_h"]).mean()
        print(f"  among nhc_applied: frac(|dv_nhc|>|dv_pred|)={frac:.3f}  n={len(both)}")

    print("\n=== First time vel_after_nhc_h exceeds threshold (full log) ===")
    full = df.copy()
    full["vel_after_nhc_h"] = h(full["vel_after_nhc_n"], full["vel_after_nhc_e"])
    for thr in (0.5, 1.0, 2.0, 5.0):
        hit = full[full["vel_after_nhc_h"] >= thr]
        if hit.empty:
            print(f"  >={thr} m/s: never")
        else:
            r = hit.iloc[0]
            print(
                f"  >={thr} m/s: t={r['timestamp_s']:.3f}  "
                f"v={r['vel_after_nhc_h']:.4g}  gps={r['gps_speed_mps']:.3f}"
            )

    # Net effect of NHC on speed each tick
    w["speed_change_nhc"] = w["vel_after_nhc_h"] - w["vel_after_pred_h"]
    print(
        f"\n=== NHC effect on |v|_h (t<={args.t_max}) ===\n"
        f"  mean(after_nhc - after_pred)={w['speed_change_nhc'].mean():.4g}  "
        f"frac_reduces={(w['speed_change_nhc'] < 0).mean():.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
