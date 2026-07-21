#!/usr/bin/env python3
"""Summarize post-predict tick-stage audit (A→B NHC/ZUPT, B→C GNSS).

A = post-predict pre-NHC (from predict_audit)
B = after NHC/ZUPT
C = after GNSS update
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--t-max", type=float, default=6.0)
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    w = df[df["timestamp_s"] <= args.t_max]
    print(f"rows_t<={args.t_max}: {len(w)} / {len(df)}")

    for edge in ("B_post_nhc_zupt", "C_post_gnss"):
        e = w[w["edge"] == edge]
        if e.empty:
            print(f"\n{edge}: (none)")
            continue
        print(f"\n{edge} n={len(e)}")
        print(
            f"  dpos_m:   max={e['dpos_m'].max():.4g}  p95={e['dpos_m'].quantile(0.95):.4g}  "
            f"mean={e['dpos_m'].mean():.4g}"
        )
        print(
            f"  dvel_mps: max={e['dvel_mps'].max():.4g}  p95={e['dvel_mps'].quantile(0.95):.4g}  "
            f"mean={e['dvel_mps'].mean():.4g}"
        )
        print(
            f"  dyaw_deg: max={e['dyaw_deg'].abs().max():.4g}  "
            f"p95={e['dyaw_deg'].abs().quantile(0.95):.4g}"
        )
        if edge == "B_post_nhc_zupt":
            print(
                f"  nhc_applied_frac={e['nhc_applied'].mean():.3f}  "
                f"zupt_applied_frac={e['zupt_applied'].mean():.3f}"
            )
        if edge == "C_post_gnss":
            acc = e[e["gnss_accepted"] == 1]
            rej = e[e["gnss_accepted"] == 0]
            print(f"  accepted={len(acc)} rejected={len(rej)}")
            if len(acc):
                print(
                    f"  ACCEPT dpos max/mean={acc['dpos_m'].max():.4g}/"
                    f"{acc['dpos_m'].mean():.4g}  "
                    f"dvel max/mean={acc['dvel_mps'].max():.4g}/"
                    f"{acc['dvel_mps'].mean():.4g}"
                )
            if len(rej):
                print(
                    f"  REJECT dpos max/mean={rej['dpos_m'].max():.4g}/"
                    f"{rej['dpos_m'].mean():.4g}"
                )

        # First large move
        thr = 0.05 if edge.startswith("B") else 0.5
        big = e[e["dpos_m"] >= thr].sort_values("timestamp_s")
        if len(big):
            r = big.iloc[0]
            print(
                f"  first dpos>={thr} at t={r['timestamp_s']:.3f}  "
                f"dpos={r['dpos_m']:.4g} dvel={r['dvel_mps']:.4g} "
                f"dyaw={r['dyaw_deg']:.3g}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
