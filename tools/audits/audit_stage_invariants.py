#!/usr/bin/env python3
"""Find first stage that breaks a physical invariant previously held.

Input: tick_stage_audit.csv (with inv_* columns).
Question: at which (t, edge) does |course−yaw| or |dp/dt−v| first fail
after having been OK earlier in the same tick / previous tick?
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

EDGE_ORDER = ["A_post_predict", "B_post_nhc_zupt", "C_post_gnss"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--t-max", type=float, default=6.0)
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    need = {"inv_course_yaw_ok", "inv_dpdt_v_ok", "edge", "timestamp_s"}
    if not need.issubset(df.columns):
        print("ERROR: CSV missing invariant columns — rebuild replay with new schema")
        print("columns:", list(df.columns))
        return 1

    w = df[df["timestamp_s"] <= args.t_max].copy()
    print(f"rows t<={args.t_max}: {len(w)}")
    print(
        "thresholds: |course-yaw|<=45 deg if speed>=0.25; "
        "|dp/dt-v|<=1; speed>=0.25*gps if gps_speed>=3"
    )
    if "speed_mps" in w.columns:
        print(
            f"speed_mps: max={w['speed_mps'].max():.4g}  "
            f"p95={w['speed_mps'].quantile(0.95):.4g}"
        )

    props = [
        ("course_yaw", "inv_course_yaw_ok", "abs_course_yaw_deg"),
        ("dpdt_v", "inv_dpdt_v_ok", "dpdt_minus_v_mps"),
    ]
    if "inv_speed_vs_gps_ok" in w.columns:
        props.append(("speed_vs_gps", "inv_speed_vs_gps_ok", "speed_mps"))

    for prop, col, mag in props:
        if col not in w.columns:
            continue
        valid = w[w[col] >= 0]
        if valid.empty:
            print(f"\n{prop}: no valid samples (speed floor / no prev / no gps)")
            continue
        print(f"\n{prop}: valid={len(valid)}  fail={int((valid[col] == 0).sum())}")
        for edge in EDGE_ORDER:
            e = valid[valid["edge"] == edge]
            if e.empty:
                continue
            fail = e[e[col] == 0]
            print(
                f"  {edge}: n={len(e)} fail={len(fail)} "
                f"fail_frac={len(fail)/len(e):.3f}"
            )

    for prop, col, mag in props:
        if col not in w.columns:
            continue
        print(f"\n=== First invariant break ({prop}) ===")
        find_first_break(w, col, mag)

    for prop, col, _mag in props:
        if col not in w.columns:
            continue
        print(f"\n=== Same-tick A->B ({prop}) ===")
        count_ab_transitions(w, col)

    return 0


def find_first_break(w: pd.DataFrame, col: str, mag_col: str) -> None:
    prev_ok = None
    prev_edge = None
    prev_t = None
    for _, r in w.sort_values(["timestamp_s", "imu_seq"]).iterrows():
        v = int(r[col])
        if v < 0:
            continue
        ok = v == 1
        if prev_ok is True and not ok:
            print(
                f"  BREAK at t={r['timestamp_s']:.6f} edge={r['edge']} "
                f"{mag_col}={r[mag_col]:.4g}  "
                f"(was OK at t={prev_t:.6f} {prev_edge})"
            )
            return
        prev_ok = ok
        prev_edge = r["edge"]
        prev_t = float(r["timestamp_s"])
    print("  (no OK→FAIL transition in window)")


def count_ab_transitions(w: pd.DataFrame, col: str) -> None:
    a = w[w["edge"] == "A_post_predict"][["imu_seq", "timestamp_s", col]].rename(
        columns={col: "a"}
    )
    b = w[w["edge"] == "B_post_nhc_zupt"][["imu_seq", col]].rename(columns={col: "b"})
    m = a.merge(b, on="imu_seq")
    both = m[(m["a"] >= 0) & (m["b"] >= 0)]
    if both.empty:
        print("  (no paired A/B with valid prop)")
        return
    ok_ok = ((both["a"] == 1) & (both["b"] == 1)).sum()
    ok_fail = ((both["a"] == 1) & (both["b"] == 0)).sum()
    fail_ok = ((both["a"] == 0) & (both["b"] == 1)).sum()
    fail_fail = ((both["a"] == 0) & (both["b"] == 0)).sum()
    print(f"  pairs={len(both)}  A✔B✔={ok_ok}  A✔B✘={ok_fail}  A✘B✔={fail_ok}  A✘B✘={fail_fail}")
    if ok_fail:
        r = both[(both["a"] == 1) & (both["b"] == 0)].iloc[0]
        print(f"  first A✔→B✘ t={r['timestamp_s']:.6f} imu_seq={int(r['imu_seq'])}")


if __name__ == "__main__":
    raise SystemExit(main())
