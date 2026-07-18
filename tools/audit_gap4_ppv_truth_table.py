#!/usr/bin/env python3
"""GAP-4 — Truth table 1d vs 1d′ with filter-logged cos / P_pv gate (not offline recon)."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PPV_COLS = [
    "ppv_policy",
    "ppv_triggered",
    "ppv_effective_gap_s",
    "cos_dv_pos_err_pre",
    "cos_dv_tot_err_pre",
    "ppv_frob_pre",
    "ppv_frob_post",
]
ANCHORS = [2, 3, 4, 5, 6, 7, 32]


def parse_gnss_row(line: str, header: list[str]) -> dict:
    parts = line.split(",")
    if len(parts) < len(PPV_COLS) + 10:
        raise ValueError(f"short row ({len(parts)} fields)")
    ppv = dict(zip(PPV_COLS, parts[-7:]))
    base = dict(zip(header[: len(parts) - 7], parts[: len(parts) - 7]))
    base.update(ppv)
    return base


def load_arm(path: Path, accepted_only: bool = False) -> dict[int, dict]:
    lines = path.read_text(encoding="utf-8").splitlines()
    header = lines[0].split(",")
    out: dict[int, dict] = {}
    for ln in lines[1:]:
        if not ln.strip():
            continue
        row = parse_gnss_row(ln, header)
        if accepted_only and row["accepted"] != "1":
            continue
        out[int(float(row["gps_index"]))] = row
    return out


def fval(row: dict, key: str) -> float:
    return float(row[key])


def summarize(row: dict) -> dict:
    pre = fval(row, "ppv_frob_pre")
    post = fval(row, "ppv_frob_post")
    return {
        "accepted": int(float(row["accepted"])),
        "cos_pos": fval(row, "cos_dv_pos_err_pre"),
        "cos_tot": fval(row, "cos_dv_tot_err_pre"),
        "triggered": int(float(row["ppv_triggered"])),
        "ppv_pre": pre,
        "ppv_post": post,
        "delta_ppv": post - pre,
        "innov_h_m": fval(row, "innov_h_m"),
        "timestamp_s": fval(row, "timestamp_s"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--1d-dir",
        type=Path,
        default=REPO_ROOT
        / "docs/benchmarks/gap4_gnss_velocity/G1_intervention/arm_1d_cos_pos_0_40s_logged",
    )
    parser.add_argument(
        "--1d-prime-dir",
        type=Path,
        default=REPO_ROOT
        / "docs/benchmarks/gap4_gnss_velocity/G1_intervention/arm_1d_prime_cos_tot_0_40s_logged",
    )
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    d1 = load_arm(args.__dict__["1d_dir"] / "gnss_nis_audit.csv")
    d1p = load_arm(args.__dict__["1d_prime_dir"] / "gnss_nis_audit.csv")

    rows = []
    for fix in ANCHORS:
        r1 = d1.get(fix)
        r1p = d1p.get(fix)
        if r1 is None and r1p is None:
            continue
        s1 = summarize(r1) if r1 else None
        s1p = summarize(r1p) if r1p else None
        entry = {
            "gps_index": fix,
            "accepted_1d": s1["accepted"] if s1 else None,
            "accepted_1d_prime": s1p["accepted"] if s1p else None,
            "t_s_1d": s1["timestamp_s"] if s1 else None,
            "t_s_1d_prime": s1p["timestamp_s"] if s1p else None,
            "cos_pos_1d": s1["cos_pos"] if s1 else None,
            "cos_tot_1d": s1["cos_tot"] if s1 else None,
            "trigger_1d": s1["triggered"] if s1 else None,
            "delta_ppv_1d": s1["delta_ppv"] if s1 else None,
            "ppv_pre_1d": s1["ppv_pre"] if s1 else None,
            "ppv_post_1d": s1["ppv_post"] if s1 else None,
            "innov_h_1d": s1["innov_h_m"] if s1 else None,
            "cos_pos_1d_prime": s1p["cos_pos"] if s1p else None,
            "cos_tot_1d_prime": s1p["cos_tot"] if s1p else None,
            "trigger_1d_prime": s1p["triggered"] if s1p else None,
            "delta_ppv_1d_prime": s1p["delta_ppv"] if s1p else None,
            "ppv_pre_1d_prime": s1p["ppv_pre"] if s1p else None,
            "ppv_post_1d_prime": s1p["ppv_post"] if s1p else None,
            "innov_h_1d_prime": s1p["innov_h_m"] if s1p else None,
        }
        if s1 and s1p:
            entry["cos_pos_match"] = abs(s1["cos_pos"] - s1p["cos_pos"]) < 1e-3
            entry["trajectories_aligned_pre_fix"] = (
                abs(s1["cos_pos"] - s1p["cos_pos"]) < 1e-3
                and abs(s1["cos_tot"] - s1p["cos_tot"]) < 1e-3
            )
        rows.append(entry)

    out_path = args.out or (
        REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G1_intervention/ppv_truth_table_1d_vs_1dprime.json"
    )
    import json

    report = {
        "method": "Filter-logged ppv_triggered and cos_* from gnss_nis_audit.csv (tail parse). "
        "Offline K-block reconstruction MUST NOT be used for gate diagnosis.",
        "arms": {
            "1d": str(args.__dict__["1d_dir"]),
            "1d_prime": str(args.__dict__["1d_prime_dir"]),
        },
        "anchors": rows,
    }
    out_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    def fmt(v, spec: str, na: str = "   —") -> str:
        return na if v is None else format(v, spec)

    print(
        f"{'fix':>4} | {'acc':>3} | {'cos_pos':>8} {'cos_tot':>8} | "
        f"{'tr1d':>4} {'dPpv1d':>8} | {'tr1dp':>5} {'dPpv1dp':>8} | note"
    )
    print("-" * 88)
    for e in rows:
        fix = e["gps_index"]
        cp = e["cos_pos_1d"]
        ct = e["cos_tot_1d"]
        cp_p = e["cos_pos_1d_prime"]
        ct_p = e["cos_tot_1d_prime"]
        acc = f"{e.get('accepted_1d', '—')}/{e.get('accepted_1d_prime', '—')}"
        note = ""
        if cp is not None and cp_p is not None:
            if abs(cp - cp_p) > 0.05 or abs((ct or 0) - (ct_p or 0)) > 0.05:
                note = "trajectories diverged"
        if fix == 32:
            note = (note + "; " if note else "") + "1d REJECT, 1d-prime ACCEPT"
        if e.get("trigger_1d") != e.get("trigger_1d_prime"):
            note = (note + "; " if note else "") + "trigger split"
        print(
            f"{fix:4d} | {acc:>3} | "
            f"{fmt(cp, '+8.3f')} {fmt(ct, '+8.3f')} | "
            f"{fmt(e['trigger_1d'], '4d')} {fmt(e['delta_ppv_1d'], '8.2f')} | "
            f"{fmt(e['trigger_1d_prime'], '5d')} {fmt(e['delta_ppv_1d_prime'], '8.2f')} | "
            f"{note}"
        )
        if cp_p is not None and cp is not None and abs(cp - cp_p) > 0.001:
            print(
                f"     | 1d': cos_pos={cp_p:+.3f} cos_tot={ct_p:+.3f} "
                f"innov_h={e['innov_h_1d_prime']:.0f}m"
            )
        elif cp is None and cp_p is not None:
            print(
                f"     | 1d only REJECT: cos_pos={cp_p:+.3f} cos_tot={ct_p:+.3f} "
                f"(1d′ innov_h={e['innov_h_1d_prime']:.0f}m)"
            )

    print(f"\nWrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
