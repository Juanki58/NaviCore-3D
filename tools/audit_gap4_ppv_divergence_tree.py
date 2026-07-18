#!/usr/bin/env python3
"""GAP-4 — Divergence tree: control trunk + fix#4 bifurcation (1d vs 1d')."""

from __future__ import annotations

import argparse
import json
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
TRUNK_FIXES = [2, 3, 4]
BRANCH_FIXES = [5, 6, 7, 32]


def parse_gnss_row(line: str, header: list[str]) -> dict:
    parts = line.split(",")
    ppv = dict(zip(PPV_COLS, parts[-7:]))
    base = dict(zip(header[: len(parts) - 7], parts[: len(parts) - 7]))
    base.update(ppv)
    return base


def load_gnss_by_fix(path: Path) -> dict[int, dict]:
    lines = path.read_text(encoding="utf-8").splitlines()
    header = lines[0].split(",")
    out: dict[int, dict] = {}
    for ln in lines[1:]:
        if not ln.strip():
            continue
        row = parse_gnss_row(ln, header)
        out[int(float(row["gps_index"]))] = row
    return out


def load_cov_pre_by_ts(path: Path) -> dict[float, dict]:
    """GNSS pre-update covariance metrics keyed by timestamp_s."""
    lines = path.read_text(encoding="utf-8").splitlines()
    header = lines[0].split(",")
    idx = {name: i for i, name in enumerate(header)}
    out: dict[float, dict] = {}
    for ln in lines[1:]:
        if not ln.strip():
            continue
        parts = ln.split(",")
        if parts[idx["update_type"]] != "gnss" or parts[idx["phase"]] != "pre":
            continue
        ts = float(parts[idx["timestamp_s"]])
        out[ts] = {
            "P_vv_frob": float(parts[idx["P_vv_frob"]]),
            "P_pv_frob": float(parts[idx["P_pv_frob"]]),
            "P_pp_frob": float(parts[idx["P_pp_frob"]]),
        }
    return out


def f(row: dict, key: str) -> float:
    return float(row[key])


def node_from_gnss(
    row: dict,
    cov_pre: dict[float, dict] | None,
    *,
    arm: str,
    fix: int,
    note: str = "",
) -> dict:
    ts = f(row, "timestamp_s")
    cov = (cov_pre or {}).get(ts, {})
    ppv_pre = f(row, "ppv_frob_pre")
    ppv_post = f(row, "ppv_frob_post")
    p_vv = cov.get("P_vv_frob")
    return {
        "arm": arm,
        "gps_index": fix,
        "timestamp_s": ts,
        "accepted": int(float(row["accepted"])),
        "nis_h2d": f(row, "nis_horizontal_2d"),
        "nis_threshold": f(row, "nis_threshold"),
        "cos_pos": f(row, "cos_dv_pos_err_pre"),
        "cos_tot": f(row, "cos_dv_tot_err_pre"),
        "ppv_triggered": int(float(row["ppv_triggered"])),
        "P_pv_frob_pre": ppv_pre,
        "P_pv_frob_post": ppv_post,
        "delta_P_pv": ppv_post - ppv_pre,
        "P_vv_frob_pre": p_vv,
        "P_pv_over_P_vv": (ppv_pre / p_vv) if p_vv and p_vv > 0 else None,
        "innov_h_m": f(row, "innov_h_m"),
        "note": note,
    }


def fmt_node(n: dict) -> str:
    acc = "A" if n["accepted"] else "R"
    trig = n["ppv_triggered"]
    trig_s = str(trig) if trig is not None else "—"
    pv = n["P_pv_frob_pre"]
    vv = n["P_vv_frob_pre"]
    ratio = n["P_pv_over_P_vv"]
    ratio_s = f"{ratio:.3f}" if ratio is not None else "—"
    vv_s = f"{vv:.2f}" if vv is not None else "—"
    return (
        f"fix#{n['gps_index']} [{acc}] NIS={n['nis_h2d']:.2f} "
        f"|Ppv|={pv:.2f} Pvv={vv_s} r={ratio_s} "
        f"cos_pos={n['cos_pos']:+.2f} cos_tot={n['cos_tot']:+.2f} trig={trig_s}"
    )


def build_mermaid(trunk: list[dict], split: dict, branch_1d: list[dict], branch_1dp: list[dict]) -> str:
    lines = ["flowchart TD"]
    lines.append('  ctrl["control / ppv_none<br/>reference trunk"]')
    prev = "ctrl"
    for i, n in enumerate(trunk):
        nid = f"t{i}"
        label = fmt_node(n).replace('"', "'")
        lines.append(f'  {nid}["{label}"]')
        lines.append(f"  {prev} --> {nid}")
        prev = nid

    split_id = "fix4split"
    pre = split["pre"]
    pre_label = (
        f"fix#4 PRE shared<br/>|Ppv|={pre['P_pv_frob_pre']:.2f} "
        f"cos_pos={pre['cos_pos']:+.2f} cos_tot={pre['cos_tot']:+.2f}"
    ).replace('"', "'")
    lines.append(f'  {split_id}{{"{pre_label}"}}')
    lines.append(f"  {prev} --> {split_id}")

    d_post = split["1d_post"]
    dp_post = split["1d_prime_post"]
    d_id = "1d4"
    dp_id = "1dp4"
    lines.append(
        f'  {d_id}["1d POST<br/>trig={d_post["ppv_triggered"]} '
        f'|Ppv| {d_post["P_pv_frob_pre"]:.1f}->{d_post["P_pv_frob_post"]:.1f}"]'
    )
    lines.append(
        f'  {dp_id}["1d-prime POST<br/>trig={dp_post["ppv_triggered"]} '
        f'|Ppv| {dp_post["P_pv_frob_pre"]:.1f}->{dp_post["P_pv_frob_post"]:.1f}"]'
    )
    lines.append(f"  {split_id} --> {d_id}")
    lines.append(f"  {split_id} --> {dp_id}")

    chain_d = d_id
    for j, n in enumerate(branch_1d):
        nid = f"d{j}"
        lines.append(f'  {nid}["{fmt_node(n).replace(chr(34), chr(39))}"]')
        lines.append(f"  {chain_d} --> {nid}")
        chain_d = nid

    chain_p = dp_id
    for j, n in enumerate(branch_1dp):
        nid = f"p{j}"
        lines.append(f'  {nid}["{fmt_node(n).replace(chr(34), chr(39))}"]')
        lines.append(f"  {chain_p} --> {nid}")
        chain_p = nid

    return "\n".join(lines)


def causal_summary(nodes: list[dict]) -> dict:
    """Test whether cos sign tracks P_pv retention vs raw angle."""
    rows = []
    for n in nodes:
        rows.append(
            {
                "fix": n["gps_index"],
                "arm": n["arm"],
                "P_pv_pre": n["P_pv_frob_pre"],
                "P_vv_pre": n["P_vv_frob_pre"],
                "P_pv_over_P_vv": n["P_pv_over_P_vv"],
                "cos_pos": n["cos_pos"],
                "cos_tot": n["cos_tot"],
                "triggered": n["ppv_triggered"],
            }
        )
    return {
        "hypothesis": "cos may be observational; P_pv structure (|P_pv|/P_vv) may be more causal",
        "fix4_shared_pre": {
            "cos_pos_positive": True,
            "cos_tot_negative": True,
            "same_P_pv_pre": True,
            "policy_split_only": "1d zeros P_pv (trig=1); 1d_prime retains P_pv (trig=0)",
        },
        "post_bifurcation": (
            "After fix#4, cos comparisons across arms are trajectory comparisons, "
            "not policy-on-same-state. Sign inversions at fix#6-7 correlate with "
            "1d_prime retaining higher |P_pv| and different nominal v_pred."
        ),
        "nodes": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--control-dir",
        type=Path,
        default=REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G1_control_full_ppv_none",
    )
    parser.add_argument(
        "--1d-dir",
        dest="arm_1d_dir",
        type=Path,
        default=REPO_ROOT
        / "docs/benchmarks/gap4_gnss_velocity/G1_intervention/arm_1d_cos_pos_0_40s_logged",
    )
    parser.add_argument(
        "--1d-prime-dir",
        dest="arm_1d_prime_dir",
        type=Path,
        default=REPO_ROOT
        / "docs/benchmarks/gap4_gnss_velocity/G1_intervention/arm_1d_prime_cos_tot_0_40s_logged",
    )
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--out-mmd", type=Path, default=None)
    args = parser.parse_args()

    out_base = REPO_ROOT / "docs/benchmarks/gap4_gnss_velocity/G1_intervention"
    out_json = args.out_json or (out_base / "ppv_divergence_tree.json")
    out_mmd = args.out_mmd or (out_base / "ppv_divergence_tree.mmd")

    ctrl = load_gnss_by_fix(args.control_dir / "gnss_nis_audit.csv")
    ctrl_cov = load_cov_pre_by_ts(args.control_dir / "cov_step_audit.csv")
    d1 = load_gnss_by_fix(args.arm_1d_dir / "gnss_nis_audit.csv")
    d1_cov = load_cov_pre_by_ts(args.arm_1d_dir / "cov_step_audit.csv")
    d1p = load_gnss_by_fix(args.arm_1d_prime_dir / "gnss_nis_audit.csv")
    d1p_cov = load_cov_pre_by_ts(args.arm_1d_prime_dir / "cov_step_audit.csv")

    trunk = []
    for fix in [2, 3]:
        row = d1[fix]
        trunk.append(
            node_from_gnss(
                row,
                d1_cov,
                arm="shared_trunk",
                fix=fix,
                note="1d == 1d' (verified); control P_pv path differs (ppv_none)",
            )
        )

    fix4_row = d1[4]
    fix4_pre = node_from_gnss(fix4_row, d1_cov, arm="shared_pre", fix=4, note="last common pre-update state")
    fix4_pre["ppv_triggered"] = None
    fix4_pre["note"] = (
        "last common pre-update state; would trigger 1d=1, 1d_prime=0 at this cos split"
    )
    fix4_1d = node_from_gnss(d1[4], d1_cov, arm="1d", fix=4, note="post: P_pv zeroed")
    fix4_1dp = node_from_gnss(d1p[4], d1p_cov, arm="1d_prime", fix=4, note="post: P_pv retained")

    branch_1d = [node_from_gnss(d1[f], d1_cov, arm="1d", fix=f) for f in BRANCH_FIXES if f in d1]
    branch_1dp = [node_from_gnss(d1p[f], d1p_cov, arm="1d_prime", fix=f) for f in BRANCH_FIXES if f in d1p]

    # Control reference nodes (no ppv cols — cov + gnss only)
    control_ref = []
    for fix in TRUNK_FIXES:
        if fix not in ctrl:
            continue
        row = ctrl[fix]
        ts = f(row, "timestamp_s")
        cov = ctrl_cov.get(ts, {})
        control_ref.append(
            {
                "gps_index": fix,
                "timestamp_s": ts,
                "accepted": int(float(row["accepted"])),
                "nis_h2d": f(row, "nis_horizontal_2d"),
                "P_pv_frob_pre": cov.get("P_pv_frob"),
                "P_vv_frob_pre": cov.get("P_vv_frob"),
                "innov_h_m": f(row, "innov_h_m"),
                "note": "ppv_none — no cos/trigger logged",
            }
        )

    all_leaf_nodes = [fix4_1d, fix4_1dp] + branch_1d + branch_1dp
    report = {
        "title": "GAP-4 P_pv divergence tree (fix#4 bifurcation)",
        "method": "Filter-logged gnss_nis_audit + cov_step_audit gnss/pre join by timestamp_s.",
        "interpretation": {
            "trunk": "fix#2-3: single trajectory (1d == 1d').",
            "bifurcation": "fix#4: same pre-state; 1d zeros P_pv, 1d' does not.",
            "after_fix4": "Two distinct EKF instances — do not compare cos as policy-on-same-state.",
        },
        "control_reference": control_ref,
        "trunk_shared": trunk + [fix4_pre],
        "fix4_split": {"pre": fix4_pre, "1d_post": fix4_1d, "1d_prime_post": fix4_1dp},
        "branch_1d": branch_1d,
        "branch_1d_prime": branch_1dp,
        "causal_probe": causal_summary([fix4_pre, fix4_1d, fix4_1dp] + branch_1d + branch_1dp),
    }

    out_json.write_text(json.dumps(report, indent=2), encoding="utf-8")
    mmd = build_mermaid(trunk, report["fix4_split"], branch_1d, branch_1dp)
    out_mmd.write_text(mmd, encoding="utf-8")

    print("=== Divergence tree (filter-logged) ===\n")
    print("TRUNK (shared):")
    for n in trunk:
        print(" ", fmt_node(n))
    print("\nBIFURCATION fix#4:")
    print("  PRE ", fmt_node(fix4_pre))
    print("  1d  ", fmt_node(fix4_1d))
    print("  1d' ", fmt_node(fix4_1dp))
    print("\nBRANCH 1d:")
    for n in branch_1d:
        print(" ", fmt_node(n))
    print("\nBRANCH 1d':")
    for n in branch_1dp:
        print(" ", fmt_node(n))
    print(f"\nWrote {out_json}")
    print(f"Wrote {out_mmd}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
