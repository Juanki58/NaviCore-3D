#!/usr/bin/env python3
"""K_bias_gz path decompose (H_vel vs H_att) in R1/R2/R3 — protocol §13.7.

Frozen S split: dx_bias_gz = dx_via_vel + dx_via_att (exact for NHC H).
Arms: ctrl vs latch λ=1 @ T2, SLALOM A jcorrect seed 71.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"
SEED = 71
T2 = 3.736646e-6
TMAX = 0.65

WINDOWS = {
    "R1": (1.34, 1.59, "[t0,t1)"),  # onset → break
    "R2": (1.59, 1.74, "[t0,t1)"),  # break → pre-explosion
    "R3": (1.74, 2.00, "[t0,t1]"),  # explosion → ω peak
}

NEED = [
    "dx_bias_gz",
    "dx_bias_gz_via_vel",
    "dx_bias_gz_via_att",
    "k_bias_gz_via_vel",
    "k_bias_gz_via_att",
    "innov_norm_mps",
]

sys.path.insert(0, str(REPO))
from run_all_benchmarks import run_benchmark  # noqa: E402


def mask_window(t: np.ndarray, t0: float, t1: float, half_open: str) -> np.ndarray:
    if half_open == "[t0,t1)":
        return (t >= t0) & (t < t1)
    return (t >= t0) & (t <= t1)


def run_arm(name: str, *, lam: float, gate: float | None) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    audit = OUT / f"{name}_nhc_block_audit.csv"
    if audit.exists():
        audit.unlink()
    env = os.environ.copy()
    env["NAVICORE_NHC_BLOCK_AUDIT_CSV"] = str(audit)
    r = run_benchmark(
        f"KBiasR123 {name}",
        "SLALOM",
        seed=SEED,
        imu_mode="ideal",
        nhc_jacobian="correct",
        nhc_att_z_forget=lam if gate else 0.0,
        nhc_att_z_forget_gate=gate if gate else 0.0,
        nhc_att_z_forget_tmax=TMAX if gate else None,
        archive_suffix=f"pattbias_{name}_s{SEED}",
        env=env,
    )
    if r.error:
        raise RuntimeError(r.error)
    if not audit.is_file():
        raise FileNotFoundError(audit)
    return audit


def arm_stats(df: pd.DataFrame, mask: np.ndarray) -> dict:
    if not mask.any():
        return {"n": 0}
    d = df.loc[mask]
    via_v = d["dx_bias_gz_via_vel"].to_numpy(float)
    via_a = d["dx_bias_gz_via_att"].to_numpy(float)
    tot = d["dx_bias_gz"].to_numpy(float)
    resid = tot - (via_v + via_a)
    # within-arm half-split robustness
    t = d["timestamp_s"].to_numpy(float)
    mid = 0.5 * (t.min() + t.max())
    m1 = t < mid
    m2 = ~m1

    def signed_sums(m):
        return {
            "sum_via_vel": float(via_v[m].sum()) if m.any() else 0.0,
            "sum_via_att": float(via_a[m].sum()) if m.any() else 0.0,
            "sum_total": float(tot[m].sum()) if m.any() else 0.0,
            "n": int(m.sum()),
        }

    h1, h2 = signed_sums(m1), signed_sums(m2)
    # dominant path: larger |sum|
    sv, sa = float(via_v.sum()), float(via_a.sum())
    if abs(sv) + abs(sa) < 1e-30:
        dom = "none"
    elif abs(sv) >= abs(sa):
        dom = "via_vel"
    else:
        dom = "via_att"

    # flag if halves disagree on dominant path or sign of dominant
    def dom_of(h):
        if abs(h["sum_via_vel"]) + abs(h["sum_via_att"]) < 1e-30:
            return "none"
        return "via_vel" if abs(h["sum_via_vel"]) >= abs(h["sum_via_att"]) else "via_att"

    d1, d2 = dom_of(h1), dom_of(h2)
    sign_flip_vel = (h1["sum_via_vel"] * h2["sum_via_vel"] < 0) and (
        abs(h1["sum_via_vel"]) > 1e-6 or abs(h2["sum_via_vel"]) > 1e-6
    )
    sign_flip_att = (h1["sum_via_att"] * h2["sum_via_att"] < 0) and (
        abs(h1["sum_via_att"]) > 1e-6 or abs(h2["sum_via_att"]) > 1e-6
    )
    # magnitude disparity: one half >> other for dominant path
    mag_disparate = False
    if d1 == d2 and d1 != "none":
        key = "sum_via_vel" if d1 == "via_vel" else "sum_via_att"
        a, b = abs(h1[key]), abs(h2[key])
        if max(a, b) > 1e-9 and min(a, b) / max(a, b) < 0.25:
            mag_disparate = True

    need_subdivide = (d1 != d2 and d1 != "none" and d2 != "none") or sign_flip_vel or sign_flip_att or mag_disparate

    return {
        "n": int(mask.sum()),
        "t_span": [float(t.min()), float(t.max())],
        "sum_dx_bias_gz": float(tot.sum()),
        "sum_via_vel": sv,
        "sum_via_att": sa,
        "frac_abs_via_vel": float(abs(sv) / max(abs(sv) + abs(sa), 1e-30)),
        "frac_abs_via_att": float(abs(sa) / max(abs(sv) + abs(sa), 1e-30)),
        "dominant_path": dom,
        "mean_k_via_vel": float(d["k_bias_gz_via_vel"].mean()),
        "mean_k_via_att": float(d["k_bias_gz_via_att"].mean()),
        "mean_innov": float(d["innov_norm_mps"].mean()),
        "max_abs_resid_path_sum": float(np.max(np.abs(resid))),
        "rms_resid_path_sum": float(np.sqrt(np.mean(resid**2))),
        "half_split": {"first": h1, "second": h2, "dom_first": d1, "dom_second": d2},
        "robustness": {
            "sign_flip_via_vel_across_halves": bool(sign_flip_vel),
            "sign_flip_via_att_across_halves": bool(sign_flip_att),
            "dominant_path_disagrees_across_halves": bool(d1 != d2 and d1 != "none" and d2 != "none"),
            "mag_disparate_halves": bool(mag_disparate),
            "need_subdivide": bool(need_subdivide),
        },
    }


def analyze(ctrl: Path, latch: Path) -> dict:
    dc = pd.read_csv(ctrl)
    dl = pd.read_csv(latch)
    for name, d in ("ctrl", dc), ("latch", dl):
        miss = [c for c in NEED if c not in d.columns]
        if miss:
            raise KeyError(f"{name} missing {miss} — rebuild sim with path-split audit")

    report = {
        "definition": {
            "split": "frozen-S: K_via_X = P H_X^T S^{-1}, dx = K_via_X · y, X∈{vel,att}",
            "identity": "dx_bias_gz = dx_via_vel + dx_via_att (NHC H has only vel+att)",
            "windows": WINDOWS,
        },
        "arms": {},
    }
    for arm_name, d in ("ctrl", dc), ("latch", dl):
        t = d["timestamp_s"].to_numpy(float)
        arm = {}
        for wname, (t0, t1, ho) in WINDOWS.items():
            m = mask_window(t, t0, t1, ho)
            arm[wname] = arm_stats(d, m)
        report["arms"][arm_name] = arm

    # latch vs ctrl deltas
    cmp = {}
    for w in WINDOWS:
        lc = report["arms"]["latch"][w]
        cc = report["arms"]["ctrl"][w]
        if lc.get("n", 0) == 0 or cc.get("n", 0) == 0:
            continue
        cmp[w] = {
            "delta_sum_total_L_minus_C": lc["sum_dx_bias_gz"] - cc["sum_dx_bias_gz"],
            "delta_sum_via_vel_L_minus_C": lc["sum_via_vel"] - cc["sum_via_vel"],
            "delta_sum_via_att_L_minus_C": lc["sum_via_att"] - cc["sum_via_att"],
            "dom_ctrl": cc["dominant_path"],
            "dom_latch": lc["dominant_path"],
            "dom_changed": cc["dominant_path"] != lc["dominant_path"],
            "latch_need_subdivide": lc["robustness"]["need_subdivide"],
            "ctrl_need_subdivide": cc["robustness"]["need_subdivide"],
        }
    report["latch_vs_ctrl"] = cmp

    # motor change across rise under latch?
    doms = [report["arms"]["latch"][w]["dominant_path"] for w in WINDOWS]
    report["verdict"] = {
        "latch_dominant_by_tramo": {w: report["arms"]["latch"][w]["dominant_path"] for w in WINDOWS},
        "ctrl_dominant_by_tramo": {w: report["arms"]["ctrl"][w]["dominant_path"] for w in WINDOWS},
        "motor_changes_across_tramos_latch": len(set(doms)) > 1,
        "any_subdivide_flag": any(
            report["arms"][a][w]["robustness"]["need_subdivide"]
            for a in ("ctrl", "latch")
            for w in WINDOWS
        ),
    }
    # reading
    L = report["arms"]["latch"]
    if report["verdict"]["motor_changes_across_tramos_latch"]:
        reading = (
            "Dominant K_bias path changes across R1/R2/R3 under latch — not a single "
            "homogeneous motor over the rise. Interpret each tramo separately; do not "
            "average path fractions over [1.34,2]."
        )
    else:
        reading = (
            f"Dominant path under latch is stable ({doms[0]}) across R1–R3. "
            "Check whether magnitude/sign still differ by tramo and vs ctrl."
        )
    # is latch inducing a new path vs amplifying ctrl?
    induced = []
    for w in WINDOWS:
        if cmp[w]["dom_changed"]:
            induced.append(f"{w}: ctrl={cmp[w]['dom_ctrl']} → latch={cmp[w]['dom_latch']}")
    report["verdict"]["reading"] = reading
    report["verdict"]["dom_change_vs_ctrl"] = induced or ["none — same dominant path as ctrl in every tramo"]
    return report


def write_md(report: dict) -> None:
    lines = [
        "# K_bias_gz path decompose — R1 / R2 / R3",
        "",
        f"**Split:** `{report['definition']['split']}`",
        "",
        f"**Motor changes across tramos (latch):** "
        f"{report['verdict']['motor_changes_across_tramos_latch']}",
        "",
        report["verdict"]["reading"],
        "",
        "## Signed Σdx_bias_gz by path",
        "",
        "| Arm | Tramo | n | Σ total | Σ via_vel | Σ via_att | |via_v|/(|v|+|a|) | dominant | subdivide? |",
        "|-----|-------|---|---------|-----------|-----------|-------------------|----------|------------|",
    ]
    for arm in ("ctrl", "latch"):
        for w in WINDOWS:
            p = report["arms"][arm][w]
            lines.append(
                f"| {arm} | {w} [{WINDOWS[w][0]},{WINDOWS[w][1]}{' )' if WINDOWS[w][2]=='[t0,t1)' else ']'}] "
                f"| {p['n']} | {p['sum_dx_bias_gz']:+.5f} | {p['sum_via_vel']:+.5f} | "
                f"{p['sum_via_att']:+.5f} | {p['frac_abs_via_vel']:.2f} | "
                f"**{p['dominant_path']}** | "
                f"{'YES' if p['robustness']['need_subdivide'] else 'no'} |"
            )
    lines += [
        "",
        "## Latch − ctrl (Δ signed sums)",
        "",
        "| Tramo | ΔΣ total | ΔΣ via_vel | ΔΣ via_att | dom ctrl→latch |",
        "|-------|----------|------------|------------|----------------|",
    ]
    for w, c in report["latch_vs_ctrl"].items():
        lines.append(
            f"| {w} | {c['delta_sum_total_L_minus_C']:+.5f} | "
            f"{c['delta_sum_via_vel_L_minus_C']:+.5f} | "
            f"{c['delta_sum_via_att_L_minus_C']:+.5f} | "
            f"{c['dom_ctrl']} → {c['dom_latch']} |"
        )
    lines += [
        "",
        "## Dom vs ctrl",
        "",
        *[f"- {x}" for x in report["verdict"]["dom_change_vs_ctrl"]],
        "",
        "Figure: `fig_k_bias_r123_paths.png`",
        "",
    ]
    (OUT / "k_bias_r123_decompose.md").write_text("\n".join(lines), encoding="utf-8")


def plot(report: dict, ctrl: Path, latch: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
    tramos = list(WINDOWS.keys())
    x = np.arange(len(tramos))
    width = 0.35
    for ax, arm, color_v, color_a in (
        (axes[0], "ctrl", "C0", "C1"),
        (axes[1], "latch", "C0", "C1"),
    ):
        vv = [report["arms"][arm][w]["sum_via_vel"] for w in tramos]
        va = [report["arms"][arm][w]["sum_via_att"] for w in tramos]
        ax.bar(x - width / 2, vv, width, label="Σ via_vel", color=color_v)
        ax.bar(x + width / 2, va, width, label="Σ via_att", color=color_a)
        ax.axhline(0, color="gray", lw=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels(tramos)
        ax.set_ylabel("Σ dx_bias_gz")
        ax.set_title(f"{arm}: path contribution by tramo")
        ax.legend(fontsize=8)
    fig.suptitle("K_bias_gz H_vel vs H_att paths — R1/R2/R3 (seed 71)")
    fig.tight_layout()
    fig.savefig(OUT / "fig_k_bias_r123_paths.png", dpi=140)
    plt.close(fig)


def main() -> None:
    skip_run = "--reuse-audit" in sys.argv
    if skip_run:
        ctrl = OUT / "ctrl_nhc_block_audit.csv"
        latch = OUT / "latch_nhc_block_audit.csv"
    else:
        print("Running ctrl…")
        ctrl = run_arm("ctrl", lam=0.0, gate=None)
        print("Running latch λ=1 @ T2…")
        latch = run_arm("latch", lam=1.0, gate=T2)

    report = analyze(ctrl, latch)
    report["figure"] = str(OUT / "fig_k_bias_r123_paths.png")
    (OUT / "k_bias_r123_decompose.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    write_md(report)
    plot(report, ctrl, latch)
    print(json.dumps(report["verdict"], indent=2))
    for arm in ("ctrl", "latch"):
        print(f"=== {arm} ===")
        for w in WINDOWS:
            p = report["arms"][arm][w]
            print(
                f"  {w}: tot={p['sum_dx_bias_gz']:+.5f} "
                f"vel={p['sum_via_vel']:+.5f} att={p['sum_via_att']:+.5f} "
                f"dom={p['dominant_path']} sub={p['robustness']['need_subdivide']}"
            )


if __name__ == "__main__":
    main()
