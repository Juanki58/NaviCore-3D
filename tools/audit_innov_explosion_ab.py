#!/usr/bin/env python3
"""Discriminant A vs B: truth v_body vs filter v_body in [1.69, 1.79]s.

Protocol §13.10 / innov_explosion_next_freeze.json
NHC observes body lat/vert (y,z). Truth from slalom_kinematics_at_time.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

OUT = Path(__file__).resolve().parents[1] / "docs/benchmarks/jacobian_imu_ab/patt_bias_g"
T0, T1 = 1.69, 1.79

# Match slalom_scenario.cpp / burstiness autopsy
TC04_SPEED_MPS = 50.0 / 3.6
TC04_SLALOM_PERIOD_S = 4.0
TC04_MAX_LATERAL_ACCEL_MPS2 = 3.0
TC04_BASE_COURSE_DEG = 45.0  # verify
OMEGA = 2.0 * np.pi / TC04_SLALOM_PERIOD_S
YAW_AMP = TC04_MAX_LATERAL_ACCEL_MPS2 / (TC04_SPEED_MPS * OMEGA)
BASE_COURSE = np.deg2rad(TC04_BASE_COURSE_DEG)


def load_base_course() -> float:
    """Prefer header constant if available."""
    try:
        from pathlib import Path as P
        text = (P(__file__).resolve().parents[1] / "src/scenarios/slalom_benchmark.hpp").read_text(
            encoding="utf-8", errors="ignore"
        )
        for line in text.splitlines():
            if "TC04_BASE_COURSE_DEG" in line and "define" in line.lower() or "=" in line:
                # parse number
                import re

                m = re.search(r"TC04_BASE_COURSE_DEG\s+(\d+(?:\.\d+)?)", line)
                if m:
                    return float(np.deg2rad(float(m.group(1))))
                m = re.search(r"TC04_BASE_COURSE_DEG\s*=\s*([0-9.]+)", line)
                if m:
                    return float(np.deg2rad(float(m.group(1))))
    except Exception:
        pass
    return BASE_COURSE


def truth_v_body(t: np.ndarray, base_course: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (v_fwd, v_lat, v_vert) truth in body — yaw-only DCM, vel along heading."""
    phase = OMEGA * t
    yaw = base_course + YAW_AMP * np.sin(phase)
    # vel NED along heading
    vn = TC04_SPEED_MPS * np.cos(yaw)
    ve = TC04_SPEED_MPS * np.sin(yaw)
    vd = np.zeros_like(t)
    # yaw-only quat → DCM_bn; body = C_nb * ned = columns of C_bn
    # For yaw ψ: body_x = vn*cos + ve*sin = V
    #            body_y = -vn*sin + ve*cos = 0
    #            body_z = vd = 0
    c, s = np.cos(yaw), np.sin(yaw)
    v_fwd = vn * c + ve * s
    v_lat = -vn * s + ve * c
    v_vert = vd.copy()
    return v_fwd, v_lat, v_vert


def arm_table(name: str, audit: Path) -> pd.DataFrame:
    d = pd.read_csv(audit)
    m = (d["timestamp_s"] >= T0) & (d["timestamp_s"] <= T1)
    w = d.loc[m].copy()
    t = w["timestamp_s"].to_numpy(float)
    base = load_base_course()
    tf, tl, tv = truth_v_body(t, base)
    # filter: audit before NHC update
    fl = w["v_body_y_before_mps"].to_numpy(float)
    fv = w["v_body_z_before_mps"].to_numpy(float)
    # consistency: innov should be -v_body
    iy = w["innov_y_mps"].to_numpy(float)
    iz = w["innov_z_mps"].to_numpy(float)
    out = pd.DataFrame(
        {
            "t_s": t,
            "truth_v_lat": tl,
            "truth_v_vert": tv,
            "filter_v_lat": fl,
            "filter_v_vert": fv,
            "resid_lat_filter_minus_truth": fl - tl,
            "resid_vert_filter_minus_truth": fv - tv,
            "innov_y": iy,
            "innov_z": iz,
            "innov_plus_filter_v_lat": iy + fl,  # ~0 if consistent
            "innov_plus_filter_v_vert": iz + fv,
            "abs_truth_lat": np.abs(tl),
            "abs_truth_vert": np.abs(tv),
            "abs_filter_lat": np.abs(fl),
            "abs_filter_vert": np.abs(fv),
            "innov_norm": w["innov_norm_mps"].to_numpy(float),
        }
    )
    out.attrs["arm"] = name
    return out


def sub_split(df: pd.DataFrame) -> dict:
    """Three equal sub-windows inside [1.69, 1.79] — avoid homogeneous average."""
    edges = np.linspace(T0, T1, 4)
    labels = ["S1", "S2", "S3"]
    out = {}
    for i, lab in enumerate(labels):
        m = (df["t_s"] >= edges[i]) & (df["t_s"] < edges[i + 1] if i < 2 else df["t_s"] <= edges[i + 1])
        if i == 2:
            m = (df["t_s"] >= edges[i]) & (df["t_s"] <= edges[i + 1])
        g = df.loc[m]
        if g.empty:
            out[lab] = {"n": 0}
            continue
        out[lab] = {
            "n": int(len(g)),
            "t_span": [float(g["t_s"].min()), float(g["t_s"].max())],
            "max_abs_truth_lat": float(g["abs_truth_lat"].max()),
            "max_abs_truth_vert": float(g["abs_truth_vert"].max()),
            "mean_abs_truth_lat": float(g["abs_truth_lat"].mean()),
            "max_abs_filter_lat": float(g["abs_filter_lat"].max()),
            "max_abs_filter_vert": float(g["abs_filter_vert"].max()),
            "mean_abs_filter_lat": float(g["abs_filter_lat"].mean()),
            "mean_innov_norm": float(g["innov_norm"].mean()),
            "max_innov_norm": float(g["innov_norm"].max()),
        }
    return out


def classify(df: pd.DataFrame, truth_tol: float = 0.05) -> dict:
    """truth_tol m/s: 'near zero' for NHC assumption on truth."""
    max_t_lat = float(df["abs_truth_lat"].max())
    max_t_vert = float(df["abs_truth_vert"].max())
    max_f_lat = float(df["abs_filter_lat"].max())
    max_f_vert = float(df["abs_filter_vert"].max())
    truth_ok = max_t_lat <= truth_tol and max_t_vert <= truth_tol
    filter_bad = max_f_lat > truth_tol or max_f_vert > truth_tol

    # per-sub mixed?
    subs = sub_split(df)
    sub_labels = []
    for lab, s in subs.items():
        if s.get("n", 0) == 0:
            continue
        t_ok = s["max_abs_truth_lat"] <= truth_tol and s["max_abs_truth_vert"] <= truth_tol
        f_bad = s["max_abs_filter_lat"] > truth_tol or s["max_abs_filter_vert"] > truth_tol
        if t_ok and f_bad:
            sub_labels.append(f"{lab}:A")
        elif (not t_ok) and f_bad:
            sub_labels.append(f"{lab}:B_or_both")
        elif not t_ok:
            sub_labels.append(f"{lab}:B")
        else:
            sub_labels.append(f"{lab}:quiet")

    if truth_ok and filter_bad:
        label = "A_CASCADE"
        reading = (
            "Truth v_body lat/vert stays ~0 (NHC assumption holds for scenario kinematics); "
            "filter v_body diverges — innov explosion is filter/state cascade, not NHC-too-rigid."
        )
    elif (not truth_ok) and filter_bad:
        # check if filter error >> truth violation
        if max_f_lat > 3 * max(max_t_lat, 1e-9) or max_f_vert > 3 * max(max_t_vert, 1e-9):
            label = "MIXED_A_DOMINATES"
            reading = (
                "Truth has some non-zero body lat/vert, but filter deviation is much larger — "
                "cascade still primary; do not jump to B alone."
            )
        else:
            label = "B_NHC_TOO_RIGID"
            reading = (
                "Truth itself has non-negligible body lat/vert in window — NHC penalizes "
                "real scenario kinematics (design limit, not EKF bug)."
            )
    elif truth_ok and not filter_bad:
        label = "NEITHER_QUIET"
        reading = "Both truth and filter near zero in window — innov explosion not located here."
    else:
        label = "AMBIGUOUS"
        reading = "Pattern unclear; see tick table and sub-tramos."

    return {
        "label": label,
        "reading": reading,
        "truth_tol_mps": truth_tol,
        "max_abs_truth_lat": max_t_lat,
        "max_abs_truth_vert": max_t_vert,
        "max_abs_filter_lat": max_f_lat,
        "max_abs_filter_vert": max_f_vert,
        "mean_abs_filter_lat": float(df["abs_filter_lat"].mean()),
        "mean_abs_filter_vert": float(df["abs_filter_vert"].mean()),
        "max_innov_norm": float(df["innov_norm"].max()),
        "sub_tramo_tags": sub_labels,
        "subs": subs,
        "scenario_note": (
            "Ideal SLALOM truth is yaw-only with vel along heading → truth v_lat=v_vert≡0 "
            "by construction. B cannot appear unless kinematics change; a 'B' verdict here "
            "would require numerical artifact or a different scenario."
        ),
    }


def main() -> None:
    base = load_base_course()
    report = {
        "window_s": [T0, T1],
        "truth_kinematics": {
            "source": "slalom_kinematics_at_time (yaw-only, vel along heading)",
            "base_course_rad": base,
            "speed_mps": TC04_SPEED_MPS,
            "yaw_amp_rad": YAW_AMP,
            "omega_radps": OMEGA,
            "analytic": "truth v_lat ≡ 0, v_vert ≡ 0",
        },
        "arms": {},
    }

    tables = {}
    for arm in ("ctrl", "latch"):
        path = OUT / f"{arm}_nhc_block_audit.csv"
        df = arm_table(arm, path)
        tables[arm] = df
        # save tick CSV
        tick_path = OUT / f"innov_explosion_ab_{arm}_ticks.csv"
        df.to_csv(tick_path, index=False)
        cl = classify(df)
        report["arms"][arm] = {
            "tick_csv": str(tick_path),
            "n_ticks": int(len(df)),
            "verdict": cl,
            "max_abs_innov_plus_v_resid": float(
                np.max(
                    np.abs(
                        np.column_stack(
                            [
                                df["innov_plus_filter_v_lat"],
                                df["innov_plus_filter_v_vert"],
                            ]
                        )
                    )
                )
            ),
        }

    # overall: latch is the arm of interest; ctrl for comparison
    lv = report["arms"]["latch"]["verdict"]["label"]
    cv = report["arms"]["ctrl"]["verdict"]["label"]
    report["verdict"] = {
        "latch": lv,
        "ctrl": cv,
        "reading": report["arms"]["latch"]["verdict"]["reading"],
        "design_implication": (
            "A_CASCADE → resume attitude-Z cascade thread (onset→break→innov lag); "
            "do not open NHC-too-rigid design conversation from this ideal-slalom evidence. "
            "B would require a scenario with real sideslip/bank in truth."
        ),
    }

    # figure
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for arm, color in ("ctrl", "C0"), ("latch", "C3"):
        df = tables[arm]
        axes[0].plot(df["t_s"], df["truth_v_lat"], color="k", ls="--", lw=1, alpha=0.5)
        axes[0].plot(df["t_s"], df["filter_v_lat"], color=color, lw=1.2, label=f"{arm} filter")
        axes[1].plot(df["t_s"], df["truth_v_vert"], color="k", ls="--", lw=1, alpha=0.5)
        axes[1].plot(df["t_s"], df["filter_v_vert"], color=color, lw=1.2, label=f"{arm} filter")
    axes[0].axhline(0, color="gray", lw=0.6)
    axes[1].axhline(0, color="gray", lw=0.6)
    axes[0].set_ylabel("v_lat body [m/s]")
    axes[1].set_ylabel("v_vert body [m/s]")
    axes[1].set_xlabel("t [s]")
    axes[0].set_title("A vs B: truth (black dashed ≡0) vs filter v_body — [1.69, 1.79]")
    axes[0].legend(fontsize=8)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig_path = OUT / "fig_innov_explosion_ab.png"
    fig.savefig(fig_path, dpi=140)
    plt.close(fig)
    report["figure"] = str(fig_path)

    (OUT / "innov_explosion_ab.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    # markdown with tick table (latch primary)
    dl = tables["latch"]
    lines = [
        "# Innov explosion A vs B — truth vs filter v_body [1.69, 1.79]s",
        "",
        f"**Verdict (latch):** `{lv}`",
        f"**Verdict (ctrl):** `{cv}`",
        "",
        report["verdict"]["reading"],
        "",
        "## Framing",
        "",
        "- Truth: `slalom_kinematics_at_time` — yaw-only, velocity along heading → "
        "**v_lat ≡ 0, v_vert ≡ 0 by construction**.",
        "- Filter: `v_body_y/z_before` from NHC audit (= −innov).",
        "- Window: [1.69, 1.79] s; sub-tramos S1/S2/S3 to avoid homogeneous average.",
        "",
        "## Summary latch",
        "",
        f"| qty | max | mean |",
        f"|-----|-----|------|",
        f"| |truth v_lat| | {dl['abs_truth_lat'].max():.3e} | {dl['abs_truth_lat'].mean():.3e} |",
        f"| |truth v_vert| | {dl['abs_truth_vert'].max():.3e} | {dl['abs_truth_vert'].mean():.3e} |",
        f"| |filter v_lat| | {dl['abs_filter_lat'].max():.4f} | {dl['abs_filter_lat'].mean():.4f} |",
        f"| |filter v_vert| | {dl['abs_filter_vert'].max():.4f} | {dl['abs_filter_vert'].mean():.4f} |",
        f"| innov_norm | {dl['innov_norm'].max():.4f} | {dl['innov_norm'].mean():.4f} |",
        "",
        "## Sub-tramos latch",
        "",
        "| Sub | t | max|truth_lat| | max|filt_lat| | max|filt_vert| | mean‖y‖ | tag |",
        "|-----|---|-----------------|----------------|------------------|---------|-----|",
    ]
    tags = report["arms"]["latch"]["verdict"]["sub_tramo_tags"]
    for i, (lab, s) in enumerate(report["arms"]["latch"]["verdict"]["subs"].items()):
        tag = tags[i] if i < len(tags) else "?"
        lines.append(
            f"| {lab} | [{s['t_span'][0]:.3f},{s['t_span'][1]:.3f}] | "
            f"{s['max_abs_truth_lat']:.2e} | {s['max_abs_filter_lat']:.3f} | "
            f"{s['max_abs_filter_vert']:.3f} | {s['mean_innov_norm']:.3f} | {tag} |"
        )
    lines += [
        "",
        "## Tick table — latch (primary)",
        "",
        "| t | truth_v_lat | truth_v_vert | filter_v_lat | filter_v_vert | resid_lat | resid_vert | ‖y‖ |",
        "|---|-------------|--------------|--------------|--------------|-----------|------------|-----|",
    ]
    for _, r in dl.iterrows():
        lines.append(
            f"| {r['t_s']:.3f} | {r['truth_v_lat']:+.3e} | {r['truth_v_vert']:+.3e} | "
            f"{r['filter_v_lat']:+.4f} | {r['filter_v_vert']:+.4f} | "
            f"{r['resid_lat_filter_minus_truth']:+.4f} | "
            f"{r['resid_vert_filter_minus_truth']:+.4f} | {r['innov_norm']:.4f} |"
        )
    lines += [
        "",
        "## Tick table — ctrl (comparison)",
        "",
        "| t | truth_v_lat | filter_v_lat | filter_v_vert | resid_lat | ‖y‖ |",
        "|---|-------------|--------------|--------------|-----------|-----|",
    ]
    for _, r in tables["ctrl"].iterrows():
        lines.append(
            f"| {r['t_s']:.3f} | {r['truth_v_lat']:+.3e} | {r['filter_v_lat']:+.4f} | "
            f"{r['filter_v_vert']:+.4f} | {r['resid_lat_filter_minus_truth']:+.4f} | "
            f"{r['innov_norm']:.4f} |"
        )
    lines += [
        "",
        "## Implication",
        "",
        report["verdict"]["design_implication"],
        "",
        f"Figure: `{fig_path.name}`",
        f"JSON: `innov_explosion_ab.json`",
        "",
    ]
    (OUT / "innov_explosion_ab.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report["verdict"], indent=2))
    print("latch max |filter_lat|", dl["abs_filter_lat"].max())
    print("latch max |truth_lat|", dl["abs_truth_lat"].max())


if __name__ == "__main__":
    main()
