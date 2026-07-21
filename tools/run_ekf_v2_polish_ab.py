#!/usr/bin/env python3
"""A/B REF: EKF v2 baseline vs v2 --polish (coherent vel + NHC gap + adaptive R)."""

from __future__ import annotations

import json
import math
import re
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
REPLAY = REPO / "build" / "NaviCore3D_Replay.exe"
MOUNT = REPO / "calibration" / "imu_mount.json"
INPUT = (
    REPO
    / "docs"
    / "benchmarks"
    / "real_run_19082026_baseline"
    / "real_run_replay.csv"
)
OUT = REPO / "docs" / "benchmarks" / "ekf_v2_polish_ab"
EXPORT = REPO / "tools" / "export_ekf_explorer_session.py"
SESSIONS = (
    REPO
    / "ekf_explorer"
    / "EkfExplorer"
    / "Assets"
    / "StreamingAssets"
    / "Sessions"
)


def run_arm(name: str, polish: bool) -> Path:
    arm = OUT / name
    arm.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(REPLAY),
        "--input",
        str(INPUT),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(MOUNT),
        "--yaw-init",
        "zero",
        "--h9a-gravity-tilt-init",
        "--constraint-policy",
        "disabled",
        "--nhc-policy",
        "enabled" if polish else "disabled",
        "--gnss-obs-mode",
        "pos_vel",
        "--p-pv-policy",
        "none",
        "--ekf-core",
        "v2",
        "--output",
        str(arm / "replay_output.csv"),
        "--gap3-gnss-nis-audit-csv",
        str(arm / "gnss_nis_audit.csv"),
        "--gap3-constraint-pipeline-audit-csv",
        str(arm / "constraint_pipeline_audit.csv"),
    ]
    if polish:
        cmd.append("--v2-polish")
    print("RUN", name, flush=True)
    log = subprocess.run(cmd, cwd=str(REPO), check=True, capture_output=True, text=True)
    text = (log.stdout or "") + (log.stderr or "")
    (arm / "replay.log").write_text(text, encoding="utf-8", errors="replace")
    return arm


def parse_drift(log_text: str) -> float | None:
    m = re.search(r"Deriva final H:\s*([0-9.eE+-]+)\s*m", log_text)
    return float(m.group(1)) if m else None


def parse_nhc(log_text: str) -> int | None:
    m = re.search(r"NHC updates:\s*(\d+)", log_text)
    if not m:
        m = re.search(r"nhc_update[s]?[=:]\s*(\d+)", log_text, re.I)
    return int(m.group(1)) if m else None


def track_sep_from_replay(arm: Path) -> dict:
    """Horizontal |ekf-gps| from replay_output if columns exist; else from export session."""
    out = pd.read_csv(arm / "replay_output.csv")
    # Try common column names
    pairs = [
        ("pos_n_m", "gps_n_m", "pos_e_m", "gps_e_m"),
        ("ekf_pos_n_m", "gps_pos_n_m", "ekf_pos_e_m", "gps_pos_e_m"),
        ("n_m", "gps_n", "e_m", "gps_e"),
    ]
    for pn, gn, pe, ge in pairs:
        if pn in out.columns and gn in out.columns:
            dn = out[pn].to_numpy(float) - out[gn].to_numpy(float)
            de = out[pe].to_numpy(float) - out[ge].to_numpy(float)
            # only rows with finite gps
            mask = np.isfinite(out[gn]) & np.isfinite(out[ge])
            sep = np.hypot(dn[mask], de[mask])
            if len(sep) == 0:
                continue
            return {
                "median_m": float(np.median(sep)),
                "p95_m": float(np.percentile(sep, 95)),
                "max_m": float(np.max(sep)),
                "n": int(len(sep)),
            }
    return {"median_m": None, "p95_m": None, "max_m": None, "n": 0}


def metrics(arm: Path) -> dict:
    gnss = pd.read_csv(arm / "gnss_nis_audit.csv")
    accepts = int((gnss["accepted"] == 1).sum())
    rejects = int((gnss["accepted"] == 0).sum())
    total = accepts + rejects
    log_text = (arm / "replay.log").read_text(encoding="utf-8", errors="replace")
    # vel accepts from v2 flags if present; else infer from n_meas/contrib
    vel_acc = None
    if "accepted_vel" in gnss.columns:
        vel_acc = int((gnss["accepted_vel"] == 1).sum())
    m = {
        "accept_rate": accepts / total if total else 0.0,
        "accepts": accepts,
        "rejects": rejects,
        "final_drift_h_m": parse_drift(log_text),
        "nhc_updates": parse_nhc(log_text),
        "vel_accepts": vel_acc,
        **{f"sep_{k}": v for k, v in track_sep_from_replay(arm).items()},
    }
    return m


def export_pack(arm: Path, name: str) -> Path:
    loc = REPO / "data" / "real_run" / "19082026" / "Location.csv"
    cmd = [
        sys.executable,
        str(EXPORT),
        "--bundle-dir",
        str(arm),
        "--name",
        name,
    ]
    if loc.exists():
        cmd.extend(["--location-csv", str(loc)])
    subprocess.run(cmd, cwd=str(REPO), check=True)
    src = REPO / "docs" / "ekf_explorer" / "sessions" / name
    dst = SESSIONS / name
    dst.mkdir(parents=True, exist_ok=True)
    for f in src.iterdir():
        if f.is_file():
            (dst / f.name).write_bytes(f.read_bytes())
    return dst


def plot_compare(base_session: Path, polish_session: Path, out_png: Path) -> None:
    def load(sid: Path):
        pack = json.loads((sid / "session.json").read_text(encoding="utf-8"))
        by_id = {t["id"]: t for t in pack["tracks"]}
        return by_id["ekf_replay"], by_id["gnss_phone"], pack.get("name")

    def xy(track):
        lat = np.array([float(s["lat_deg"]) for s in track["samples"]])
        lon = np.array([float(s["lon_deg"]) for s in track["samples"]])
        return lat, lon

    def sep(est, truth):
        te = np.array([float(s["t_s"]) for s in est["samples"]])
        seps = []
        for s in truth["samples"]:
            i = int(np.argmin(np.abs(te - float(s["t_s"]))))
            e = est["samples"][i]
            seps.append(
                math.hypot(float(e["n_m"]) - float(s["n_m"]), float(e["e_m"]) - float(s["e_m"]))
            )
        a = np.array(seps)
        return float(np.median(a)), float(np.percentile(a, 95)), float(np.max(a))

    fig, axes = plt.subplots(1, 2, figsize=(14, 7), dpi=140)
    fig.patch.set_facecolor("#12151a")
    for ax, sid, title in zip(
        axes,
        (base_session, polish_session),
        ("v2 baseline", "v2 polish"),
    ):
        est, truth, name = load(sid)
        lat_e, lon_e = xy(est)
        lat_g, lon_g = xy(truth)
        med, p95, mx = sep(est, truth)
        ax.set_facecolor("#1a1f24")
        ax.plot(lon_g, lat_g, color="#f0d040", lw=2.0, label="GNSS")
        ax.plot(lon_e, lat_e, color="#e85a9b", lw=1.4, alpha=0.9, label="EKF")
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(
            f"{title}\nmed={med:.1f} p95={p95:.1f} max={mx:.1f} m",
            color="#e8eef4",
            fontsize=11,
        )
        ax.tick_params(colors="#aab4c0", labelsize=7)
        for sp in ax.spines.values():
            sp.set_color("#3a4550")
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)


def main() -> int:
    if not REPLAY.exists():
        print("ERROR missing", REPLAY, file=sys.stderr)
        return 1
    OUT.mkdir(parents=True, exist_ok=True)

    base = run_arm("A_v2_baseline", polish=False)
    pol = run_arm("B_v2_polish", polish=True)
    mb, mp = metrics(base), metrics(pol)

    summary = {"A_v2_baseline": mb, "B_v2_polish": mp}
    (OUT / "SUMMARY.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    # Prefer session-based sep after export
    pack_a = export_pack(base, "REF_19082026_v2")
    pack_b = export_pack(pol, "REF_19082026_v2_polish")
    plot_compare(pack_a, pack_b, OUT / "REF_v2_baseline_vs_polish.png")

    # Refresh sep from sessions
    def session_sep(p: Path) -> dict:
        pack = json.loads((p / "session.json").read_text(encoding="utf-8"))
        by_id = {t["id"]: t for t in pack["tracks"]}
        est, truth = by_id["ekf_replay"], by_id["gnss_phone"]
        te = np.array([float(s["t_s"]) for s in est["samples"]])
        seps = []
        for s in truth["samples"]:
            i = int(np.argmin(np.abs(te - float(s["t_s"]))))
            e = est["samples"][i]
            seps.append(
                math.hypot(float(e["n_m"]) - float(s["n_m"]), float(e["e_m"]) - float(s["e_m"]))
            )
        a = np.asarray(seps, dtype=float)
        return {
            "median_m": float(np.median(a)),
            "p95_m": float(np.percentile(a, 95)),
            "max_m": float(np.max(a)),
        }

    sa, sb = session_sep(pack_a), session_sep(pack_b)
    summary["A_v2_baseline"]["session_sep"] = sa
    summary["B_v2_polish"]["session_sep"] = sb
    (OUT / "SUMMARY.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    lines = [
        "# REF A/B: v2 baseline vs v2 polish",
        "",
        "| Arm | Accept | Drift H [m] | sep med/p95/max [m] | NHC |",
        "|-----|--------|-------------|---------------------|-----|",
        f"| A baseline | {mb['accept_rate']:.4f} | {mb['final_drift_h_m']} | "
        f"{sa['median_m']:.1f}/{sa['p95_m']:.1f}/{sa['max_m']:.1f} | {mb['nhc_updates']} |",
        f"| B polish | {mp['accept_rate']:.4f} | {mp['final_drift_h_m']} | "
        f"{sb['median_m']:.1f}/{sb['p95_m']:.1f}/{sb['max_m']:.1f} | {mp['nhc_updates']} |",
        "",
        "Packs: `REF_19082026_v2`, `REF_19082026_v2_polish`",
        f"Plot: `{OUT / 'REF_v2_baseline_vs_polish.png'}`",
    ]
    (OUT / "SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines), flush=True)
    improved = sb["median_m"] < sa["median_m"] and sb["p95_m"] <= sa["p95_m"] * 1.05
    return 0 if improved else 2


if __name__ == "__main__":
    raise SystemExit(main())
