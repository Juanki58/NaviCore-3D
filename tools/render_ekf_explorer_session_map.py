#!/usr/bin/env python3
"""Render pink(EKF) vs yellow(GNSS) map from Explorer session.json — Unity-grade visual check without GUI."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "docs" / "benchmarks" / "ekf_v2_unity_visual"
SESSIONS = (
    REPO
    / "ekf_explorer"
    / "EkfExplorer"
    / "Assets"
    / "StreamingAssets"
    / "Sessions"
)


def track_xy(track: dict) -> tuple[np.ndarray, np.ndarray]:
    samples = track.get("samples") or []
    lat = np.array([float(s["lat_deg"]) for s in samples], dtype=float)
    lon = np.array([float(s["lon_deg"]) for s in samples], dtype=float)
    return lat, lon


def horiz_sep_m(est: dict, truth: dict) -> dict:
    """Nearest-time horizontal separation using n_m/e_m."""
    es = est["samples"]
    ts = truth["samples"]
    te = np.array([float(s["t_s"]) for s in es])
    seps = []
    for s in ts:
        t = float(s["t_s"])
        i = int(np.argmin(np.abs(te - t)))
        e = es[i]
        if "n_m" in e and "n_m" in s:
            dn = float(e["n_m"]) - float(s["n_m"])
            de = float(e["e_m"]) - float(s["e_m"])
            seps.append(math.hypot(dn, de))
    arr = np.array(seps, dtype=float) if seps else np.array([float("nan")])
    return {
        "median_m": float(np.nanmedian(arr)),
        "p95_m": float(np.nanpercentile(arr, 95)),
        "max_m": float(np.nanmax(arr)),
        "n": int(np.sum(np.isfinite(arr))),
    }


def render(session_id: str, out_png: Path) -> dict:
    path = SESSIONS / session_id / "session.json"
    if not path.exists():
        raise FileNotFoundError(path)
    pack = json.loads(path.read_text(encoding="utf-8"))
    name = pack.get("name") or session_id
    tracks = {t["role"]: t for t in pack.get("tracks") or []}
    # also by id
    by_id = {t["id"]: t for t in pack.get("tracks") or []}
    est = tracks.get("estimate") or by_id.get("ekf_replay")
    truth = tracks.get("truth") or by_id.get("gnss_phone")
    if est is None or truth is None:
        raise RuntimeError(f"missing tracks in {session_id}: roles={list(tracks)}")

    lat_e, lon_e = track_xy(est)
    lat_g, lon_g = track_xy(truth)
    stats = horiz_sep_m(est, truth)
    provenance = (pack.get("provenance") or {}).get("baseline_dir", "")

    fig, ax = plt.subplots(figsize=(10, 10), dpi=140)
    ax.set_facecolor("#1a1f24")
    fig.patch.set_facecolor("#12151a")
    ax.plot(lon_g, lat_g, color="#f0d040", linewidth=2.2, label="GNSS (amarillo)", zorder=2)
    ax.plot(lon_e, lat_e, color="#e85a9b", linewidth=1.6, alpha=0.9, label="EKF (rosa)", zorder=3)
    ax.scatter([lon_g[0]], [lat_g[0]], c="#f0d040", s=36, zorder=4, edgecolors="k", linewidths=0.4)
    ax.scatter([lon_e[-1]], [lat_e[-1]], c="#e85a9b", s=36, zorder=4, edgecolors="k", linewidths=0.4)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color="#2a323c", linewidth=0.5)
    ax.tick_params(colors="#aab4c0", labelsize=8)
    for spine in ax.spines.values():
        spine.set_color("#3a4550")
    ax.set_xlabel("lon", color="#aab4c0")
    ax.set_ylabel("lat", color="#aab4c0")
    ax.set_title(
        f"{name}\nmed={stats['median_m']:.1f} m  p95={stats['p95_m']:.1f} m  max={stats['max_m']:.1f} m",
        color="#e8eef4",
        fontsize=12,
        pad=10,
    )
    leg = ax.legend(loc="upper right", facecolor="#1e252c", edgecolor="#3a4550", labelcolor="#e8eef4")
    fig.text(
        0.02,
        0.01,
        f"provenance: {provenance}",
        color="#7a8794",
        fontsize=7,
        ha="left",
    )
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)
    return {"name": name, "provenance": provenance, **stats, "png": str(out_png)}


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    results = []
    for sid in ("REF_19082026_v2", "real_run_19082026_nhc_off", "ALT_16072026_v2", "JUL17_20260717_v2"):
        try:
            r = render(sid, OUT / f"{sid}_pink_yellow.png")
            results.append(r)
            print(json.dumps(r, ensure_ascii=False), flush=True)
        except Exception as exc:
            print(f"FAIL {sid}: {exc}", file=sys.stderr, flush=True)

    # Side-by-side REF v2 vs v1 control
    fig, axes = plt.subplots(1, 2, figsize=(14, 7), dpi=140)
    fig.patch.set_facecolor("#12151a")
    for ax, sid, title in zip(
        axes,
        ("REF_19082026_v2", "real_run_19082026_nhc_off"),
        ("REF v2 (candidato)", "REF v1 NHC-off (control)"),
    ):
        pack = json.loads((SESSIONS / sid / "session.json").read_text(encoding="utf-8"))
        by_id = {t["id"]: t for t in pack["tracks"]}
        est, truth = by_id["ekf_replay"], by_id["gnss_phone"]
        lat_e, lon_e = track_xy(est)
        lat_g, lon_g = track_xy(truth)
        stats = horiz_sep_m(est, truth)
        ax.set_facecolor("#1a1f24")
        ax.plot(lon_g, lat_g, color="#f0d040", linewidth=2.0)
        ax.plot(lon_e, lat_e, color="#e85a9b", linewidth=1.4, alpha=0.9)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(
            f"{title}\nmax={stats['max_m']:.0f} m",
            color="#e8eef4",
            fontsize=11,
        )
        ax.tick_params(colors="#aab4c0", labelsize=7)
        for spine in ax.spines.values():
            spine.set_color("#3a4550")
    cmp = OUT / "REF_v2_vs_v1_side_by_side.png"
    fig.tight_layout()
    fig.savefig(cmp, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("WROTE", cmp, flush=True)
    (OUT / "visual_stats.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
