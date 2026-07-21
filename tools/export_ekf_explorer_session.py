#!/usr/bin/env python3
"""Export NaviCore CSV artefacts → EKF Explorer session pack v1.

Schema: navicore.ekf_explorer.session/v1
See docs/diagnostics/19-ekf-explorer-protocol.md
"""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pandas as pd

REPO = Path(__file__).resolve().parents[1]
OUT_ROOT = REPO / "docs" / "ekf_explorer" / "sessions"

# WGS84 (same spirit as geodesy.hpp)
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)

SIM_ORIGIN = {"lat_deg": 41.3874, "lon_deg": 2.1686, "alt_m": 12.0}


def ned_to_lla(
    lat0_deg: float,
    lon0_deg: float,
    alt0_m: float,
    north_m: float,
    east_m: float,
    down_m: float,
) -> tuple[float, float, float]:
    """Local tangent NED → LLA degrees / meters (flat-Earth small-area)."""
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    s = math.sin(lat0)
    c = math.cos(lat0)
    n_radius = _A / math.sqrt(1.0 - _E2 * s * s)
    m_radius = _A * (1.0 - _E2) / ((1.0 - _E2 * s * s) ** 1.5)
    dlat = north_m / m_radius
    dlon = east_m / (n_radius * max(c, 1e-12))
    lat_deg = math.degrees(lat0 + dlat)
    lon_deg = math.degrees(lon0 + dlon)
    alt_m = alt0_m - down_m
    return lat_deg, lon_deg, alt_m


def _downsample(df: pd.DataFrame, max_points: int) -> pd.DataFrame:
    if len(df) <= max_points:
        return df
    step = max(1, len(df) // max_points)
    return df.iloc[::step].copy()


def track_from_ned_df(
    df: pd.DataFrame,
    *,
    origin: dict,
    track_id: str,
    role: str,
    t_col: str,
    n_col: str,
    e_col: str,
    d_col: str,
    yaw_col: str | None = None,
    yaw_in_deg: bool = False,
    max_points: int = 8000,
) -> dict[str, Any]:
    df = _downsample(df, max_points)
    samples = []
    for row in df.itertuples(index=False):
        t = float(getattr(row, t_col))
        n = float(getattr(row, n_col))
        e = float(getattr(row, e_col))
        d = float(getattr(row, d_col))
        lat, lon, alt = ned_to_lla(
            origin["lat_deg"], origin["lon_deg"], origin["alt_m"], n, e, d
        )
        sample: dict[str, Any] = {
            "t_s": t,
            "lat_deg": lat,
            "lon_deg": lon,
            "alt_m": alt,
            "n_m": n,
            "e_m": e,
            "d_m": d,
        }
        if yaw_col and hasattr(row, yaw_col):
            yaw = float(getattr(row, yaw_col))
            sample["yaw_rad"] = math.radians(yaw) if yaw_in_deg else yaw
        samples.append(sample)
    return {"id": track_id, "role": role, "samples": samples}


def _interp_ned_at(samples: list[dict], t: float) -> tuple[float, float, float] | None:
    """Linear NED interpolation on a track sample list. None if t out of range."""
    if not samples:
        return None
    if t <= samples[0]["t_s"]:
        s = samples[0]
        return float(s["n_m"]), float(s["e_m"]), float(s["d_m"])
    if t >= samples[-1]["t_s"]:
        s = samples[-1]
        return float(s["n_m"]), float(s["e_m"]), float(s["d_m"])
    lo, hi = 0, len(samples) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if samples[mid]["t_s"] < t:
            lo = mid + 1
        else:
            hi = mid
    b = samples[lo]
    a = samples[max(0, lo - 1)]
    dt = b["t_s"] - a["t_s"]
    u = 0.0 if dt < 1e-12 else (t - a["t_s"]) / dt
    n = a["n_m"] + u * (b["n_m"] - a["n_m"])
    e = a["e_m"] + u * (b["e_m"] - a["e_m"])
    d = a["d_m"] + u * (b["d_m"] - a["d_m"])
    return float(n), float(e), float(d)


def residual_horizontal_m(
    estimate: dict,
    truth: dict,
    *,
    max_points: int = 8000,
) -> list[dict]:
    """Diagnostic: ||x_estimate − x_truth|| horizontal (m) at estimate times.

    Computed in the exporter (pipeline side). Explorer only sees series t→v.
    """
    est = estimate.get("samples") or []
    tru = truth.get("samples") or []
    if len(est) < 2 or len(tru) < 1:
        return []
    step = max(1, len(est) // max_points) if len(est) > max_points else 1
    out: list[dict] = []
    for i in range(0, len(est), step):
        s = est[i]
        t = float(s["t_s"])
        ned = _interp_ned_at(tru, t)
        if ned is None:
            continue
        dn = float(s["n_m"]) - ned[0]
        de = float(s["e_m"]) - ned[1]
        out.append({"t_s": t, "v": math.hypot(dn, de)})
    return out


def accept_reject_series(events: list[dict]) -> list[dict]:
    """Binary diagnostic from events: accept=0, reject=1 (for segment coloring)."""
    pts: list[dict] = []
    for e in events or []:
        typ = e.get("type") or ""
        if typ == "gnss_accept":
            pts.append({"t_s": float(e["t_s"]), "v": 0.0})
        elif typ == "gnss_reject":
            pts.append({"t_s": float(e["t_s"]), "v": 1.0})
    pts.sort(key=lambda p: p["t_s"])
    return pts


def _series_values(pts: list[dict]) -> list[float]:
    return [
        float(p["v"])
        for p in pts
        if p.get("v") is not None and math.isfinite(float(p["v"]))
    ]


def _auto_color_vmax(pts: list[dict], fallback: float = 1.0) -> float:
    """Color-scale hint only — never used to clamp stored series values."""
    vs = _series_values(pts)
    if not vs:
        return fallback
    vs.sort()
    p95 = vs[min(len(vs) - 1, int(0.95 * (len(vs) - 1)))]
    return float(max(p95, 1e-6))


def _value_stats(pts: list[dict]) -> dict[str, float | None]:
    """Scientific stats from raw series values (never from color clamp)."""
    vs = _series_values(pts)
    if not vs:
        return {"value_min": None, "value_max": None, "value_mean": None, "value_p95": None}
    vs_sorted = sorted(vs)
    p95 = vs_sorted[min(len(vs_sorted) - 1, int(0.95 * (len(vs_sorted) - 1)))]
    return {
        "value_min": float(vs_sorted[0]),
        "value_max": float(vs_sorted[-1]),
        "value_mean": float(sum(vs) / len(vs)),
        "value_p95": float(p95),
    }


def build_series_meta(series: dict[str, list], *, kinds: dict[str, str] | None = None) -> list[dict]:
    """Opaque display hints for the Explorer. Renderer must not hardcode names.

    Dual scale (contract):
      - series CSV / session.series[*].v  → always the real value
      - color_vmin / color_vmax           → map paint only (may saturate)
      - value_*                          → stats from real values for Inspector/analysis
    Never use the color clamp for analysis.
    """
    kinds = kinds or {}
    # Known defaults (pipeline semantics); unknown names get auto color scale.
    presets = {
        "nis": {"kind": "observable", "label": "NIS", "color_vmin": 0.0, "color_vmax": 12.0},
        "lambda_n": {"kind": "observable", "label": "ΛN", "color_vmin": 0.0, "color_vmax": None},
        "gamma": {"kind": "observable", "label": "Γ", "color_vmin": 0.0, "color_vmax": 1.0},
        "ppv_frob": {"kind": "observable", "label": "||Ppv||", "color_vmin": 0.0, "color_vmax": None},
        "pvv": {"kind": "observable", "label": "Pvv", "color_vmin": 0.0, "color_vmax": None},
        "drift_m": {"kind": "observable", "label": "drift_m", "color_vmin": 0.0, "color_vmax": None},
        "nav_mode": {"kind": "observable", "label": "nav_mode", "color_vmin": 0.0, "color_vmax": 4.0},
        "nhc_applied": {"kind": "observable", "label": "NHC", "color_vmin": 0.0, "color_vmax": 1.0},
        "zupt_applied": {"kind": "observable", "label": "ZUPT", "color_vmin": 0.0, "color_vmax": 1.0},
        "residual_m": {
            "kind": "diagnostic",
            "label": "||est−truth||",
            "color_vmin": 0.0,
            "color_vmax": 50.0,  # paint only; raw residual_m can be km-scale
        },
        "accept_reject": {
            "kind": "diagnostic",
            "label": "Accept/Reject",
            "color_vmin": 0.0,
            "color_vmax": 1.0,
        },
    }
    kind_order = {"observable": 0, "diagnostic": 1, "state": 2}

    meta: list[dict] = []
    for name, pts in series.items():
        pre = dict(presets.get(name, {}))
        kind = kinds.get(name) or pre.get("kind") or "observable"
        label = pre.get("label") or name
        # Compat: old vmin/vmax keys in presets mean color scale
        cmin = pre.get("color_vmin", pre.get("vmin"))
        cmax = pre.get("color_vmax", pre.get("vmax"))
        color_vmin = float(cmin if cmin is not None else 0.0)
        color_vmax = cmax
        if color_vmax is None:
            color_vmax = _auto_color_vmax(pts, fallback=1.0)
        color_vmax = float(color_vmax)
        stats = _value_stats(pts)
        meta.append(
            {
                "name": name,
                "kind": kind,
                "label": label,
                "color_vmin": color_vmin,
                "color_vmax": color_vmax,
                # Aliases — same as color_*; never mean "clamped stored value"
                "vmin": color_vmin,
                "vmax": color_vmax,
                **stats,
                "colormap": "diverging",
            }
        )
    meta.sort(key=lambda m: (kind_order.get(m["kind"], 9), m["label"]))
    return meta


def series_from_col(df: pd.DataFrame, t_col: str, v_col: str, max_points: int = 8000) -> list[dict]:
    if v_col not in df.columns:
        return []
    df = _downsample(df[[t_col, v_col]].dropna(), max_points)
    return [{"t_s": float(r[t_col]), "v": float(r[v_col])} for _, r in df.iterrows()]


def events_from_constraints(df: pd.DataFrame) -> list[dict]:
    """Edge-detect NHC/ZUPT applied and simple regime markers."""
    events: list[dict] = []
    if df.empty:
        return events
    prev_nhc = 0
    prev_zupt = 0
    for row in df.itertuples(index=False):
        t = float(row.timestamp_s)
        nhc = int(getattr(row, "nhc_applied", 0))
        zupt = int(getattr(row, "zupt_applied", 0))
        if nhc and not prev_nhc:
            events.append({"t_s": t, "type": "nhc_applied_rise", "label": "NHC applied"})
        if zupt and not prev_zupt:
            events.append({"t_s": t, "type": "zupt_applied_rise", "label": "ZUPT applied"})
        prev_nhc, prev_zupt = nhc, zupt
    # Cap dense edges
    if len(events) > 200:
        step = len(events) // 200
        events = events[::step]
    return events


def events_from_sim_telemetry(df: pd.DataFrame) -> list[dict]:
    """nav_mode transitions as coarse events."""
    events = []
    if "nav_mode" not in df.columns or "t_s" not in df.columns:
        return events
    prev = None
    for row in df.itertuples(index=False):
        mode = int(row.nav_mode)
        if prev is None or mode != prev:
            events.append(
                {
                    "t_s": float(row.t_s),
                    "type": "nav_mode",
                    "label": f"nav_mode={mode}",
                }
            )
            prev = mode
    return events[:100]


def to_czml(session: dict) -> list[dict]:
    """Minimal CZML document for Cesium (tracks as polylines)."""
    doc = [
        {
            "id": "document",
            "name": session.get("name", "ekf_explorer"),
            "version": "1.0",
            "clock": {
                "interval": None,
                "currentTime": None,
                "multiplier": 1,
                "range": "LOOP_STOP",
                "step": "SYSTEM_CLOCK_MULTIPLIER",
            },
        }
    ]
    t0 = None
    t1 = None
    for track in session.get("tracks", []):
        samples = track.get("samples") or []
        if not samples:
            continue
        times = [s["t_s"] for s in samples]
        t0 = min(times) if t0 is None else min(t0, min(times))
        t1 = max(times) if t1 is None else max(t1, max(times))
        # Cartographic degrees in CZML: lon, lat, height
        positions = []
        for s in samples:
            positions.extend([s["lon_deg"], s["lat_deg"], s["alt_m"]])
        color = {
            "estimate": [0, 255, 255, 255],
            "truth": [255, 255, 0, 255],
            "compare": [255, 64, 64, 255],
        }.get(track.get("role"), [200, 200, 200, 255])
        doc.append(
            {
                "id": track["id"],
                "name": f"{track['id']} ({track.get('role')})",
                "polyline": {
                    "positions": {"cartographicDegrees": positions},
                    "width": 3,
                    "material": {
                        "solidColor": {
                            "color": {"rgba": color},
                        }
                    },
                    "clampToGround": False,
                },
            }
        )
    if t0 is not None and t1 is not None:
        # ISO-ish relative clock using epoch day
        doc[0]["clock"]["interval"] = f"{t0}/{t1}"
        doc[0]["clock"]["currentTime"] = str(t0)
    for ev in session.get("events", [])[:80]:
        # Place event near first track sample closest in time
        track = session["tracks"][0] if session.get("tracks") else None
        if not track:
            break
        samples = track["samples"]
        nearest = min(samples, key=lambda s: abs(s["t_s"] - ev["t_s"]))
        doc.append(
            {
                "id": f"event_{ev['type']}_{ev['t_s']}",
                "name": ev.get("label", ev["type"]),
                "position": {
                    "cartographicDegrees": [
                        nearest["lon_deg"],
                        nearest["lat_deg"],
                        nearest["alt_m"] + 20.0,
                    ]
                },
                "point": {
                    "pixelSize": 10,
                    "color": {"rgba": [255, 128, 0, 255]},
                },
                "label": {
                    "text": ev.get("label", ev["type"]),
                    "font": "12px sans-serif",
                    "show": True,
                },
            }
        )
    return doc


def build_session(
    *,
    name: str,
    origin: dict,
    tracks: list[dict],
    events: list[dict],
    series: dict[str, list],
    provenance: dict,
    series_meta: list[dict] | None = None,
) -> dict:
    all_t = []
    for tr in tracks:
        all_t.extend(s["t_s"] for s in tr.get("samples", []))
    if series_meta is None:
        series_meta = build_series_meta(series)
    return {
        "schema": "navicore.ekf_explorer.session/v1",
        "name": name,
        "exported_at": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "origin": origin,
        "t_range_s": [min(all_t), max(all_t)] if all_t else [0.0, 0.0],
        "tracks": tracks,
        "events": sorted(events, key=lambda e: e["t_s"]),
        "series": series,
        "series_meta": series_meta,
        "provenance": provenance,
        "protocol": "docs/diagnostics/19-ekf-explorer-protocol.md",
    }


def export_sim_telemetry(
    csv_path: Path,
    *,
    name: str,
    origin: dict = SIM_ORIGIN,
    max_points: int = 8000,
) -> dict:
    df = pd.read_csv(csv_path)
    df["t_s"] = df["time_us"].astype(float) * 1e-6
    track = track_from_ned_df(
        df,
        origin=origin,
        track_id="ekf",
        role="estimate",
        t_col="t_s",
        n_col="pos_x",
        e_col="pos_y",
        d_col="pos_z",
        yaw_col="yaw",
        yaw_in_deg=False,
        max_points=max_points,
    )
    series = {
        "nis": series_from_col(df, "t_s", "nis", max_points),
        "drift_m": series_from_col(df, "t_s", "drift_m", max_points),
        "nav_mode": series_from_col(df, "t_s", "nav_mode", max_points),
    }
    events = events_from_sim_telemetry(df)
    return build_session(
        name=name,
        origin=origin,
        tracks=[track],
        events=events,
        series=series,
        provenance={
            "sources": [str(csv_path.relative_to(REPO)).replace("\\", "/")],
            "kind": "sim_telemetry_ned",
            "notes": "pos_* are NED meters; converted with session origin",
        },
    )


def lla_to_ned(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    origin: dict,
) -> tuple[float, float, float]:
    """Inverse of ned_to_lla (small-area flat Earth)."""
    lat0 = math.radians(origin["lat_deg"])
    lon0 = math.radians(origin["lon_deg"])
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat0)
    c = math.cos(lat0)
    a = 6378137.0
    e2 = 6.69437999014e-3
    n_radius = a / math.sqrt(1.0 - e2 * s * s)
    m_radius = a * (1.0 - e2) / ((1.0 - e2 * s * s) ** 1.5)
    north_m = (lat - lat0) * m_radius
    east_m = (lon - lon0) * n_radius * max(c, 1e-12)
    down_m = float(origin["alt_m"]) - alt_m
    return north_m, east_m, down_m


def track_from_location_csv(
    loc_path: Path,
    *,
    origin: dict,
    track_id: str = "gnss_phone",
    role: str = "truth",
    max_points: int = 8000,
) -> dict:
    """Phone GNSS LLA → session track (truth layer)."""
    df = pd.read_csv(loc_path)
    df = df.dropna(subset=["latitude", "longitude", "seconds_elapsed"])
    df = _downsample(df, max_points)
    samples = []
    for _, r in df.iterrows():
        lat = float(r["latitude"])
        lon = float(r["longitude"])
        alt = float(r["altitude"]) if "altitude" in r and pd.notna(r["altitude"]) else float(origin["alt_m"])
        n, e, d = lla_to_ned(lat, lon, alt, origin)
        yaw = 0.0
        if "bearing" in r and pd.notna(r["bearing"]):
            yaw = math.radians(float(r["bearing"]))
        samples.append(
            {
                "t_s": float(r["seconds_elapsed"]),
                "lat_deg": lat,
                "lon_deg": lon,
                "alt_m": alt,
                "n_m": n,
                "e_m": e,
                "d_m": d,
                "yaw_rad": yaw,
            }
        )
    return {"id": track_id, "role": role, "samples": samples}


def events_from_gnss_audit(gdf: pd.DataFrame, *, max_reject: int = 200, max_accept: int = 200) -> list[dict]:
    """GNSS accept (green) / reject (red) markers for Explorer timeline + map."""
    events: list[dict] = []
    if gdf.empty or "accepted" not in gdf.columns or "timestamp_s" not in gdf.columns:
        return events
    acc = gdf[gdf["accepted"].astype(int) == 1]
    rej = gdf[gdf["accepted"].astype(int) == 0]
    if len(acc) > max_accept:
        acc = acc.iloc[:: max(1, len(acc) // max_accept)]
    if len(rej) > max_reject:
        rej = rej.iloc[:: max(1, len(rej) // max_reject)]
    for _, r in acc.iterrows():
        events.append(
            {
                "t_s": float(r["timestamp_s"]),
                "type": "gnss_accept",
                "label": "GNSS accept",
                "track_id": "gnss_phone",
            }
        )
    for _, r in rej.iterrows():
        events.append(
            {
                "t_s": float(r["timestamp_s"]),
                "type": "gnss_reject",
                "label": str(r.get("reject_reason", "reject")),
                "track_id": "gnss_phone",
            }
        )
    return events


def export_real_run_baseline(
    baseline_dir: Path,
    *,
    name: str,
    origin: dict | None = None,
    max_points: int = 8000,
    location_csv: Path | None = None,
) -> dict:
    replay = baseline_dir / "replay_output.csv"
    constr = baseline_dir / "constraint_pipeline_audit.csv"
    gnss = baseline_dir / "gnss_nis_audit.csv"
    loc = location_csv or (REPO / "data" / "real_run" / "19082026" / "Location.csv")
    df = pd.read_csv(replay)
    # Infer origin from first GPS row if not given: treat NED as relative to phone first fix.
    if origin is None:
        if loc.is_file():
            loc_df = pd.read_csv(loc)
            origin = {
                "lat_deg": float(loc_df["latitude"].iloc[0]),
                "lon_deg": float(loc_df["longitude"].iloc[0]),
                "alt_m": float(loc_df["altitude"].iloc[0]),
            }
        else:
            origin = dict(SIM_ORIGIN)

    tracks = [
        track_from_ned_df(
            df,
            origin=origin,
            track_id="ekf_replay",
            role="estimate",
            t_col="timestamp_s",
            n_col="pos_n_m",
            e_col="pos_e_m",
            d_col="pos_d_m",
            yaw_col="yaw_deg",
            yaw_in_deg=True,
            max_points=max_points,
        )
    ]
    series: dict[str, list] = {
        "nis": series_from_col(df, "timestamp_s", "nis", max_points),
    }
    events: list[dict] = []
    sources = [str(replay.relative_to(REPO)).replace("\\", "/")]

    if loc.is_file():
        tracks.append(
            track_from_location_csv(
                loc, origin=origin, track_id="gnss_phone", role="truth", max_points=max_points
            )
        )
        sources.append(str(loc.relative_to(REPO)).replace("\\", "/"))

    if constr.is_file():
        cdf = pd.read_csv(constr)
        series["nhc_applied"] = series_from_col(cdf, "timestamp_s", "nhc_applied", max_points)
        series["zupt_applied"] = series_from_col(cdf, "timestamp_s", "zupt_applied", max_points)
        events.extend(events_from_constraints(cdf))
        sources.append(str(constr.relative_to(REPO)).replace("\\", "/"))
    if gnss.is_file():
        gdf = pd.read_csv(gnss)
        events.extend(events_from_gnss_audit(gdf))
        sources.append(str(gnss.relative_to(REPO)).replace("\\", "/"))

    # External diagnostic: spatial residual estimate vs truth (pipeline-side)
    est = next((t for t in tracks if t.get("role") == "estimate"), None)
    tru = next((t for t in tracks if t.get("role") == "truth"), None)
    if est and tru:
        series["residual_m"] = residual_horizontal_m(est, tru, max_points=max_points)
    ar = accept_reject_series(events)
    if ar:
        series["accept_reject"] = ar

    return build_session(
        name=name,
        origin=origin,
        tracks=tracks,
        events=events,
        series=series,
        provenance={
            "sources": sources,
            "kind": "real_run_replay_bundle",
            "baseline_dir": str(baseline_dir.relative_to(REPO)).replace("\\", "/"),
            "notes": (
                "tracks: ekf_replay=estimate, gnss_phone=truth; "
                "diagnostic residual_m = ||est−truth|| horizontal"
            ),
        },
    )


def write_run_package(session: dict, out_dir: Path) -> Path:
    """Write RunPackage layout (tracks/ events / observables|diagnostics / metadata).

    Explorer-facing contract: docs/ekf_explorer/RUN_PACKAGE.md
    Visor must not need EKF knowledge — only temporal layers.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    tracks_dir = out_dir / "tracks"
    obs_dir = out_dir / "observables"
    diag_dir = out_dir / "diagnostics"
    tracks_dir.mkdir(exist_ok=True)
    obs_dir.mkdir(exist_ok=True)
    diag_dir.mkdir(exist_ok=True)

    track_meta = []
    for tr in session.get("tracks") or []:
        tid = tr.get("id") or "track"
        role = tr.get("role") or "estimate"
        rel = f"tracks/{tid}.csv"
        rows = []
        for s in tr.get("samples") or []:
            rows.append(
                {
                    "t_s": s["t_s"],
                    "lat_deg": s["lat_deg"],
                    "lon_deg": s["lon_deg"],
                    "alt_m": s["alt_m"],
                    "n_m": s.get("n_m", 0.0),
                    "e_m": s.get("e_m", 0.0),
                    "d_m": s.get("d_m", 0.0),
                    "yaw_rad": s.get("yaw_rad", 0.0),
                    "role": role,
                }
            )
        pd.DataFrame(rows).to_csv(out_dir / rel, index=False)
        track_meta.append({"id": tid, "role": role, "file": rel})

    ev_rows = session.get("events") or []
    if not ev_rows:
        (out_dir / "events.csv").write_text("t_s,type,label,track_id\n", encoding="utf-8")
    else:
        pd.DataFrame(ev_rows).to_csv(out_dir / "events.csv", index=False)

    series = session.get("series") or {}
    series_meta = session.get("series_meta") or build_series_meta(series)
    by_name = {m["name"]: m for m in series_meta}

    layer_rows = []
    for name, pts in series.items():
        sm = by_name.get(name) or {
            "name": name,
            "kind": "observable",
            "label": name,
            "color_vmin": 0.0,
            "color_vmax": _auto_color_vmax(pts),
            "vmin": 0.0,
            "vmax": _auto_color_vmax(pts),
            **_value_stats(pts),
            "colormap": "diverging",
        }
        folder = "diagnostics" if sm.get("kind") == "diagnostic" else "observables"
        rel = f"{folder}/{name}.csv"
        if not pts:
            (out_dir / rel).write_text("t_s,v\n", encoding="utf-8")
        else:
            pd.DataFrame(pts).to_csv(out_dir / rel, index=False)
        layer_rows.append({**sm, "file": rel})

    layer_rows.sort(key=lambda m: (0 if m.get("kind") == "observable" else 1, m.get("label") or m["name"]))
    default_obs = next((m["name"] for m in layer_rows if m["name"] == "nis"), None)
    if default_obs is None and layer_rows:
        default_obs = layer_rows[0]["name"]

    meta = {
        "schema": "navicore.ekf_explorer.run_package/v1",
        "run_id": session.get("name"),
        "origin": session.get("origin"),
        "t_range_s": session.get("t_range_s"),
        "tracks": track_meta,
        "events_file": "events.csv",
        "series_meta": layer_rows,
        "observables": [m for m in layer_rows if m.get("kind") != "diagnostic"],
        "diagnostics": [m for m in layer_rows if m.get("kind") == "diagnostic"],
        "provenance": session.get("provenance") or {},
        "ui_hints": {
            "default_tracks": ["estimate", "truth"],
            "default_observable": default_obs,
        },
        "protocol": "docs/ekf_explorer/RUN_PACKAGE.md",
        "exported_at": session.get("exported_at"),
    }
    meta_path = out_dir / "metadata.json"
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    return meta_path


def write_session(session: dict, out_dir: Path) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "session.json"
    czml_path = out_dir / "session.czml"
    json_path.write_text(json.dumps(session, indent=2), encoding="utf-8")
    czml_path.write_text(json.dumps(to_czml(session), indent=2), encoding="utf-8")
    write_run_package(session, out_dir)
    return json_path, czml_path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--preset",
        choices=("slalom", "tunnel", "real_run", "all"),
        default="all",
    )
    ap.add_argument(
        "--bundle-dir",
        type=Path,
        default=None,
        help="Export one real_run bundle (replay_output + audits) instead of presets",
    )
    ap.add_argument("--name", type=str, default=None, help="Session pack name (with --bundle-dir)")
    ap.add_argument(
        "--location-csv",
        type=Path,
        default=None,
        help="Phone Location.csv for GNSS truth track / origin (with --bundle-dir)",
    )
    ap.add_argument("--max-points", type=int, default=8000)
    args = ap.parse_args()

    if args.bundle_dir is not None:
        name = args.name or args.bundle_dir.name
        session = export_real_run_baseline(
            args.bundle_dir.resolve(),
            name=name,
            max_points=args.max_points,
            location_csv=args.location_csv.resolve() if args.location_csv else None,
        )
        out = OUT_ROOT / name
        jp, cp = write_session(session, out)
        n_tr = sum(len(t["samples"]) for t in session["tracks"])
        print(
            f"OK {name}: tracks_samples={n_tr} events={len(session['events'])} "
            f"series={list(session['series'])} -> {jp.relative_to(REPO)} + {cp.name}"
        )
        return 0

    jobs = []
    if args.preset in ("slalom", "all"):
        jobs.append(
            (
                "slalom_default",
                lambda: export_sim_telemetry(
                    REPO / "docs" / "benchmarks" / "slalom_telemetry.csv",
                    name="slalom_default",
                    max_points=args.max_points,
                ),
            )
        )
    if args.preset in ("tunnel", "all"):
        tun = REPO / "docs" / "benchmarks" / "tunnel_stress_telemetry.csv"
        if tun.is_file():
            jobs.append(
                (
                    "tunnel_stress_default",
                    lambda: export_sim_telemetry(
                        tun, name="tunnel_stress_default", max_points=args.max_points
                    ),
                )
            )
    if args.preset in ("real_run", "all"):
        base = REPO / "docs" / "benchmarks" / "real_run_19082026_baseline"
        if (base / "replay_output.csv").is_file():
            jobs.append(
                (
                    "real_run_19082026_baseline",
                    lambda: export_real_run_baseline(
                        base, name="real_run_19082026_baseline", max_points=args.max_points
                    ),
                )
            )

    if not jobs:
        print("No sources found for preset", args.preset)
        return 1

    for name, fn in jobs:
        session = fn()
        out = OUT_ROOT / name
        jp, cp = write_session(session, out)
        n_tr = sum(len(t["samples"]) for t in session["tracks"])
        print(
            f"OK {name}: tracks_samples={n_tr} events={len(session['events'])} "
            f"series={list(session['series'])} -> {jp.relative_to(REPO)} + {cp.name}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
