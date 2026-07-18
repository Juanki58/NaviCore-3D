#!/usr/bin/env python3
"""Descompone a_nav_pre_g (H9d) en ejes vehiculo: longitudinal, lateral, vertical."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
H9D_CSV = REPO_ROOT / "docs" / "benchmarks" / "h9d_gravity_subtraction.csv"
LOCATION_CSV = REPO_ROOT / "data" / "real_run" / "Location.csv"
OUT_JSON = REPO_ROOT / "docs" / "benchmarks" / "h9d_vehicle_frame_decomposition.json"


def load_gps_bearing(path: Path) -> tuple[np.ndarray, np.ndarray]:
    times: list[float] = []
    bearings: list[float] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        t0: float | None = None
        for row in reader:
            if not row.get("bearing"):
                continue
            if row.get("seconds_elapsed"):
                t = float(row["seconds_elapsed"])
            elif row.get("time"):
                ns = float(row["time"])
                if t0 is None:
                    t0 = ns
                t = (ns - t0) * 1e-9
            else:
                continue
            times.append(t)
            bearings.append(float(row["bearing"]))
    return np.array(times, dtype=float), np.array(bearings, dtype=float)


def interp_bearing(t: float, gps_times: np.ndarray, bearings: np.ndarray) -> float | None:
    if gps_times.size == 0:
        return None
    if t < gps_times[0] or t > gps_times[-1]:
        return None
    return float(np.interp(t, gps_times, bearings))


def ned_to_vehicle(an: float, ae: float, ad: float, heading_deg: float) -> tuple[float, float, float]:
    """Heading: azimuth from North, clockwise (GPS bearing convention)."""
    h = math.radians(heading_deg)
    c, s = math.cos(h), math.sin(h)
    along = an * c + ae * s
    lat = -an * s + ae * c
    return along, lat, ad


def summarize(label: str, along: np.ndarray, lat: np.ndarray, vert: np.ndarray, alin_h: np.ndarray) -> dict:
    return {
        "label": label,
        "n": int(along.size),
        "along_mean_mps2": float(np.mean(along)),
        "along_std_mps2": float(np.std(along)),
        "along_abs_mean_mps2": float(np.mean(np.abs(along))),
        "lateral_mean_mps2": float(np.mean(lat)),
        "lateral_std_mps2": float(np.std(lat)),
        "lateral_abs_mean_mps2": float(np.mean(np.abs(lat))),
        "vertical_mean_mps2": float(np.mean(vert)),
        "vertical_std_mps2": float(np.std(vert)),
        "a_lin_h_mean_mps2": float(np.mean(alin_h)),
    }


def main() -> int:
    if not H9D_CSV.is_file():
        raise FileNotFoundError(H9D_CSV)

    gps_times = np.array([], dtype=float)
    gps_bearings = np.array([], dtype=float)
    if LOCATION_CSV.is_file():
        gps_times, gps_bearings = load_gps_bearing(LOCATION_CSV)

    ekf_rows: list[dict] = []
    gps_rows: list[dict] = []

    with H9D_CSV.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t = float(raw["timestamp_s"])
            an = float(raw["a_nav_pre_g_n"])
            ae = float(raw["a_nav_pre_g_e"])
            ad = float(raw["a_nav_pre_g_d"])
            alin_h = float(raw["a_lin_h"])
            yaw = float(raw["yaw_deg"])

            along_e, lat_e, vert = ned_to_vehicle(an, ae, ad, yaw)
            ekf_rows.append(
                {"t": t, "along": along_e, "lat": lat_e, "vert": vert, "alin_h": alin_h}
            )

            bearing = interp_bearing(t, gps_times, gps_bearings)
            if bearing is not None:
                along_g, lat_g, _ = ned_to_vehicle(an, ae, ad, bearing)
                gps_rows.append(
                    {"t": t, "along": along_g, "lat": lat_g, "vert": vert, "alin_h": alin_h}
                )

    def window(rows: list[dict], t0: float, t1: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        subset = [r for r in rows if t0 <= r["t"] <= t1]
        if not subset:
            return np.array([]), np.array([]), np.array([]), np.array([])
        return (
            np.array([r["along"] for r in subset]),
            np.array([r["lat"] for r in subset]),
            np.array([r["vert"] for r in subset]),
            np.array([r["alin_h"] for r in subset]),
        )

    windows = [
        ("static_0_2s", 0.0, 2.0),
        ("motion_2_10s", 2.0, 10.0),
        ("cruise_11_25s", 11.4, 25.4),
        ("full_0_60s", 0.0, 60.0),
    ]

    report = {
        "source_csv": str(H9D_CSV),
        "signal": "a_nav_pre_g = R_bn * a_corr (before gravity subtraction)",
        "frames": {},
    }

    for frame_name, rows in [("vehicle_heading_ekf_yaw", ekf_rows), ("vehicle_heading_gps_bearing", gps_rows)]:
        frame_out: dict = {}
        for label, t0, t1 in windows:
            a, l, v, h = window(rows, t0, t1)
            if a.size:
                frame_out[label] = summarize(label, a, l, v, h)
        if rows:
            motion = [r for r in rows if 2.0 < r["t"] <= 10.0]
            if motion:
                lat = np.array([r["lat"] for r in motion])
                along = np.array([r["along"] for r in motion])
                ah = np.array([r["alin_h"] for r in motion])
                frame_out["correlations_motion_2_10s"] = {
                    "corr_lateral_vs_a_lin_h": float(np.corrcoef(lat, ah)[0, 1]),
                    "corr_longitudinal_vs_a_lin_h": float(np.corrcoef(along, ah)[0, 1]),
                    "corr_abs_lateral_vs_a_lin_h": float(np.corrcoef(np.abs(lat), ah)[0, 1]),
                }
        report["frames"][frame_name] = frame_out

    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with OUT_JSON.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")

    print("=" * 72)
    print(" a_nav_pre_g en marco vehiculo (H9d)")
    print("=" * 72)
    for frame_name in report["frames"]:
        print(f"\n Referencia heading: {frame_name}")
        for label, t0, t1 in windows:
            stats = report["frames"][frame_name].get(label)
            if not stats:
                continue
            print(f"  [{label}]")
            print(
                f"    longitudinal: {stats['along_mean_mps2']:+.4f} m/s2  "
                f"|abs|={stats['along_abs_mean_mps2']:.4f}"
            )
            print(
                f"    lateral:      {stats['lateral_mean_mps2']:+.4f} m/s2  "
                f"|abs|={stats['lateral_abs_mean_mps2']:.4f}"
            )
            print(f"    vertical(D):  {stats['vertical_mean_mps2']:+.4f} m/s2")
            print(f"    a_lin_h:      {stats['a_lin_h_mean_mps2']:.4f} m/s2")
        corr = report["frames"][frame_name].get("correlations_motion_2_10s")
        if corr:
            print(
                f"  corr(lateral, a_lin_h) [2-10s]: {corr['corr_lateral_vs_a_lin_h']:.3f}  "
                f"corr(along, a_lin_h): {corr['corr_longitudinal_vs_a_lin_h']:.3f}"
            )
    print(f"\n JSON: {OUT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
