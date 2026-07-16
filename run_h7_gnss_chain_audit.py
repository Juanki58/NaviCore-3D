#!/usr/bin/env python3
"""H7 — Auditoria independiente de la cadena GNSS y del modelo de observacion.

Parte A: verificacion geometrica LLA->ECEF->NED vs implementacion independiente.
Parte B: descomposicion del update (z, Hx, nu, S, NIS, gate) y primera innovacion absurda.
No modifica el EKF.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
DEFAULT_LOCATION = REPO_ROOT / "data" / "real_run" / "Location.csv"
DEFAULT_REPLAY = REPO_ROOT / "docs" / "benchmarks" / "real_run_replay.csv"

BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
GEO_AUDIT_CSV = BENCH_DIR / "h7_geo_audit.csv"
UPDATE_AUDIT_CSV = BENCH_DIR / "h7_update_audit.csv"
REPORT_JSON = BENCH_DIR / "h7_gnss_chain_report.json"
ANALYSIS_PNG = BENCH_DIR / "h7_gnss_chain_analysis.png"

METERS_PER_DEG_LAT = 111_132.954
WGS84_A = 6_378_137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

# seed_from_ned_fix en ins_ekf_15_state.cpp (placeholder Barcelona)
EKF_REF_LAT_DEG = 41.3874
EKF_REF_LON_DEG = 2.1686
EKF_REF_ALT_M = 12.0

ABSURD_INNOV_H_M = 20.0
NIS_THRESHOLD = 11.345
MOVING_SPEED_MPS = 3.0

from parse_mobile_log import (  # noqa: E402
    is_valid_gnss,
    latlonalt_to_ned as navicore_latlonalt_to_ned,
    load_location_csv,
)
from analyze_real_run import resolve_replay_path  # noqa: E402


@dataclass
class GeoAuditRow:
    timestamp_s: float
    lat_deg: float
    lon_deg: float
    alt_m: float
    ned_navicore_n: float
    ned_navicore_e: float
    ned_navicore_d: float
    ned_independent_n: float
    ned_independent_e: float
    ned_independent_d: float
    delta_n_m: float
    delta_e_m: float
    delta_d_m: float
    delta_horiz_m: float
    replay_pos_n: float | None
    replay_pos_e: float | None
    replay_pos_d: float | None
    delta_replay_n_m: float | None
    delta_replay_e_m: float | None
    delta_replay_horiz_m: float | None
    z_adapter_n: float
    z_adapter_e: float
    z_adapter_d: float
    delta_z_vs_navicore_h_m: float
    ned_ekf_ref_n: float
    ned_ekf_ref_e: float
    ned_ekf_ref_d: float
    delta_parse_vs_ekf_ref_h_m: float


@dataclass
class UpdateAuditRow:
    timestamp_s: float
    gps_index: int
    z_n: float
    z_e: float
    z_d: float
    hx_n: float
    hx_e: float
    hx_d: float
    innov_n: float
    innov_e: float
    innov_d: float
    s_nn: float
    s_ee: float
    s_dd: float
    nis: float
    nis_threshold: float
    gnss_accepted: bool
    innov_h_m: float
    gps_pos_n: float
    gps_pos_e: float
    gps_pos_d: float


def parse_float(text: str | None) -> float | None:
    if text is None:
        return None
    value = text.strip()
    if not value:
        return None
    try:
        out = float(value)
    except ValueError:
        return None
    if not math.isfinite(out):
        return None
    return out


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n_radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n_radius + alt_m) * cos_lat * math.cos(lon)
    y = (n_radius + alt_m) * cos_lat * math.sin(lon)
    z = (n_radius * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_ned(
    x: float,
    y: float,
    z: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
) -> tuple[float, float, float]:
    ref_x, ref_y, ref_z = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    dx = x - ref_x
    dy = y - ref_y
    dz = z - ref_z

    lat0 = math.radians(ref_lat_deg)
    lon0 = math.radians(ref_lon_deg)
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)

    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    east = -sin_lon * dx + cos_lon * dy
    down = -cos_lat * cos_lon * dx - cos_lat * sin_lon * dy - sin_lat * dz
    return north, east, down


def independent_latlonalt_to_ned(
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
) -> tuple[float, float, float]:
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    return ecef_to_ned(x, y, z, ref_lat_deg, ref_lon_deg, ref_alt_m)


def flat_ned_to_geodetic(
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
    north_m: float,
    east_m: float,
    down_m: float,
) -> tuple[float, float, float]:
    lat_rad = math.radians(ref_lat_deg)
    cos_lat = math.cos(lat_rad)
    lat_deg = ref_lat_deg + (north_m / METERS_PER_DEG_LAT)
    if abs(cos_lat) > 1.0e-6:
        lon_deg = ref_lon_deg + (east_m / (METERS_PER_DEG_LAT * cos_lat))
    else:
        lon_deg = ref_lon_deg
    alt_m = ref_alt_m - down_m
    return lat_deg, lon_deg, alt_m


def simulate_adapter_z(
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
    pos_n: float,
    pos_e: float,
    pos_d: float,
) -> tuple[float, float, float]:
    lat, lon, alt = flat_ned_to_geodetic(ref_lat_deg, ref_lon_deg, ref_alt_m, pos_n, pos_e, pos_d)
    return navicore_latlonalt_to_ned(ref_lat_deg, ref_lon_deg, ref_alt_m, lat, lon, alt)


def load_replay_gps_rows(path: Path) -> dict[float, tuple[float, float, float]]:
    lookup: dict[float, tuple[float, float, float]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("type") != "GPS":
                continue
            t = parse_float(row.get("timestamp_s"))
            pn = parse_float(row.get("pos_n"))
            pe = parse_float(row.get("pos_e"))
            pd = parse_float(row.get("pos_d"))
            if t is None or pn is None or pe is None or pd is None:
                continue
            lookup[round(t, 6)] = (pn, pe, pd)
    return lookup


def build_geo_audit(
    location_csv: Path,
    replay_csv: Path,
    parse_ref: tuple[float, float, float],
    t0_ns: int,
) -> list[GeoAuditRow]:
    locations = load_location_csv(location_csv)
    if not locations:
        raise ValueError("Location.csv sin fixes validos")

    ref_lat, ref_lon, ref_alt = parse_ref
    t0_ns = locations[0].time_ns
    replay_gps = load_replay_gps_rows(replay_csv)

    rows: list[GeoAuditRow] = []
    for fix in locations:
        if not is_valid_gnss(fix.latitude, fix.longitude, fix.altitude):
            continue

        timestamp_s = (fix.time_ns - t0_ns) * 1e-9
        nav_n, nav_e, nav_d = navicore_latlonalt_to_ned(
            ref_lat, ref_lon, ref_alt, fix.latitude, fix.longitude, fix.altitude
        )
        ind_n, ind_e, ind_d = independent_latlonalt_to_ned(
            ref_lat, ref_lon, ref_alt, fix.latitude, fix.longitude, fix.altitude
        )
        ekf_n, ekf_e, ekf_d = navicore_latlonalt_to_ned(
            EKF_REF_LAT_DEG,
            EKF_REF_LON_DEG,
            EKF_REF_ALT_M,
            fix.latitude,
            fix.longitude,
            fix.altitude,
        )
        delta_n = nav_n - ind_n
        delta_e = nav_e - ind_e
        delta_d = nav_d - ind_d
        delta_h = math.hypot(delta_n, delta_e)

        replay_pos = None
        best_key = min(replay_gps.keys(), key=lambda key: abs(key - timestamp_s), default=None)
        if best_key is not None and abs(best_key - timestamp_s) < 0.05:
            replay_pos = replay_gps[best_key]
        delta_replay_n = None
        delta_replay_e = None
        delta_replay_h = None
        replay_n = replay_e = replay_d = None
        if replay_pos is not None:
            replay_n, replay_e, replay_d = replay_pos
            delta_replay_n = replay_n - nav_n
            delta_replay_e = replay_e - nav_e
            delta_replay_h = math.hypot(delta_replay_n, delta_replay_e)

        z_n, z_e, z_d = simulate_adapter_z(ref_lat, ref_lon, ref_alt, nav_n, nav_e, nav_d)
        delta_z_h = math.hypot(z_n - nav_n, z_e - nav_e)
        delta_frame_h = math.hypot(ekf_n - nav_n, ekf_e - nav_e)

        rows.append(
            GeoAuditRow(
                timestamp_s=timestamp_s,
                lat_deg=fix.latitude,
                lon_deg=fix.longitude,
                alt_m=fix.altitude,
                ned_navicore_n=nav_n,
                ned_navicore_e=nav_e,
                ned_navicore_d=nav_d,
                ned_independent_n=ind_n,
                ned_independent_e=ind_e,
                ned_independent_d=ind_d,
                delta_n_m=delta_n,
                delta_e_m=delta_e,
                delta_d_m=delta_d,
                delta_horiz_m=delta_h,
                replay_pos_n=replay_n,
                replay_pos_e=replay_e,
                replay_pos_d=replay_d,
                delta_replay_n_m=delta_replay_n,
                delta_replay_e_m=delta_replay_e,
                delta_replay_horiz_m=delta_replay_h,
                z_adapter_n=z_n,
                z_adapter_e=z_e,
                z_adapter_d=z_d,
                delta_z_vs_navicore_h_m=delta_z_h,
                ned_ekf_ref_n=ekf_n,
                ned_ekf_ref_e=ekf_e,
                ned_ekf_ref_d=ekf_d,
                delta_parse_vs_ekf_ref_h_m=delta_frame_h,
            )
        )
    return rows


def write_geo_audit_csv(rows: list[GeoAuditRow], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "timestamp_s",
                "lat_deg",
                "lon_deg",
                "alt_m",
                "ned_navicore_n",
                "ned_navicore_e",
                "ned_navicore_d",
                "ned_independent_n",
                "ned_independent_e",
                "ned_independent_d",
                "delta_n_m",
                "delta_e_m",
                "delta_d_m",
                "delta_horiz_m",
                "replay_pos_n",
                "replay_pos_e",
                "replay_pos_d",
                "delta_replay_horiz_m",
                "z_adapter_n",
                "z_adapter_e",
                "z_adapter_d",
                "delta_z_vs_navicore_h_m",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    f"{row.timestamp_s:.9f}",
                    f"{row.lat_deg:.9f}",
                    f"{row.lon_deg:.9f}",
                    f"{row.alt_m:.6f}",
                    f"{row.ned_navicore_n:.6f}",
                    f"{row.ned_navicore_e:.6f}",
                    f"{row.ned_navicore_d:.6f}",
                    f"{row.ned_independent_n:.6f}",
                    f"{row.ned_independent_e:.6f}",
                    f"{row.ned_independent_d:.6f}",
                    f"{row.delta_n_m:.6f}",
                    f"{row.delta_e_m:.6f}",
                    f"{row.delta_d_m:.6f}",
                    f"{row.delta_horiz_m:.6f}",
                    "" if row.replay_pos_n is None else f"{row.replay_pos_n:.6f}",
                    "" if row.replay_pos_e is None else f"{row.replay_pos_e:.6f}",
                    "" if row.replay_pos_d is None else f"{row.replay_pos_d:.6f}",
                    "" if row.delta_replay_horiz_m is None else f"{row.delta_replay_horiz_m:.6f}",
                    f"{row.z_adapter_n:.6f}",
                    f"{row.z_adapter_e:.6f}",
                    f"{row.z_adapter_d:.6f}",
                    f"{row.delta_z_vs_navicore_h_m:.6f}",
                ]
            )


def ensure_calibration(path: Path) -> None:
    if path.is_file():
        return
    subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "audit_imu_chain.py"),
            "--export-calibration",
            str(path),
        ],
        cwd=REPO_ROOT,
        check=True,
    )


def run_h7_replay(replay_csv: Path, replay_exe: Path, calibration: Path) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")

    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(BENCH_DIR / "h7_replay_output.csv"),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--h7-update-audit-csv",
        str(UPDATE_AUDIT_CSV),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def load_update_audit(path: Path) -> list[UpdateAuditRow]:
    rows: list[UpdateAuditRow] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            t = parse_float(raw.get("timestamp_s"))
            if t is None:
                continue
            rows.append(
                UpdateAuditRow(
                    timestamp_s=t,
                    gps_index=int(parse_float(raw.get("gps_index")) or 0),
                    z_n=parse_float(raw.get("z_n")) or 0.0,
                    z_e=parse_float(raw.get("z_e")) or 0.0,
                    z_d=parse_float(raw.get("z_d")) or 0.0,
                    hx_n=parse_float(raw.get("hx_n")) or 0.0,
                    hx_e=parse_float(raw.get("hx_e")) or 0.0,
                    hx_d=parse_float(raw.get("hx_d")) or 0.0,
                    innov_n=parse_float(raw.get("innov_n")) or 0.0,
                    innov_e=parse_float(raw.get("innov_e")) or 0.0,
                    innov_d=parse_float(raw.get("innov_d")) or 0.0,
                    s_nn=parse_float(raw.get("S_nn")) or 0.0,
                    s_ee=parse_float(raw.get("S_ee")) or 0.0,
                    s_dd=parse_float(raw.get("S_dd")) or 0.0,
                    nis=parse_float(raw.get("nis")) or 0.0,
                    nis_threshold=parse_float(raw.get("nis_threshold")) or NIS_THRESHOLD,
                    gnss_accepted=bool(int(parse_float(raw.get("gnss_accepted")) or 0)),
                    innov_h_m=parse_float(raw.get("innov_h_m")) or 0.0,
                    gps_pos_n=parse_float(raw.get("gps_pos_n")) or 0.0,
                    gps_pos_e=parse_float(raw.get("gps_pos_e")) or 0.0,
                    gps_pos_d=parse_float(raw.get("gps_pos_d")) or 0.0,
                )
            )
    return rows


def origin_offset_m(parse_ref: tuple[float, float, float]) -> tuple[float, float, float]:
    ref_lat, ref_lon, ref_alt = parse_ref
    n, e, d = navicore_latlonalt_to_ned(
        EKF_REF_LAT_DEG,
        EKF_REF_LON_DEG,
        EKF_REF_ALT_M,
        ref_lat,
        ref_lon,
        ref_alt,
    )
    return n, e, d


def summarize_geo(rows: list[GeoAuditRow]) -> dict[str, float]:
    delta_h = np.array([r.delta_horiz_m for r in rows], dtype=float)
    delta_replay = np.array(
        [r.delta_replay_horiz_m for r in rows if r.delta_replay_horiz_m is not None],
        dtype=float,
    )
    delta_z = np.array([r.delta_z_vs_navicore_h_m for r in rows], dtype=float)
    delta_frame = np.array([r.delta_parse_vs_ekf_ref_h_m for r in rows], dtype=float)
    return {
        "samples": float(len(rows)),
        "nav_vs_independent_h_max_m": float(np.max(delta_h)) if delta_h.size else float("nan"),
        "nav_vs_independent_h_mean_m": float(np.mean(delta_h)) if delta_h.size else float("nan"),
        "replay_vs_nav_h_max_m": float(np.max(delta_replay)) if delta_replay.size else float("nan"),
        "adapter_roundtrip_h_max_m": float(np.max(delta_z)) if delta_z.size else float("nan"),
        "parse_vs_ekf_ref_h_max_m": float(np.max(delta_frame)) if delta_frame.size else float("nan"),
        "parse_vs_ekf_ref_h_at_origin_m": float(delta_frame[0]) if delta_frame.size else float("nan"),
    }


def find_first_absurd_update(rows: list[UpdateAuditRow]) -> UpdateAuditRow | None:
    for row in rows:
        if row.innov_h_m >= ABSURD_INNOV_H_M:
            return row
    return None


def plot_analysis(
    geo_rows: list[GeoAuditRow],
    update_rows: list[UpdateAuditRow],
    first_absurd: UpdateAuditRow | None,
    plot_path: Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("H7 — Cadena GNSS: geometria + update", fontsize=13)

    delta_h_cm = np.array([r.delta_horiz_m * 100.0 for r in geo_rows], dtype=float)
    axes[0, 0].hist(delta_h_cm, bins=40, color="#3498db", edgecolor="white")
    axes[0, 0].set_title("Navicore vs WGS84 ECEF (horizontal)")
    axes[0, 0].set_xlabel("Delta (cm)")
    axes[0, 0].set_ylabel("Recuento")
    axes[0, 0].grid(True, alpha=0.25)

    times = np.array([r.timestamp_s for r in update_rows], dtype=float)
    innov_h = np.array([r.innov_h_m for r in update_rows], dtype=float)
    axes[0, 1].plot(times, innov_h, color="#e74c3c", linewidth=1.0)
    axes[0, 1].axhline(ABSURD_INNOV_H_M, color="#7f8c8d", linestyle="--", label=f"umbral {ABSURD_INNOV_H_M:.0f} m")
    if first_absurd is not None:
        axes[0, 1].axvline(
            first_absurd.timestamp_s,
            color="#9b59b6",
            linestyle=":",
            label=f"1er absurdo t={first_absurd.timestamp_s:.1f}s",
        )
    axes[0, 1].set_title("Innovacion horizontal vs tiempo")
    axes[0, 1].set_xlabel("Tiempo (s)")
    axes[0, 1].set_ylabel("Innov H (m)")
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True, alpha=0.25)

    z_minus_gps_n = np.array([r.z_n - r.gps_pos_n for r in update_rows], dtype=float)
    z_minus_gps_e = np.array([r.z_e - r.gps_pos_e for r in update_rows], dtype=float)
    axes[1, 0].plot(times, z_minus_gps_n, label="z_n - gps_n", alpha=0.8)
    axes[1, 0].plot(times, z_minus_gps_e, label="z_e - gps_e", alpha=0.8)
    axes[1, 0].set_title("Medicion z vs posicion GPS en CSV")
    axes[1, 0].set_xlabel("Tiempo (s)")
    axes[1, 0].set_ylabel("Metros")
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.25)

    nis = np.array([r.nis for r in update_rows], dtype=float)
    accepted = np.array([1 if r.gnss_accepted else 0 for r in update_rows], dtype=float)
    axes[1, 1].plot(times, nis, color="#e67e22", label="NIS")
    axes[1, 1].axhline(NIS_THRESHOLD, color="#7f8c8d", linestyle="--", label="chi2 gate")
    axes[1, 1].step(times, accepted * np.max(nis) if nis.size else accepted, where="post", alpha=0.35, label="accepted")
    axes[1, 1].set_title("NIS y aceptacion GNSS")
    axes[1, 1].set_xlabel("Tiempo (s)")
    axes[1, 1].set_ylabel("NIS")
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True, alpha=0.25)

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="H7 GNSS chain audit")
    parser.add_argument("--skip-replay", action="store_true")
    parser.add_argument("--location", type=Path, default=DEFAULT_LOCATION)
    args = parser.parse_args()

    try:
        replay_csv = resolve_replay_path(None)
        ensure_calibration(DEFAULT_CALIBRATION)
        BENCH_DIR.mkdir(parents=True, exist_ok=True)

        locations = load_location_csv(args.location)
        if not locations:
            raise ValueError("Location.csv sin fixes validos")
        parse_ref = (locations[0].latitude, locations[0].longitude, locations[0].altitude)
        t0_ns = locations[0].time_ns
        ekf_origin_offset = origin_offset_m(parse_ref)

        geo_rows = build_geo_audit(args.location, replay_csv, parse_ref, t0_ns)
        write_geo_audit_csv(geo_rows, GEO_AUDIT_CSV)
        geo_stats = summarize_geo(geo_rows)

        if not args.skip_replay:
            run_h7_replay(replay_csv, DEFAULT_REPLAY_EXE, DEFAULT_CALIBRATION)

        if not UPDATE_AUDIT_CSV.is_file():
            raise FileNotFoundError(f"Falta {UPDATE_AUDIT_CSV}")

        update_rows = load_update_audit(UPDATE_AUDIT_CSV)
        if not update_rows:
            raise ValueError("Update audit vacio")

        first_absurd = find_first_absurd_update(update_rows)
        first_reject = next((r for r in update_rows if not r.gnss_accepted), None)
        first_accept = next((r for r in update_rows if r.gnss_accepted), None)

        plot_analysis(geo_rows, update_rows, first_absurd, ANALYSIS_PNG)

        payload = {
            "experiment": "H7_gnss_chain_audit",
            "parse_origin_deg": {
                "lat": parse_ref[0],
                "lon": parse_ref[1],
                "alt_m": parse_ref[2],
            },
            "ekf_seed_ref_deg": {
                "lat": EKF_REF_LAT_DEG,
                "lon": EKF_REF_LON_DEG,
                "alt_m": EKF_REF_ALT_M,
            },
            "ekf_ref_expressed_in_parse_ned_m": {
                "n": ekf_origin_offset[0],
                "e": ekf_origin_offset[1],
                "d": ekf_origin_offset[2],
            },
            "geo_audit": geo_stats,
            "update_audit": {
                "samples": len(update_rows),
                "absurd_innov_h_threshold_m": ABSURD_INNOV_H_M,
                "first_absurd": None
                if first_absurd is None
                else {
                    "timestamp_s": first_absurd.timestamp_s,
                    "gps_index": first_absurd.gps_index,
                    "innov_h_m": first_absurd.innov_h_m,
                    "innov_n_m": first_absurd.innov_n,
                    "innov_e_m": first_absurd.innov_e,
                    "nis": first_absurd.nis,
                    "gnss_accepted": first_absurd.gnss_accepted,
                    "z_n": first_absurd.z_n,
                    "hx_n": first_absurd.hx_n,
                    "gps_pos_n": first_absurd.gps_pos_n,
                },
                "first_reject": None
                if first_reject is None
                else {
                    "timestamp_s": first_reject.timestamp_s,
                    "gps_index": first_reject.gps_index,
                    "innov_h_m": first_reject.innov_h_m,
                    "nis": first_reject.nis,
                },
                "first_accept": None
                if first_accept is None
                else {
                    "timestamp_s": first_accept.timestamp_s,
                    "gps_index": first_accept.gps_index,
                    "innov_h_m": first_accept.innov_h_m,
                    "nis": first_accept.nis,
                },
            },
        }
        with REPORT_JSON.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
            handle.write("\n")

        print("=" * 78)
        print(" H7 - AUDITORIA CADENA GNSS")
        print("=" * 78)
        print(
            f"  Origen parse (1er fix): lat={parse_ref[0]:.7f} lon={parse_ref[1]:.7f} alt={parse_ref[2]:.1f} m"
        )
        print(
            f"  Origen EKF seed:        lat={EKF_REF_LAT_DEG:.4f} lon={EKF_REF_LON_DEG:.4f} alt={EKF_REF_ALT_M:.1f} m"
        )
        print(
            f"  Offset EKF en NED parse: N={ekf_origin_offset[0]:.1f} m  E={ekf_origin_offset[1]:.1f} m  D={ekf_origin_offset[2]:.1f} m"
        )
        print("-" * 78)
        print("  Parte A — Geometria (Navicore vs WGS84 ECEF):")
        print(
            f"    delta_h max={geo_stats['nav_vs_independent_h_max_m']*100:.2f} cm  "
            f"mean={geo_stats['nav_vs_independent_h_mean_m']*100:.2f} cm"
        )
        print(
            f"    replay vs nav max={geo_stats['replay_vs_nav_h_max_m']*100:.2f} cm  "
            f"adapter roundtrip max={geo_stats['adapter_roundtrip_h_max_m']*100:.4f} cm"
        )
        print(
            f"    parse NED vs EKF-ref NED max={geo_stats['parse_vs_ekf_ref_h_max_m']:.1f} m  "
            f"(crece con recorrido; no es error de conversion local)"
        )
        print("-" * 78)
        print("  Parte B — Update audit:")
        if first_absurd is not None:
            print(
                f"    1er innov absurda (>{ABSURD_INNOV_H_M:.0f} m): t={first_absurd.timestamp_s:.2f} s  "
                f"idx={first_absurd.gps_index}  innov_h={first_absurd.innov_h_m:.1f} m  "
                f"accepted={first_absurd.gnss_accepted}"
            )
        else:
            print(f"    Sin innovacion > {ABSURD_INNOV_H_M:.0f} m")
        if first_reject is not None:
            print(
                f"    1er rechazo NIS: t={first_reject.timestamp_s:.2f} s  "
                f"idx={first_reject.gps_index}  innov_h={first_reject.innov_h_m:.1f} m  nis={first_reject.nis:.1f}"
            )
        print("=" * 78)
        print(f"Geo CSV:    {GEO_AUDIT_CSV}")
        print(f"Update CSV: {UPDATE_AUDIT_CSV}")
        print(f"Reporte:    {REPORT_JSON}")
        print(f"Grafico:    {ANALYSIS_PNG}")
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
