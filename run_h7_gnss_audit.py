#!/usr/bin/env python3
"""H7 - Auditoria geometrica GNSS e inspeccion del primer fix en movimiento.

Parte 1: verifica que el replay CSV usa la misma geodesia WGS84 ECEF->NED que Location.csv.
Parte 2: inspeccion forense del primer update GPS en movimiento (t > 5 s, v > 1 m/s).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_LOCATION = REPO_ROOT / "data" / "real_run" / "Location.csv"
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DEFAULT_REPLAY = BENCH_DIR / "real_run_replay.csv"
DEFAULT_CONSISTENCY = BENCH_DIR / "h4_consistency.csv"
DEFAULT_DIAGNOSTICS = BENCH_DIR / "h4_consistency_diagnostics.csv"
DEFAULT_GNSS_AUDIT = BENCH_DIR / "gnss_innovation_audit.csv"
DEFAULT_H7_UPDATE = BENCH_DIR / "h7_update_audit.csv"
GEO_AUDIT_CSV = BENCH_DIR / "h7_gnss_geo_audit.csv"
REPORT_JSON = BENCH_DIR / "h7_gnss_audit_report.json"
PLOT_PATH = BENCH_DIR / "h7_gnss_audit_error.png"

GEO_ALERT_THRESHOLD_M = 0.1
MOVING_MIN_TIME_S = 5.0
MOVING_MIN_SPEED_MPS = 1.0
TIMESTAMP_MATCH_EPS_S = 0.002

from geodesy import LLA, lla, lla_to_ned  # noqa: E402
from parse_mobile_log import load_location_csv  # noqa: E402
from analyze_real_run import resolve_replay_path  # noqa: E402


@dataclass
class GeoAuditRow:
    timestamp_s: float
    lat_deg: float
    lon_deg: float
    alt_m: float
    ned_strict_n: float
    ned_strict_e: float
    ned_strict_d: float
    parser_n: float
    parser_e: float
    parser_d: float
    delta_n_m: float
    delta_e_m: float
    delta_d_m: float
    delta_horiz_m: float


@dataclass
class MovingUpdateInspection:
    source_file: str
    timestamp_s: float
    speed_mps: float
    ekf_n_m: float
    ekf_e_m: float
    gps_n_m: float
    gps_e_m: float
    innov_n_m: float
    innov_e_m: float
    s_nn: float
    s_ee: float
    s_dd: float | None
    nis: float
    gnss_accepted: bool | None


def parse_float(value: str | None) -> float | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        out = float(text)
    except ValueError:
        return None
    if not math.isfinite(out):
        return None
    return out


def geodesy_ned_from_fix(ref: LLA, fix_lat: float, fix_lon: float, fix_alt: float) -> tuple[float, float, float]:
    ned = lla_to_ned(lla(fix_lat, fix_lon, fix_alt), ref)
    return ned.north_m, ned.east_m, ned.down_m


def load_replay_gps_rows(replay_csv: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with replay_csv.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            if raw.get("type") != "GPS":
                continue
            t = parse_float(raw.get("timestamp_s"))
            pn = parse_float(raw.get("pos_n"))
            pe = parse_float(raw.get("pos_e"))
            pd = parse_float(raw.get("pos_d"))
            speed = parse_float(raw.get("speed"))
            if t is None or pn is None or pe is None or pd is None:
                continue
            rows.append(
                {
                    "timestamp_s": t,
                    "pos_n": pn,
                    "pos_e": pe,
                    "pos_d": pd,
                    "speed": speed if speed is not None else 0.0,
                }
            )
    rows.sort(key=lambda item: item["timestamp_s"])
    return rows


def find_replay_gps_match(
    replay_rows: list[dict[str, float]],
    timestamp_s: float,
) -> dict[str, float] | None:
    best: dict[str, float] | None = None
    best_dt = TIMESTAMP_MATCH_EPS_S + 1.0
    for row in replay_rows:
        dt = abs(row["timestamp_s"] - timestamp_s)
        if dt <= TIMESTAMP_MATCH_EPS_S and dt < best_dt:
            best = row
            best_dt = dt
    return best


def build_geo_audit(
    location_csv: Path,
    replay_csv: Path,
) -> tuple[list[GeoAuditRow], tuple[float, float, float]]:
    locations = load_location_csv(location_csv)
    if not locations:
        raise ValueError(f"{location_csv}: sin fixes GNSS validos")

    first_fix = locations[0]
    ref_lat = first_fix.latitude
    ref_lon = first_fix.longitude
    ref_alt = first_fix.altitude
    t0_ns = first_fix.time_ns
    ref_lla = lla(ref_lat, ref_lon, ref_alt)

    replay_gps = load_replay_gps_rows(replay_csv)
    if not replay_gps:
        raise ValueError(f"{replay_csv}: sin filas GPS")

    audit_rows: list[GeoAuditRow] = []
    for fix in locations:
        timestamp_s = (fix.time_ns - t0_ns) / 1e9
        strict_n, strict_e, strict_d = geodesy_ned_from_fix(
            ref_lla,
            fix.latitude,
            fix.longitude,
            fix.altitude,
        )

        replay_match = find_replay_gps_match(replay_gps, timestamp_s)
        if replay_match is None:
            parser_n, parser_e, parser_d = strict_n, strict_e, strict_d
        else:
            parser_n = replay_match["pos_n"]
            parser_e = replay_match["pos_e"]
            parser_d = replay_match["pos_d"]

        delta_n = parser_n - strict_n
        delta_e = parser_e - strict_e
        delta_d = parser_d - strict_d
        audit_rows.append(
            GeoAuditRow(
                timestamp_s=timestamp_s,
                lat_deg=fix.latitude,
                lon_deg=fix.longitude,
                alt_m=fix.altitude,
                ned_strict_n=strict_n,
                ned_strict_e=strict_e,
                ned_strict_d=strict_d,
                parser_n=parser_n,
                parser_e=parser_e,
                parser_d=parser_d,
                delta_n_m=delta_n,
                delta_e_m=delta_e,
                delta_d_m=delta_d,
                delta_horiz_m=math.hypot(delta_n, delta_e),
            )
        )

    return audit_rows, (ref_lat, ref_lon, ref_alt)


def write_geo_audit_csv(rows: list[GeoAuditRow], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "timestamp_s",
                "lat_deg",
                "lon_deg",
                "alt_m",
                "ned_strict_n",
                "ned_strict_e",
                "ned_strict_d",
                "parser_n",
                "parser_e",
                "parser_d",
                "delta_n_m",
                "delta_e_m",
                "delta_d_m",
                "delta_horiz_m",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def summarize_geo(rows: list[GeoAuditRow]) -> dict[str, float]:
    delta_n = np.array([r.delta_n_m for r in rows], dtype=float)
    delta_e = np.array([r.delta_e_m for r in rows], dtype=float)
    delta_d = np.array([r.delta_d_m for r in rows], dtype=float)
    delta_h = np.array([r.delta_horiz_m for r in rows], dtype=float)
    return {
        "samples": float(len(rows)),
        "delta_n_max_m": float(np.max(np.abs(delta_n))) if delta_n.size else float("nan"),
        "delta_e_max_m": float(np.max(np.abs(delta_e))) if delta_e.size else float("nan"),
        "delta_d_max_m": float(np.max(np.abs(delta_d))) if delta_d.size else float("nan"),
        "delta_horiz_max_m": float(np.max(delta_h)) if delta_h.size else float("nan"),
        "delta_horiz_mean_m": float(np.mean(delta_h)) if delta_h.size else float("nan"),
        "delta_horiz_rms_m": float(np.sqrt(np.mean(delta_h * delta_h))) if delta_h.size else float("nan"),
    }


def resolve_consistency_path(explicit: Path | None) -> Path | None:
    candidates: list[Path] = []
    if explicit is not None:
        candidates.append(explicit)
    candidates.extend(
        [
            DEFAULT_CONSISTENCY,
            DEFAULT_DIAGNOSTICS,
            DEFAULT_GNSS_AUDIT,
            DEFAULT_H7_UPDATE,
        ]
    )
    for path in candidates:
        if path.is_file():
            return path
    return None


def load_consistency_rows(path: Path) -> tuple[list[dict[str, str]], str]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        rows = list(reader)
    return rows, ",".join(fieldnames)


def inspect_first_moving_update(
    consistency_path: Path,
    replay_gps_rows: list[dict[str, float]],
) -> MovingUpdateInspection:
    rows, header = load_consistency_rows(consistency_path)

    if "ekf_pos_n_m" in header and "gps_pos_n_m" in header:
        return _inspect_from_gnss_audit(consistency_path.name, rows, replay_gps_rows)
    if "hx_n" in header and "gps_pos_n" in header:
        return _inspect_from_h7_update(consistency_path.name, rows, replay_gps_rows)
    if "innovation_n_m" in header or "innov_n" in header:
        return _inspect_from_h4_consistency(consistency_path.name, rows, replay_gps_rows)

    raise ValueError(f"Formato de consistencia no reconocido: {consistency_path}")


def _speed_for_timestamp(replay_gps_rows: list[dict[str, float]], timestamp_s: float) -> float:
    match = find_replay_gps_match(replay_gps_rows, timestamp_s)
    if match is None:
        return 0.0
    return match["speed"]


def _inspect_from_gnss_audit(
    source_name: str,
    rows: list[dict[str, str]],
    replay_gps_rows: list[dict[str, float]],
) -> MovingUpdateInspection:
    for raw in rows:
        t = parse_float(raw.get("timestamp_s"))
        if t is None or t <= MOVING_MIN_TIME_S:
            continue
        speed = _speed_for_timestamp(replay_gps_rows, t)
        if speed <= MOVING_MIN_SPEED_MPS:
            continue

        return MovingUpdateInspection(
            source_file=source_name,
            timestamp_s=t,
            speed_mps=speed,
            ekf_n_m=parse_float(raw.get("ekf_pos_n_m")) or 0.0,
            ekf_e_m=parse_float(raw.get("ekf_pos_e_m")) or 0.0,
            gps_n_m=parse_float(raw.get("gps_pos_n_m")) or 0.0,
            gps_e_m=parse_float(raw.get("gps_pos_e_m")) or 0.0,
            innov_n_m=parse_float(raw.get("innovation_n_m")) or 0.0,
            innov_e_m=parse_float(raw.get("innovation_e_m")) or 0.0,
            s_nn=parse_float(raw.get("S_nn")) or float("nan"),
            s_ee=parse_float(raw.get("S_ee")) or float("nan"),
            s_dd=parse_float(raw.get("S_dd")),
            nis=parse_float(raw.get("nis")) or float("nan"),
            gnss_accepted=bool(int(parse_float(raw.get("gnss_accepted")) or 0)),
        )
    raise ValueError("No se encontro update GPS en movimiento en gnss_innovation_audit")


def _inspect_from_h7_update(
    source_name: str,
    rows: list[dict[str, str]],
    replay_gps_rows: list[dict[str, float]],
) -> MovingUpdateInspection:
    for raw in rows:
        t = parse_float(raw.get("timestamp_s"))
        if t is None or t <= MOVING_MIN_TIME_S:
            continue
        speed = _speed_for_timestamp(replay_gps_rows, t)
        if speed <= MOVING_MIN_SPEED_MPS:
            continue

        return MovingUpdateInspection(
            source_file=source_name,
            timestamp_s=t,
            speed_mps=speed,
            ekf_n_m=parse_float(raw.get("hx_n")) or 0.0,
            ekf_e_m=parse_float(raw.get("hx_e")) or 0.0,
            gps_n_m=parse_float(raw.get("gps_pos_n")) or 0.0,
            gps_e_m=parse_float(raw.get("gps_pos_e")) or 0.0,
            innov_n_m=parse_float(raw.get("innov_n")) or 0.0,
            innov_e_m=parse_float(raw.get("innov_e")) or 0.0,
            s_nn=parse_float(raw.get("S_nn")) or float("nan"),
            s_ee=parse_float(raw.get("S_ee")) or float("nan"),
            s_dd=parse_float(raw.get("S_dd")),
            nis=parse_float(raw.get("nis")) or float("nan"),
            gnss_accepted=bool(int(parse_float(raw.get("gnss_accepted")) or 0)),
        )
    raise ValueError("No se encontro update GPS en movimiento en h7_update_audit")


def _inspect_from_h4_consistency(
    source_name: str,
    rows: list[dict[str, str]],
    replay_gps_rows: list[dict[str, float]],
) -> MovingUpdateInspection:
    innov_n_key = "innovation_n_m" if "innovation_n_m" in (rows[0].keys() if rows else {}) else None
    innov_e_key = "innovation_e_m" if "innovation_e_m" in (rows[0].keys() if rows else {}) else None
    if innov_n_key is None:
        raise ValueError("h4_consistency sin columnas de innovacion")

    for raw in rows:
        t = parse_float(raw.get("timestamp_s"))
        if t is None or t <= MOVING_MIN_TIME_S:
            continue
        speed = _speed_for_timestamp(replay_gps_rows, t)
        if speed <= MOVING_MIN_SPEED_MPS:
            continue

        replay_match = find_replay_gps_match(replay_gps_rows, t)
        if replay_match is None:
            continue

        innov_n = parse_float(raw.get(innov_n_key)) or 0.0
        innov_e = parse_float(raw.get(innov_e_key)) or 0.0
        gps_n = replay_match["pos_n"]
        gps_e = replay_match["pos_e"]
        ekf_n = gps_n - innov_n
        ekf_e = gps_e - innov_e

        return MovingUpdateInspection(
            source_file=source_name,
            timestamp_s=t,
            speed_mps=speed,
            ekf_n_m=ekf_n,
            ekf_e_m=ekf_e,
            gps_n_m=gps_n,
            gps_e_m=gps_e,
            innov_n_m=innov_n,
            innov_e_m=innov_e,
            s_nn=parse_float(raw.get("S_nn")) or float("nan"),
            s_ee=parse_float(raw.get("S_ee")) or float("nan"),
            s_dd=parse_float(raw.get("S_dd")),
            nis=parse_float(raw.get("nis")) or float("nan"),
            gnss_accepted=bool(int(parse_float(raw.get("gnss_accepted")) or 0))
            if raw.get("gnss_accepted") is not None
            else None,
        )
    raise ValueError("No se encontro update GPS en movimiento en h4_consistency")


def plot_geo_error(rows: list[GeoAuditRow], plot_path: Path) -> None:
    times = np.array([r.timestamp_s for r in rows], dtype=float)
    delta_n_cm = np.array([r.delta_n_m * 100.0 for r in rows], dtype=float)
    delta_e_cm = np.array([r.delta_e_m * 100.0 for r in rows], dtype=float)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times, delta_n_cm, label="Delta N (replay - geodesy)", linewidth=1.0)
    ax.plot(times, delta_e_cm, label="Delta E (replay - geodesy)", linewidth=1.0, alpha=0.85)
    ax.axhline(GEO_ALERT_THRESHOLD_M * 100.0, color="#c0392b", linestyle="--", linewidth=1.0, label="Umbral 10 cm")
    ax.axhline(-GEO_ALERT_THRESHOLD_M * 100.0, color="#c0392b", linestyle="--", linewidth=1.0)
    ax.set_title("H7 - Replay vs geodesy WGS84 ECEF->NED (cadena unificada)")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Discrepancia (cm)")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=9)
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def print_geo_alert(rows: list[GeoAuditRow], stats: dict[str, float]) -> None:
    if stats["delta_horiz_max_m"] <= GEO_ALERT_THRESHOLD_M:
        print(
            f"OK geometria: delta horizontal maxima {stats['delta_horiz_max_m']*100:.2f} cm "
            f"(<= {GEO_ALERT_THRESHOLD_M*100:.0f} cm)"
        )
        return

    worst = max(rows, key=lambda row: row.delta_horiz_m)
    print("=" * 78)
    print(" ALERTA H7 - DISCREPANCIA GEODESICA > 10 cm")
    print("=" * 78)
    print(
        f"  Delta horizontal maxima: {stats['delta_horiz_max_m']:.3f} m "
        f"({stats['delta_horiz_max_m']*100:.1f} cm)"
    )
    print(
        f"  Delta N max |abs|: {stats['delta_n_max_m']:.3f} m   "
        f"Delta E max |abs|: {stats['delta_e_max_m']:.3f} m   "
        f"Delta D max |abs|: {stats['delta_d_max_m']:.3f} m"
    )
    print(
        f"  Peor fix: t={worst.timestamp_s:.3f} s  "
        f"lat={worst.lat_deg:.7f} lon={worst.lon_deg:.7f} alt={worst.alt_m:.1f} m"
    )
    print(
        f"    parser NED=({worst.parser_n:.3f}, {worst.parser_e:.3f}, {worst.parser_d:.3f}) m"
    )
    print(
        f"    strict NED=({worst.ned_strict_n:.3f}, {worst.ned_strict_e:.3f}, {worst.ned_strict_d:.3f}) m"
    )
    print(
        f"    delta=({worst.delta_n_m:.3f}, {worst.delta_e_m:.3f}, {worst.delta_d_m:.3f}) m  "
        f"|H|={worst.delta_horiz_m:.3f} m"
    )
    print("=" * 78)


def print_moving_update_inspection(item: MovingUpdateInspection) -> None:
    print("=" * 78)
    print(" H7 - INSPECCION FORENSE: PRIMER UPDATE GPS EN MOVIMIENTO")
    print("=" * 78)
    print(f"  Fuente:           {item.source_file}")
    print(f"  timestamp_s (t):  {item.timestamp_s:.9f}")
    print(f"  velocidad GPS:    {item.speed_mps:.3f} m/s  (filtro: t>{MOVING_MIN_TIME_S:.0f}s, v>{MOVING_MIN_SPEED_MPS:.1f} m/s)")
    print("-" * 78)
    print(f"  EKF antes update: N={item.ekf_n_m:+.6f} m   E={item.ekf_e_m:+.6f} m")
    print(f"  GPS reportado:    N={item.gps_n_m:+.6f} m   E={item.gps_e_m:+.6f} m")
    print(f"  Innovacion:       innov_n={item.innov_n_m:+.6f} m   innov_e={item.innov_e_m:+.6f} m")
    print(
        f"  Covarianza S:     S_nn={item.s_nn:.6f}   S_ee={item.s_ee:.6f}"
        + (f"   S_dd={item.s_dd:.6f}" if item.s_dd is not None else "")
    )
    print(f"  NIS:              {item.nis:.6f}")
    if item.gnss_accepted is not None:
        print(f"  GNSS accepted:    {item.gnss_accepted}")
    print("=" * 78)


def main() -> int:
    parser = argparse.ArgumentParser(description="H7 GNSS geometric audit")
    parser.add_argument("--location", type=Path, default=DEFAULT_LOCATION)
    parser.add_argument("--replay", type=Path, default=None)
    parser.add_argument("--consistency", type=Path, default=None)
    args = parser.parse_args()

    try:
        replay_csv = resolve_replay_path(args.replay)
        if not args.location.is_file():
            raise FileNotFoundError(f"No existe Location.csv: {args.location}")

        BENCH_DIR.mkdir(parents=True, exist_ok=True)

        geo_rows, parse_ref = build_geo_audit(args.location, replay_csv)
        geo_stats = summarize_geo(geo_rows)
        write_geo_audit_csv(geo_rows, GEO_AUDIT_CSV)
        plot_geo_error(geo_rows, PLOT_PATH)

        consistency_path = resolve_consistency_path(args.consistency)
        if consistency_path is None:
            raise FileNotFoundError(
                "No se encontro CSV de consistencia H4/H2. "
                "Ejecuta replay con --consistency-csv o --gnss-audit-csv."
            )

        replay_gps_rows = load_replay_gps_rows(replay_csv)
        moving_update = inspect_first_moving_update(consistency_path, replay_gps_rows)

        payload = {
            "experiment": "H7_gnss_audit",
            "parse_origin_deg": {
                "lat": parse_ref[0],
                "lon": parse_ref[1],
                "alt_m": parse_ref[2],
            },
            "geo_audit": geo_stats,
            "geo_alert_threshold_m": GEO_ALERT_THRESHOLD_M,
            "geo_alert_triggered": geo_stats["delta_horiz_max_m"] > GEO_ALERT_THRESHOLD_M,
            "geodesy_model": "WGS84 ECEF->NED (geodesy.hpp / geodesy.py)",
            "consistency_source": str(consistency_path),
            "first_moving_update": asdict(moving_update),
            "artifacts": {
                "geo_audit_csv": str(GEO_AUDIT_CSV),
                "plot_png": str(PLOT_PATH),
            },
        }
        with REPORT_JSON.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
            handle.write("\n")

        print("=" * 78)
        print(" H7 - AUDITORIA GEOMETRICA GNSS")
        print("=" * 78)
        print(
            f"  Origen (1er fix): lat={parse_ref[0]:.7f} lon={parse_ref[1]:.7f} alt={parse_ref[2]:.1f} m"
        )
        print(f"  Fixes auditados: {int(geo_stats['samples'])}")
        print(
            f"  Delta horiz max={geo_stats['delta_horiz_max_m']*100:.2f} cm  "
            f"mean={geo_stats['delta_horiz_mean_m']*100:.2f} cm  "
            f"RMS={geo_stats['delta_horiz_rms_m']*100:.2f} cm"
        )
        print_geo_alert(geo_rows, geo_stats)
        print("-" * 78)
        print_moving_update_inspection(moving_update)
        print(f"  Geo audit CSV: {GEO_AUDIT_CSV}")
        print(f"  Grafico:       {PLOT_PATH}")
        print(f"  Reporte JSON:  {REPORT_JSON}")
        return 0
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
