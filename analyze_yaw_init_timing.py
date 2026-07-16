#!/usr/bin/env python3
"""Explica por que H2 aplico yaw tan tarde (p.ej. t=31.27 s).

Analiza el CSV replay: speed GNSS, fixes disponibles, ventana de heading.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY = REPO_ROOT / "docs" / "benchmarks" / "real_run_replay.csv"

MIN_SPEED_MPS = 3.0
MIN_SAMPLES = 20
MIN_DISPLACEMENT_M = 0.3
MAX_HEADING_STD_DEG = 5.0


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


def wrap_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def circular_std_deg(headings_deg: list[float]) -> float:
    if not headings_deg:
        return float("nan")
    rads = [math.radians(h) for h in headings_deg]
    sin_m = statistics.mean(math.sin(r) for r in rads)
    cos_m = statistics.mean(math.cos(r) for r in rads)
    resultant = math.hypot(sin_m, cos_m)
    if resultant <= 1e-9 or resultant >= 1.0:
        return 0.0
    return math.degrees(math.sqrt(-2.0 * math.log(resultant)))


def load_gps_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("type") != "GPS":
                continue
            t = parse_float(row.get("timestamp_s"))
            pn = parse_float(row.get("pos_n"))
            pe = parse_float(row.get("pos_e"))
            pd = parse_float(row.get("pos_d"))
            speed = parse_float(row.get("speed")) or 0.0
            if t is None or pn is None or pe is None or pd is None:
                continue
            rows.append({
                "t": t,
                "n": pn,
                "e": pe,
                "d": pd,
                "speed": speed,
            })
    return rows


def analyze_timing(gps_rows: list[dict[str, float]]) -> None:
    if not gps_rows:
        print("Sin filas GPS.")
        return

    print("=" * 72)
    print(" ANALISIS TIMING — H2 yaw inicial (GNSS estable)")
    print("=" * 72)
    print(f"  Fixes GNSS totales:     {len(gps_rows)}")
    print(f"  Primer fix:             t={gps_rows[0]['t']:.3f} s  speed={gps_rows[0]['speed']:.2f} m/s")
    print(f"  Ultimo fix:             t={gps_rows[-1]['t']:.3f} s  speed={gps_rows[-1]['speed']:.2f} m/s")

    moving = [r for r in gps_rows if r["speed"] >= MIN_SPEED_MPS]
    print(f"  Fixes con speed>={MIN_SPEED_MPS}: {len(moving)}")
    if moving:
        print(f"  Primer fix en marcha:   t={moving[0]['t']:.3f} s  speed={moving[0]['speed']:.2f} m/s")
    else:
        print("  -> Nunca se alcanzo speed>=3 m/s; H2 no podria aplicar yaw.")
        return

    dt_samples = [gps_rows[i]["t"] - gps_rows[i - 1]["t"] for i in range(1, len(gps_rows))]
    if dt_samples:
        print(f"  Cadencia GNSS media:    {statistics.mean(dt_samples):.2f} s")
        print(f"  Cadencia GNSS mediana:  {statistics.median(dt_samples):.2f} s")

    heading_window: list[float] = []
    ref: dict[str, float] | None = None
    apply_t: float | None = None
    apply_heading: float | None = None
    apply_std: float | None = None

    for row in gps_rows:
        if row["speed"] < MIN_SPEED_MPS:
            continue
        if ref is None:
            ref = row
            continue

        dn = row["n"] - ref["n"]
        de = row["e"] - ref["e"]
        dist = math.hypot(dn, de)
        if dist < MIN_DISPLACEMENT_M:
            continue

        heading = math.degrees(math.atan2(de, dn))
        heading_window.append(heading)
        if len(heading_window) > MIN_SAMPLES:
            heading_window.pop(0)

        ref = row

        if len(heading_window) >= MIN_SAMPLES and apply_t is None:
            std_deg = circular_std_deg(heading_window)
            if std_deg <= MAX_HEADING_STD_DEG:
                apply_t = row["t"]
                mean_rad = math.atan2(
                    statistics.mean(math.sin(math.radians(h)) for h in heading_window),
                    statistics.mean(math.cos(math.radians(h)) for h in heading_window),
                )
                apply_heading = math.degrees(mean_rad)
                apply_std = std_deg

    print("-" * 72)
    print(" Fase estatica / baja velocidad")
    static_end = moving[0]["t"]
    print(f"  Vehiculo parado o <3 m/s hasta: t~{static_end:.1f} s")
    print(f"  Tiempo estacionado aprox:       {static_end - gps_rows[0]['t']:.1f} s")

    print("-" * 72)
    print(" Ventana H2 (20 muestras heading @ speed>=3 m/s)")
    if apply_t is not None:
        print(f"  Yaw aplicable en:       t={apply_t:.3f} s")
        print(f"  Heading medio:          {apply_heading:.2f} deg")
        print(f"  Std heading:            {apply_std:.2f} deg")
        print(f"  Retardo desde marcha:   {apply_t - moving[0]['t']:.1f} s (~{MIN_SAMPLES} fixes @ ~1 Hz)")
    else:
        print("  No se cumplieron condiciones de estabilidad en el log.")

    print("-" * 72)
    print(" Por que ~31 s y no antes?")
    print("  1. GPS ~1 Hz: 20 muestras estables ~ 20 s de marcha continua.")
    print("  2. Hasta t~11 s el coche estaba parado (speed=0 en primeros fixes).")
    print("  3. H2 exige speed>=3 m/s + desplazamiento>=0.3 m entre fixes + std<5 deg.")
    print("  4. No usa la primera velocidad GNSS: espera ventana completa.")
    if apply_t is not None:
        print(f"  => Esperado: primer fix movimiento ~{moving[0]['t']:.0f}s + 20s ~ {apply_t:.0f}s")
    print("=" * 72)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analiza timing yaw H2")
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if not args.replay.is_file():
        print(f"ERROR: no existe {args.replay}", file=sys.stderr)
        return 1
    gps_rows = load_gps_rows(args.replay)
    analyze_timing(gps_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
