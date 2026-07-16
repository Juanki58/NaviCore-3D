#!/usr/bin/env python3
"""Preprocesa grabaciones de Sensor Logger (Android) para replay NaviCore.

Lee AccelerometerUncalibrated.csv, Gyroscope.csv y Location.csv, sincroniza
por timestamp en nanosegundos, convierte GNSS a NED respecto al primer fix
valido y escribe un CSV consolidado para Software-in-the-Loop.

Conversion geodesica: geodesy WGS84 ECEF->NED (src/core/geodesy.hpp, geodesy.py).
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT = REPO_ROOT / "docs" / "benchmarks" / "real_run_replay.csv"

from geodesy import lla_to_ned_scalars as latlonalt_to_ned  # noqa: E402

ACCEL_FILE = "AccelerometerUncalibrated.csv"
GYRO_FILE = "Gyroscope.csv"
LOCATION_FILE = "Location.csv"

SEARCH_DIRS = (
    REPO_ROOT / "data" / "real_run",
    REPO_ROOT,
    REPO_ROOT / "docs",
)

# Tolerancia para emparejar accel + gyro (~100 Hz con jitter de reloj).
IMU_PAIR_TOLERANCE_NS = 12_000_000  # 12 ms
# Minimo intervalo entre filas IMU emitidas (evita duplicados al fusionar).
IMU_MIN_INTERVAL_S = 0.005

OUTPUT_COLUMNS = (
    "timestamp_s",
    "type",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "pos_n",
    "pos_e",
    "pos_d",
    "accuracy_horizontal",
    "accuracy_vertical",
    "speed",
)


@dataclass(frozen=True)
class Vec3Sample:
    time_ns: int
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class GnssSample:
    time_ns: int
    latitude: float
    longitude: float
    altitude: float
    horizontal_accuracy: float
    vertical_accuracy: float
    speed: float


@dataclass(frozen=True)
class ReplayRow:
    time_ns: int
    row_type: str
    accel: tuple[float, float, float] | None = None
    gyro: tuple[float, float, float] | None = None
    pos_ned: tuple[float, float, float] | None = None
    accuracy_horizontal: float | None = None
    accuracy_vertical: float | None = None
    speed: float | None = None

    @property
    def timestamp_s(self) -> float:
        return self.time_ns / 1e9


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


def parse_int(value: str | None) -> int | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        return int(float(text))
    except ValueError:
        return None


def discover_input_dir(explicit: Path | None) -> Path:
    if explicit is not None:
        if not explicit.is_dir():
            raise FileNotFoundError(f"Directorio de entrada no encontrado: {explicit}")
        return explicit

    for candidate in SEARCH_DIRS:
        if candidate.is_dir() and all((candidate / name).is_file() for name in (
            ACCEL_FILE,
            GYRO_FILE,
            LOCATION_FILE,
        )):
            return candidate

    searched = ", ".join(str(path) for path in SEARCH_DIRS)
    raise FileNotFoundError(
        "No se encontraron los tres CSV de Sensor Logger. "
        f"Directorios buscados: {searched}. "
        "Usa --input-dir para indicar la carpeta de la grabacion."
    )


def load_vec3_csv(path: Path) -> list[Vec3Sample]:
    rows: list[Vec3Sample] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV sin cabecera: {path}")

        required = {"time", "x", "y", "z"}
        missing = required - set(reader.fieldnames)
        if missing:
            raise ValueError(f"{path.name}: faltan columnas {sorted(missing)}")

        for line_no, row in enumerate(reader, start=2):
            time_ns = parse_int(row.get("time"))
            x = parse_float(row.get("x"))
            y = parse_float(row.get("y"))
            z = parse_float(row.get("z"))
            if time_ns is None or x is None or y is None or z is None:
                continue
            rows.append(Vec3Sample(time_ns=time_ns, x=x, y=y, z=z))

    rows.sort(key=lambda sample: sample.time_ns)
    if not rows:
        raise ValueError(f"{path.name}: no hay muestras validas")
    return rows


def is_valid_gnss(latitude: float, longitude: float, altitude: float) -> bool:
    if abs(latitude) < 1.0e-6 and abs(longitude) < 1.0e-6:
        return False
    if abs(latitude) > 90.0 or abs(longitude) > 180.0:
        return False
    return math.isfinite(latitude) and math.isfinite(longitude) and math.isfinite(altitude)


def load_location_csv(path: Path) -> list[GnssSample]:
    rows: list[GnssSample] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV sin cabecera: {path}")

        required = {
            "time",
            "latitude",
            "longitude",
            "altitude",
            "horizontalAccuracy",
            "verticalAccuracy",
            "speed",
        }
        missing = required - set(reader.fieldnames)
        if missing:
            raise ValueError(f"{path.name}: faltan columnas {sorted(missing)}")

        for row in reader:
            time_ns = parse_int(row.get("time"))
            latitude = parse_float(row.get("latitude"))
            longitude = parse_float(row.get("longitude"))
            altitude = parse_float(row.get("altitude"))
            if (
                time_ns is None
                or latitude is None
                or longitude is None
                or altitude is None
            ):
                continue
            if not is_valid_gnss(latitude, longitude, altitude):
                continue

            rows.append(
                GnssSample(
                    time_ns=time_ns,
                    latitude=latitude,
                    longitude=longitude,
                    altitude=altitude,
                    horizontal_accuracy=parse_float(row.get("horizontalAccuracy")) or 0.0,
                    vertical_accuracy=parse_float(row.get("verticalAccuracy")) or 0.0,
                    speed=max(0.0, parse_float(row.get("speed")) or 0.0),
                )
            )

    rows.sort(key=lambda sample: sample.time_ns)
    if not rows:
        raise ValueError(f"{path.name}: no hay fixes GNSS validos")
    return rows


def nearest_index(times_ns: Sequence[int], target_ns: int) -> int | None:
    if not times_ns:
        return None

    pos = bisect.bisect_left(times_ns, target_ns)
    candidates: list[int] = []
    if pos < len(times_ns):
        candidates.append(pos)
    if pos > 0:
        candidates.append(pos - 1)

    best_idx: int | None = None
    best_delta = IMU_PAIR_TOLERANCE_NS + 1
    for idx in candidates:
        delta = abs(times_ns[idx] - target_ns)
        if delta <= IMU_PAIR_TOLERANCE_NS and delta < best_delta:
            best_idx = idx
            best_delta = delta
    return best_idx


def build_imu_rows(accel: Sequence[Vec3Sample], gyro: Sequence[Vec3Sample]) -> list[ReplayRow]:
    gyro_times = [sample.time_ns for sample in gyro]
    rows: list[ReplayRow] = []
    last_emit_time_ns: int | None = None

    for acc in accel:
        gyro_idx = nearest_index(gyro_times, acc.time_ns)
        if gyro_idx is None:
            continue

        if (
            last_emit_time_ns is not None
            and (acc.time_ns - last_emit_time_ns) < int(IMU_MIN_INTERVAL_S * 1e9)
        ):
            continue

        gyr = gyro[gyro_idx]
        rows.append(
            ReplayRow(
                time_ns=acc.time_ns,
                row_type="IMU",
                accel=(acc.x, acc.y, acc.z),
                gyro=(gyr.x, gyr.y, gyr.z),
            )
        )
        last_emit_time_ns = acc.time_ns

    if not rows:
        raise ValueError("No se pudo emparejar accel y gyro dentro de la tolerancia temporal")
    return rows


def build_gnss_rows(
    locations: Sequence[GnssSample],
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
) -> list[ReplayRow]:
    rows: list[ReplayRow] = []
    for fix in locations:
        pos_n, pos_e, pos_d = latlonalt_to_ned(
            ref_lat_deg,
            ref_lon_deg,
            ref_alt_m,
            fix.latitude,
            fix.longitude,
            fix.altitude,
        )
        rows.append(
            ReplayRow(
                time_ns=fix.time_ns,
                row_type="GPS",
                pos_ned=(pos_n, pos_e, pos_d),
                accuracy_horizontal=fix.horizontal_accuracy,
                accuracy_vertical=fix.vertical_accuracy,
                speed=fix.speed,
            )
        )
    return rows


def normalize_timestamps(rows: Iterable[ReplayRow], t0_ns: int) -> list[ReplayRow]:
    normalized: list[ReplayRow] = []
    for row in rows:
        normalized.append(
            ReplayRow(
                time_ns=row.time_ns - t0_ns,
                row_type=row.row_type,
                accel=row.accel,
                gyro=row.gyro,
                pos_ned=row.pos_ned,
                accuracy_horizontal=row.accuracy_horizontal,
                accuracy_vertical=row.accuracy_vertical,
                speed=row.speed,
            )
        )
    normalized.sort(key=lambda item: item.time_ns)
    return normalized


def format_cell(value: float | None) -> str:
    if value is None:
        return ""
    return f"{value:.9g}"


def row_to_csv_dict(row: ReplayRow) -> dict[str, str]:
    out = {column: "" for column in OUTPUT_COLUMNS}
    out["timestamp_s"] = f"{row.timestamp_s:.9f}"
    out["type"] = row.row_type

    if row.accel is not None:
        out["accel_x"], out["accel_y"], out["accel_z"] = (
            format_cell(row.accel[0]),
            format_cell(row.accel[1]),
            format_cell(row.accel[2]),
        )
    if row.gyro is not None:
        out["gyro_x"], out["gyro_y"], out["gyro_z"] = (
            format_cell(row.gyro[0]),
            format_cell(row.gyro[1]),
            format_cell(row.gyro[2]),
        )
    if row.pos_ned is not None:
        out["pos_n"], out["pos_e"], out["pos_d"] = (
            format_cell(row.pos_ned[0]),
            format_cell(row.pos_ned[1]),
            format_cell(row.pos_ned[2]),
        )
    if row.accuracy_horizontal is not None:
        out["accuracy_horizontal"] = format_cell(row.accuracy_horizontal)
    if row.accuracy_vertical is not None:
        out["accuracy_vertical"] = format_cell(row.accuracy_vertical)
    if row.speed is not None:
        out["speed"] = format_cell(row.speed)
    return out


def write_replay_csv(path: Path, rows: Sequence[ReplayRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row_to_csv_dict(row))


def summarize_rates(rows: Sequence[ReplayRow]) -> tuple[float, float]:
    imu_times = [row.timestamp_s for row in rows if row.row_type == "IMU"]
    gps_times = [row.timestamp_s for row in rows if row.row_type == "GPS"]

    def median_dt(times: Sequence[float]) -> float:
        if len(times) < 2:
            return 0.0
        deltas = [times[i + 1] - times[i] for i in range(len(times) - 1) if times[i + 1] > times[i]]
        if not deltas:
            return 0.0
        deltas.sort()
        return deltas[len(deltas) // 2]

    imu_dt = median_dt(imu_times)
    gps_dt = median_dt(gps_times)
    imu_hz = (1.0 / imu_dt) if imu_dt > 0.0 else 0.0
    gps_hz = (1.0 / gps_dt) if gps_dt > 0.0 else 0.0
    return imu_hz, gps_hz


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convierte logs de Sensor Logger a docs/benchmarks/real_run_replay.csv"
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=None,
        help="Carpeta con AccelerometerUncalibrated.csv, Gyroscope.csv y Location.csv "
        f"(por defecto: {SEARCH_DIRS[0]} o raiz/docs si existen los archivos)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"CSV consolidado de salida (por defecto: {DEFAULT_OUTPUT})",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    try:
        input_dir = discover_input_dir(args.input_dir)
        accel = load_vec3_csv(input_dir / ACCEL_FILE)
        gyro = load_vec3_csv(input_dir / GYRO_FILE)
        locations = load_location_csv(input_dir / LOCATION_FILE)
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    ref = locations[0]
    ref_lat_deg = ref.latitude
    ref_lon_deg = ref.longitude
    ref_alt_m = ref.altitude

    imu_rows = build_imu_rows(accel, gyro)
    gnss_rows = build_gnss_rows(locations, ref_lat_deg, ref_lon_deg, ref_alt_m)
    merged = imu_rows + gnss_rows

    t0_ns = min(
        accel[0].time_ns,
        gyro[0].time_ns,
        locations[0].time_ns,
    )
    rows = normalize_timestamps(merged, t0_ns)

    write_replay_csv(args.output, rows)

    imu_hz, gps_hz = summarize_rates(rows)
    duration_s = rows[-1].timestamp_s if rows else 0.0
    imu_count = sum(1 for row in rows if row.row_type == "IMU")
    gps_count = sum(1 for row in rows if row.row_type == "GPS")

    print(f"Entrada:   {input_dir}")
    print(f"Salida:    {args.output}")
    print(
        "Origen GNSS: "
        f"lat={ref_lat_deg:.7f} deg, lon={ref_lon_deg:.7f} deg, alt={ref_alt_m:.3f} m"
    )
    print(
        f"Muestras:  IMU={imu_count} (~{imu_hz:.1f} Hz) | GPS={gps_count} (~{gps_hz:.2f} Hz) | "
        f"duracion={duration_s:.2f} s"
    )
    print("Conversion NED: geodesy WGS84 ECEF->NED (geodesy.py / src/core/geodesy.hpp)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
