#!/usr/bin/env python3
"""Geodesia WGS84 compartida — ECEF <-> LLA <-> NED (Python + herramientas de auditoria)."""

from __future__ import annotations

import math
from dataclasses import dataclass

WGS84_A = 6_378_137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass(frozen=True)
class LLA:
    lat_deg: float
    lon_deg: float
    alt_m: float


@dataclass(frozen=True)
class ECEF:
    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True)
class NED:
    north_m: float
    east_m: float
    down_m: float


def lla(lat_deg: float, lon_deg: float, alt_m: float) -> LLA:
    return LLA(lat_deg, lon_deg, alt_m)


def lla_to_ecef(point: LLA) -> ECEF:
    lat = math.radians(point.lat_deg)
    lon = math.radians(point.lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n_radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n_radius + point.alt_m) * cos_lat * cos_lon
    y = (n_radius + point.alt_m) * cos_lat * sin_lon
    z = (n_radius * (1.0 - WGS84_E2) + point.alt_m) * sin_lat
    return ECEF(x, y, z)


def ecef_to_lla(point: ECEF) -> LLA:
    x = point.x_m
    y = point.y_m
    z = point.z_m
    p = math.hypot(x, y)
    theta = math.atan2(z * WGS84_A, p * (WGS84_A * (1.0 - WGS84_E2)))
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    lat = math.atan2(
        z
        + (WGS84_E2 * (1.0 - WGS84_E2) * WGS84_A * sin_theta**3) / (1.0 - WGS84_E2),
        p - WGS84_E2 * WGS84_A * cos_theta**3,
    )
    lon = math.atan2(y, x)
    sin_lat = math.sin(lat)
    n_radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = (p / math.cos(lat)) - n_radius
    return LLA(math.degrees(lat), math.degrees(lon), alt)


def ecef_to_ned(point: ECEF, ref: LLA) -> NED:
    ref_ecef = lla_to_ecef(ref)
    dx = point.x_m - ref_ecef.x_m
    dy = point.y_m - ref_ecef.y_m
    dz = point.z_m - ref_ecef.z_m

    lat0 = math.radians(ref.lat_deg)
    lon0 = math.radians(ref.lon_deg)
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)

    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    east = -sin_lon * dx + cos_lon * dy
    down = -cos_lat * cos_lon * dx - cos_lat * sin_lon * dy - sin_lat * dz
    return NED(north, east, down)


def ned_to_ecef(ned: NED, ref: LLA) -> ECEF:
    ref_ecef = lla_to_ecef(ref)
    lat0 = math.radians(ref.lat_deg)
    lon0 = math.radians(ref.lon_deg)
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)

    north = ned.north_m
    east = ned.east_m
    down = ned.down_m

    dx = -sin_lat * cos_lon * north - sin_lon * east - cos_lat * cos_lon * down
    dy = -sin_lat * sin_lon * north + cos_lon * east - cos_lat * sin_lon * down
    dz = cos_lat * north - sin_lat * down
    return ECEF(ref_ecef.x_m + dx, ref_ecef.y_m + dy, ref_ecef.z_m + dz)


def lla_to_ned(point: LLA, ref: LLA) -> NED:
    return ecef_to_ned(lla_to_ecef(point), ref)


def ned_to_lla(ned: NED, ref: LLA) -> LLA:
    return ecef_to_lla(ned_to_ecef(ned, ref))


def lla_to_ned_scalars(
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
) -> tuple[float, float, float]:
    ned = lla_to_ned(
        lla(lat_deg, lon_deg, alt_m),
        lla(ref_lat_deg, ref_lon_deg, ref_alt_m),
    )
    return ned.north_m, ned.east_m, ned.down_m


def ned_to_lla_scalars(
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
    north_m: float,
    east_m: float,
    down_m: float,
) -> tuple[float, float, float]:
    point = ned_to_lla(
        NED(north_m, east_m, down_m),
        lla(ref_lat_deg, ref_lon_deg, ref_alt_m),
    )
    return point.lat_deg, point.lon_deg, point.alt_m


def lla_to_ned_flat_legacy(
    point: LLA,
    ref: LLA,
) -> NED:
    """Aproximacion plana historica (solo validacion / comparacion H8)."""
    meters_per_deg_lat = 111_132.954
    dlat_m = (point.lat_deg - ref.lat_deg) * meters_per_deg_lat
    lat_rad = math.radians((ref.lat_deg + point.lat_deg) * 0.5)
    dlon_m = (point.lon_deg - ref.lon_deg) * meters_per_deg_lat * math.cos(lat_rad)
    return NED(dlat_m, dlon_m, ref.alt_m - point.alt_m)
