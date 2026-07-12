#!/usr/bin/env python3
"""Protocolo UDP de telemetría NaviCore-3D v3 (espejo de telemetry_udp.hpp)."""

from __future__ import annotations

import struct
from typing import TypedDict

TELEMETRY_UDP_MAGIC = 0x4E43
TELEMETRY_UDP_DEFAULT_PORT = 5005
TELEMETRY_UDP_DEFAULT_HOST = "127.0.0.1"

PACKET_FMT = "<HHIfffHHBBhhH"
PACKET_SIZE = struct.calcsize(PACKET_FMT)

TELEMETRY_SCENARIO_HIGH_DEMAND = 0
TELEMETRY_SCENARIO_FAULT_INJECTION = 1
TELEMETRY_SCENARIO_CLEAN = 2
TELEMETRY_SCENARIO_GPS_LOSS = 3
TELEMETRY_SCENARIO_IMU_DRIFT = 4
TELEMETRY_SCENARIO_ODOM_LOSS = 5
TELEMETRY_SCENARIO_SUBMARINE = 6
TELEMETRY_SCENARIO_UNKNOWN = 255

SCENARIO_NAMES = {
    TELEMETRY_SCENARIO_HIGH_DEMAND: "HIGH_DEMAND_STRESS_TEST",
    TELEMETRY_SCENARIO_FAULT_INJECTION: "FAULT_INJECTION",
    TELEMETRY_SCENARIO_CLEAN: "CLEAN",
    TELEMETRY_SCENARIO_GPS_LOSS: "GPS_LOSS",
    TELEMETRY_SCENARIO_IMU_DRIFT: "IMU_DRIFT",
    TELEMETRY_SCENARIO_ODOM_LOSS: "ODOM_LOSS",
    TELEMETRY_SCENARIO_SUBMARINE: "SUBMARINE",
    TELEMETRY_SCENARIO_UNKNOWN: "UNKNOWN",
}

NAV_MODE_NAMES = {
    0: "INITIALIZING",
    1: "GPS",
    2: "DEAD_RECKONING",
    3: "HYBRID",
}

HEALTH_MODES = {0: "NOMINAL", 1: "DEGRADED", 2: "CRITICAL"}
COLOR_MAP = {"NOMINAL": "green", "DEGRADED": "orange", "CRITICAL": "red"}


class DecodedPacket(TypedDict):
    magic: int
    seq: int
    timestamp_ms: int
    x: float
    y: float
    z: float
    cross_track_m: float
    along_track_m: float
    score: int
    flags: int
    scenario_id: int
    nav_mode: int
    temperature_c: float
    mode_bits: int
    mode_str: str
    color: str
    dropped_packets: int
    scenario_name: str
    nav_mode_name: str


def encode_flags(health_mode: int, dropped: int) -> int:
    return (health_mode & 0x03) | ((dropped & 0x3FFF) << 2)


def encode_temperature_deci_c(temperature_c: float) -> int:
    clamped = max(-327.0, min(327.0, temperature_c))
    return int(round(clamped * 10.0))


def encode_cross_deci_m(value_m: float) -> int:
    clamped = max(-3276.7, min(3276.7, value_m))
    return int(round(clamped * 10.0))


def encode_along_deci_m(value_m: float) -> int:
    clamped = max(0.0, min(6553.5, value_m))
    return int(round(clamped * 10.0))


def pack_packet(
    timestamp_ms: int,
    x: float,
    y: float,
    z: float,
    cross_track_m: float,
    along_track_m: float,
    score: int,
    health_mode: int,
    dropped: int,
    scenario_id: int,
    nav_mode: int,
    temperature_c: float,
    seq: int = 0,
    magic: int = TELEMETRY_UDP_MAGIC,
) -> bytes:
    flags = encode_flags(health_mode, dropped)
    return struct.pack(
        PACKET_FMT,
        magic,
        seq & 0xFFFF,
        timestamp_ms & 0xFFFFFFFF,
        x,
        y,
        z,
        score & 0xFFFF,
        flags & 0xFFFF,
        scenario_id & 0xFF,
        nav_mode & 0xFF,
        encode_temperature_deci_c(temperature_c),
        encode_cross_deci_m(cross_track_m),
        encode_along_deci_m(along_track_m),
    )


def unpack_packet(data: bytes) -> DecodedPacket:
    if len(data) != PACKET_SIZE:
        raise ValueError(f"Paquete invalido: {len(data)} bytes (esperado {PACKET_SIZE})")

    (
        magic,
        seq,
        timestamp_ms,
        x,
        y,
        z,
        score,
        flags,
        scenario_id,
        nav_mode,
        temperature_deci_c,
        cross_track_deci_m,
        along_track_deci_m,
    ) = struct.unpack(PACKET_FMT, data)

    if magic != TELEMETRY_UDP_MAGIC:
        raise ValueError(f"Magic invalido: 0x{magic:04X} (esperado 0x{TELEMETRY_UDP_MAGIC:04X})")

    mode_bits = flags & 0x03
    dropped_packets = flags >> 2
    mode_str = HEALTH_MODES.get(mode_bits, "CRITICAL")

    return {
        "magic": magic,
        "seq": seq,
        "timestamp_ms": timestamp_ms,
        "x": x,
        "y": y,
        "z": z,
        "cross_track_m": cross_track_deci_m / 10.0,
        "along_track_m": along_track_deci_m / 10.0,
        "score": score,
        "flags": flags,
        "scenario_id": scenario_id,
        "nav_mode": nav_mode,
        "temperature_c": temperature_deci_c / 10.0,
        "mode_bits": mode_bits,
        "mode_str": mode_str,
        "color": COLOR_MAP[mode_str],
        "dropped_packets": dropped_packets,
        "scenario_name": SCENARIO_NAMES.get(scenario_id, SCENARIO_NAMES[TELEMETRY_SCENARIO_UNKNOWN]),
        "nav_mode_name": NAV_MODE_NAMES.get(nav_mode, "UNKNOWN"),
    }
