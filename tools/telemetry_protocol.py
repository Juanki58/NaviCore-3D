#!/usr/bin/env python3
"""Protocolo UDP de telemetría NaviCore-3D — Unity unified + eventos legacy."""

from __future__ import annotations

import math
import struct
from typing import TypedDict

# --- Unity unified telemetry (espejo de src/core/telemetry_interface.hpp) ---
UNITY_TELEMETRY_MAGIC = 0x4E55
UNITY_TELEMETRY_DEFAULT_PORT = 5556
UNITY_TELEMETRY_DEFAULT_HOST = "127.0.0.1"

UNITY_PACKET_FMT = "<HHIffffffffffBBBBH"
UNITY_PACKET_SIZE = struct.calcsize(UNITY_PACKET_FMT)

UNITY_TELEM_FLAG_EKF_VALID = 0x01
UNITY_TELEM_FLAG_POS_VALID = 0x02
UNITY_TELEM_FLAG_VEL_VALID = 0x04
UNITY_TELEM_FLAG_ATT_VALID = 0x08

# --- Legacy v3 (32 B) — mantenido para pruebas retrocompatibles ---
TELEMETRY_UDP_MAGIC = 0x4E43
TELEMETRY_UDP_EVENT_MAGIC = 0x4E45
TELEMETRY_UDP_DEFAULT_PORT = 5005
TELEMETRY_UDP_DEFAULT_HOST = "127.0.0.1"

LEGACY_PACKET_FMT = "<HHIfffHHBBhhH"
LEGACY_PACKET_SIZE = struct.calcsize(LEGACY_PACKET_FMT)

EVENT_FMT = "<HHI"
EVENT_SIZE = struct.calcsize(EVENT_FMT)

# Alias del receptor unificado
PACKET_SIZE = UNITY_PACKET_SIZE
PACKET_FMT = UNITY_PACKET_FMT

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

MISSION_STATE_NAMES = {
    0: "INIT",
    1: "WAIT_GPS",
    2: "READY",
    3: "NAVIGATE",
    4: "RETURN_HOME",
    5: "SAFE_MODE",
}

HEALTH_MODES = {0: "NOMINAL", 1: "DEGRADED", 2: "CRITICAL"}
COLOR_MAP = {"NOMINAL": "green", "DEGRADED": "orange", "CRITICAL": "red"}

TELEM_EVENT_SAFE_STOP = 1
TELEM_EVENT_HOT_RESTART = 2
TELEM_EVENT_HEALTH_DEGRADED = 3
TELEM_EVENT_HEALTH_CRITICAL = 4
TELEM_EVENT_HEALTH_NOMINAL = 5
TELEM_EVENT_GPS_LOST = 6
TELEM_EVENT_GPS_RESTORED = 7
TELEM_EVENT_WCET_VIOLATION = 8
TELEM_EVENT_POWER_CONSERVATION = 9
TELEM_EVENT_PREDICTIVE_DEGRADE = 10

EVENT_NAMES = {
    TELEM_EVENT_SAFE_STOP: "SAFE_STOP",
    TELEM_EVENT_HOT_RESTART: "HOT_RESTART",
    TELEM_EVENT_HEALTH_DEGRADED: "HEALTH_DEGRADED",
    TELEM_EVENT_HEALTH_CRITICAL: "HEALTH_CRITICAL",
    TELEM_EVENT_HEALTH_NOMINAL: "HEALTH_NOMINAL",
    TELEM_EVENT_GPS_LOST: "GPS_LOST",
    TELEM_EVENT_GPS_RESTORED: "GPS_RESTORED",
    TELEM_EVENT_WCET_VIOLATION: "WCET_VIOLATION",
    TELEM_EVENT_POWER_CONSERVATION: "POWER_CONSERVATION",
    TELEM_EVENT_PREDICTIVE_DEGRADE: "PREDICTIVE_DEGRADE",
}


class DecodedPacket(TypedDict):
    magic: int
    seq: int
    timestamp_ms: int
    pos_n_m: float
    pos_e_m: float
    pos_d_m: float
    vel_n_mps: float
    vel_e_mps: float
    vel_d_mps: float
    quat_w: float
    quat_x: float
    quat_y: float
    quat_z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    speed_mps: float
    nav_mode: int
    mission_state: int
    health_mode: int
    flags: int
    score: int
    mode_bits: int
    mode_str: str
    color: str
    mission_state_name: str
    nav_mode_name: str
    # Alias retrocompatibles con el visualizador anterior
    x: float
    y: float
    z: float


class DecodedEvent(TypedDict):
    magic: int
    event_id: int
    param: int
    timestamp_ms: int
    event_name: str


def quat_to_euler_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Cuaternión Hamilton (w,x,y,z) → roll/pitch/yaw en grados (convención NED / ZYX)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def ned_speed_mps(vel_n_mps: float, vel_e_mps: float, vel_d_mps: float) -> float:
    return math.sqrt((vel_n_mps * vel_n_mps) + (vel_e_mps * vel_e_mps) + (vel_d_mps * vel_d_mps))


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


def pack_legacy_packet(
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
        LEGACY_PACKET_FMT,
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
    """Alias retrocompatible — empaqueta telemetría legacy v3 (32 B)."""
    return pack_legacy_packet(
        timestamp_ms,
        x,
        y,
        z,
        cross_track_m,
        along_track_m,
        score,
        health_mode,
        dropped,
        scenario_id,
        nav_mode,
        temperature_c,
        seq=seq,
        magic=magic,
    )


def pack_unity_packet(
    timestamp_ms: int,
    pos_n_m: float,
    pos_e_m: float,
    pos_d_m: float,
    vel_n_mps: float,
    vel_e_mps: float,
    vel_d_mps: float,
    quat_w: float,
    quat_x: float,
    quat_y: float,
    quat_z: float,
    nav_mode: int,
    mission_state: int,
    health_mode: int,
    flags: int,
    health_score: int,
    seq: int = 0,
    magic: int = UNITY_TELEMETRY_MAGIC,
) -> bytes:
    return struct.pack(
        UNITY_PACKET_FMT,
        magic,
        seq & 0xFFFF,
        timestamp_ms & 0xFFFFFFFF,
        pos_n_m,
        pos_e_m,
        pos_d_m,
        vel_n_mps,
        vel_e_mps,
        vel_d_mps,
        quat_w,
        quat_x,
        quat_y,
        quat_z,
        nav_mode & 0xFF,
        mission_state & 0xFF,
        health_mode & 0xFF,
        flags & 0xFF,
        health_score & 0xFFFF,
    )


def unpack_packet(data: bytes) -> DecodedPacket:
    if len(data) != UNITY_PACKET_SIZE:
        raise ValueError(f"Paquete invalido: {len(data)} bytes (esperado {UNITY_PACKET_SIZE})")

    (
        magic,
        seq,
        timestamp_ms,
        pos_n_m,
        pos_e_m,
        pos_d_m,
        vel_n_mps,
        vel_e_mps,
        vel_d_mps,
        quat_w,
        quat_x,
        quat_y,
        quat_z,
        nav_mode,
        mission_state,
        health_mode,
        flags,
        health_score,
    ) = struct.unpack(UNITY_PACKET_FMT, data)

    if magic != UNITY_TELEMETRY_MAGIC:
        raise ValueError(f"Magic invalido: 0x{magic:04X} (esperado 0x{UNITY_TELEMETRY_MAGIC:04X})")

    mode_str = HEALTH_MODES.get(health_mode, "CRITICAL")
    roll_deg, pitch_deg, yaw_deg = quat_to_euler_deg(quat_w, quat_x, quat_y, quat_z)
    speed_mps = ned_speed_mps(vel_n_mps, vel_e_mps, vel_d_mps)

    return {
        "magic": magic,
        "seq": seq,
        "timestamp_ms": timestamp_ms,
        "pos_n_m": pos_n_m,
        "pos_e_m": pos_e_m,
        "pos_d_m": pos_d_m,
        "vel_n_mps": vel_n_mps,
        "vel_e_mps": vel_e_mps,
        "vel_d_mps": vel_d_mps,
        "quat_w": quat_w,
        "quat_x": quat_x,
        "quat_y": quat_y,
        "quat_z": quat_z,
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg,
        "speed_mps": speed_mps,
        "nav_mode": nav_mode,
        "mission_state": mission_state,
        "health_mode": health_mode,
        "flags": flags,
        "score": health_score,
        "mode_bits": health_mode,
        "mode_str": mode_str,
        "color": COLOR_MAP[mode_str],
        "mission_state_name": MISSION_STATE_NAMES.get(mission_state, f"STATE_{mission_state}"),
        "nav_mode_name": NAV_MODE_NAMES.get(nav_mode, "UNKNOWN"),
        "x": pos_n_m,
        "y": pos_e_m,
        "z": pos_d_m,
    }


def unpack_legacy_packet(data: bytes) -> dict:
    if len(data) != LEGACY_PACKET_SIZE:
        raise ValueError(f"Paquete legacy invalido: {len(data)} bytes (esperado {LEGACY_PACKET_SIZE})")

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
    ) = struct.unpack(LEGACY_PACKET_FMT, data)

    if magic != TELEMETRY_UDP_MAGIC:
        raise ValueError(f"Magic legacy invalido: 0x{magic:04X}")

    mode_bits = flags & 0x03
    mode_str = HEALTH_MODES.get(mode_bits, "CRITICAL")

    return {
        "magic": magic,
        "seq": seq,
        "timestamp_ms": timestamp_ms,
        "x": x,
        "y": y,
        "z": z,
        "score": score,
        "mode_str": mode_str,
        "color": COLOR_MAP[mode_str],
        "scenario_name": SCENARIO_NAMES.get(scenario_id, SCENARIO_NAMES[TELEMETRY_SCENARIO_UNKNOWN]),
        "nav_mode_name": NAV_MODE_NAMES.get(nav_mode, "UNKNOWN"),
        "temperature_c": temperature_deci_c / 10.0,
        "cross_track_m": cross_track_deci_m / 10.0,
        "along_track_m": along_track_deci_m / 10.0,
        "dropped_packets": flags >> 2,
    }


def pack_event(
    timestamp_ms: int,
    event_id: int,
    param: int = 0,
    magic: int = TELEMETRY_UDP_EVENT_MAGIC,
) -> bytes:
    packed = ((event_id & 0xFF) << 8) | (param & 0xFF)
    return struct.pack(EVENT_FMT, magic, packed, timestamp_ms & 0xFFFFFFFF)


def unpack_event(data: bytes) -> DecodedEvent:
    if len(data) != EVENT_SIZE:
        raise ValueError(f"Evento invalido: {len(data)} bytes (esperado {EVENT_SIZE})")

    magic, packed, timestamp_ms = struct.unpack(EVENT_FMT, data)
    if magic != TELEMETRY_UDP_EVENT_MAGIC:
        raise ValueError(
            f"Magic de evento invalido: 0x{magic:04X} (esperado 0x{TELEMETRY_UDP_EVENT_MAGIC:04X})"
        )

    event_id = (packed >> 8) & 0xFF
    param = packed & 0xFF
    return {
        "magic": magic,
        "event_id": event_id,
        "param": param,
        "timestamp_ms": timestamp_ms,
        "event_name": EVENT_NAMES.get(event_id, f"EVENT_{event_id}"),
    }
