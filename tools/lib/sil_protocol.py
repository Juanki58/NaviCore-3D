#!/usr/bin/env python3
"""Protocolo UDP SIL — espejo de src/core/sil_protocol.hpp."""

from __future__ import annotations

import struct
from typing import TypedDict

SIL_TRUTH_MAGIC = 0x4E54
SIL_SENSOR_MAGIC = 0x4E53
SIL_ACTUATOR_MAGIC = 0x4E41

SIL_MAX_UAV_ID = 7

SIL_TRUTH_BASE_PORT = 5301
SIL_SENSOR_BASE_PORT = 5401
SIL_ACTUATOR_BASE_PORT = 5501
SIL_NAVICORE_TELEM_BASE_PORT = 5201

SIL_FLAG_POS_VALID = 0x01
SIL_FLAG_ATT_VALID = 0x02
SIL_FLAG_VEL_VALID = 0x04
SIL_FLAG_IMU_VALID = 0x01
SIL_FLAG_GPS_VALID = 0x02
SIL_FLAG_MAG_VALID = 0x04

TRUTH_FMT = "<HBBHIfffffffffH"
TRUTH_SIZE = struct.calcsize(TRUTH_FMT)

SENSOR_FMT = "<HBBHIffffffffffffffBBH"
SENSOR_SIZE = struct.calcsize(SENSOR_FMT)

ACTUATOR_FMT = "<HBBHIfH"
ACTUATOR_SIZE = struct.calcsize(ACTUATOR_FMT)

SIL_SURFACE_THROTTLE = 0
SIL_SURFACE_AILERON = 1
SIL_SURFACE_ELEVATOR = 2
SIL_SURFACE_RUDDER = 3


class DecodedTruth(TypedDict):
    magic: int
    uav_id: int
    flags: int
    seq: int
    timestamp_ms: int
    pos_n_m: float
    pos_e_m: float
    pos_d_m: float
    vel_n_mps: float
    vel_e_mps: float
    vel_d_mps: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    status_flags: int


class DecodedSensor(TypedDict):
    magic: int
    uav_id: int
    flags: int
    seq: int
    timestamp_ms: int
    accel_mps2: tuple[float, float, float]
    gyro_radps: tuple[float, float, float]
    mag_ut: tuple[float, float, float]
    lat_deg: float
    lon_deg: float
    alt_m: float
    speed_mps: float
    course_deg: float
    satellites: int
    fix_valid: bool


class DecodedActuator(TypedDict):
    magic: int
    uav_id: int
    surface_id: int
    seq: int
    timestamp_ms: int
    command_norm: float


def sil_truth_port(uav_id: int) -> int:
    return SIL_TRUTH_BASE_PORT + (uav_id - 1)


def sil_sensor_port(uav_id: int) -> int:
    return SIL_SENSOR_BASE_PORT + (uav_id - 1)


def sil_actuator_port(uav_id: int) -> int:
    return SIL_ACTUATOR_BASE_PORT + (uav_id - 1)


def sil_navicore_telem_port(uav_id: int) -> int:
    return SIL_NAVICORE_TELEM_BASE_PORT + (uav_id - 1)


def pack_truth(
    uav_id: int,
    timestamp_ms: int,
    pos_n_m: float,
    pos_e_m: float,
    pos_d_m: float,
    vel_n_mps: float,
    vel_e_mps: float,
    vel_d_mps: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    flags: int = SIL_FLAG_POS_VALID | SIL_FLAG_ATT_VALID | SIL_FLAG_VEL_VALID,
    status_flags: int = 0,
    seq: int = 0,
    magic: int = SIL_TRUTH_MAGIC,
) -> bytes:
    return struct.pack(
        TRUTH_FMT,
        magic,
        uav_id & 0xFF,
        flags & 0xFF,
        seq & 0xFFFF,
        timestamp_ms & 0xFFFFFFFF,
        pos_n_m,
        pos_e_m,
        pos_d_m,
        vel_n_mps,
        vel_e_mps,
        vel_d_mps,
        roll_deg,
        pitch_deg,
        yaw_deg,
        status_flags & 0xFFFF,
    )


def unpack_truth(data: bytes) -> DecodedTruth:
    if len(data) != TRUTH_SIZE:
        raise ValueError(f"SilTruth invalido: {len(data)} bytes (esperado {TRUTH_SIZE})")

    fields = struct.unpack(TRUTH_FMT, data)
    magic = fields[0]
    if magic != SIL_TRUTH_MAGIC:
        raise ValueError(f"Magic truth invalido: 0x{magic:04X}")

    return {
        "magic": magic,
        "uav_id": fields[1],
        "flags": fields[2],
        "seq": fields[3],
        "timestamp_ms": fields[4],
        "pos_n_m": fields[5],
        "pos_e_m": fields[6],
        "pos_d_m": fields[7],
        "vel_n_mps": fields[8],
        "vel_e_mps": fields[9],
        "vel_d_mps": fields[10],
        "roll_deg": fields[11],
        "pitch_deg": fields[12],
        "yaw_deg": fields[13],
        "status_flags": fields[14],
    }


def pack_sensor(
    uav_id: int,
    timestamp_ms: int,
    accel_mps2: tuple[float, float, float],
    gyro_radps: tuple[float, float, float],
    mag_ut: tuple[float, float, float],
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    speed_mps: float,
    course_deg: float,
    satellites: int = 12,
    fix_valid: bool = True,
    flags: int = SIL_FLAG_IMU_VALID | SIL_FLAG_GPS_VALID | SIL_FLAG_MAG_VALID,
    seq: int = 0,
    magic: int = SIL_SENSOR_MAGIC,
) -> bytes:
    return struct.pack(
        SENSOR_FMT,
        magic,
        uav_id & 0xFF,
        flags & 0xFF,
        seq & 0xFFFF,
        timestamp_ms & 0xFFFFFFFF,
        *accel_mps2,
        *gyro_radps,
        *mag_ut,
        lat_deg,
        lon_deg,
        alt_m,
        speed_mps,
        course_deg,
        satellites & 0xFF,
        1 if fix_valid else 0,
        0,
    )


def unpack_sensor(data: bytes) -> DecodedSensor:
    if len(data) != SENSOR_SIZE:
        raise ValueError(f"SilSensor invalido: {len(data)} bytes (esperado {SENSOR_SIZE})")

    fields = struct.unpack(SENSOR_FMT, data)
    magic = fields[0]
    if magic != SIL_SENSOR_MAGIC:
        raise ValueError(f"Magic sensor invalido: 0x{magic:04X}")

    return {
        "magic": magic,
        "uav_id": fields[1],
        "flags": fields[2],
        "seq": fields[3],
        "timestamp_ms": fields[4],
        "accel_mps2": (fields[5], fields[6], fields[7]),
        "gyro_radps": (fields[8], fields[9], fields[10]),
        "mag_ut": (fields[11], fields[12], fields[13]),
        "lat_deg": fields[14],
        "lon_deg": fields[15],
        "alt_m": fields[16],
        "speed_mps": fields[17],
        "course_deg": fields[18],
        "satellites": fields[19],
        "fix_valid": bool(fields[20]),
    }


def pack_actuator(
    uav_id: int,
    timestamp_ms: int,
    surface_id: int,
    command_norm: float,
    seq: int = 0,
    magic: int = SIL_ACTUATOR_MAGIC,
) -> bytes:
    return struct.pack(
        ACTUATOR_FMT,
        magic,
        uav_id & 0xFF,
        surface_id & 0xFF,
        seq & 0xFFFF,
        timestamp_ms & 0xFFFFFFFF,
        command_norm,
        0,
    )


def unpack_actuator(data: bytes) -> DecodedActuator:
    if len(data) != ACTUATOR_SIZE:
        raise ValueError(f"SilActuator invalido: {len(data)} bytes (esperado {ACTUATOR_SIZE})")

    fields = struct.unpack(ACTUATOR_FMT, data)
    magic = fields[0]
    if magic != SIL_ACTUATOR_MAGIC:
        raise ValueError(f"Magic actuator invalido: 0x{magic:04X}")

    return {
        "magic": magic,
        "uav_id": fields[1],
        "surface_id": fields[2],
        "seq": fields[3],
        "timestamp_ms": fields[4],
        "command_norm": fields[5],
    }
