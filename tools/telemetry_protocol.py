#!/usr/bin/env python3
"""Protocolo UDP de telemetría NaviCore-3D (fuente de verdad Python, espejo de telemetry_udp.hpp)."""

from __future__ import annotations

import struct
from typing import TypedDict

TELEMETRY_UDP_MAGIC = 0x4E43
TELEMETRY_UDP_DEFAULT_PORT = 5005
TELEMETRY_UDP_DEFAULT_HOST = "127.0.0.1"

PACKET_FMT = "<HHIfffHH"
PACKET_SIZE = struct.calcsize(PACKET_FMT)

HEALTH_MODES = {0: "NOMINAL", 1: "DEGRADED", 2: "CRITICAL"}
COLOR_MAP = {"NOMINAL": "green", "DEGRADED": "orange", "CRITICAL": "red"}


class DecodedPacket(TypedDict):
    magic: int
    seq: int
    timestamp_ms: int
    x: float
    y: float
    z: float
    score: int
    flags: int
    mode_bits: int
    mode_str: str
    color: str
    dropped_packets: int


def encode_flags(health_mode: int, dropped: int) -> int:
    return (health_mode & 0x03) | ((dropped & 0x3FFF) << 2)


def pack_packet(
    timestamp_ms: int,
    x: float,
    y: float,
    z: float,
    score: int,
    health_mode: int,
    dropped: int,
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
    )


def unpack_packet(data: bytes) -> DecodedPacket:
    if len(data) != PACKET_SIZE:
        raise ValueError(f"Paquete invalido: {len(data)} bytes (esperado {PACKET_SIZE})")

    magic, seq, timestamp_ms, x, y, z, score, flags = struct.unpack(PACKET_FMT, data)
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
        "score": score,
        "flags": flags,
        "mode_bits": mode_bits,
        "mode_str": mode_str,
        "color": COLOR_MAP[mode_str],
        "dropped_packets": dropped_packets,
    }
