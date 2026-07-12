#!/usr/bin/env python3
"""Capa de transporte UDP: decode, ring buffer y estadísticas de enlace."""

from __future__ import annotations

import socket
from collections import deque
from dataclasses import dataclass
from typing import Deque

from telemetry_protocol import (
    COLOR_MAP,
    PACKET_SIZE,
    TELEMETRY_UDP_MAGIC,
    unpack_packet,
)


@dataclass(frozen=True, slots=True)
class TelemetrySample:
    timestamp_s: float
    x: float
    y: float
    z: float
    score: int
    mode: str
    color: str
    dropped_packets: int
    seq: int


class TelemetryReceiver:
    def __init__(self, host: str = "0.0.0.0", port: int = 5005, max_samples: int = 200):
        self.max_samples = max_samples
        self.samples: Deque[TelemetrySample] = deque(maxlen=max_samples)
        self.recovery_points: Deque[tuple[float, float]] = deque(maxlen=50)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)

        self._dirty = False
        self._last_score = 100
        self._last_seq: int | None = None

        self.packets_ok = 0
        self.packets_invalid = 0
        self.seq_gaps = 0

    @property
    def dirty(self) -> bool:
        return self._dirty

    def clear_dirty(self) -> None:
        self._dirty = False

    def drain(self) -> int:
        """Vacía el socket UDP hasta EWOULDBLOCK. Devuelve muestras nuevas aceptadas."""
        accepted = 0
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
            except BlockingIOError:
                break
            except InterruptedError:
                break

            if len(data) != PACKET_SIZE:
                self.packets_invalid += 1
                continue

            try:
                decoded = unpack_packet(data)
            except ValueError:
                self.packets_invalid += 1
                continue

            if self._last_seq is not None:
                expected = (self._last_seq + 1) & 0xFFFF
                if decoded["seq"] != expected:
                    self.seq_gaps += 1
            self._last_seq = decoded["seq"]

            sample = TelemetrySample(
                timestamp_s=decoded["timestamp_ms"] * 1e-3,
                x=decoded["x"],
                y=decoded["y"],
                z=decoded["z"],
                score=decoded["score"],
                mode=decoded["mode_str"],
                color=decoded["color"],
                dropped_packets=decoded["dropped_packets"],
                seq=decoded["seq"],
            )
            self.samples.append(sample)
            self._dirty = True
            self.packets_ok += 1
            accepted += 1

            if (self._last_score <= 10 and sample.score >= 75) or (
                sample.mode == "NOMINAL"
                and len(self.samples) > 1
                and self.samples[-2].color != COLOR_MAP["NOMINAL"]
            ):
                self.recovery_points.append((sample.x, sample.y))

            self._last_score = sample.score

        return accepted

    def close(self) -> None:
        self.sock.close()

    def link_status(self) -> str:
        if self.packets_ok == 0 and self.packets_invalid == 0:
            return "esperando"
        if self.packets_ok == 0 and self.packets_invalid > 0:
            return "sin tramas validas"
        if self.seq_gaps > 0:
            return f"degradado ({self.seq_gaps} huecos)"
        return "nominal"
