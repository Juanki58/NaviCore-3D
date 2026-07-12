#!/usr/bin/env python3
"""Capa de transporte UDP: decode, ring buffer y estadísticas de enlace."""

from __future__ import annotations

import socket
from collections import deque
from dataclasses import dataclass
from typing import Deque

from telemetry_protocol import (
    COLOR_MAP,
    EVENT_SIZE,
    PACKET_SIZE,
    SCENARIO_NAMES,
    TELEMETRY_SCENARIO_UNKNOWN,
    TELEM_EVENT_HOT_RESTART,
    unpack_event,
    unpack_packet,
)


@dataclass(frozen=True, slots=True)
class TelemetrySample:
    timestamp_s: float
    x: float
    y: float
    z: float
    cross_track_m: float
    along_track_m: float
    score: int
    mode: str
    color: str
    dropped_packets: int
    seq: int
    scenario_id: int
    scenario_name: str
    nav_mode: int
    nav_mode_name: str
    temperature_c: float


@dataclass(frozen=True, slots=True)
class TelemetryEvent:
    timestamp_s: float
    event_id: int
    event_name: str
    param: int


class TelemetryReceiver:
    def __init__(self, host: str = "0.0.0.0", port: int = 5005, max_samples: int = 200):
        self.max_samples = max_samples
        self.samples: Deque[TelemetrySample] = deque(maxlen=max_samples)
        self.events: Deque[TelemetryEvent] = deque(maxlen=50)
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
        self.events_ok = 0
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

            if len(data) == EVENT_SIZE:
                try:
                    decoded_event = unpack_event(data)
                except ValueError:
                    self.packets_invalid += 1
                    continue

                event = TelemetryEvent(
                    timestamp_s=decoded_event["timestamp_ms"] * 1e-3,
                    event_id=decoded_event["event_id"],
                    event_name=decoded_event["event_name"],
                    param=decoded_event["param"],
                )
                self.events.append(event)
                self._dirty = True
                self.events_ok += 1

                if event.event_id == TELEM_EVENT_HOT_RESTART and self.samples:
                    last = self.samples[-1]
                    self.recovery_points.append((last.x, last.y))
                continue

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
                cross_track_m=decoded["cross_track_m"],
                along_track_m=decoded["along_track_m"],
                score=decoded["score"],
                mode=decoded["mode_str"],
                color=decoded["color"],
                dropped_packets=decoded["dropped_packets"],
                seq=decoded["seq"],
                scenario_id=decoded["scenario_id"],
                scenario_name=decoded["scenario_name"],
                nav_mode=decoded["nav_mode"],
                nav_mode_name=decoded["nav_mode_name"],
                temperature_c=decoded["temperature_c"],
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
        if self.packets_ok == 0 and self.events_ok == 0 and self.packets_invalid == 0:
            return "esperando"
        if self.packets_ok == 0 and self.events_ok == 0 and self.packets_invalid > 0:
            return "sin tramas validas"
        if self.seq_gaps > 0:
            return f"degradado ({self.seq_gaps} huecos)"
        return "nominal"

    def latest_event_summary(self) -> str:
        if not self.events:
            return "sin eventos"
        last = self.events[-1]
        return f"{last.event_name}({last.param}) @{last.timestamp_s:.1f}s"

    def latest_scenario_name(self) -> str:
        if not self.samples:
            return SCENARIO_NAMES[TELEMETRY_SCENARIO_UNKNOWN]
        return self.samples[-1].scenario_name
