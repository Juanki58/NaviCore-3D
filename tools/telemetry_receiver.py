#!/usr/bin/env python3
"""Capa de transporte UDP: decode UnityTelemetryPacket, ring buffer y estadísticas."""

from __future__ import annotations

import socket
from collections import deque
from dataclasses import dataclass
from typing import Deque

from telemetry_protocol import (
    COLOR_MAP,
    EVENT_SIZE,
    UNITY_PACKET_SIZE,
    UNITY_TELEMETRY_DEFAULT_PORT,
    TELEM_EVENT_HOT_RESTART,
    unpack_event,
    unpack_packet,
)


@dataclass(frozen=True, slots=True)
class TelemetrySample:
    timestamp_s: float
    pos_n_m: float
    pos_e_m: float
    pos_d_m: float
    vel_n_mps: float
    vel_e_mps: float
    vel_d_mps: float
    speed_mps: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    score: int
    mode: str
    color: str
    seq: int
    nav_mode: int
    nav_mode_name: str
    mission_state: int
    mission_state_name: str
    flags: int
    # Alias para paneles que usaban x/y
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class TelemetryEvent:
    timestamp_s: float
    event_id: int
    event_name: str
    param: int


class TelemetryReceiver:
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = UNITY_TELEMETRY_DEFAULT_PORT,
        max_samples: int = 500,
    ):
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
                    self.recovery_points.append((last.pos_n_m, last.pos_e_m))
                continue

            if len(data) != UNITY_PACKET_SIZE:
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
                pos_n_m=decoded["pos_n_m"],
                pos_e_m=decoded["pos_e_m"],
                pos_d_m=decoded["pos_d_m"],
                vel_n_mps=decoded["vel_n_mps"],
                vel_e_mps=decoded["vel_e_mps"],
                vel_d_mps=decoded["vel_d_mps"],
                speed_mps=decoded["speed_mps"],
                roll_deg=decoded["roll_deg"],
                pitch_deg=decoded["pitch_deg"],
                yaw_deg=decoded["yaw_deg"],
                score=decoded["score"],
                mode=decoded["mode_str"],
                color=decoded["color"],
                seq=decoded["seq"],
                nav_mode=decoded["nav_mode"],
                nav_mode_name=decoded["nav_mode_name"],
                mission_state=decoded["mission_state"],
                mission_state_name=decoded["mission_state_name"],
                flags=decoded["flags"],
                x=decoded["x"],
                y=decoded["y"],
                z=decoded["z"],
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
                self.recovery_points.append((sample.pos_n_m, sample.pos_e_m))

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

    def latest_mission_state_name(self) -> str:
        if not self.samples:
            return "UNKNOWN"
        return self.samples[-1].mission_state_name
