#!/usr/bin/env python3
"""Receptor UDP para NavigationState (64 bytes) emitido por NaviCore3D_Sim."""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
from dataclasses import dataclass
from typing import Iterable

NAVIGATION_STATE_SIZE = 64
NAVIGATION_STATE_STRUCT = struct.Struct("<QddfffffffIff")

NAV_STATE_FLAG_EKF_VALID = 1 << 0
NAV_STATE_FLAG_GPS_FIX = 1 << 1
NAV_STATE_FLAG_NHC_ENABLED = 1 << 2
NAV_STATE_FLAG_GNSS_OUTLIER = 1 << 3
NAV_STATE_FLAG_DEAD_RECKONING = 1 << 4

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 9090


@dataclass(frozen=True, slots=True)
class NavigationState:
    timestamp_us: int
    lat_rad: float
    lon_rad: float
    alt_m: float
    vn_mps: float
    ve_mps: float
    vd_mps: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    health_flags: int
    pos_uncertainty_m: float
    att_uncertainty_rad: float

    @property
    def timestamp_s(self) -> float:
        return self.timestamp_us * 1e-6

    @property
    def lat_deg(self) -> float:
        return math.degrees(self.lat_rad)

    @property
    def lon_deg(self) -> float:
        return math.degrees(self.lon_rad)

    @property
    def speed_mps(self) -> float:
        return math.sqrt(
            (self.vn_mps * self.vn_mps)
            + (self.ve_mps * self.ve_mps)
            + (self.vd_mps * self.vd_mps)
        )


def decode_health_flags(flags: int) -> list[str]:
    labels: list[tuple[int, str]] = [
        (NAV_STATE_FLAG_EKF_VALID, "EKF_VALID"),
        (NAV_STATE_FLAG_GPS_FIX, "GPS_FIX"),
        (NAV_STATE_FLAG_NHC_ENABLED, "NHC_ENABLED"),
        (NAV_STATE_FLAG_GNSS_OUTLIER, "GNSS_OUTLIER"),
        (NAV_STATE_FLAG_DEAD_RECKONING, "DEAD_RECKONING"),
    ]
    active = [name for bit, name in labels if flags & bit]
    return active if active else ["NONE"]


def unpack_navigation_state(payload: bytes) -> NavigationState:
    if len(payload) != NAVIGATION_STATE_SIZE:
        raise ValueError(
            f"paquete invalido: esperados {NAVIGATION_STATE_SIZE} bytes, recibidos {len(payload)}"
        )

    (
        timestamp_us,
        lat_rad,
        lon_rad,
        alt_m,
        vn_mps,
        ve_mps,
        vd_mps,
        roll_rad,
        pitch_rad,
        yaw_rad,
        health_flags,
        pos_uncertainty_m,
        att_uncertainty_rad,
    ) = NAVIGATION_STATE_STRUCT.unpack(payload)

    return NavigationState(
        timestamp_us=timestamp_us,
        lat_rad=lat_rad,
        lon_rad=lon_rad,
        alt_m=alt_m,
        vn_mps=vn_mps,
        ve_mps=ve_mps,
        vd_mps=vd_mps,
        roll_rad=roll_rad,
        pitch_rad=pitch_rad,
        yaw_rad=yaw_rad,
        health_flags=health_flags,
        pos_uncertainty_m=pos_uncertainty_m,
        att_uncertainty_rad=att_uncertainty_rad,
    )


def format_navigation_state(state: NavigationState, source: str) -> str:
    flags = ",".join(decode_health_flags(state.health_flags))
    return (
        f"[{source}] t={state.timestamp_s:8.3f}s | "
        f"lat={state.lat_deg:+.6f} lon={state.lon_deg:+.6f} alt={state.alt_m:7.2f} m | "
        f"vel_ned=({state.vn_mps:+.2f},{state.ve_mps:+.2f},{state.vd_mps:+.2f}) m/s "
        f"| speed={state.speed_mps:.2f} m/s | "
        f"att_rpy=({math.degrees(state.roll_rad):+.1f},"
        f"{math.degrees(state.pitch_rad):+.1f},"
        f"{math.degrees(state.yaw_rad):+.1f}) deg | "
        f"flags=0x{state.health_flags:02X} [{flags}] | "
        f"sigma_pos={state.pos_uncertainty_m:.3f} m "
        f"sigma_att={math.degrees(state.att_uncertainty_rad):.3f} deg"
    )


def listen(host: str, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))

    print(f"Escuchando NavigationState UDP en {host}:{port} ({NAVIGATION_STATE_SIZE} bytes/paquete)")
    print("Esperando datos de NaviCore3D_Sim... (Ctrl+C para salir)")

    packets_ok = 0
    packets_invalid = 0

    try:
        while True:
            payload, addr = sock.recvfrom(NAVIGATION_STATE_SIZE + 64)
            source = f"{addr[0]}:{addr[1]}"

            try:
                state = unpack_navigation_state(payload)
            except ValueError as exc:
                packets_invalid += 1
                print(f"[WARN] {source}: {exc}", file=sys.stderr)
                continue

            packets_ok += 1
            print(format_navigation_state(state, source))
    except KeyboardInterrupt:
        print()
        print(
            f"Fin. paquetes_ok={packets_ok} paquetes_invalidos={packets_invalid}"
        )
    finally:
        sock.close()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Receptor UDP del struct NavigationState (64 bytes) de NaviCore-3D."
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"Direccion de escucha (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"Puerto UDP (default: {DEFAULT_PORT})",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_arg_parser().parse_args(list(argv) if argv is not None else None)
    listen(args.host, args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
