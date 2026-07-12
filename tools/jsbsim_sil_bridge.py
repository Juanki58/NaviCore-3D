#!/usr/bin/env python3
"""Bridge SIL: publica SilTruthPacket y SilSensorPacket por UAV (sintético o JSBSim)."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path
from typing import Any

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sil_protocol import (  # noqa: E402
    SIL_FLAG_ATT_VALID,
    SIL_FLAG_GPS_VALID,
    SIL_FLAG_IMU_VALID,
    SIL_FLAG_MAG_VALID,
    SIL_FLAG_POS_VALID,
    SIL_FLAG_VEL_VALID,
    SIL_SURFACE_THROTTLE,
    pack_sensor,
    pack_truth,
    unpack_actuator,
)

GRAVITY_MPS2 = 9.80665
METERS_PER_DEG_LAT = 111_320.0

SURFACE_MAP = {
    "throttle": SIL_SURFACE_THROTTLE,
    "aileron": 1,
    "elevator": 2,
    "rudder": 3,
}


def load_manifest(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def find_uav_entry(manifest: dict[str, Any], uav_id: int) -> dict[str, Any]:
    for entry in manifest.get("simulation_fleet", []):
        if int(entry["uav_id"]) == uav_id:
            return entry
    raise KeyError(f"uav_id={uav_id} no encontrado en el manifiesto")


def latlon_to_ned(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    origin_lat: float,
    origin_lon: float,
    origin_alt_m: float,
) -> tuple[float, float, float]:
    cos_lat = math.cos(math.radians(origin_lat))
    north_m = (lat_deg - origin_lat) * METERS_PER_DEG_LAT
    east_m = (lon_deg - origin_lon) * METERS_PER_DEG_LAT * cos_lat
    down_m = origin_alt_m - alt_m
    return north_m, east_m, down_m


def ned_to_latlon(
    north_m: float,
    east_m: float,
    down_m: float,
    origin_lat: float,
    origin_lon: float,
    origin_alt_m: float,
) -> tuple[float, float, float]:
    cos_lat = math.cos(math.radians(origin_lat))
    lat_deg = origin_lat + (north_m / METERS_PER_DEG_LAT)
    lon_deg = origin_lon + (east_m / (METERS_PER_DEG_LAT * cos_lat))
    alt_m = origin_alt_m - down_m
    return lat_deg, lon_deg, alt_m


def synthetic_state(
    t_s: float,
    uav_id: int,
    entry: dict[str, Any],
    origin: dict[str, float],
    throttle_cmd: float,
) -> tuple[dict[str, float], dict[str, float]]:
    init = entry["initial_position"]
    radius_m = 8.0 + uav_id * 2.0
    omega = 0.15 + uav_id * 0.02
    phase = uav_id * 0.9

    north_m = radius_m * math.cos(omega * t_s + phase)
    east_m = radius_m * math.sin(omega * t_s + phase)
    climb_mps = max(0.0, throttle_cmd) * 3.0
    alt_m = init["alt_m"] + climb_mps * t_s
    down_m = origin["alt_m"] - alt_m

    vel_n = -radius_m * omega * math.sin(omega * t_s + phase)
    vel_e = radius_m * omega * math.cos(omega * t_s + phase)
    vel_d = -climb_mps

    yaw_deg = math.degrees(math.atan2(vel_e, vel_n)) if abs(vel_n) + abs(vel_e) > 0.01 else 0.0
    lat_deg, lon_deg, alt_m = ned_to_latlon(north_m, east_m, down_m, origin["lat_deg"], origin["lon_deg"], origin["alt_m"])

    truth = {
        "pos_n_m": north_m,
        "pos_e_m": east_m,
        "pos_d_m": down_m,
        "vel_n_mps": vel_n,
        "vel_e_mps": vel_e,
        "vel_d_mps": vel_d,
        "roll_deg": 2.0 * math.sin(t_s * 0.5),
        "pitch_deg": 3.0 * throttle_cmd,
        "yaw_deg": yaw_deg,
    }
    gps = {
        "lat_deg": lat_deg,
        "lon_deg": lon_deg,
        "alt_m": alt_m,
        "speed_mps": math.sqrt(vel_n * vel_n + vel_e * vel_e + vel_d * vel_d),
        "course_deg": (yaw_deg + 360.0) % 360.0,
    }
    return truth, gps


class SilBridge:
    def __init__(
        self,
        entry: dict[str, Any],
        origin: dict[str, float],
        tick_rate_hz: float,
        host: str,
        mode: str,
    ):
        self.entry = entry
        self.origin = origin
        self.tick_rate_hz = tick_rate_hz
        self.host = host
        self.mode = mode
        self.uav_id = int(entry["uav_id"])
        self.seq = 0
        self.throttle_cmd = 0.0

        self.truth_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.actuator_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.actuator_sock.bind(("0.0.0.0", int(entry["actuator_port"])))
        self.actuator_sock.setblocking(False)

        self.truth_addr = (host, int(entry["truth_port"]))
        self.sensor_addr = (host, int(entry["sensor_port"]))

        step = entry.get("control_step")
        self.control_step = step
        if step and step.get("surface") == "throttle":
            self._step_value = float(step.get("value", 1.0))
            self._step_start_ms = int(step.get("start_ms", 5000))
        else:
            self._step_value = 0.0
            self._step_start_ms = -1

    def _drain_actuator(self) -> None:
        while True:
            try:
                data, _ = self.actuator_sock.recvfrom(256)
            except BlockingIOError:
                break
            except InterruptedError:
                break

            try:
                cmd = unpack_actuator(data)
            except ValueError:
                continue

            if cmd["uav_id"] != self.uav_id:
                continue
            if cmd["surface_id"] == SIL_SURFACE_THROTTLE:
                self.throttle_cmd = max(-1.0, min(1.0, cmd["command_norm"]))

    def _apply_control_step(self, t_ms: int) -> None:
        if self._step_start_ms >= 0 and t_ms >= self._step_start_ms:
            self.throttle_cmd = max(self.throttle_cmd, self._step_value)

    def tick(self, t_ms: int) -> None:
        self._drain_actuator()
        self._apply_control_step(t_ms)
        t_s = t_ms * 0.001

        if self.mode == "jsbsim":
            raise NotImplementedError(
                "Modo jsbsim: conecte JSBSim vía FGNative o script propio; use --mode synthetic para validar red."
            )

        truth, gps = synthetic_state(t_s, self.uav_id, self.entry, self.origin, self.throttle_cmd)

        truth_payload = pack_truth(
            uav_id=self.uav_id,
            timestamp_ms=t_ms,
            pos_n_m=truth["pos_n_m"],
            pos_e_m=truth["pos_e_m"],
            pos_d_m=truth["pos_d_m"],
            vel_n_mps=truth["vel_n_mps"],
            vel_e_mps=truth["vel_e_mps"],
            vel_d_mps=truth["vel_d_mps"],
            roll_deg=truth["roll_deg"],
            pitch_deg=truth["pitch_deg"],
            yaw_deg=truth["yaw_deg"],
            flags=SIL_FLAG_POS_VALID | SIL_FLAG_ATT_VALID | SIL_FLAG_VEL_VALID,
            seq=self.seq,
        )
        sensor_payload = pack_sensor(
            uav_id=self.uav_id,
            timestamp_ms=t_ms,
            accel_mps2=(0.0, 0.0, GRAVITY_MPS2),
            gyro_radps=(0.0, 0.0, 0.02 * math.sin(t_s)),
            mag_ut=(22.0, 4.0, 42.0),
            lat_deg=gps["lat_deg"],
            lon_deg=gps["lon_deg"],
            alt_m=gps["alt_m"],
            speed_mps=gps["speed_mps"],
            course_deg=gps["course_deg"],
            satellites=14,
            fix_valid=True,
            flags=SIL_FLAG_IMU_VALID | SIL_FLAG_GPS_VALID | SIL_FLAG_MAG_VALID,
            seq=self.seq,
        )

        self.truth_sock.sendto(truth_payload, self.truth_addr)
        self.sensor_sock.sendto(sensor_payload, self.sensor_addr)
        self.seq = (self.seq + 1) & 0xFFFF

    def run(self, duration_s: float) -> None:
        dt_s = 1.0 / self.tick_rate_hz
        deadline = time.perf_counter() + duration_s
        t_ms = 0

        print(
            f"[*] SIL bridge UAV {self.uav_id} | mode={self.mode} | "
            f"truth->{self.truth_addr} sensor->{self.sensor_addr} | {self.tick_rate_hz:.0f} Hz"
        )

        while time.perf_counter() < deadline:
            tick_start = time.perf_counter()
            self.tick(t_ms)
            t_ms += int(dt_s * 1000.0)

            elapsed = time.perf_counter() - tick_start
            sleep_s = dt_s - elapsed
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    def close(self) -> None:
        self.truth_sock.close()
        self.sensor_sock.close()
        self.actuator_sock.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="NaviCore-3D JSBSim SIL bridge (UDP truth + sensor)")
    parser.add_argument("--manifest", type=Path, default=Path("docs/sil_fleet_manifest.example.json"))
    parser.add_argument("--uav-id", type=int, required=True, choices=range(1, 8))
    parser.add_argument("--host", default="127.0.0.1", help="Destino UDP (motor gráfico / NaviCore)")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--mode", choices=["synthetic", "jsbsim"], default="synthetic")
    args = parser.parse_args()

    manifest_path = args.manifest
    if not manifest_path.is_file():
        manifest_path = Path(__file__).resolve().parent.parent / args.manifest

    manifest = load_manifest(manifest_path)
    entry = find_uav_entry(manifest, args.uav_id)
    origin = manifest.get("origin", entry["initial_position"])
    tick_rate_hz = float(manifest.get("tick_rate_hz", 100.0))

    bridge = SilBridge(entry, origin, tick_rate_hz, args.host, args.mode)
    try:
        bridge.run(args.duration_s)
    except KeyboardInterrupt:
        print("\n[*] Bridge detenido por usuario")
    finally:
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
