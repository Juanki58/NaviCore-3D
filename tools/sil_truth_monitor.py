#!/usr/bin/env python3
"""Monitor de consola para SilTruthPacket multi-UAV (validación Paso 2/3)."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sil_protocol import TRUTH_SIZE, unpack_truth  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor SIL truth UDP")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "docs" / "sil_fleet_manifest.example.json",
    )
    parser.add_argument("--duration-s", type=float, default=15.0)
    args = parser.parse_args()

    with args.manifest.open(encoding="utf-8") as handle:
        manifest = json.load(handle)

    sockets: list[tuple[int, socket.socket]] = []
    for entry in manifest["simulation_fleet"]:
        uav_id = int(entry["uav_id"])
        port = int(entry["truth_port"])
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", port))
        sock.setblocking(False)
        sockets.append((uav_id, sock))

    print(f"[*] Escuchando truth en puertos {[entry['truth_port'] for entry in manifest['simulation_fleet']]}")
    print(f"[*] Duración {args.duration_s:.0f}s — Ctrl+C para salir")

    last_alt: dict[int, float] = {}
    deadline = time.time() + args.duration_s

    try:
        while time.time() < deadline:
            for uav_id, sock in sockets:
                while True:
                    try:
                        data, _ = sock.recvfrom(256)
                    except BlockingIOError:
                        break
                    except InterruptedError:
                        break

                    if len(data) != TRUTH_SIZE:
                        continue
                    try:
                        pkt = unpack_truth(data)
                    except ValueError:
                        continue

                    alt_m = -pkt["pos_d_m"]
                    prev = last_alt.get(uav_id)
                    delta = "" if prev is None else f" dAlt={alt_m - prev:+.2f}m"
                    last_alt[uav_id] = alt_m

                    print(
                        f"UAV{uav_id} t={pkt['timestamp_ms']:5d}ms "
                        f"N={pkt['pos_n_m']:7.1f} E={pkt['pos_e_m']:7.1f} "
                        f"alt={alt_m:6.1f}m yaw={pkt['yaw_deg']:6.1f}{delta}"
                    )
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n[*] Monitor detenido")

    for _, sock in sockets:
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
