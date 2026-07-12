#!/usr/bin/env python3
"""Lanza una instancia del bridge SIL por cada UAV del manifiesto."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BRIDGE = Path(__file__).resolve().parent / "jsbsim_sil_bridge.py"


def main() -> int:
    parser = argparse.ArgumentParser(description="NaviCore-3D SIL fleet launcher")
    parser.add_argument(
        "--manifest",
        "--config",
        type=Path,
        dest="manifest",
        default=ROOT / "docs" / "sil_fleet_manifest.example.json",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--mode", choices=["synthetic", "jsbsim"], default="synthetic")
    parser.add_argument("--uav-ids", type=str, default="", help="Lista opcional, ej. 1,3,7")
    args = parser.parse_args()

    with args.manifest.open(encoding="utf-8") as handle:
        manifest = json.load(handle)

    if args.uav_ids.strip():
        selected = {int(x.strip()) for x in args.uav_ids.split(",") if x.strip()}
    else:
        selected = {int(entry["uav_id"]) for entry in manifest["simulation_fleet"]}

    procs: list[subprocess.Popen[bytes]] = []
    print(f"[*] Lanzando {len(selected)} bridge(s) SIL desde {args.manifest}")

    for entry in manifest["simulation_fleet"]:
        uav_id = int(entry["uav_id"])
        if uav_id not in selected:
            continue
        cmd = [
            sys.executable,
            str(BRIDGE),
            "--manifest",
            str(args.manifest),
            "--uav-id",
            str(uav_id),
            "--host",
            args.host,
            "--duration-s",
            str(args.duration_s),
            "--mode",
            args.mode,
        ]
        procs.append(subprocess.Popen(cmd))
        print(f"    UAV {uav_id} -> truth:{entry['truth_port']} sensor:{entry['sensor_port']}")

    try:
        while any(p.poll() is None for p in procs):
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[*] Deteniendo flota SIL...")
        for proc in procs:
            if proc.poll() is None:
                proc.terminate()
        for proc in procs:
            proc.wait(timeout=5)

    failed = [p.returncode for p in procs if p.returncode not in (0, None)]
    if failed:
        print(f"[-] Algunos bridges terminaron con error: {failed}")
        return 1

    print("[+] Flota SIL finalizada")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
