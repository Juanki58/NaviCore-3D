#!/usr/bin/env python3
"""Capture fused NavState CSV from a single USB CDC serial port (Pico / Artemis).

Firmware runs the EKF on-device and prints one NavigationState row per line
(same schema as TelemetryFileLogger). This host tool only opens the port and
appends lines to a file — no second port, no on-PC fusion.

Usage:
  python tools/serial_navstate_capture.py --port COM7 --out docs/benchmarks/field/artemis_run.csv
  python tools/serial_navstate_capture.py --port /dev/ttyUSB0 --baud 115200 --duration 120
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

NAVSTATE_CSV_HEADER = (
    "timestamp_us,lat_rad,lon_rad,alt_m,vn_mps,ve_mps,vd_mps,"
    "roll_rad,pitch_rad,yaw_rad,health_flags,pos_uncertainty_m,att_uncertainty_rad"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dump on-device NavState CSV lines from one serial port."
    )
    parser.add_argument("--port", required=True, help="Serial port (e.g. COM7, /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output CSV path (default: docs/benchmarks/field/navstate_<utc>.csv)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after N seconds (0 = until Ctrl+C)",
    )
    parser.add_argument(
        "--write-header",
        action="store_true",
        help="Write NavigationState header if the output file is new/empty",
    )
    parser.add_argument(
        "--echo",
        action="store_true",
        help="Also print each captured line to stdout",
    )
    return parser.parse_args()


def default_out_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("docs/benchmarks/field") / f"navstate_{stamp}.csv"


def looks_like_navstate_row(line: str) -> bool:
    if not line or line.startswith("#") or "timestamp_us" in line:
        return False
    parts = line.split(",")
    return len(parts) >= 13


def main() -> int:
    args = parse_args()
    try:
        import serial  # type: ignore
    except ImportError:
        print("Missing dependency: pip install pyserial", file=sys.stderr)
        return 2

    out_path = args.out if args.out is not None else default_out_path()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    write_header = args.write_header or (not out_path.exists()) or out_path.stat().st_size == 0

    print(f"[*] Opening {args.port} @ {args.baud}")
    print(f"[*] Writing NavState CSV -> {out_path}")
    print("[*] Expect on-device EKF rows (not raw IMU/GPS fusion on PC). Ctrl+C to stop.")

    rows = 0
    t0 = time.monotonic()
    try:
        with serial.Serial(args.port, args.baud, timeout=1.0) as ser, out_path.open(
            "a", encoding="utf-8", newline="\n"
        ) as fp:
            if write_header:
                fp.write(NAVSTATE_CSV_HEADER + "\n")
                fp.flush()

            while True:
                if args.duration > 0.0 and (time.monotonic() - t0) >= args.duration:
                    break

                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8", errors="replace").strip()
                except Exception:
                    continue
                if not line:
                    continue

                if looks_like_navstate_row(line) or line.startswith("timestamp_us"):
                    if line.startswith("timestamp_us"):
                        continue
                    fp.write(line + "\n")
                    fp.flush()
                    rows += 1
                    if args.echo:
                        print(line)
                elif args.echo:
                    print(f"# {line}")
    except KeyboardInterrupt:
        print("\n[*] Stopped by user")
    except serial.SerialException as exc:
        print(f"[-] Serial error: {exc}", file=sys.stderr)
        return 1

    elapsed = time.monotonic() - t0
    print(f"[*] Captured {rows} NavState rows in {elapsed:.1f}s -> {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
