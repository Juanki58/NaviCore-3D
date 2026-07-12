#!/usr/bin/env python3
"""Prueba de integración: simulador C++ → receptor UDP local."""

from __future__ import annotations

import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

PACKET_FMT = "<fffHH"
PACKET_SIZE = struct.calcsize(PACKET_FMT)
LISTEN_PORT = 5005
SIM_PATH = Path(__file__).resolve().parent.parent / "build" / "NaviCore3D_Sim.exe"


def main() -> int:
    if not SIM_PATH.is_file():
        print(f"[-] No se encontró el simulador: {SIM_PATH}")
        return 1

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", LISTEN_PORT))
    receiver.settimeout(0.2)

    print(f"[*] Escuchando UDP en 127.0.0.1:{LISTEN_PORT}")
    print(f"[*] Lanzando {SIM_PATH.name}...")

    packets: list[bytes] = []
    deadline = time.time() + 25.0
    stdout = ""

    proc = subprocess.Popen(
        [str(SIM_PATH)],
        cwd=SIM_PATH.parent.parent,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        while time.time() < deadline:
            if proc.poll() is not None and not packets:
                break
            try:
                data, _ = receiver.recvfrom(1024)
                if len(data) == PACKET_SIZE:
                    packets.append(data)
            except TimeoutError:
                if proc.poll() is not None:
                    break
    finally:
        if proc.poll() is None:
            proc.kill()
        stdout, _ = proc.communicate(timeout=5)

    receiver.close()

    if proc.returncode not in (None, 0):
        print(f"[-] Simulador terminó con código {proc.returncode}")
        if proc.returncode == 3221225781:
            print("[-] Falta una DLL de runtime (ej. libstdc++ / MinGW). Ejecute el sim desde su entorno habitual.")
        return 1

    if not packets:
        print("[-] No se recibieron paquetes UDP del simulador")
        if stdout:
            print(stdout[-2000:])
        return 1

    print(f"[+] Paquetes UDP recibidos: {len(packets)}")

    first = struct.unpack(PACKET_FMT, packets[0])
    last = struct.unpack(PACKET_FMT, packets[-1])
    print(f"[+] Primer paquete: pos=({first[0]:.4f}, {first[1]:.4f}) score={first[3]} flags={first[4]}")
    print(f"[+] Último paquete:  pos=({last[0]:.4f}, {last[1]:.4f}) score={last[3]} flags={last[4]}")

    invalid = [p for p in packets if len(p) != PACKET_SIZE]
    if invalid:
        print(f"[-] Paquetes con tamaño inválido: {len(invalid)}")
        return 1

    if len(packets) < 100:
        print(f"[-] Muy pocos paquetes para un escenario de 20 s a 10 Hz: {len(packets)}")
        return 1

    print("[+] Integración UDP simulador → receptor: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
