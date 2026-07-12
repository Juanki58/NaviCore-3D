#!/usr/bin/env python3
"""Pruebas del protocolo UDP de telemetría NaviCore-3D (16 bytes, little-endian)."""

from __future__ import annotations

import socket
import struct
import sys
import unittest

PACKET_FMT = "<fffHH"
PACKET_SIZE = struct.calcsize(PACKET_FMT)

HEALTH_MODES = {0: "NOMINAL", 1: "DEGRADED", 2: "CRITICAL"}
COLOR_MAP = {"NOMINAL": "green", "DEGRADED": "orange", "CRITICAL": "red"}


def pack_packet(
    x: float,
    y: float,
    z: float,
    score: int,
    health_mode: int,
    dropped: int,
) -> bytes:
    flags = (health_mode & 0x03) | ((dropped & 0x3FFF) << 2)
    return struct.pack(PACKET_FMT, x, y, z, score, flags)


def unpack_packet(data: bytes) -> dict:
    if len(data) != PACKET_SIZE:
        raise ValueError(f"Paquete invalido: {len(data)} bytes (esperado {PACKET_SIZE})")

    x, y, z, score, flags = struct.unpack(PACKET_FMT, data)
    mode_bits = flags & 0x03
    dropped_packets = flags >> 2
    mode_str = HEALTH_MODES.get(mode_bits, "CRITICAL")

    return {
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


class TestPacketCodec(unittest.TestCase):
    def test_packet_size(self) -> None:
        self.assertEqual(PACKET_SIZE, 16)

    def test_nominal_roundtrip(self) -> None:
        payload = pack_packet(10.0, 20.0, 0.0, 95, 0, 0)
        decoded = unpack_packet(payload)
        self.assertAlmostEqual(decoded["x"], 10.0)
        self.assertAlmostEqual(decoded["y"], 20.0)
        self.assertEqual(decoded["score"], 95)
        self.assertEqual(decoded["mode_str"], "NOMINAL")
        self.assertEqual(decoded["color"], "green")
        self.assertEqual(decoded["dropped_packets"], 0)

    def test_all_health_modes(self) -> None:
        expected = {
            0: ("NOMINAL", "green"),
            1: ("DEGRADED", "orange"),
            2: ("CRITICAL", "red"),
        }
        for mode, (name, color) in expected.items():
            decoded = unpack_packet(pack_packet(1.0, 2.0, 3.0, 50, mode, 0))
            self.assertEqual(decoded["mode_str"], name)
            self.assertEqual(decoded["color"], color)

    def test_unknown_mode_defaults_to_critical(self) -> None:
        decoded = unpack_packet(pack_packet(0.0, 0.0, 0.0, 10, 3, 0))
        self.assertEqual(decoded["mode_str"], "CRITICAL")
        self.assertEqual(decoded["color"], "red")

    def test_dropped_packets_encoding(self) -> None:
        decoded = unpack_packet(pack_packet(0.0, 0.0, 0.0, 80, 1, 42))
        self.assertEqual(decoded["mode_str"], "DEGRADED")
        self.assertEqual(decoded["dropped_packets"], 42)

    def test_max_dropped_packets(self) -> None:
        dropped = 16383
        decoded = unpack_packet(pack_packet(0.0, 0.0, 0.0, 100, 0, dropped))
        self.assertEqual(decoded["dropped_packets"], dropped)

    def test_invalid_packet_length_raises(self) -> None:
        with self.assertRaises(ValueError):
            unpack_packet(b"\x00" * 8)


class TestUdpLoopback(unittest.TestCase):
    def test_send_receive_loopback(self) -> None:
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2.0)
        port = receiver.getsockname()[1]

        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payload = pack_packet(10.0, 20.0, 0.0, 95, 0, 0)
        sender.sendto(payload, ("127.0.0.1", port))

        data, _ = receiver.recvfrom(1024)
        decoded = unpack_packet(data)
        self.assertEqual(decoded["mode_str"], "NOMINAL")
        self.assertEqual(decoded["color"], "green")

        sender.close()
        receiver.close()


def main() -> int:
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
