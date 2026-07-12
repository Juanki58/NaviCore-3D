#!/usr/bin/env python3
"""Pruebas del protocolo UDP de telemetría NaviCore-3D (24 bytes, little-endian)."""

from __future__ import annotations

import socket
import struct
import sys
import unittest
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

from telemetry_protocol import (  # noqa: E402
    COLOR_MAP,
    PACKET_SIZE,
    TELEMETRY_UDP_MAGIC,
    pack_packet,
    unpack_packet,
)
from telemetry_receiver import TelemetryReceiver  # noqa: E402


class TestPacketCodec(unittest.TestCase):
    def test_packet_size(self) -> None:
        self.assertEqual(PACKET_SIZE, 24)

    def test_nominal_roundtrip(self) -> None:
        payload = pack_packet(1000, 10.0, 20.0, 0.0, 95, 0, 0, seq=7)
        decoded = unpack_packet(payload)
        self.assertEqual(decoded["magic"], TELEMETRY_UDP_MAGIC)
        self.assertEqual(decoded["seq"], 7)
        self.assertEqual(decoded["timestamp_ms"], 1000)
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
            decoded = unpack_packet(pack_packet(0, 1.0, 2.0, 3.0, 50, mode, 0))
            self.assertEqual(decoded["mode_str"], name)
            self.assertEqual(decoded["color"], color)

    def test_unknown_mode_defaults_to_critical(self) -> None:
        decoded = unpack_packet(pack_packet(0, 0.0, 0.0, 0.0, 10, 3, 0))
        self.assertEqual(decoded["mode_str"], "CRITICAL")
        self.assertEqual(decoded["color"], "red")

    def test_dropped_packets_encoding(self) -> None:
        decoded = unpack_packet(pack_packet(0, 0.0, 0.0, 0.0, 80, 1, 42))
        self.assertEqual(decoded["mode_str"], "DEGRADED")
        self.assertEqual(decoded["dropped_packets"], 42)

    def test_max_dropped_packets(self) -> None:
        dropped = 16383
        decoded = unpack_packet(pack_packet(0, 0.0, 0.0, 0.0, 100, 0, dropped))
        self.assertEqual(decoded["dropped_packets"], dropped)

    def test_invalid_packet_length_raises(self) -> None:
        with self.assertRaises(ValueError):
            unpack_packet(b"\x00" * 8)

    def test_invalid_magic_raises(self) -> None:
        bad = pack_packet(0, 0.0, 0.0, 0.0, 50, 0, 0, magic=0x0000)
        with self.assertRaises(ValueError):
            unpack_packet(bad)


class TestUdpLoopback(unittest.TestCase):
    def test_send_receive_loopback(self) -> None:
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2.0)
        port = receiver.getsockname()[1]

        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payload = pack_packet(500, 10.0, 20.0, 0.0, 95, 0, 0)
        sender.sendto(payload, ("127.0.0.1", port))

        data, _ = receiver.recvfrom(1024)
        decoded = unpack_packet(data)
        self.assertEqual(decoded["mode_str"], "NOMINAL")
        self.assertEqual(decoded["color"], "green")

        sender.close()
        receiver.close()


class TestTelemetryReceiver(unittest.TestCase):
    def test_receiver_validates_magic_and_seq_gaps(self) -> None:
        rx = TelemetryReceiver(host="127.0.0.1", port=0)
        port = rx.sock.getsockname()[1]
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        tx.sendto(pack_packet(100, 1.0, 2.0, 0.0, 90, 0, 0, seq=1), ("127.0.0.1", port))
        tx.sendto(pack_packet(200, 2.0, 3.0, 0.0, 80, 1, 0, seq=5), ("127.0.0.1", port))
        tx.sendto(b"\x00" * 8, ("127.0.0.1", port))

        self.assertEqual(rx.drain(), 2)
        self.assertEqual(rx.packets_ok, 2)
        self.assertEqual(rx.packets_invalid, 1)
        self.assertEqual(rx.seq_gaps, 1)
        self.assertEqual(len(rx.samples), 2)
        self.assertAlmostEqual(rx.samples[-1].timestamp_s, 0.2)

        tx.close()
        rx.close()


def main() -> int:
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
