#!/usr/bin/env python3
"""
Batería de inyección de fallos — telemetría UDP NaviCore-3D v3.
"""

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
    TELEMETRY_SCENARIO_HIGH_DEMAND,
    pack_packet,
)
from telemetry_receiver import TelemetryReceiver  # noqa: E402


def _open_tx_rx() -> tuple[socket.socket, TelemetryReceiver, int]:
    rx = TelemetryReceiver(host="127.0.0.1", port=0, max_samples=200)
    port = rx.sock.getsockname()[1]
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return tx, rx, port


class TestFaultInjection(unittest.TestCase):
    def test_reject_legacy_16_and_24_byte_packets(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(struct.pack("<fffHH", 1.0, 2.0, 3.0, 90, 0), ("127.0.0.1", port))
        tx.sendto(pack_packet(0, 1.0, 2.0, 3.0, 0, 0, 90, 0, 0, 0, 0, 25.0)[:24], ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 0)
        self.assertEqual(rx.packets_invalid, 2)
        tx.close()
        rx.close()

    def test_reject_truncated_and_oversized(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(b"\x00" * 31, ("127.0.0.1", port))
        tx.sendto(b"\x00" * 40, ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 0)
        self.assertEqual(rx.packets_invalid, 2)
        tx.close()
        rx.close()

    def test_reject_bad_magic(self) -> None:
        tx, rx, port = _open_tx_rx()
        bad = pack_packet(0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 0, 0, 0, 0, 25.0, magic=0xDEAD)
        tx.sendto(bad, ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 0)
        self.assertEqual(rx.packets_invalid, 1)
        tx.close()
        rx.close()

    def test_garbage_burst_then_valid(self) -> None:
        tx, rx, port = _open_tx_rx()
        for _ in range(20):
            tx.sendto(b"\xFF" * 8, ("127.0.0.1", port))
        good = pack_packet(300, 10.0, 20.0, 0.0, 0.1, 5.0, 95, 0, 0, TELEMETRY_SCENARIO_HIGH_DEMAND, 3, 25.0, seq=1)
        tx.sendto(good, ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 1)
        self.assertEqual(rx.packets_invalid, 20)
        self.assertEqual(rx.samples[-1].scenario_name, "HIGH_DEMAND_STRESS_TEST")
        tx.close()
        rx.close()

    def test_seq_gap_detection(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(pack_packet(0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 25.0, seq=10), ("127.0.0.1", port))
        tx.sendto(pack_packet(100, 1, 1, 0, 0, 0, 90, 0, 0, 0, 0, 25.0, seq=12), ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 2)
        self.assertEqual(rx.seq_gaps, 1)
        tx.close()
        rx.close()

    def test_seq_wraparound_no_false_gap(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(pack_packet(0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 25.0, seq=0xFFFF), ("127.0.0.1", port))
        tx.sendto(pack_packet(100, 1, 0, 0, 0, 0, 95, 0, 0, 0, 0, 25.0, seq=0), ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 2)
        self.assertEqual(rx.seq_gaps, 0)
        tx.close()
        rx.close()

    def test_ring_buffer_caps_at_max_samples(self) -> None:
        tx, rx, port = _open_tx_rx()
        rx.close()
        tx.close()

        rx = TelemetryReceiver(host="127.0.0.1", port=0, max_samples=10)
        port = rx.sock.getsockname()[1]
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for i in range(25):
            tx.sendto(
                pack_packet(i * 100, float(i), 0.0, 0.0, 0.0, float(i), 80, 0, 0, 0, 0, 25.0, seq=i + 1),
                ("127.0.0.1", port),
            )
        self.assertEqual(rx.drain(), 25)
        self.assertEqual(len(rx.samples), 10)
        self.assertEqual(rx.samples[-1].x, 24.0)
        tx.close()
        rx.close()

    def test_hot_restart_marker_after_critical(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(pack_packet(0, 1.0, 2.0, 0.0, 0, 0, 5, 2, 0, 0, 0, 25.0, seq=1), ("127.0.0.1", port))
        tx.sendto(pack_packet(100, 1.5, 2.5, 0.0, 0, 0, 80, 0, 0, 0, 0, 25.0, seq=2), ("127.0.0.1", port))
        rx.drain()
        self.assertEqual(len(rx.recovery_points), 1)
        tx.close()
        rx.close()

    def test_critical_mode_survives_mixed_traffic(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(b"\x00" * 4, ("127.0.0.1", port))
        tx.sendto(pack_packet(0, 5.0, 5.0, 0.0, 2.0, 8.0, 10, 2, 99, 0, 2, 25.0, seq=1), ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 1)
        sample = rx.samples[-1]
        self.assertEqual(sample.mode, "CRITICAL")
        self.assertAlmostEqual(sample.cross_track_m, 2.0)
        self.assertAlmostEqual(sample.along_track_m, 8.0)
        tx.close()
        rx.close()


def main() -> int:
    print("=" * 60)
    print("NaviCore-3D — Batería de inyección de fallos UDP v3")
    print("=" * 60)
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    print("=" * 60)
    print(f"RESULTADO: {result.testsRun} pruebas — {'OK' if result.wasSuccessful() else 'FALLÓ'}")
    print("=" * 60)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
