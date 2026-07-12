#!/usr/bin/env python3
"""
Batería de inyección de fallos — telemetría UDP NaviCore-3D.

Simula condiciones adversas de red y valida que el receptor descarte basura,
detecte huecos de secuencia y mantenga el buffer acotado.
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
    TELEMETRY_UDP_MAGIC,
    pack_packet,
)
from telemetry_receiver import TelemetryReceiver  # noqa: E402


def _open_tx_rx() -> tuple[socket.socket, TelemetryReceiver, int]:
    rx = TelemetryReceiver(host="127.0.0.1", port=0, max_samples=200)
    port = rx.sock.getsockname()[1]
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return tx, rx, port


class TestFaultInjection(unittest.TestCase):
    def test_reject_legacy_16_byte_packets(self) -> None:
        tx, rx, port = _open_tx_rx()
        legacy = struct.pack("<fffHH", 1.0, 2.0, 3.0, 90, 0)
        tx.sendto(legacy, ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 0)
        self.assertEqual(rx.packets_invalid, 1)
        self.assertEqual(rx.link_status(), "sin tramas validas")
        tx.close()
        rx.close()

    def test_reject_truncated_and_oversized(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(b"\x00" * 23, ("127.0.0.1", port))
        tx.sendto(b"\x00" * 32, ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 0)
        self.assertEqual(rx.packets_invalid, 2)
        tx.close()
        rx.close()

    def test_reject_bad_magic(self) -> None:
        tx, rx, port = _open_tx_rx()
        bad = pack_packet(0, 0.0, 0.0, 0.0, 50, 0, 0, magic=0xDEAD)
        tx.sendto(bad, ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 0)
        self.assertEqual(rx.packets_invalid, 1)
        tx.close()
        rx.close()

    def test_garbage_burst_then_valid(self) -> None:
        tx, rx, port = _open_tx_rx()
        for _ in range(20):
            tx.sendto(b"\xFF" * 8, ("127.0.0.1", port))
        good = pack_packet(300, 10.0, 20.0, 0.0, 95, 0, 0, seq=1)
        tx.sendto(good, ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 1)
        self.assertEqual(rx.packets_invalid, 20)
        self.assertEqual(rx.packets_ok, 1)
        self.assertEqual(rx.samples[-1].mode, "NOMINAL")
        self.assertEqual(rx.link_status(), "nominal")
        tx.close()
        rx.close()

    def test_seq_gap_detection(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(pack_packet(0, 0, 0, 0, 100, 0, 0, seq=10), ("127.0.0.1", port))
        tx.sendto(pack_packet(100, 1, 1, 0, 90, 0, 0, seq=12), ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 2)
        self.assertEqual(rx.seq_gaps, 1)
        self.assertEqual(rx.link_status(), "degradado (1 huecos)")
        tx.close()
        rx.close()

    def test_seq_wraparound_no_false_gap(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(pack_packet(0, 0, 0, 0, 100, 0, 0, seq=0xFFFF), ("127.0.0.1", port))
        tx.sendto(pack_packet(100, 1, 0, 0, 95, 0, 0, seq=0), ("127.0.0.1", port))
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
                pack_packet(i * 100, float(i), 0.0, 0.0, 80, 0, 0, seq=i + 1),
                ("127.0.0.1", port),
            )
        self.assertEqual(rx.drain(), 25)
        self.assertEqual(len(rx.samples), 10)
        self.assertEqual(rx.samples[0].x, 15.0)
        self.assertEqual(rx.samples[-1].x, 24.0)
        tx.close()
        rx.close()

    def test_hot_restart_marker_after_critical(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(pack_packet(0, 1.0, 2.0, 0.0, 5, 2, 0, seq=1), ("127.0.0.1", port))
        tx.sendto(pack_packet(100, 1.5, 2.5, 0.0, 80, 0, 0, seq=2), ("127.0.0.1", port))
        rx.drain()
        self.assertEqual(len(rx.recovery_points), 1)
        self.assertAlmostEqual(rx.recovery_points[0][0], 1.5)
        tx.close()
        rx.close()

    def test_recovery_points_capped(self) -> None:
        tx, rx, port = _open_tx_rx()
        seq = 1
        for i in range(60):
            tx.sendto(pack_packet(i * 100, float(i), 0.0, 0.0, 5, 2, 0, seq=seq), ("127.0.0.1", port))
            seq += 1
            tx.sendto(pack_packet(i * 100 + 50, float(i) + 0.5, 0.0, 0.0, 80, 0, 0, seq=seq), ("127.0.0.1", port))
            seq += 1
        rx.drain()
        self.assertEqual(len(rx.recovery_points), 50)
        tx.close()
        rx.close()

    def test_drain_empty_socket_returns_zero(self) -> None:
        rx = TelemetryReceiver(host="127.0.0.1", port=0)
        self.assertEqual(rx.drain(), 0)
        self.assertFalse(rx.dirty)
        rx.close()

    def test_critical_mode_survives_mixed_traffic(self) -> None:
        tx, rx, port = _open_tx_rx()
        tx.sendto(b"\x00" * 4, ("127.0.0.1", port))
        tx.sendto(pack_packet(0, 5.0, 5.0, 0.0, 10, 2, 99, seq=1), ("127.0.0.1", port))
        tx.sendto(pack_packet(0, 0, 0, 0, 50, 0, 0, magic=0x0000), ("127.0.0.1", port))
        self.assertEqual(rx.drain(), 1)
        sample = rx.samples[-1]
        self.assertEqual(sample.mode, "CRITICAL")
        self.assertEqual(sample.color, COLOR_MAP["CRITICAL"])
        self.assertEqual(sample.dropped_packets, 99)
        tx.close()
        rx.close()


def main() -> int:
    print("=" * 60)
    print("NaviCore-3D — Batería de inyección de fallos UDP")
    print("=" * 60)
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    print("=" * 60)
    if result.wasSuccessful():
        print(f"RESULTADO: {result.testsRun} pruebas OK — receptor robusto ante fallos")
    else:
        print(f"RESULTADO: {len(result.failures)} fallos, {len(result.errors)} errores")
    print("=" * 60)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
