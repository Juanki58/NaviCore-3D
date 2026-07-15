#!/usr/bin/env python3
"""Pruebas del protocolo UDP Unity unificado (54 B) y legacy v3 (32 B)."""

from __future__ import annotations

import math
import socket
import sys
import unittest
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

from telemetry_protocol import (  # noqa: E402
    COLOR_MAP,
    EVENT_SIZE,
    LEGACY_PACKET_SIZE,
    TELEMETRY_UDP_EVENT_MAGIC,
    TELEMETRY_UDP_MAGIC,
    TELEM_EVENT_GPS_LOST,
    TELEM_EVENT_HOT_RESTART,
    UNITY_PACKET_SIZE,
    UNITY_TELEMETRY_MAGIC,
    pack_event,
    pack_legacy_packet,
    pack_unity_packet,
    quat_to_euler_deg,
    unpack_event,
    unpack_legacy_packet,
    unpack_packet,
)
from telemetry_receiver import TelemetryReceiver  # noqa: E402


class TestUnityPacketCodec(unittest.TestCase):
    def test_packet_size(self) -> None:
        self.assertEqual(UNITY_PACKET_SIZE, 54)

    def test_nominal_roundtrip(self) -> None:
        payload = pack_unity_packet(
            timestamp_ms=1000,
            pos_n_m=10.0,
            pos_e_m=20.0,
            pos_d_m=-3.0,
            vel_n_mps=5.0,
            vel_e_mps=1.0,
            vel_d_mps=-0.2,
            quat_w=1.0,
            quat_x=0.0,
            quat_y=0.0,
            quat_z=0.0,
            nav_mode=3,
            mission_state=3,
            health_mode=0,
            flags=0x0F,
            health_score=95,
            seq=7,
        )
        decoded = unpack_packet(payload)
        self.assertEqual(decoded["magic"], UNITY_TELEMETRY_MAGIC)
        self.assertEqual(decoded["seq"], 7)
        self.assertEqual(decoded["timestamp_ms"], 1000)
        self.assertAlmostEqual(decoded["pos_n_m"], 10.0)
        self.assertAlmostEqual(decoded["pos_e_m"], 20.0)
        self.assertAlmostEqual(decoded["pos_d_m"], -3.0)
        self.assertAlmostEqual(decoded["vel_n_mps"], 5.0)
        self.assertAlmostEqual(decoded["speed_mps"], math.sqrt(5 * 5 + 1 * 1 + 0.2 * 0.2))
        self.assertEqual(decoded["score"], 95)
        self.assertEqual(decoded["mode_str"], "NOMINAL")
        self.assertEqual(decoded["color"], "green")
        self.assertEqual(decoded["mission_state_name"], "NAVIGATE")
        self.assertEqual(decoded["nav_mode_name"], "HYBRID")

    def test_quaternion_to_euler(self) -> None:
        roll, pitch, yaw = quat_to_euler_deg(1.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(roll, 0.0, places=3)
        self.assertAlmostEqual(pitch, 0.0, places=3)
        self.assertAlmostEqual(yaw, 0.0, places=3)

    def test_all_health_modes(self) -> None:
        expected = {
            0: ("NOMINAL", "green"),
            1: ("DEGRADED", "orange"),
            2: ("CRITICAL", "red"),
        }
        for mode, (name, color) in expected.items():
            decoded = unpack_packet(
                pack_unity_packet(
                    0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0, 0, mode, 0, 50
                )
            )
            self.assertEqual(decoded["mode_str"], name)
            self.assertEqual(decoded["color"], color)

    def test_invalid_packet_length_raises(self) -> None:
        with self.assertRaises(ValueError):
            unpack_packet(b"\x00" * 16)

    def test_invalid_magic_raises(self) -> None:
        bad = pack_unity_packet(
            0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 50, magic=0x0000
        )
        with self.assertRaises(ValueError):
            unpack_packet(bad)


class TestLegacyPacketCodec(unittest.TestCase):
    def test_legacy_size(self) -> None:
        self.assertEqual(LEGACY_PACKET_SIZE, 32)

    def test_legacy_roundtrip(self) -> None:
        payload = pack_legacy_packet(
            500, 10.0, 20.0, 0.0, 0.5, 10.0, 95, 0, 0, 0, 3, 25.0
        )
        decoded = unpack_legacy_packet(payload)
        self.assertEqual(decoded["magic"], TELEMETRY_UDP_MAGIC)
        self.assertEqual(decoded["mode_str"], "NOMINAL")


class TestEventCodec(unittest.TestCase):
    def test_event_roundtrip(self) -> None:
        payload = pack_event(2500, TELEM_EVENT_GPS_LOST, 7)
        decoded = unpack_event(payload)
        self.assertEqual(len(payload), EVENT_SIZE)
        self.assertEqual(decoded["magic"], TELEMETRY_UDP_EVENT_MAGIC)
        self.assertEqual(decoded["event_id"], TELEM_EVENT_GPS_LOST)
        self.assertEqual(decoded["param"], 7)
        self.assertEqual(decoded["event_name"], "GPS_LOST")

    def test_hot_restart_event(self) -> None:
        decoded = unpack_event(pack_event(100, TELEM_EVENT_HOT_RESTART, 85))
        self.assertEqual(decoded["event_name"], "HOT_RESTART")
        self.assertEqual(decoded["param"], 85)


class TestUdpLoopback(unittest.TestCase):
    def test_send_receive_loopback(self) -> None:
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2.0)
        port = receiver.getsockname()[1]

        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payload = pack_unity_packet(
            500, 10.0, 20.0, -1.0, 3.0, 4.0, 0.0, 1.0, 0.0, 0.0, 0.0, 3, 2, 0, 0x0F, 95
        )
        sender.sendto(payload, ("127.0.0.1", port))

        data, _ = receiver.recvfrom(1024)
        decoded = unpack_packet(data)
        self.assertEqual(decoded["mode_str"], "NOMINAL")
        self.assertEqual(decoded["mission_state_name"], "READY")

        sender.close()
        receiver.close()


class TestTelemetryReceiver(unittest.TestCase):
    def test_receiver_validates_magic_and_seq_gaps(self) -> None:
        rx = TelemetryReceiver(host="127.0.0.1", port=0)
        port = rx.sock.getsockname()[1]
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        tx.sendto(
            pack_unity_packet(100, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1, 0, 0, 0, 90, seq=1),
            ("127.0.0.1", port),
        )
        tx.sendto(
            pack_unity_packet(200, 2.0, 3.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1, 0, 1, 0, 80, seq=5),
            ("127.0.0.1", port),
        )
        tx.sendto(pack_event(300, TELEM_EVENT_HOT_RESTART, 90), ("127.0.0.1", port))
        tx.sendto(b"\x00" * 4, ("127.0.0.1", port))

        self.assertEqual(rx.drain(), 2)
        self.assertEqual(rx.packets_ok, 2)
        self.assertEqual(rx.events_ok, 1)
        self.assertEqual(rx.packets_invalid, 1)
        self.assertEqual(rx.seq_gaps, 1)
        self.assertAlmostEqual(rx.samples[-1].speed_mps, 1.0)
        self.assertEqual(rx.events[-1].event_name, "HOT_RESTART")

        tx.close()
        rx.close()


def main() -> int:
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
