#!/usr/bin/env python3
"""Pruebas del protocolo SIL (truth / sensor / actuator)."""

from __future__ import annotations

import socket
import sys
import unittest
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

from sil_protocol import (  # noqa: E402
    ACTUATOR_SIZE,
    SENSOR_SIZE,
    SIL_ACTUATOR_MAGIC,
    SIL_SENSOR_MAGIC,
    SIL_TRUTH_MAGIC,
    TRUTH_SIZE,
    pack_actuator,
    pack_sensor,
    pack_truth,
    sil_sensor_port,
    sil_truth_port,
    unpack_actuator,
    unpack_sensor,
    unpack_truth,
)


class TestSilPacketSizes(unittest.TestCase):
    def test_sizes(self) -> None:
        self.assertEqual(TRUTH_SIZE, 48)
        self.assertEqual(SENSOR_SIZE, 70)
        self.assertEqual(ACTUATOR_SIZE, 16)


class TestSilCodec(unittest.TestCase):
    def test_truth_roundtrip(self) -> None:
        payload = pack_truth(
            3, 5000, 10.0, -5.0, -15.0, 1.0, 0.5, -2.0, 2.0, -3.0, 90.0, seq=12
        )
        decoded = unpack_truth(payload)
        self.assertEqual(decoded["magic"], SIL_TRUTH_MAGIC)
        self.assertEqual(decoded["uav_id"], 3)
        self.assertEqual(decoded["seq"], 12)
        self.assertAlmostEqual(decoded["pos_d_m"], -15.0)
        self.assertAlmostEqual(decoded["yaw_deg"], 90.0)

    def test_sensor_roundtrip(self) -> None:
        payload = pack_sensor(
            5,
            1000,
            (0.0, 0.0, 9.81),
            (0.0, 0.0, 0.01),
            (20.0, 3.0, 40.0),
            41.3878,
            2.1690,
            13.0,
            12.5,
            45.0,
            satellites=11,
            fix_valid=True,
            seq=3,
        )
        decoded = unpack_sensor(payload)
        self.assertEqual(decoded["magic"], SIL_SENSOR_MAGIC)
        self.assertEqual(decoded["uav_id"], 5)
        self.assertAlmostEqual(decoded["lat_deg"], 41.3878, places=4)
        self.assertTrue(decoded["fix_valid"])

    def test_actuator_roundtrip(self) -> None:
        payload = pack_actuator(3, 5000, 0, 1.0, seq=1)
        decoded = unpack_actuator(payload)
        self.assertEqual(decoded["magic"], SIL_ACTUATOR_MAGIC)
        self.assertEqual(decoded["surface_id"], 0)
        self.assertAlmostEqual(decoded["command_norm"], 1.0)

    def test_invalid_truth_length(self) -> None:
        with self.assertRaises(ValueError):
            unpack_truth(b"\x00" * 16)


class TestSilMultiUavIsolation(unittest.TestCase):
    def test_truth_ports_isolated(self) -> None:
        rx1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx1.bind(("127.0.0.1", 0))
        rx2.bind(("127.0.0.1", 0))
        port1 = rx1.getsockname()[1]
        port2 = rx2.getsockname()[1]

        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx.sendto(pack_truth(1, 100, 0, 0, -10, 0, 0, 0, 0, 0, 0, 0), ("127.0.0.1", port1))
        tx.sendto(pack_truth(2, 100, 50, 0, -20, 0, 0, 0, 0, 0, 0, 0), ("127.0.0.1", port2))

        d1 = unpack_truth(rx1.recvfrom(256)[0])
        d2 = unpack_truth(rx2.recvfrom(256)[0])
        self.assertEqual(d1["uav_id"], 1)
        self.assertEqual(d2["uav_id"], 2)
        self.assertAlmostEqual(d1["pos_d_m"], -10.0)
        self.assertAlmostEqual(d2["pos_d_m"], -20.0)

        rx1.close()
        rx2.close()
        tx.close()

    def test_default_port_convention(self) -> None:
        self.assertEqual(sil_truth_port(1), 5301)
        self.assertEqual(sil_truth_port(7), 5307)
        self.assertEqual(sil_sensor_port(3), 5403)


def main() -> int:
    suite = unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
