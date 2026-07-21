#!/usr/bin/env python3
"""Orquestador de la suite de regresion NaviCore-3D."""

from __future__ import annotations

import argparse
import subprocess
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = REPO_ROOT / "build"
TOOLS_DIR = REPO_ROOT / "tools"


def run_command(label: str, command: list[str], cwd: Path | None = None) -> int:
    print(f"\n== {label} ==")
    print(" ".join(command))
    completed = subprocess.run(command, cwd=cwd or REPO_ROOT, check=False)
    if completed.returncode == 0:
        print(f"OK: {label}")
    else:
        print(f"FAIL: {label} (exit={completed.returncode})")
    return completed.returncode


def run_python_unittests(pattern: str) -> int:
    print(f"\n== python unittest ({pattern}) ==")
    suite = unittest.TestLoader().discover(str(TOOLS_DIR), pattern=pattern)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    if result.wasSuccessful():
        print(f"OK: python unittest ({pattern})")
        return 0
    print(f"FAIL: python unittest ({pattern})")
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Ejecuta la suite de regresion NaviCore-3D")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=BUILD_DIR,
        help="Directorio de build CMake (default: build/)",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="No recompilar antes de ejecutar pruebas C++",
    )
    parser.add_argument(
        "--skip-python",
        action="store_true",
        help="Omitir pruebas Python de protocolo/telemetria",
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="Incluir ring_stress_test (60 s, host UART SPSC)",
    )
    parser.add_argument(
        "--safety-inject",
        action="store_true",
        default=True,
        help="C++: solo suite safety-inject (default; CI-friendly)",
    )
    parser.add_argument(
        "--full-cpp",
        action="store_true",
        help="C++: suite completa (incluye NHC/TC legacy que pueden FAIL)",
    )
    parser.add_argument(
        "--unit",
        action="store_true",
        default=True,
        help="También ejecutar navicore_unit_tests (Catch2; default on)",
    )
    parser.add_argument(
        "--skip-unit",
        action="store_true",
        help="Omitir Catch2 unit tests",
    )
    args = parser.parse_args()

    build_dir = args.build_dir.resolve()
    regression_bin = build_dir / (
        "navicore_regression_test.exe" if sys.platform == "win32" else "navicore_regression_test"
    )
    unit_bin = build_dir / (
        "navicore_unit_tests.exe" if sys.platform == "win32" else "navicore_unit_tests"
    )
    ring_stress_bin = build_dir / (
        "ring_stress_test.exe" if sys.platform == "win32" else "ring_stress_test"
    )

    failures = 0

    if not args.skip_build:
        failures += run_command(
            "cmake build (regression targets)",
            [
                "cmake",
                "--build",
                str(build_dir),
                "--target",
                "navicore_regression_test",
            ],
        )
        if not args.skip_unit:
            failures += run_command(
                "cmake build (navicore_unit_tests)",
                [
                    "cmake",
                    "--build",
                    str(build_dir),
                    "--target",
                    "navicore_unit_tests",
                ],
            )
        if args.full:
            failures += run_command(
                "cmake build (ring_stress_test)",
                ["cmake", "--build", str(build_dir), "--target", "ring_stress_test"],
            )

    if failures == 0 and not args.skip_unit:
        if unit_bin.exists():
            failures += run_command("navicore_unit_tests (Catch2)", [str(unit_bin)])
        else:
            print(f"WARN: no existe {unit_bin} (¿NAVICORE_BUILD_UNIT_TESTS=OFF?)")

    if failures == 0:
        if not regression_bin.exists():
            print(f"FAIL: no existe {regression_bin}")
            failures += 1
        else:
            cpp_cmd = [str(regression_bin)]
            if not args.full_cpp:
                cpp_cmd.append("--safety-inject")
            label = (
                "navicore_regression_test (full)"
                if args.full_cpp
                else "navicore_regression_test --safety-inject"
            )
            failures += run_command(label, cpp_cmd)

    if failures == 0 and args.full:
        if not ring_stress_bin.exists():
            print(f"FAIL: no existe {ring_stress_bin}")
            failures += 1
        else:
            failures += run_command("ring_stress_test", [str(ring_stress_bin)])

    if not args.skip_python and failures == 0:
        failures += run_python_unittests("test_sil_protocol.py")
        failures += run_python_unittests("test_udp_telemetry.py")

    print("\n============================")
    if failures == 0:
        print("REGRESSION SUITE: OK")
        return 0

    print(f"REGRESSION SUITE: FAIL ({failures} stage(s))")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
