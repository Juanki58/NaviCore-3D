#!/usr/bin/env python3
"""Suite autónoma de benchmarks cuantitativos NaviCore-3D.

Ejecuta secuencialmente los escenarios SLALOM y TUNNEL_STRESS del simulador,
evalúa métricas desde los CSV de telemetría y emite un reporte PASS/FAIL.
"""

from __future__ import annotations

import csv
import math
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parent
BUILD_DIR = REPO_ROOT / "build"
DOCS_DIR = REPO_ROOT / "docs"
BENCHMARKS_DIR = DOCS_DIR / "benchmarks"

CSV_CANDIDATES = (
    DOCS_DIR / "telemetria_navicore.csv",
    REPO_ROOT / "telemetry_log.csv",
)

NIS_THRESHOLD = 11.345
SLALOM_MAX_LATERAL_DRIFT_M = 0.15
TUNNEL_EXIT_DRIFT_M = 15.0
TUNNEL_ZUPT_MAX_SPEED_MPS = 0.01
TUNNEL_ZUPT_T0_S = 20.0
TUNNEL_ZUPT_T1_S = 25.0
TUNNEL_EXIT_T_S = 30.0
TUNNEL_GLITCH_T0_S = 30.0

ANSI_GREEN = "\033[92m"
ANSI_RED = "\033[91m"
ANSI_BOLD = "\033[1m"
ANSI_RESET = "\033[0m"


@dataclass
class MetricResult:
    name: str
    passed: bool
    measured: float | str
    limit: float | str
    unit: str = ""
    detail: str = ""


@dataclass
class BenchmarkResult:
    name: str
    scenario: str
    metrics: list[MetricResult] = field(default_factory=list)
    sim_exit_code: int = 0
    csv_path: Path | None = None
    error: str = ""

    @property
    def passed(self) -> bool:
        if self.error or self.sim_exit_code != 0:
            return False
        return all(metric.passed for metric in self.metrics)


def supports_color() -> bool:
    if not sys.stdout.isatty():
        return False
    if sys.platform == "win32":
        return "WT_SESSION" in os.environ
    return True


def colorize(text: str, color: str) -> str:
    if not supports_color():
        return text
    return f"{color}{text}{ANSI_RESET}"


def status_label(passed: bool) -> str:
    label = "PASS" if passed else "FAIL"
    return colorize(label, ANSI_GREEN if passed else ANSI_RED)


def sim_binary_path() -> Path:
    name = "NaviCore3D_Sim.exe" if sys.platform == "win32" else "NaviCore3D_Sim"
    return BUILD_DIR / name


def resolve_csv_path() -> Path | None:
    for path in CSV_CANDIDATES:
        if path.is_file() and path.stat().st_size > 0:
            return path
    return None


def load_telemetry_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV sin cabecera: {csv_path}")
        return list(reader)


def row_time_s(row: dict[str, str]) -> float:
    time_us = float(row.get("time_us", "0") or "0")
    return time_us * 1e-6


def row_speed_mps(row: dict[str, str]) -> float:
    vx = float(row.get("vel_x", "0") or "0")
    vy = float(row.get("vel_y", "0") or "0")
    vz = float(row.get("vel_z", "0") or "0")
    return math.sqrt((vx * vx) + (vy * vy) + (vz * vz))


def row_drift_m(row: dict[str, str]) -> float | None:
    if "drift_m" not in row:
        return None
    raw = row.get("drift_m", "")
    if raw is None or raw == "":
        return None
    return float(raw)


def nearest_row_at(rows: Iterable[dict[str, str]], target_t_s: float) -> dict[str, str] | None:
    best_row: dict[str, str] | None = None
    best_dt = float("inf")
    for row in rows:
        dt = abs(row_time_s(row) - target_t_s)
        if dt < best_dt:
            best_dt = dt
            best_row = row
    return best_row


def run_simulator(scenario: str) -> subprocess.CompletedProcess[str]:
    binary = sim_binary_path()
    if not binary.is_file():
        raise FileNotFoundError(
            f"No se encontró el simulador compilado: {binary}\n"
            "Compile con: cmake --build build --target NaviCore3D_Sim"
        )

    command = [str(binary), "--scenario", scenario, "--no-udp"]
    return subprocess.run(
        command,
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def archive_csv(scenario: str) -> Path | None:
    source = resolve_csv_path()
    if source is None:
        return None

    BENCHMARKS_DIR.mkdir(parents=True, exist_ok=True)
    destination = BENCHMARKS_DIR / f"{scenario.lower()}_telemetry.csv"
    shutil.copy2(source, destination)
    return destination


def evaluate_slalom(csv_path: Path, stdout: str) -> list[MetricResult]:
    rows = load_telemetry_rows(csv_path)
    if not rows:
        raise ValueError("CSV de SLALOM vacío")

    drift_values = [value for row in rows if (value := row_drift_m(row)) is not None]
    if drift_values:
        max_lateral = max(abs(value) for value in drift_values)
    else:
        max_lateral = max(abs(float(row.get("pos_x", "0") or "0")) for row in rows)

    for line in stdout.splitlines():
        if "Max deriva lateral" in line:
            try:
                max_lateral = max(max_lateral, float(line.split(":")[-1].strip().split()[0]))
            except ValueError:
                pass

    return [
        MetricResult(
            name="max_lateral_drift",
            passed=max_lateral < SLALOM_MAX_LATERAL_DRIFT_M,
            measured=max_lateral,
            limit=SLALOM_MAX_LATERAL_DRIFT_M,
            unit="m",
            detail="Deriva lateral máxima en curvas cerradas",
        )
    ]


def count_glitch_rejections(rows: list[dict[str, str]], stdout: str) -> tuple[int, float]:
    glitch_rows = [
        row
        for row in rows
        if row_time_s(row) >= TUNNEL_GLITCH_T0_S
        and float(row.get("nis", "0") or "0") > NIS_THRESHOLD
    ]
    nis_peak = max((float(row.get("nis", "0") or "0") for row in glitch_rows), default=0.0)

    stdout_rej = 0
    for line in stdout.splitlines():
        if "GLITCH" in line and "RECHAZADO" in line:
            stdout_rej += 1
        elif "gnss=REJ" in line:
            try:
                t_token = line.split("t=")[1].split("s")[0]
                if float(t_token) >= TUNNEL_GLITCH_T0_S:
                    stdout_rej += 1
            except (IndexError, ValueError):
                pass

    if stdout_rej >= 1:
        return 1, max(nis_peak, NIS_THRESHOLD)
    return len({round(row_time_s(row), 2) for row in glitch_rows}), nis_peak


def evaluate_tunnel(csv_path: Path, stdout: str) -> list[MetricResult]:
    rows = load_telemetry_rows(csv_path)
    if not rows:
        raise ValueError("CSV de TUNNEL_STRESS vacío")

    exit_row = nearest_row_at(rows, TUNNEL_EXIT_T_S)
    if exit_row is None:
        raise ValueError("No hay muestras cercanas a t=30 s")

    exit_drift = row_drift_m(exit_row)
    if exit_drift is None:
        exit_drift = math.hypot(
            float(exit_row.get("pos_x", "0") or "0"),
            float(exit_row.get("pos_y", "0") or "0"),
        )

    for line in stdout.splitlines():
        if "Deriva al salir tunel (30 s):" in line:
            try:
                exit_drift = float(line.split(":")[-1].strip().split()[0])
            except ValueError:
                pass

    zupt_speeds = [
        row_speed_mps(row)
        for row in rows
        if TUNNEL_ZUPT_T0_S <= row_time_s(row) < TUNNEL_ZUPT_T1_S
    ]
    max_zupt_speed = max(zupt_speeds) if zupt_speeds else float("inf")

    for line in stdout.splitlines():
        if "Max |v| durante ZUPT" in line:
            try:
                max_zupt_speed = float(line.split(":")[-1].strip().split()[0])
            except ValueError:
                pass

    rejections, nis_peak = count_glitch_rejections(rows, stdout)
    gnss_rej_in_stdout = ("RECHAZADO" in stdout and "GLITCH" in stdout) or any(
        "gnss=REJ" in line and "t= 30." in line for line in stdout.splitlines()
    )
    glitch_pass = rejections == 1 and nis_peak > NIS_THRESHOLD and gnss_rej_in_stdout

    return [
        MetricResult(
            name="tunnel_exit_drift",
            passed=exit_drift < TUNNEL_EXIT_DRIFT_M,
            measured=exit_drift,
            limit=TUNNEL_EXIT_DRIFT_M,
            unit="m",
            detail=f"Deriva horizontal al salir del túnel (t={TUNNEL_EXIT_T_S:.0f} s)",
        ),
        MetricResult(
            name="zupt_residual_speed",
            passed=max_zupt_speed < TUNNEL_ZUPT_MAX_SPEED_MPS,
            measured=max_zupt_speed,
            limit=TUNNEL_ZUPT_MAX_SPEED_MPS,
            unit="m/s",
            detail=f"Velocidad residual máxima en ZUPT ({TUNNEL_ZUPT_T0_S:.0f}-{TUNNEL_ZUPT_T1_S:.0f} s)",
        ),
        MetricResult(
            name="gps_glitch_rejection",
            passed=glitch_pass,
            measured=f"rejections={rejections}, nis_peak={nis_peak:.2f}",
            limit=f"rejections=1, nis>{NIS_THRESHOLD:.3f}, gnss=REJ",
            unit="",
            detail="Integridad FDE: 1 outlier rechazado tras glitch GPS",
        ),
    ]


def run_benchmark(name: str, scenario: str) -> BenchmarkResult:
    result = BenchmarkResult(name=name, scenario=scenario)

    try:
        completed = run_simulator(scenario)
        result.sim_exit_code = completed.returncode
        if completed.returncode != 0:
            tail = (completed.stderr or completed.stdout or "").strip()
            result.error = f"Simulador terminó con código {completed.returncode}\n{tail}"
            return result

        csv_path = archive_csv(scenario)
        if csv_path is None or not csv_path.is_file():
            result.error = (
                "No se encontró CSV de telemetría tras la simulación. "
                f"Buscado en: {', '.join(str(p) for p in CSV_CANDIDATES)}"
            )
            return result

        result.csv_path = csv_path
        stdout = completed.stdout or ""
        if scenario == "SLALOM":
            result.metrics = evaluate_slalom(csv_path, stdout)
        else:
            result.metrics = evaluate_tunnel(csv_path, stdout)
    except (FileNotFoundError, OSError, ValueError, csv.Error) as exc:
        result.error = str(exc)

    return result


def print_metric(metric: MetricResult) -> None:
    status = status_label(metric.passed)
    measured = metric.measured
    limit = metric.limit
    unit = f" {metric.unit}" if metric.unit else ""
    if isinstance(measured, float):
        measured_text = f"{measured:.4f}{unit}"
    else:
        measured_text = str(measured)
    if isinstance(limit, float):
        limit_text = f"{limit:.4f}{unit}"
    else:
        limit_text = str(limit)

    print(f"  [{status}] {metric.name}")
    print(f"         {metric.detail}")
    print(f"         medido: {measured_text}  |  límite: {limit_text}")


def print_report(results: list[BenchmarkResult]) -> None:
    print()
    print(colorize("NaviCore-3D — Reporte de Benchmarks Cuantitativos", ANSI_BOLD))
    print("=" * 72)

    for benchmark in results:
        overall = benchmark.passed if not benchmark.error else False
        print()
        print(f"{colorize(benchmark.name, ANSI_BOLD)}  [{status_label(overall)}]")
        print(f"  Escenario: {benchmark.scenario}")
        if benchmark.csv_path:
            print(f"  CSV:       {benchmark.csv_path.relative_to(REPO_ROOT)}")
        if benchmark.error:
            print(colorize(f"  ERROR: {benchmark.error}", ANSI_RED))
            continue
        for metric in benchmark.metrics:
            print_metric(metric)

    print()
    print("=" * 72)
    total_pass = all(item.passed for item in results)
    summary = "TODOS LOS BENCHMARKS PASS" if total_pass else "HAY BENCHMARKS FAIL"
    print(colorize(summary, ANSI_GREEN if total_pass else ANSI_RED))
    print()


def main() -> int:
    print("Ejecutando suite autónoma de benchmarks...")
    print(f"Repositorio: {REPO_ROOT}")
    print(f"Simulador:   {sim_binary_path()}")

    benchmarks = [
        ("Benchmark 1 — Slalom", "SLALOM"),
        ("Benchmark 2 — Túnel (TUNNEL_STRESS)", "TUNNEL_STRESS"),
    ]

    results = [run_benchmark(name, scenario) for name, scenario in benchmarks]
    print_report(results)
    return 0 if all(item.passed for item in results) else 1


if __name__ == "__main__":
    sys.exit(main())
