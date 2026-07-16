#!/usr/bin/env python3
"""Campana de Monte Carlo para TUNNEL_STRESS en NaviCore-3D.

Ejecuta N corridas en paralelo del escenario TUNNEL_STRESS con semillas distintas,
recolecta la deriva horizontal al salir del túnel (t=30 s) y emite un reporte
estadístico en consola.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parent
BUILD_DIR = REPO_ROOT / "build"
MONTE_CARLO_DIR = REPO_ROOT / "docs" / "monte_carlo"

SCENARIO = "TUNNEL_STRESS"
TUNNEL_EXIT_T_S = 30.0
DIVERGENCE_DRIFT_M = 30.0
DEFAULT_RUNS = 50

ANSI_GREEN = "\033[92m"
ANSI_RED = "\033[91m"
ANSI_YELLOW = "\033[93m"
ANSI_BOLD = "\033[1m"
ANSI_RESET = "\033[0m"


@dataclass
class RunResult:
    run_id: int
    seed: int
    drift_m: float | None
    diverged: bool
    exit_code: int
    csv_path: Path | None
    error: str = ""


@dataclass
class MonteCarloStats:
    runs: int
    valid_runs: int
    mean_drift_m: float
    std_drift_m: float
    p95_drift_m: float
    p99_drift_m: float
    min_drift_m: float
    max_drift_m: float
    divergence_rate_pct: float
    diverged_count: int


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


def sim_binary_path() -> Path:
    name = "NaviCore3D_Sim.exe" if sys.platform == "win32" else "NaviCore3D_Sim"
    return BUILD_DIR / name


def row_time_s(row: dict[str, str]) -> float:
    time_us = float(row.get("time_us", "0") or "0")
    return time_us * 1e-6


def nearest_row_at(rows: Iterable[dict[str, str]], target_t_s: float) -> dict[str, str] | None:
    best_row: dict[str, str] | None = None
    best_dt = float("inf")
    for row in rows:
        dt = abs(row_time_s(row) - target_t_s)
        if dt < best_dt:
            best_dt = dt
            best_row = row
    return best_row


def parse_float_or_nan(raw: str | None) -> float:
    if raw is None or raw == "":
        return float("nan")
    try:
        value = float(raw)
    except ValueError:
        return float("nan")
    if math.isnan(value) or math.isinf(value):
        return float("nan")
    return value


def csv_has_nan(rows: list[dict[str, str]]) -> bool:
    for row in rows:
        for value in row.values():
            if value is None or value == "":
                continue
            try:
                number = float(value)
            except ValueError:
                continue
            if math.isnan(number) or math.isinf(number):
                return True
    return False


def load_telemetry_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV sin cabecera: {csv_path}")
        return list(reader)


def extract_exit_drift(csv_path: Path, stdout: str) -> float | None:
    rows = load_telemetry_rows(csv_path)
    if not rows:
        return None

    exit_row = nearest_row_at(rows, TUNNEL_EXIT_T_S)
    if exit_row is None:
        return None

    drift = parse_float_or_nan(exit_row.get("drift_m"))
    if not math.isnan(drift):
        return drift

    pos_x = parse_float_or_nan(exit_row.get("pos_x"))
    pos_y = parse_float_or_nan(exit_row.get("pos_y"))
    if math.isnan(pos_x) or math.isnan(pos_y):
        return None
    return math.hypot(pos_x, pos_y)


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]

    sorted_values = sorted(values)
    rank = (pct / 100.0) * (len(sorted_values) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return sorted_values[lower]

    weight = rank - lower
    return sorted_values[lower] + (weight * (sorted_values[upper] - sorted_values[lower]))


def run_git_short_hash() -> str:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        if completed.returncode == 0:
            return (completed.stdout or "").strip() or "N/A"
    except (OSError, subprocess.SubprocessError):
        pass
    return "N/A"


def execute_single_run(run_id: int) -> RunResult:
    seed = run_id
    csv_path = MONTE_CARLO_DIR / f"run_{run_id:04d}_seed_{seed}.csv"
    binary = sim_binary_path()

    if not binary.is_file():
        return RunResult(
            run_id=run_id,
            seed=seed,
            drift_m=None,
            diverged=True,
            exit_code=-1,
            csv_path=None,
            error=f"Simulador no encontrado: {binary}",
        )

    MONTE_CARLO_DIR.mkdir(parents=True, exist_ok=True)

    command = [
        str(binary),
        "--scenario",
        SCENARIO,
        "--no-udp",
        "--seed",
        str(seed),
        "--csv-out",
        str(csv_path),
    ]

    try:
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return RunResult(
            run_id=run_id,
            seed=seed,
            drift_m=None,
            diverged=True,
            exit_code=-1,
            csv_path=None,
            error=str(exc),
        )

    if completed.returncode != 0:
        tail = (completed.stderr or completed.stdout or "").strip()
        return RunResult(
            run_id=run_id,
            seed=seed,
            drift_m=None,
            diverged=True,
            exit_code=completed.returncode,
            csv_path=csv_path if csv_path.is_file() else None,
            error=f"Simulador terminó con código {completed.returncode}\n{tail}",
        )

    if not csv_path.is_file() or csv_path.stat().st_size == 0:
        return RunResult(
            run_id=run_id,
            seed=seed,
            drift_m=None,
            diverged=True,
            exit_code=completed.returncode,
            csv_path=None,
            error="CSV de telemetría no generado",
        )

    stdout = completed.stdout or ""
    try:
        rows = load_telemetry_rows(csv_path)
        drift = extract_exit_drift(csv_path, stdout)
        has_nan = csv_has_nan(rows)
        diverged = (
            drift is None
            or math.isnan(drift)
            or drift > DIVERGENCE_DRIFT_M
            or has_nan
        )
        return RunResult(
            run_id=run_id,
            seed=seed,
            drift_m=drift,
            diverged=diverged,
            exit_code=completed.returncode,
            csv_path=csv_path,
        )
    except (OSError, ValueError, csv.Error) as exc:
        return RunResult(
            run_id=run_id,
            seed=seed,
            drift_m=None,
            diverged=True,
            exit_code=completed.returncode,
            csv_path=csv_path,
            error=str(exc),
        )


def compute_stats(results: list[RunResult]) -> MonteCarloStats:
    valid_drifts = [
        result.drift_m
        for result in results
        if result.drift_m is not None and not math.isnan(result.drift_m)
    ]
    diverged_count = sum(1 for result in results if result.diverged)

    if valid_drifts:
        mean_drift = statistics.fmean(valid_drifts)
        std_drift = statistics.pstdev(valid_drifts) if len(valid_drifts) > 1 else 0.0
        p95 = percentile(valid_drifts, 95.0)
        p99 = percentile(valid_drifts, 99.0)
        min_drift = min(valid_drifts)
        max_drift = max(valid_drifts)
    else:
        mean_drift = float("nan")
        std_drift = float("nan")
        p95 = float("nan")
        p99 = float("nan")
        min_drift = float("nan")
        max_drift = float("nan")

    divergence_rate = (diverged_count / len(results)) * 100.0 if results else 0.0

    return MonteCarloStats(
        runs=len(results),
        valid_runs=len(valid_drifts),
        mean_drift_m=mean_drift,
        std_drift_m=std_drift,
        p95_drift_m=p95,
        p99_drift_m=p99,
        min_drift_m=min_drift,
        max_drift_m=max_drift,
        divergence_rate_pct=divergence_rate,
        diverged_count=diverged_count,
    )


def format_metric(value: float, unit: str = "m") -> str:
    if math.isnan(value):
        return "N/A"
    return f"{value:.4f} {unit}"


def status_for_divergence(rate_pct: float) -> str:
    if rate_pct <= 1.0:
        return colorize("ACEPTABLE", ANSI_GREEN)
    if rate_pct <= 5.0:
        return colorize("ATENCIÓN", ANSI_YELLOW)
    return colorize("CRÍTICO", ANSI_RED)


def print_report(
    results: list[RunResult],
    stats: MonteCarloStats,
    jobs: int,
    commit_hash: str,
) -> None:
    timestamp = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
    failed_runs = [result for result in results if result.error]

    print()
    print(colorize("NaviCore-3D — Reporte Monte Carlo (TUNNEL_STRESS)", ANSI_BOLD))
    print("=" * 72)
    print()
    print(f"  Escenario:          {SCENARIO}")
    print(f"  Corridas (N):       {stats.runs}")
    print(f"  Paralelismo:        {jobs} procesos")
    print(f"  Punto de análisis:  t = {TUNNEL_EXIT_T_S:.0f} s (salida del túnel)")
    print(f"  Semillas:           --seed i  (i = 0 … {stats.runs - 1})")
    print(f"  CSV:                {MONTE_CARLO_DIR.relative_to(REPO_ROOT)}")
    print(f"  Commit:             {commit_hash}")
    print(f"  Timestamp:          {timestamp}")
    print()
    print(colorize("Métricas de deriva horizontal", ANSI_BOLD))
    print("-" * 72)
    print(f"  Media (Mean Drift):           {format_metric(stats.mean_drift_m)}")
    print(f"  Desviación Estándar (Std):    {format_metric(stats.std_drift_m)}")
    print(f"  Percentil 95 (límite oper.):  {format_metric(stats.p95_drift_m)}")
    print(f"  Percentil 99 (peor caso):     {format_metric(stats.p99_drift_m)}")
    print(f"  Mínimo / Máximo:              {format_metric(stats.min_drift_m)} / {format_metric(stats.max_drift_m)}")
    print()
    print(colorize("Robustez del filtro", ANSI_BOLD))
    print("-" * 72)
    print(
        f"  Tasa de Divergencia:         {stats.divergence_rate_pct:.2f}% "
        f"({stats.diverged_count}/{stats.runs} corridas)"
    )
    print(
        f"  Criterio divergencia:       deriva > {DIVERGENCE_DRIFT_M:.0f} m "
        "o telemetría con NaN/Inf"
    )
    print(f"  Evaluación:                 {status_for_divergence(stats.divergence_rate_pct)}")
    print(f"  Corridas válidas:           {stats.valid_runs}/{stats.runs}")

    if failed_runs:
        print()
        print(colorize("Corridas con error de ejecución", ANSI_BOLD))
        print("-" * 72)
        for result in failed_runs[:5]:
            print(f"  run={result.run_id:04d} seed={result.seed}: {result.error.splitlines()[0]}")
        if len(failed_runs) > 5:
            print(f"  ... y {len(failed_runs) - 5} más")

    print()
    print("=" * 72)
    summary = (
        "MONTE CARLO COMPLETADO — FILTRO ROBUSTO"
        if stats.divergence_rate_pct <= 1.0
        else "MONTE CARLO COMPLETADO — REVISAR DIVERGENCIAS"
    )
    summary_color = ANSI_GREEN if stats.divergence_rate_pct <= 1.0 else ANSI_RED
    print(colorize(summary, summary_color))
    print()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Campana de Monte Carlo para TUNNEL_STRESS con semillas reproducibles.",
    )
    parser.add_argument(
        "-n",
        "--runs",
        type=int,
        default=DEFAULT_RUNS,
        help=f"Número de corridas (default: {DEFAULT_RUNS})",
    )
    parser.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=None,
        help="Procesos en paralelo (default: núm. de CPUs)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.runs < 1:
        print("ERROR: --runs debe ser >= 1", file=sys.stderr)
        return 1

    jobs = args.jobs if args.jobs is not None else os.cpu_count() or 4
    jobs = max(1, min(jobs, args.runs))

    binary = sim_binary_path()
    commit_hash = run_git_short_hash()

    print("Ejecutando campaña Monte Carlo...")
    print(f"Repositorio: {REPO_ROOT}")
    print(f"Simulador:   {binary}")
    print(f"Corridas:    {args.runs}")
    print(f"Paralelo:    {jobs} procesos")
    print(f"Commit:      {commit_hash}")

    results: list[RunResult] = []
    with ProcessPoolExecutor(max_workers=jobs) as executor:
        futures = {
            executor.submit(execute_single_run, run_id): run_id
            for run_id in range(args.runs)
        }
        completed = 0
        for future in as_completed(futures):
            results.append(future.result())
            completed += 1
            if completed % max(1, args.runs // 10) == 0 or completed == args.runs:
                print(f"  Progreso: {completed}/{args.runs} corridas finalizadas")

    results.sort(key=lambda item: item.run_id)
    stats = compute_stats(results)
    print_report(results, stats, jobs, commit_hash)

    return 0 if stats.divergence_rate_pct <= 5.0 else 1


if __name__ == "__main__":
    sys.exit(main())
