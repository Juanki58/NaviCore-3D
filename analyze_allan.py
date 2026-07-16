#!/usr/bin/env python3
"""Herramienta offline de calibración IMU — varianza de Allan (IEEE Std 952-1997).

Lee un CSV de telemetría inercial estática (horas), calcula la varianza de Allan
overlapping y dibuja la curva log-log con asíntotas para extraer ARW/VRW,
Bias Instability y RRW.

Fórmula (fase integrada θ = ∫ω dt, τ = k·dt):

    σ²_A(τ) = (1 / (2(N-2k)·τ²)) · Σ (θ_{i+2k} - 2θ_{i+k} + θ_i)²

La desviación Allan es σ_A(τ) = √σ²_A(τ).

Formato CSV (compatible con inertial_replay.hpp):
  tiempo: timestamp | time_us | time_ms | t_ms | t | time_s
  IMU:    acc_x/y/z, gyro_x/y/z  (m/s² o G; rad/s o deg/s — autodetectado)
  proxy:  yaw_rate (gyro Z), fwd_accel (accel X) si faltan ejes explícitos
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_CSV = REPO_ROOT / "docs" / "imu_static_log.csv"

TIME_COLUMNS = (
    "timestamp",
    "time_us",
    "time_ms",
    "t_ms",
    "t",
    "time_s",
    "time",
    "sim_time_ms",
)

GYRO_COLUMNS = ("gyro_x", "gyro_y", "gyro_z")
ACCEL_COLUMNS = ("acc_x", "acc_y", "acc_z")
GYRO_PROXY = ("yaw_rate", "gyro_z")
ACCEL_PROXY = ("fwd_accel", "acc_x")

GRAVITY_MPS2 = 9.80665
BI_MIN_SCALE = 0.664  # IEEE 952: piso de bias instability en Allan overlapping

ANSI_GREEN = "\033[92m"
ANSI_BOLD = "\033[1m"
ANSI_RESET = "\033[0m"


@dataclass
class ImuSeries:
    name: str
    rate: np.ndarray
    dt_s: float
    unit_label: str
    ieee_kind: str  # "gyro" | "accel"


@dataclass
class AllanFit:
    taus_s: np.ndarray
    adev: np.ndarray
    arw: float
    bias_instability: float
    rrw: float
    tau_arw_s: float
    tau_bi_s: float
    tau_rrw_s: float
    arw_line: np.ndarray
    bi_line: np.ndarray
    rrw_line: np.ndarray
    unit_label: str
    ieee_kind: str


def _normalize_header(name: str) -> str:
    return name.strip().lower().replace(" ", "_")


def _read_csv_headers(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV sin cabecera: {path}")
        headers = [_normalize_header(h) for h in reader.fieldnames]
        rows: list[dict[str, str]] = []
        for raw in reader:
            rows.append({_normalize_header(k): (v or "").strip() for k, v in raw.items()})
    return headers, rows


def _pick_column(headers: list[str], candidates: tuple[str, ...]) -> str | None:
    header_set = set(headers)
    for name in candidates:
        if name in header_set:
            return name
    return None


def _column_to_float(rows: list[dict[str, str]], column: str) -> np.ndarray:
    values: list[float] = []
    for row in rows:
        text = row.get(column, "")
        if not text:
            raise ValueError(f"Columna '{column}' con celdas vacías.")
        values.append(float(text))
    return np.asarray(values, dtype=np.float64)


def _infer_time_seconds(rows: list[dict[str, str]], headers: list[str]) -> np.ndarray:
    col = _pick_column(headers, TIME_COLUMNS)
    if col is None:
        raise ValueError(
            "No se encontró columna de tiempo. "
            f"Use una de: {', '.join(TIME_COLUMNS)}"
        )
    t = _column_to_float(rows, col)
    if col in ("time_us",):
        return t * 1e-6
    if col in ("time_ms", "t_ms", "sim_time_ms"):
        return t * 1e-3
    if col in ("timestamp",):
        if np.max(t) > 1e12:
            return (t - t[0]) * 1e-6
        if np.max(t) > 1e9:
            return (t - t[0]) * 1e-3
        return t - t[0]
    return t - t[0]


def _estimate_dt(time_s: np.ndarray) -> float:
    if len(time_s) < 2:
        raise ValueError("Se necesitan al menos 2 muestras para estimar dt.")
    dts = np.diff(time_s)
    dts = dts[dts > 0.0]
    if dts.size == 0:
        raise ValueError("Serie temporal sin incrementos positivos.")
    return float(np.median(dts))


def _is_accel_in_g(values: np.ndarray) -> bool:
    peak = float(np.max(np.abs(values)))
    return peak < 2.5


def _is_gyro_in_degps(values: np.ndarray) -> bool:
    peak = float(np.max(np.abs(values)))
    return peak > 0.35


def _load_axis(
    rows: list[dict[str, str]],
    headers: list[str],
    axis: str,
    ieee_kind: str,
) -> ImuSeries:
    if ieee_kind == "gyro":
        candidates = (axis,) if axis in GYRO_COLUMNS else GYRO_COLUMNS + GYRO_PROXY
        unit_base = "rad/s"
        ieee_label = "ARW"
    else:
        candidates = (axis,) if axis in ACCEL_COLUMNS else ACCEL_COLUMNS + ACCEL_PROXY
        unit_base = "m/s²"
        ieee_label = "VRW"

    col = _pick_column(headers, candidates)
    if col is None:
        raise ValueError(
            f"No hay columnas para {ieee_kind}. "
            f"Esperadas: {', '.join(candidates)}"
        )

    rate = _column_to_float(rows, col)
    time_s = _infer_time_seconds(rows, headers)
    if len(time_s) != len(rate):
        raise ValueError("Filas de tiempo y tasa IMU no coinciden.")

    dt_s = _estimate_dt(time_s)

    unit_label = unit_base
    if ieee_kind == "gyro" and _is_gyro_in_degps(rate):
        rate = np.deg2rad(rate)
        unit_label = "rad/s (convertido desde deg/s)"
    elif ieee_kind == "accel" and _is_accel_in_g(rate):
        rate = rate * GRAVITY_MPS2
        unit_label = "m/s² (convertido desde G)"

    rate = rate - float(np.mean(rate))

    return ImuSeries(
        name=col,
        rate=rate,
        dt_s=dt_s,
        unit_label=unit_label,
        ieee_kind=ieee_kind,
    )


def overlapping_allan_deviation(
    rate: np.ndarray,
    dt_s: float,
    max_m: int | None = None,
    num_points: int = 80,
) -> tuple[np.ndarray, np.ndarray]:
    """Varianza de Allan overlapping; devuelve (tau [s], sigma_A(tau))."""
    n = len(rate)
    if n < 6:
        raise ValueError("Se necesitan al menos 6 muestras para Allan variance.")

    if max_m is None:
        max_m = max(1, n // 3)
    max_m = max(1, min(max_m, n // 3))

    m_values = np.unique(
        np.round(np.logspace(0, math.log10(max_m), num=num_points)).astype(np.int64)
    )
    m_values = m_values[m_values >= 1]

    theta = np.cumsum(rate, dtype=np.float64) * dt_s

    taus: list[float] = []
    adevs: list[float] = []
    for m in m_values:
        m_int = int(m)
        count = n - 2 * m_int
        if count < 1:
            continue
        tau = m_int * dt_s
        diffs = (
            theta[2 * m_int : 2 * m_int + count]
            - 2.0 * theta[m_int : m_int + count]
            + theta[0:count]
        )
        avar = float(np.sum(diffs * diffs) / (2.0 * count * tau * tau))
        if avar <= 0.0:
            continue
        taus.append(tau)
        adevs.append(math.sqrt(avar))

    if not taus:
        raise ValueError("No se pudo calcular ningun punto de Allan (serie demasiado corta).")

    return np.asarray(taus, dtype=np.float64), np.asarray(adevs, dtype=np.float64)


def extract_allan_parameters(
    series: ImuSeries,
    taus: np.ndarray,
    adev: np.ndarray,
) -> AllanFit:
    log_t = np.log10(taus)
    log_a = np.log10(adev)
    dlog = np.diff(log_a) / np.diff(log_t)

    min_idx = int(np.argmin(adev))
    tau_bi = float(taus[min_idx])
    sigma_min = float(adev[min_idx])
    bias_instability = sigma_min / BI_MIN_SCALE

    short_count = max(3, int(len(taus) * 0.15))
    short_slopes = dlog[: max(1, short_count - 1)]
    if short_slopes.size and np.median(short_slopes) < -0.35:
        arw_samples = adev[:short_count] * np.sqrt(taus[:short_count])
        arw = float(np.median(arw_samples))
    else:
        arw = float(adev[0] * math.sqrt(taus[0]))

    long_start = max(0, int(len(taus) * 0.6))
    long_slopes = dlog[long_start:]
    tau_rrw = float(taus[long_start + len(taus[long_start:]) // 2])
    if long_slopes.size and np.median(long_slopes) > 0.35:
        rrw_samples = adev[long_start:] / np.sqrt(taus[long_start:] / 3.0)
        rrw = float(np.median(rrw_samples))
    else:
        rrw = float(adev[-1] / math.sqrt(taus[-1] / 3.0))

    arw_line = arw / np.sqrt(taus)
    bi_line = np.full_like(taus, bias_instability * BI_MIN_SCALE)
    rrw_line = rrw * np.sqrt(taus / 3.0)

    return AllanFit(
        taus_s=taus,
        adev=adev,
        arw=arw,
        bias_instability=bias_instability,
        rrw=rrw,
        tau_arw_s=float(taus[0]),
        tau_bi_s=tau_bi,
        tau_rrw_s=tau_rrw,
        arw_line=arw_line,
        bi_line=bi_line,
        rrw_line=rrw_line,
        unit_label=series.unit_label,
        ieee_kind=series.ieee_kind,
    )


def _to_gyro_ieee_units(fit: AllanFit) -> dict[str, float]:
    rad_to_deg = 180.0 / math.pi
    sec_to_hour = 3600.0
    arw_deg_sqrt_h = fit.arw * rad_to_deg * math.sqrt(sec_to_hour)
    bi_deg_h = fit.bias_instability * rad_to_deg * sec_to_hour
    rrw_deg_sqrt_h3 = fit.rrw * rad_to_deg * (sec_to_hour ** 1.5)
    return {
        "arw_deg_sqrt_h": arw_deg_sqrt_h,
        "bias_instability_deg_h": bi_deg_h,
        "rrw_deg_sqrt_h3": rrw_deg_sqrt_h3,
    }


def _to_accel_ieee_units(fit: AllanFit) -> dict[str, float]:
    sec_to_hour = 3600.0
    vrw_mps_sqrt_h = fit.arw * math.sqrt(sec_to_hour)
    bi_mps2 = fit.bias_instability
    arw_mps2_sqrt_h = fit.rrw * math.sqrt(sec_to_hour)
    return {
        "vrw_mps_sqrt_h": vrw_mps_sqrt_h,
        "bias_instability_mps2": bi_mps2,
        "arw_mps2_sqrt_h": arw_mps2_sqrt_h,
    }


def plot_allan(fit: AllanFit, title: str, output: Path | None, show: bool) -> None:
    fig, ax = plt.subplots(figsize=(9.0, 6.0))
    ax.loglog(fit.taus_s, fit.adev, "o-", color="#2c3e50", linewidth=1.5, markersize=4, label="Allan σ_A(τ)")

    label_arw = "VRW (∝ τ^{-1/2})" if fit.ieee_kind == "accel" else "ARW (∝ τ^{-1/2})"
    ax.loglog(fit.taus_s, fit.arw_line, "--", color="#2980b9", linewidth=1.2, label=label_arw)
    ax.loglog(
        fit.taus_s,
        fit.bi_line,
        "--",
        color="#27ae60",
        linewidth=1.2,
        label=f"Bias Instability (mín @ τ={fit.tau_bi_s:.2g} s)",
    )
    ax.loglog(
        fit.taus_s,
        fit.rrw_line,
        "--",
        color="#c0392b",
        linewidth=1.2,
        label="RRW (∝ τ^{+1/2})",
    )

    ax.set_xlabel("Tiempo de agrupación τ [s]")
    ax.set_ylabel(f"Desviación Allan σ_A [{fit.unit_label}]")
    ax.set_title(title)
    ax.grid(True, which="both", linestyle=":", alpha=0.5)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()

    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=150)
        print(f"Gráfica guardada en: {output}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def print_report(series: ImuSeries, fit: AllanFit, duration_s: float, sample_hz: float) -> None:
    n = len(series.rate)
    kind = series.ieee_kind
    noise_name = "VRW" if kind == "accel" else "ARW"

    print()
    print(f"{ANSI_BOLD}=== Informe Allan — {series.name} ({kind}) ==={ANSI_RESET}")
    print(f"Muestras: {n:,}  |  Duración: {duration_s/3600:.3f} h ({duration_s:.1f} s)")
    print(f"Frecuencia: {sample_hz:.2f} Hz  |  dt: {series.dt_s*1e3:.3f} ms")
    print(f"Unidad de tasa: {series.unit_label}")
    print()
    print(f"{ANSI_BOLD}Parámetros (SI, del ajuste de asíntotas):{ANSI_RESET}")
    print(f"  {noise_name:20s} = {fit.arw:.6e}  [{series.unit_label}/sqrt(s)]")
    print(f"  Bias Instability   = {fit.bias_instability:.6e}  [{series.unit_label}]")
    print(f"  RRW                = {fit.rrw:.6e}  [{series.unit_label}/s^(3/2)]")
    print()
    print(f"{ANSI_BOLD}Parametros IEEE Std 952-1997:{ANSI_RESET}")

    if kind == "gyro":
        ieee = _to_gyro_ieee_units(fit)
        print(f"  ARW                = {ieee['arw_deg_sqrt_h']:.5f} deg/sqrt(h)")
        print(f"  Bias Instability   = {ieee['bias_instability_deg_h']:.5f} deg/h")
        print(f"  RRW                = {ieee['rrw_deg_sqrt_h3']:.6f} deg/sqrt(h^3)")
        print()
        print(f"{ANSI_GREEN}Referencia simulador (sensors_sim.hpp):{ANSI_RESET}")
        print("  ARW = 0.20 deg/sqrt(h)  |  BI = 0.05 deg/h  |  RRW = 0.005 deg/sqrt(h^3)")
    else:
        ieee = _to_accel_ieee_units(fit)
        print(f"  VRW                = {ieee['vrw_mps_sqrt_h']:.5f} m/s/sqrt(h)")
        print(f"  Bias Instability   = {ieee['bias_instability_mps2']:.6f} m/s^2")
        print(f"  ARW (flicker walk) = {ieee['arw_mps2_sqrt_h']:.6f} m/s^2/sqrt(h)")
        print()
        print(f"{ANSI_GREEN}Referencia simulador (sensors_sim.hpp):{ANSI_RESET}")
        print("  VRW = 0.08 m/s/sqrt(h)  |  BI = 0.001 m/s^2  |  ARW = 0.0005 m/s^2/sqrt(h)")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibración offline IMU: varianza de Allan y extracción IEEE 952.",
    )
    parser.add_argument(
        "csv",
        nargs="?",
        type=Path,
        default=DEFAULT_CSV,
        help=f"CSV de IMU estática (default: {DEFAULT_CSV.name})",
    )
    parser.add_argument(
        "--sensor",
        choices=("gyro", "accel", "both"),
        default="gyro",
        help="Sensor a analizar (default: gyro)",
    )
    parser.add_argument(
        "--axis",
        default="auto",
        help="Eje: gyro_x/y/z, acc_x/y/z, yaw_rate, fwd_accel o auto (default: auto)",
    )
    parser.add_argument(
        "--max-tau-frac",
        type=float,
        default=1.0 / 3.0,
        help="Fracción máxima de muestras N para τ (default: 1/3)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="PNG de salida (por defecto docs/allan_<sensor>_<eje>.png)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Mostrar ventana interactiva matplotlib",
    )
    return parser.parse_args(argv)


def analyze_one(
    csv_path: Path,
    sensor: str,
    axis: str,
    max_tau_frac: float,
    output: Path | None,
    show: bool,
) -> AllanFit:
    headers, rows = _read_csv_headers(csv_path)
    if not rows:
        raise ValueError(f"CSV vacío: {csv_path}")

    ieee_kind = "gyro" if sensor == "gyro" else "accel"
    axis_name = axis
    if axis == "auto":
        axis_name = "gyro_z" if ieee_kind == "gyro" else "acc_z"

    series = _load_axis(rows, headers, axis_name, ieee_kind)
    duration_s = len(series.rate) * series.dt_s
    sample_hz = 1.0 / series.dt_s
    max_m = max(1, int(len(series.rate) * max_tau_frac))

    taus, adev = overlapping_allan_deviation(series.rate, series.dt_s, max_m=max_m)
    fit = extract_allan_parameters(series, taus, adev)
    print_report(series, fit, duration_s, sample_hz)

    if output is None:
        output = REPO_ROOT / "docs" / f"allan_{ieee_kind}_{series.name}.png"

    title = f"Allan — {csv_path.name} — {series.name}"
    plot_allan(fit, title, output, show)
    return fit


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    csv_path = args.csv if args.csv.is_absolute() else REPO_ROOT / args.csv

    if not csv_path.is_file():
        print(
            f"Error: no existe {csv_path}\n"
            "Graba IMU estática con columnas gyro_x/y/z o acc_x/y/z y tiempo en ms/us.",
            file=sys.stderr,
        )
        return 1

    try:
        if args.sensor == "both":
            for sensor in ("gyro", "accel"):
                analyze_one(
                    csv_path,
                    sensor=sensor,
                    axis=args.axis,
                    max_tau_frac=args.max_tau_frac,
                    output=args.output,
                    show=args.show,
                )
        else:
            analyze_one(
                csv_path,
                sensor=args.sensor,
                axis=args.axis,
                max_tau_frac=args.max_tau_frac,
                output=args.output,
                show=args.show,
            )
    except (ValueError, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
