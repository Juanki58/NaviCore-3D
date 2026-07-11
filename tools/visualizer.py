#!/usr/bin/env python3
"""
NaviCore-3D Digital Twin — visualizador 3D de telemetría.

Lee docs/telemetria_navicore.csv (pandas) y renderiza trayectorias 3D
por escenario con matplotlib, coloreadas según el modo de navegación.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D

MODE_COLORS: dict[str, str] = {
    "GPS": "#2ecc71",
    "DEAD_RECKONING": "#e74c3c",
    "HYBRID": "#3498db",
    "INITIALIZING": "#95a5a6",
}

DEFAULT_CSV = Path(__file__).resolve().parent.parent / "docs" / "telemetria_navicore.csv"

M_PER_DEG_LAT = 111_132.954


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualizador 3D de telemetría NaviCore (matplotlib + pandas)."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_CSV,
        help=f"Ruta al CSV de telemetría (default: {DEFAULT_CSV})",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Guardar figura en PNG en lugar de mostrar ventana interactiva.",
    )
    return parser.parse_args()


def load_telemetry(csv_path: Path) -> pd.DataFrame:
    if not csv_path.is_file():
        raise FileNotFoundError(f"No se encontró el archivo de telemetría: {csv_path}")

    df = pd.read_csv(csv_path)
    required = {
        "Timestamp_ms",
        "Escenario",
        "Modo",
        "Calidad",
        "Satelites",
        "Pos_X",
        "Pos_Y",
        "Pos_Z",
        "Vel_X",
        "Vel_Y",
        "Vel_Z",
        "Rumbo",
    }
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Columnas faltantes en CSV: {sorted(missing)}")

    df["Modo"] = df["Modo"].astype(str).str.strip()
    df["Escenario"] = df["Escenario"].astype(str).str.strip()
    return df


def latlon_to_local_m(lat_deg: np.ndarray, lon_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ref_lat = float(lat_deg[0])
    ref_lon = float(lon_deg[0])
    lat_rad = math.radians(ref_lat)
    north_m = (lat_deg - ref_lat) * M_PER_DEG_LAT
    east_m = (lon_deg - ref_lon) * M_PER_DEG_LAT * math.cos(lat_rad)
    return north_m, east_m


def scenario_to_xyz(df: pd.DataFrame) -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    scenario = df["Escenario"].iloc[0]

    if scenario == "GPS_LOSS":
        north_m, east_m = latlon_to_local_m(df["Pos_X"].to_numpy(), df["Pos_Y"].to_numpy())
        up_m = df["Pos_Z"].to_numpy(dtype=float)
        z_label = "Altitud (m)"
        return north_m, east_m, up_m, z_label

    if scenario == "SUBMARINE":
        # Pos_Z = presión hidrostática [Pa]; mostramos exceso sobre superficie.
        pressure_pa = df["Pos_Z"].to_numpy(dtype=float)
        surface_pa = float(pressure_pa[0])
        delta_pa = pressure_pa - surface_pa
        return (
            np.zeros(len(df), dtype=float),
            np.zeros(len(df), dtype=float),
            delta_pa,
            "Δ presión (Pa)",
        )

    north_m, east_m = latlon_to_local_m(df["Pos_X"].to_numpy(), df["Pos_Y"].to_numpy())
    up_m = df["Pos_Z"].to_numpy(dtype=float)
    return north_m, east_m, up_m, "Pos_Z"


def mode_color(mode: str) -> str:
    return MODE_COLORS.get(mode, "#bdc3c7")


def plot_scenario_3d(ax: plt.Axes, df: pd.DataFrame) -> None:
    x, y, z, z_label = scenario_to_xyz(df)
    scenario = df["Escenario"].iloc[0]
    time_s = df["Timestamp_ms"].to_numpy(dtype=float) * 1e-3

    modes = df["Modo"].unique()
    for mode in modes:
        mask = df["Modo"] == mode
        ax.plot(
            x[mask],
            y[mask],
            z[mask],
            color=mode_color(mode),
            linewidth=1.8,
            alpha=0.85,
            label=mode,
        )
        ax.scatter(
            x[mask],
            y[mask],
            z[mask],
            c=df.loc[mask, "Calidad"].to_numpy(),
            cmap="viridis",
            norm=Normalize(vmin=0.0, vmax=1.0),
            s=18,
            edgecolors=mode_color(mode),
            linewidths=0.4,
            alpha=0.9,
        )

    ax.scatter(x[0], y[0], z[0], c="black", s=60, marker="o", label="Inicio", zorder=5)
    ax.scatter(x[-1], y[-1], z[-1], c="black", s=60, marker="^", label="Fin", zorder=5)

    ax.set_title(f"{scenario}  ·  {len(df)} muestras  ·  {time_s[-1]:.1f} s")
    ax.set_xlabel("Norte (m)" if scenario == "GPS_LOSS" else "X")
    ax.set_ylabel("Este (m)" if scenario == "GPS_LOSS" else "Y")
    ax.set_zlabel(z_label)

    if scenario == "SUBMARINE":
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)

    mode_handles = [
        Line2D([0], [0], color=mode_color(m), linewidth=2.5, label=m) for m in modes
    ]
    ax.legend(handles=mode_handles, loc="upper left", fontsize=8, title="Modo")


def build_figure(df: pd.DataFrame) -> plt.Figure:
    scenarios = list(df["Escenario"].unique())
    fig = plt.figure(figsize=(14, 6))
    fig.suptitle("NaviCore-3D · Gemelo Digital — Telemetría 3D", fontsize=14, fontweight="bold")

    for index, scenario in enumerate(scenarios, start=1):
        subset = df[df["Escenario"] == scenario].sort_values("Timestamp_ms")
        ax = fig.add_subplot(1, len(scenarios), index, projection="3d")
        plot_scenario_3d(ax, subset)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    return fig


def main() -> int:
    args = parse_args()

    try:
        df = load_telemetry(args.csv)
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    fig = build_figure(df)

    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"Figura guardada en {args.save}")
    else:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
