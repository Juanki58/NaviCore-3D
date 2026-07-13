#!/usr/bin/env python3
"""
NaviCore-3D — Visualizador de Caja Negra EKF (Fase 2).

Lee docs/telemetria_navicore.csv (24 columnas) y reproduce la telemetría del
filtro a 100 Hz mostrando:
  1. Trayectoria N-E con elipse 2σ actual y rastro histórico de incertidumbre.
  2. Convergencia de sesgos de acelerómetro y giróscopo.
  3. Evolución de varianzas de posición y rumbo + NIS GNSS.

Esquema CSV:
  time_us,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll,pitch,yaw,
  bias_ax,bias_ay,bias_az,bias_gx,bias_gy,bias_gz,
  nis,innov_x,innov_y,innov_z,cov_pos_x,cov_pos_y,cov_pos_z,cov_yaw
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.collections import LineCollection
from matplotlib.patches import Ellipse

# ---------------------------------------------------------------------------
# Configuración
# ---------------------------------------------------------------------------

DEFAULT_CSV = Path(__file__).resolve().parent.parent / "docs" / "telemetria_navicore.csv"

# CSV generado @ 100 Hz (10 ms por fila).
DEFAULT_HZ = 100.0
SIGMA_ELLIPSE = 2.0
NIS_THRESHOLD = 11.345
DEFAULT_TRAIL_STEP = 100
DEFAULT_TRAIL_ALPHA = 0.08
TRAIL_FACE_COLOR = "#3498db"
TRAIL_EDGE_COLOR = "#2980b9"

COLUMNAS_REQUERIDAS = frozenset(
    {
        "time_us",
        "pos_x",
        "pos_y",
        "pos_z",
        "roll",
        "pitch",
        "yaw",
        "bias_ax",
        "bias_ay",
        "bias_az",
        "bias_gx",
        "bias_gy",
        "bias_gz",
        "nis",
        "innov_x",
        "innov_y",
        "innov_z",
        "cov_pos_x",
        "cov_pos_y",
        "cov_pos_z",
        "cov_yaw",
    }
)


# ---------------------------------------------------------------------------
# Carga de datos
# ---------------------------------------------------------------------------


class TelemetryIOError(Exception):
    """Error de lectura del CSV (vacío, bloqueado o columnas inválidas)."""


def cargar_caja_negra(
    csv_path: Path,
    reintentos: int = 3,
    espera_reintento_s: float = 0.5,
) -> pd.DataFrame:
    """Carga y valida el CSV de caja negra EKF."""
    if not csv_path.is_file():
        raise TelemetryIOError(f"No se encontró el archivo de telemetría: {csv_path}")

    ultimo_error: Exception | None = None

    for intento in range(1, reintentos + 1):
        try:
            df = pd.read_csv(csv_path)
            break
        except PermissionError as exc:
            ultimo_error = exc
            if intento < reintentos:
                time.sleep(espera_reintento_s)
                continue
            raise TelemetryIOError(
                f"El archivo está en uso tras {reintentos} intentos: {csv_path}\n"
                "Cierre el simulador o espere a que termine de escribir el CSV."
            ) from exc
        except pd.errors.EmptyDataError as exc:
            raise TelemetryIOError(f"El CSV está vacío: {csv_path}") from exc
    else:
        raise TelemetryIOError(f"No se pudo leer {csv_path}: {ultimo_error}")

    if df.empty:
        raise TelemetryIOError(f"El CSV no contiene filas de datos: {csv_path}")

    faltantes = COLUMNAS_REQUERIDAS - set(df.columns)
    if faltantes:
        raise TelemetryIOError(
            f"Columnas obligatorias ausentes en el CSV: {sorted(faltantes)}"
        )

    df = df.sort_values("time_us").reset_index(drop=True)

    for col in COLUMNAS_REQUERIDAS:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    if df[list(COLUMNAS_REQUERIDAS)].isna().any().any():
        raise TelemetryIOError("El CSV contiene valores no numéricos en columnas EKF.")

    return df


def sigma_desde_varianza(var: float) -> float:
    """Devuelve 1σ a partir de una varianza diagonal; clamp para valores negativos."""
    return float(np.sqrt(max(var, 0.0)))


def parametros_elipse_2sigma(
    pos_n: float,
    pos_e: float,
    cov_n: float,
    cov_e: float,
) -> tuple[tuple[float, float], float, float]:
    """Centro (N,E) y semiejes completos (ancho, alto) de la elipse 2σ."""
    sigma_n = SIGMA_ELLIPSE * sigma_desde_varianza(cov_n)
    sigma_e = SIGMA_ELLIPSE * sigma_desde_varianza(cov_e)
    return (pos_n, pos_e), max(2.0 * sigma_e, 1e-3), max(2.0 * sigma_n, 1e-3)


# ---------------------------------------------------------------------------
# Dashboard EKF
# ---------------------------------------------------------------------------


class DashboardCajaNegra:
    """Cuatro subgráficos sincronizados con reproducción secuencial."""

    def __init__(
        self,
        df: pd.DataFrame,
        titulo: str,
        trail_step: int = DEFAULT_TRAIL_STEP,
        trail_alpha: float = DEFAULT_TRAIL_ALPHA,
    ) -> None:
        self.df = df
        self.titulo = titulo
        self.num_frames = len(df)
        self.trail_step = max(1, trail_step)
        self.trail_alpha = float(np.clip(trail_alpha, 0.01, 1.0))
        self.trail_patches: list[Ellipse] = []

        self.tiempo_s = df["time_us"].to_numpy(dtype=float) * 1e-6
        self.pos_n = df["pos_x"].to_numpy(dtype=float)
        self.pos_e = df["pos_y"].to_numpy(dtype=float)
        self.pos_d = df["pos_z"].to_numpy(dtype=float)

        self.bias_a = df[["bias_ax", "bias_ay", "bias_az"]].to_numpy(dtype=float)
        self.bias_g = df[["bias_gx", "bias_gy", "bias_gz"]].to_numpy(dtype=float)

        self.nis = df["nis"].to_numpy(dtype=float)
        self.cov_pos = df[["cov_pos_x", "cov_pos_y", "cov_pos_z"]].to_numpy(dtype=float)
        self.cov_yaw = df["cov_yaw"].to_numpy(dtype=float)

        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle(titulo, fontsize=13, fontweight="bold")

        self._configurar_ejes()
        self._crear_artistas()

    def _configurar_ejes(self) -> None:
        ax_map, ax_bias_a, ax_bias_g, ax_cov = self.axes.flat

        ax_map.set_title("Trayectoria N-E + rastro histórico 2σ")
        ax_map.set_xlabel("Norte (m)")
        ax_map.set_ylabel("Este (m)")
        ax_map.set_aspect("equal", adjustable="box")
        ax_map.grid(True, alpha=0.25)

        ax_bias_a.set_title("Sesgo acelerómetro (m/s²)")
        ax_bias_a.set_xlabel("Tiempo (s)")
        ax_bias_a.set_ylabel("Bias")
        ax_bias_a.grid(True, alpha=0.25)

        ax_bias_g.set_title("Sesgo giróscopo (rad/s)")
        ax_bias_g.set_xlabel("Tiempo (s)")
        ax_bias_g.set_ylabel("Bias")
        ax_bias_g.grid(True, alpha=0.25)

        ax_cov.set_title("Varianzas P (posición + rumbo) y NIS")
        ax_cov.set_xlabel("Tiempo (s)")
        ax_cov.set_ylabel("Varianza / NIS")
        ax_cov.grid(True, alpha=0.25)
        ax_cov.axhline(NIS_THRESHOLD, color="#e74c3c", linestyle="--", linewidth=1.0, alpha=0.7)

    def _crear_artistas(self) -> None:
        ax_map, ax_bias_a, ax_bias_g, ax_cov = self.axes.flat

        self.trayectoria = LineCollection([], linewidths=1.5, colors="#2980b9", alpha=0.85)
        ax_map.add_collection(self.trayectoria)

        (self.pos_actual,) = ax_map.plot([], [], "ko", markersize=5, zorder=5)

        self.elipse_incertidumbre = Ellipse(
            (0.0, 0.0),
            width=0.0,
            height=0.0,
            angle=0.0,
            facecolor="#3498db",
            edgecolor="#2c3e50",
            alpha=0.25,
            linewidth=1.0,
            zorder=4,
        )
        ax_map.add_patch(self.elipse_incertidumbre)

        (self.linea_bias_ax,) = ax_bias_a.plot([], [], label="bias_ax", color="#e74c3c")
        (self.linea_bias_ay,) = ax_bias_a.plot([], [], label="bias_ay", color="#27ae60")
        (self.linea_bias_az,) = ax_bias_a.plot([], [], label="bias_az", color="#8e44ad")
        ax_bias_a.legend(loc="upper right", fontsize=8)

        (self.linea_bias_gx,) = ax_bias_g.plot([], [], label="bias_gx", color="#e67e22")
        (self.linea_bias_gy,) = ax_bias_g.plot([], [], label="bias_gy", color="#16a085")
        (self.linea_bias_gz,) = ax_bias_g.plot([], [], label="bias_gz", color="#34495e")
        ax_bias_g.legend(loc="upper right", fontsize=8)

        (self.linea_cov_n,) = ax_cov.plot([], [], label="cov_pos_x (N)", color="#2980b9")
        (self.linea_cov_e,) = ax_cov.plot([], [], label="cov_pos_y (E)", color="#27ae60")
        (self.linea_cov_d,) = ax_cov.plot([], [], label="cov_pos_z (D)", color="#8e44ad")
        (self.linea_cov_yaw,) = ax_cov.plot([], [], label="cov_yaw", color="#f39c12")
        (self.linea_nis,) = ax_cov.plot([], [], label="NIS", color="#c0392b", linewidth=1.2)
        ax_cov.legend(loc="upper right", fontsize=7, ncol=2)

        self.texto_estado = self.fig.text(
            0.5,
            0.01,
            "",
            ha="center",
            fontsize=10,
            color="#2c3e50",
        )

    def _crear_elipse_rastro(self, sample_idx: int) -> Ellipse:
        centro, ancho, alto = parametros_elipse_2sigma(
            self.pos_n[sample_idx],
            self.pos_e[sample_idx],
            self.cov_pos[sample_idx, 0],
            self.cov_pos[sample_idx, 1],
        )
        return Ellipse(
            centro,
            width=ancho,
            height=alto,
            angle=0.0,
            facecolor=TRAIL_FACE_COLOR,
            edgecolor=TRAIL_EDGE_COLOR,
            alpha=self.trail_alpha,
            linewidth=0.4,
            zorder=2,
        )

    def _indices_rastro_hasta(self, idx: int) -> list[int]:
        """Muestras pasadas (excluye la posición actual) espaciadas cada trail_step."""
        if idx <= 1:
            return []
        return list(range(0, idx - 1, self.trail_step))

    def _actualizar_rastro_elipses(self, idx: int) -> None:
        """
        Añade elipses al rastro de forma incremental (O(1) amortizado por frame).

        Solo crea patches nuevos cuando el índice de reproducción cruza un múltiplo
        de trail_step; las elipses ya pintadas permanecen en el eje.
        """
        objetivo = self._indices_rastro_hasta(idx)
        ax_map = self.axes[0, 0]

        while len(self.trail_patches) < len(objetivo):
            sample_idx = objetivo[len(self.trail_patches)]
            patch = self._crear_elipse_rastro(sample_idx)
            ax_map.add_patch(patch)
            self.trail_patches.append(patch)

    def _actualizar_elipse_actual(self, idx: int) -> None:
        if idx <= 0:
            self.elipse_incertidumbre.set_visible(False)
            return

        i = idx - 1
        centro, ancho, alto = parametros_elipse_2sigma(
            self.pos_n[i],
            self.pos_e[i],
            self.cov_pos[i, 0],
            self.cov_pos[i, 1],
        )

        self.elipse_incertidumbre.set_center(centro)
        self.elipse_incertidumbre.width = ancho
        self.elipse_incertidumbre.height = alto
        self.elipse_incertidumbre.set_visible(True)

    def _actualizar_marco(self, frame: int) -> list:
        idx = min(frame + 1, self.num_frames)
        sl = slice(0, idx)
        t = self.tiempo_s[sl]

        if idx >= 2:
            puntos = np.column_stack([self.pos_n[sl], self.pos_e[sl]])
            segmentos = np.stack([puntos[:-1], puntos[1:]], axis=1)
            self.trayectoria.set_segments(segmentos)
        else:
            self.trayectoria.set_segments([])

        if idx > 0:
            self.pos_actual.set_data([self.pos_n[idx - 1]], [self.pos_e[idx - 1]])
        else:
            self.pos_actual.set_data([], [])

        self._actualizar_rastro_elipses(idx)
        self._actualizar_elipse_actual(idx)

        self.linea_bias_ax.set_data(t, self.bias_a[sl, 0])
        self.linea_bias_ay.set_data(t, self.bias_a[sl, 1])
        self.linea_bias_az.set_data(t, self.bias_a[sl, 2])

        self.linea_bias_gx.set_data(t, self.bias_g[sl, 0])
        self.linea_bias_gy.set_data(t, self.bias_g[sl, 1])
        self.linea_bias_gz.set_data(t, self.bias_g[sl, 2])

        self.linea_cov_n.set_data(t, self.cov_pos[sl, 0])
        self.linea_cov_e.set_data(t, self.cov_pos[sl, 1])
        self.linea_cov_d.set_data(t, self.cov_pos[sl, 2])
        self.linea_cov_yaw.set_data(t, self.cov_yaw[sl])
        self.linea_nis.set_data(t, self.nis[sl])

        if idx >= 2:
            ax_map = self.axes[0, 0]
            margen_n = (np.max(self.pos_n[sl]) - np.min(self.pos_n[sl])) * 0.08 + 1.0
            margen_e = (np.max(self.pos_e[sl]) - np.min(self.pos_e[sl])) * 0.08 + 1.0
            ax_map.set_xlim(np.min(self.pos_n[sl]) - margen_n, np.max(self.pos_n[sl]) + margen_n)
            ax_map.set_ylim(np.min(self.pos_e[sl]) - margen_e, np.max(self.pos_e[sl]) + margen_e)

        if idx > 0:
            i = idx - 1
            t_actual = self.tiempo_s[i]
            sigma_n = sigma_desde_varianza(self.cov_pos[i, 0])
            sigma_e = sigma_desde_varianza(self.cov_pos[i, 1])
            self.texto_estado.set_text(
                f"t = {t_actual:.2f} s  |  "
                f"pos NED = ({self.pos_n[i]:.2f}, {self.pos_e[i]:.2f}, {self.pos_d[i]:.2f}) m  |  "
                f"σ_N = {sigma_n:.3f} m  σ_E = {sigma_e:.3f} m  |  "
                f"NIS = {self.nis[i]:.3f}  |  "
                f"frame {idx}/{self.num_frames}"
            )
        else:
            self.texto_estado.set_text("")

        artistas: list = [
            self.trayectoria,
            self.pos_actual,
            self.elipse_incertidumbre,
            self.linea_bias_ax,
            self.linea_bias_ay,
            self.linea_bias_az,
            self.linea_bias_gx,
            self.linea_bias_gy,
            self.linea_bias_gz,
            self.linea_cov_n,
            self.linea_cov_e,
            self.linea_cov_d,
            self.linea_cov_yaw,
            self.linea_nis,
            self.texto_estado,
        ]
        artistas.extend(self.trail_patches)
        return artistas

    def mostrar_animado(self, intervalo_ms: int) -> None:
        anim = animation.FuncAnimation(
            self.fig,
            self._actualizar_marco,
            frames=self.num_frames,
            interval=intervalo_ms,
            blit=False,
            repeat=True,
        )
        self._anim = anim
        plt.tight_layout(rect=[0, 0.03, 1, 0.96])
        plt.show()

    def mostrar_estatico(self) -> None:
        self._actualizar_marco(self.num_frames - 1)
        plt.tight_layout(rect=[0, 0.03, 1, 0.96])
        plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualizador Caja Negra EKF NaviCore-3D (trayectoria, sesgos, covarianza)."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_CSV,
        help=f"Ruta al CSV de caja negra (default: {DEFAULT_CSV})",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=DEFAULT_HZ,
        help=f"Frecuencia de reproducción en Hz (default: {DEFAULT_HZ}).",
    )
    parser.add_argument(
        "--estatico",
        action="store_true",
        help="Mostrar el recorrido completo sin animación.",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Guardar captura PNG del estado final (requiere --estatico).",
    )
    parser.add_argument(
        "--trail-step",
        type=int,
        default=DEFAULT_TRAIL_STEP,
        help=(
            f"Espaciado del rastro de elipses en frames (default: {DEFAULT_TRAIL_STEP} "
            f"→ 1 s @ 100 Hz)."
        ),
    )
    parser.add_argument(
        "--trail-alpha",
        type=float,
        default=DEFAULT_TRAIL_ALPHA,
        help=f"Transparencia del rastro histórico (default: {DEFAULT_TRAIL_ALPHA}).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.hz <= 0:
        print("Error: --hz debe ser un valor positivo.", file=sys.stderr)
        return 1

    if args.trail_step <= 0:
        print("Error: --trail-step debe ser un entero positivo.", file=sys.stderr)
        return 1

    intervalo_ms = max(1, int(round(1000.0 / args.hz)))

    try:
        df = cargar_caja_negra(args.csv)
    except TelemetryIOError as exc:
        print(f"Error de telemetría: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Error inesperado al leer el CSV: {exc}", file=sys.stderr)
        return 1

    duracion_s = float(df["time_us"].iloc[-1]) * 1e-6
    titulo = (
        f"NaviCore-3D · Caja Negra EKF "
        f"({len(df)} muestras @ 100 Hz, {duracion_s:.1f} s)"
    )

    dashboard = DashboardCajaNegra(
        df,
        titulo,
        trail_step=args.trail_step,
        trail_alpha=args.trail_alpha,
    )

    if args.estatico:
        if args.save is None:
            dashboard.mostrar_estatico()
        else:
            dashboard._actualizar_marco(dashboard.num_frames - 1)
            dashboard.fig.tight_layout(rect=[0, 0.03, 1, 0.96])
            args.save.parent.mkdir(parents=True, exist_ok=True)
            dashboard.fig.savefig(args.save, dpi=150, bbox_inches="tight")
            print(f"Captura guardada en {args.save}")
    else:
        print(
            f"Reproduciendo {len(df)} filas a {args.hz:.1f} Hz "
            f"({intervalo_ms} ms/fila), rastro cada {args.trail_step} frames "
            f"(alpha={args.trail_alpha:.2f}). Cierre la ventana para salir."
        )
        dashboard.mostrar_animado(intervalo_ms=intervalo_ms)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
