#!/usr/bin/env python3
"""
NaviCore-3D — Dashboard interactivo de telemetría.

Lee docs/telemetria_navicore.csv y muestra tres subgráficos sincronizados:
  1. Trayectoria Pos_X vs Pos_Y coloreada por salud (Nominal/Crítico).
  2. Evolución del HealthScore en el tiempo.
  3. Contador acumulado de paquetes de radio descartados.

Reproduce el CSV secuencialmente a 10 Hz (100 ms por fila) simulando telemetría en vivo.
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
from matplotlib.lines import Line2D

# ---------------------------------------------------------------------------
# Constantes de configuración
# ---------------------------------------------------------------------------

DEFAULT_CSV = Path(__file__).resolve().parent.parent / "docs" / "telemetria_navicore.csv"

# Intervalo entre frames: 100 ms → 10 Hz (coincide con el tick del simulador).
FRAME_INTERVAL_MS = 100

# Colores por modo de salud (coherencia visual con el README / dashboard ASCII).
HEALTH_COLORS: dict[str, str] = {
    "NOMINAL": "#2ecc71",   # Verde — operación nominal
    "CRITICAL": "#e74c3c",  # Rojo — parada segura / salud crítica
    "DEGRADED": "#f39c12",  # Ámbar — contingencia degradada
}

HEALTH_COLOR_DEFAULT = "#95a5a6"  # Gris para modos no mapeados
RECOVERY_MARKER_COLOR = "#3498db"  # Azul — punto de hot-restart

COLUMNAS_REQUERIDAS = frozenset(
    {
        "Timestamp_ms",
        "Escenario",
        "Pos_X",
        "Pos_Y",
        "HealthScore",
        "HealthMode",
        "RadioDroppedPackets",
    }
)


# ---------------------------------------------------------------------------
# Excepciones y carga de datos
# ---------------------------------------------------------------------------


class TelemetryIOError(Exception):
    """Error de lectura del CSV (vacío, bloqueado o columnas inválidas)."""


def health_color(mode: str) -> str:
    """Devuelve el color asociado al modo de salud (normalizado a mayúsculas)."""
    return HEALTH_COLORS.get(str(mode).strip().upper(), HEALTH_COLOR_DEFAULT)


def cargar_telemetria(
    csv_path: Path,
    escenario: str | None = None,
    reintentos: int = 3,
    espera_reintento_s: float = 0.5,
) -> pd.DataFrame:
    """
    Carga y valida el CSV de telemetría.

    Reintenta la lectura si el archivo está bloqueado por el simulador (Windows/Linux).
    """
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
                f"El archivo está en uso y no pudo leerse tras {reintentos} intentos: {csv_path}\n"
                "Cierre el simulador o espere a que termine de escribir el CSV."
            ) from exc
        except pd.errors.EmptyDataError as exc:
            raise TelemetryIOError(
                f"El archivo de telemetría está vacío (sin filas de datos): {csv_path}"
            ) from exc
    else:
        raise TelemetryIOError(f"No se pudo leer {csv_path}: {ultimo_error}")

    if df.empty:
        raise TelemetryIOError(
            f"El archivo de telemetría no contiene filas de datos: {csv_path}"
        )

    faltantes = COLUMNAS_REQUERIDAS - set(df.columns)
    if faltantes:
        raise TelemetryIOError(
            f"Columnas obligatorias ausentes en el CSV: {sorted(faltantes)}"
        )

    df = df.sort_values("Timestamp_ms").reset_index(drop=True)
    df["HealthMode"] = df["HealthMode"].astype(str).str.strip().str.upper()
    df["Escenario"] = df["Escenario"].astype(str).str.strip()

    if escenario is not None:
        df = df[df["Escenario"] == escenario].reset_index(drop=True)
        if df.empty:
            raise TelemetryIOError(
                f"No hay filas para el escenario '{escenario}' en {csv_path}"
            )

    return df


def detectar_indice_recuperacion(df: pd.DataFrame) -> int | None:
    """
    Localiza el primer tick de recuperación en caliente (Hot-Restart).

    Criterio: transición CRITICAL/DEGRADED → NOMINAL, o salto de HealthScore
    desde estado crítico (≤10) a puntuación de recuperación (≥75).
    """
    if len(df) < 2:
        return None

    modos = df["HealthMode"].to_numpy()
    scores = df["HealthScore"].to_numpy(dtype=float)

    for i in range(1, len(df)):
        prev_modo = modos[i - 1]
        curr_modo = modos[i]
        if prev_modo in ("CRITICAL", "DEGRADED") and curr_modo == "NOMINAL":
            return i
        if scores[i - 1] <= 10.0 and scores[i] >= 75.0:
            return i

    return None


def construir_segmentos_coloreados(
    x: np.ndarray,
    y: np.ndarray,
    modos: np.ndarray,
    hasta: int,
) -> tuple[list[np.ndarray], list[str]]:
    """
    Construye segmentos de línea [(x0,y0),(x1,y1)] con color según salud del punto destino.
    """
    limite = min(hasta, len(x))
    if limite < 2:
        return [], []

    segmentos: list[np.ndarray] = []
    colores: list[str] = []

    for i in range(1, limite):
        segmentos.append(np.array([[x[i - 1], y[i - 1]], [x[i], y[i]]]))
        colores.append(health_color(str(modos[i])))

    return segmentos, colores


# ---------------------------------------------------------------------------
# Dashboard matplotlib
# ---------------------------------------------------------------------------


class DashboardTelemetria:
    """Tres subgráficos sincronizados con reproducción secuencial a 10 Hz."""

    def __init__(self, df: pd.DataFrame, titulo: str) -> None:
        self.df = df
        self.titulo = titulo

        self.tiempo_s = df["Timestamp_ms"].to_numpy(dtype=float) * 1e-3
        self.pos_x = df["Pos_X"].to_numpy(dtype=float)
        self.pos_y = df["Pos_Y"].to_numpy(dtype=float)
        self.health_score = df["HealthScore"].to_numpy(dtype=float)
        self.health_mode = df["HealthMode"].to_numpy()
        self.radio_dropped = df["RadioDroppedPackets"].to_numpy(dtype=float)

        self.indice_recuperacion = detectar_indice_recuperacion(df)
        self.num_frames = len(df)

        self.fig, self.axes = plt.subplots(1, 3, figsize=(15, 5))
        self.fig.suptitle(titulo, fontsize=13, fontweight="bold")

        self._configurar_ejes()
        self._crear_artistas()

    def _configurar_ejes(self) -> None:
        ax_tray, ax_health, ax_radio = self.axes

        ax_tray.set_title("Trayectoria (Pos_X vs Pos_Y)")
        ax_tray.set_xlabel("Pos_X")
        ax_tray.set_ylabel("Pos_Y")
        ax_tray.set_aspect("equal", adjustable="box")
        ax_tray.grid(True, alpha=0.25)

        ax_health.set_title("HealthScore")
        ax_health.set_xlabel("Tiempo (s)")
        ax_health.set_ylabel("Puntuación")
        ax_health.set_ylim(-5, 105)
        ax_health.grid(True, alpha=0.25)

        ax_radio.set_title("Paquetes de radio descartados")
        ax_radio.set_xlabel("Tiempo (s)")
        ax_radio.set_ylabel("RadioDroppedPackets")
        max_drop = float(np.max(self.radio_dropped)) if len(self.radio_dropped) else 1.0
        ax_radio.set_ylim(-0.5, max(max_drop * 1.1, 1.0))
        ax_radio.grid(True, alpha=0.25)

        # Leyenda de colores de salud
        leyenda = [
            Line2D([0], [0], color=HEALTH_COLORS["NOMINAL"], linewidth=2.5, label="Nominal"),
            Line2D([0], [0], color=HEALTH_COLORS["DEGRADED"], linewidth=2.5, label="Degradado"),
            Line2D([0], [0], color=HEALTH_COLORS["CRITICAL"], linewidth=2.5, label="Crítico"),
            Line2D(
                [0],
                [0],
                marker="*",
                color="w",
                markerfacecolor=RECOVERY_MARKER_COLOR,
                markersize=12,
                label="Recuperación",
            ),
        ]
        ax_tray.legend(handles=leyenda, loc="upper left", fontsize=8)

    def _crear_artistas(self) -> None:
        ax_tray, ax_health, ax_radio = self.axes

        self.line_collection = LineCollection([], linewidths=2.0, alpha=0.9)
        ax_tray.add_collection(self.line_collection)

        (self.pos_actual,) = ax_tray.plot([], [], "ko", markersize=5, zorder=5)

        self.marcador_recuperacion = ax_tray.scatter(
            [],
            [],
            c=RECOVERY_MARKER_COLOR,
            s=180,
            marker="*",
            edgecolors="black",
            linewidths=0.5,
            zorder=6,
            visible=False,
        )

        (self.linea_health,) = ax_health.plot([], [], color="#2980b9", linewidth=2.0)
        (self.punto_health,) = ax_health.plot([], [], "o", color="#2980b9", markersize=6)

        (self.linea_radio,) = ax_radio.plot([], [], color="#8e44ad", linewidth=2.0, drawstyle="steps-post")
        (self.punto_radio,) = ax_radio.plot([], [], "s", color="#8e44ad", markersize=6)

        self.texto_tiempo = self.fig.text(
            0.5,
            0.02,
            "",
            ha="center",
            fontsize=10,
            color="#2c3e50",
        )

    def _actualizar_marco(self, frame: int) -> list:
        """Callback de FuncAnimation: muestra datos hasta la fila `frame` (inclusive)."""
        idx = min(frame + 1, self.num_frames)
        sl = slice(0, idx)

        segmentos, colores = construir_segmentos_coloreados(
            self.pos_x,
            self.pos_y,
            self.health_mode,
            idx,
        )
        self.line_collection.set_segments(segmentos)
        self.line_collection.set_color(colores)

        if idx > 0:
            self.pos_actual.set_data([self.pos_x[idx - 1]], [self.pos_y[idx - 1]])
        else:
            self.pos_actual.set_data([], [])

        # Marcador de recuperación en caliente (visible desde el tick de transición)
        if self.indice_recuperacion is not None and idx > self.indice_recuperacion:
            ri = self.indice_recuperacion
            self.marcador_recuperacion.set_offsets([[self.pos_x[ri], self.pos_y[ri]]])
            self.marcador_recuperacion.set_visible(True)
        else:
            self.marcador_recuperacion.set_visible(False)

        t = self.tiempo_s[sl]
        self.linea_health.set_data(t, self.health_score[sl])
        self.linea_radio.set_data(t, self.radio_dropped[sl])

        if idx > 0:
            self.punto_health.set_data([self.tiempo_s[idx - 1]], [self.health_score[idx - 1]])
            self.punto_radio.set_data([self.tiempo_s[idx - 1]], [self.radio_dropped[idx - 1]])
        else:
            self.punto_health.set_data([], [])
            self.punto_radio.set_data([], [])

        # Auto-zoom suave de la trayectoria conforme avanza la simulación
        if idx >= 2:
            ax_tray = self.axes[0]
            margen_x = (np.max(self.pos_x[sl]) - np.min(self.pos_x[sl])) * 0.05 + 1e-6
            margen_y = (np.max(self.pos_y[sl]) - np.min(self.pos_y[sl])) * 0.05 + 1e-6
            ax_tray.set_xlim(np.min(self.pos_x[sl]) - margen_x, np.max(self.pos_x[sl]) + margen_x)
            ax_tray.set_ylim(np.min(self.pos_y[sl]) - margen_y, np.max(self.pos_y[sl]) + margen_y)

        t_actual = self.tiempo_s[idx - 1] if idx > 0 else 0.0
        escenario = self.df["Escenario"].iloc[0] if len(self.df) else "—"
        modo = self.health_mode[idx - 1] if idx > 0 else "—"
        score = int(self.health_score[idx - 1]) if idx > 0 else 0
        drops = int(self.radio_dropped[idx - 1]) if idx > 0 else 0
        self.texto_tiempo.set_text(
            f"t = {t_actual:.1f} s  |  {escenario}  |  salud={modo}({score})  |  "
            f"radio descartados={drops}  |  frame {idx}/{self.num_frames}"
        )

        artistas = [
            self.line_collection,
            self.pos_actual,
            self.marcador_recuperacion,
            self.linea_health,
            self.punto_health,
            self.linea_radio,
            self.punto_radio,
            self.texto_tiempo,
        ]
        return artistas

    def mostrar_animado(self, intervalo_ms: int = FRAME_INTERVAL_MS) -> None:
        """Reproduce el CSV fila a fila simulando telemetría en vivo a 10 Hz."""
        anim = animation.FuncAnimation(
            self.fig,
            self._actualizar_marco,
            frames=self.num_frames,
            interval=intervalo_ms,
            blit=False,
            repeat=True,
        )
        self._anim = anim  # Referencia fuerte: evita GC prematuro de la animación
        plt.tight_layout(rect=[0, 0.04, 1, 0.95])
        plt.show()

    def mostrar_estatico(self) -> None:
        """Dibuja el recorrido completo sin animación (útil para capturas)."""
        self._actualizar_marco(self.num_frames - 1)
        plt.tight_layout(rect=[0, 0.04, 1, 0.95])
        plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dashboard interactivo NaviCore-3D (trayectoria, salud, radio)."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_CSV,
        help=f"Ruta al CSV de telemetría (default: {DEFAULT_CSV})",
    )
    parser.add_argument(
        "--escenario",
        type=str,
        default=None,
        help="Filtrar por nombre de escenario (p. ej. HIGH_DEMAND_STRESS_TEST).",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="Frecuencia de reproducción en Hz (default: 10 → 100 ms/fila).",
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
        help="Guardar captura PNG del estado final (requiere --estatico o último frame).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.hz <= 0:
        print("Error: --hz debe ser un valor positivo.", file=sys.stderr)
        return 1

    intervalo_ms = int(round(1000.0 / args.hz))

    try:
        df = cargar_telemetria(args.csv, escenario=args.escenario)
    except TelemetryIOError as exc:
        print(f"Error de telemetría: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"Error inesperado al leer el CSV: {exc}", file=sys.stderr)
        return 1

    escenario = df["Escenario"].iloc[0]
    duracion_s = float(df["Timestamp_ms"].iloc[-1]) * 1e-3
    titulo = (
        f"NaviCore-3D · Dashboard de telemetría — {escenario} "
        f"({len(df)} muestras, {duracion_s:.1f} s)"
    )

    dashboard = DashboardTelemetria(df, titulo)

    if args.estatico:
        dashboard.mostrar_estatico()
        if args.save is not None:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            dashboard.fig.savefig(args.save, dpi=150, bbox_inches="tight")
            print(f"Captura guardada en {args.save}")
    else:
        print(
            f"Reproduciendo {len(df)} filas a {args.hz:.1f} Hz "
            f"({intervalo_ms} ms/fila). Cierre la ventana para salir."
        )
        dashboard.mostrar_animado(intervalo_ms=intervalo_ms)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
