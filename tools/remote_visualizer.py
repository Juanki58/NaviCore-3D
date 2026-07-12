#!/usr/bin/env python3
"""NaviCore-3D — Visualizador remoto de telemetría UDP (10 Hz HIL)."""

import socket
import struct
import argparse
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# Configuración y mapeo de flags de salud (Fase B/X)
HEALTH_MODES = {0: "NOMINAL", 1: "DEGRADED", 2: "CRITICAL"}
COLOR_MAP = {"NOMINAL": "green", "DEGRADED": "orange", "CRITICAL": "red"}


class RemoteVisualizer:
    def __init__(self, host="0.0.0.0", port=5005, max_points=200):
        self.host = host
        self.port = port
        self.max_points = max_points

        # Buffers estáticos circulares para la visualización fluida
        self.times = deque(maxlen=max_points)
        self.pos_x = deque(maxlen=max_points)
        self.pos_y = deque(maxlen=max_points)
        self.pos_z = deque(maxlen=max_points)
        self.scores = deque(maxlen=max_points)
        self.packets = deque(maxlen=max_points)
        self.colors = deque(maxlen=max_points)

        # Puntos marcadores de recuperación en caliente (Hot-Restart)
        self.recovery_x = []
        self.recovery_y = []

        # Inicialización del socket UDP no bloqueante (Escucha en ráfaga)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self.sock.setblocking(False)

        self.current_time_tick = 0
        self.last_score = 100

    def unpack_telemetry(self):
        """Lee el buffer de red y desempaqueta el payload atómico de 16 bytes"""
        try:
            # Captura el paquete binario crudo del buffer de red
            data, _ = self.sock.recvfrom(16)
            if len(data) == 16:
                # Estructura C++: float x, float y, float z, uint16_t score, uint16_t flags
                x, y, z, score, flags = struct.unpack("<fffHH", data)

                # Decodificación de flags (Bits 0-1: HealthMode, Bits 2-15: Dropped Packets)
                mode_bits = flags & 0x03
                dropped_packets = flags >> 2
                mode_str = HEALTH_MODES.get(mode_bits, "CRITICAL")

                self.current_time_tick += 0.1
                self.times.append(self.current_time_tick)
                self.pos_x.append(x)
                self.pos_y.append(y)
                self.pos_z.append(z)
                self.scores.append(score)
                self.packets.append(dropped_packets)
                self.colors.append(COLOR_MAP[mode_str])

                # Detección del marcador de recuperación en caliente (Hot-Restart)
                if (self.last_score <= 10 and score >= 75) or (
                    mode_str == "NOMINAL"
                    and len(self.colors) > 1
                    and self.colors[-2] != "green"
                ):
                    self.recovery_x.append(x)
                    self.recovery_y.append(y)

                self.last_score = score
        except BlockingIOError:
            # El buffer de red está vacío en este tick, mantiene el flujo asíncrono
            pass

    def setup_dashboard(self):
        """Inicializa la estructura de tres subgráficos en Matplotlib"""
        self.fig, (self.ax_traj, self.ax_score, self.ax_radio) = plt.subplots(3, 1, figsize=(10, 8))
        self.fig.suptitle(
            "NaviCore-3D — Telemetría de Red Remota (10 Hz HIL)",
            fontsize=14,
            fontweight="bold",
        )

        # Subgráfico 1: Trayectoria
        self.ax_traj.set_title("Trayectoria en Tiempo Real (Pos_X vs Pos_Y)")
        self.traj_scatter = self.ax_traj.scatter([], [], c=[], s=10, cmap=None)
        (self.recovery_marker,) = self.ax_traj.plot(
            [], [], "b*", markersize=12, label="Hot-Restart Event"
        )
        self.ax_traj.legend(loc="upper left")
        self.ax_traj.grid(True)

        # Subgráfico 2: HealthScore
        self.ax_score.set_title("Evolución del Monitor de Salud (HealthScore)")
        (self.score_line,) = self.ax_score.plot([], [], "k-", label="Score")
        (self.score_dot,) = self.ax_score.plot([], [], "ro")
        self.ax_score.set_ylim(-5, 105)
        self.ax_score.grid(True)

        # Subgráfico 3: RadioDroppedPackets
        self.ax_radio.set_title("Contador Acumulado de Paquetes Descartados (Radio)")
        (self.radio_line,) = self.ax_radio.plot([], [], "tab:purple", drawstyle="steps-post")
        self.ax_radio.grid(True)

        plt.tight_layout()

    def update_plot(self, frame):
        """Bucle de refresco dinámico invocado por el temporizador de la animación"""
        # Extrae todas las ráfagas acumuladas en el buffer de red antes de redibujar
        for _ in range(10):
            self.unpack_telemetry()

        if not self.times:
            return self.traj_scatter, self.score_line, self.score_dot, self.radio_line

        # Actualiza el gráfico de trayectoria por tramos cromáticos
        if len(self.pos_x) > 0:
            self.ax_traj.cla()
            self.ax_traj.set_title("Trayectoria en Tiempo Real (Pos_X vs Pos_Y)")
            self.ax_traj.grid(True)
            self.ax_traj.scatter(list(self.pos_x), list(self.pos_y), c=list(self.colors), s=15)
            if self.recovery_x:
                self.ax_traj.plot(
                    self.recovery_x, self.recovery_y, "b*", markersize=14, label="Hot-Restart"
                )
                self.ax_traj.legend(loc="upper left")

            # Ajuste dinámico de márgenes de visión
            self.ax_traj.set_xlim(min(self.pos_x) - 5, max(self.pos_x) + 5)
            self.ax_traj.set_ylim(min(self.pos_y) - 5, max(self.pos_y) + 5)

        # Actualiza el subgráfico temporal del HealthScore
        self.score_line.set_data(list(self.times), list(self.scores))
        self.score_dot.set_data([self.times[-1]], [self.scores[-1]])
        self.ax_score.set_xlim(max(0, self.times[-1] - 20), self.times[-1] + 2)

        # Actualiza el subgráfico de descarte en formato escalonado
        self.radio_line.set_data(list(self.times), list(self.packets))
        self.ax_radio.set_xlim(max(0, self.times[-1] - 20), self.times[-1] + 2)
        self.ax_radio.set_ylim(-1, max(self.packets) + 5 if self.packets else 10)

        return self.score_line, self.score_dot, self.radio_line

    def run(self):
        self.setup_dashboard()
        # Fuerza un refresco cada 100 ms (10 Hz), emparejado con el tick del firmware
        self.ani = FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False, save_count=100
        )
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="NaviCore-3D Remote Web/Radio Telemetry Visualizer"
    )
    parser.add_argument("--port", type=int, default=5005, help="Puerto UDP de escucha")
    args = parser.parse_args()

    print(f"[*] Servidor de telemetría remota escuchando en UDP 0.0.0.0:{args.port}...")
    visualizer = RemoteVisualizer(port=args.port)
    visualizer.run()
