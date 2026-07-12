#!/usr/bin/env python3
"""NaviCore-3D — Visualizador remoto de telemetría UDP v3 (10 Hz HIL)."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from telemetry_protocol import COLOR_MAP, TELEMETRY_UDP_DEFAULT_PORT
from telemetry_receiver import TelemetryReceiver


class RemoteVisualizer:
    def __init__(self, host: str = "0.0.0.0", port: int = TELEMETRY_UDP_DEFAULT_PORT, max_points: int = 200):
        self.receiver = TelemetryReceiver(host=host, port=port, max_samples=max_points)
        self.fig = None
        self.ax_traj = None
        self.ax_score = None
        self.ax_temp = None
        self.ax_radio = None
        self.traj_scatter = None
        self.route_line = None
        self.score_line = None
        self.score_dot = None
        self.temp_line = None
        self.temp_dot = None
        self.radio_line = None
        self.status_text = None
        self.recovery_artist = None

    def setup_dashboard(self) -> None:
        self.fig, ((self.ax_traj, self.ax_score), (self.ax_temp, self.ax_radio)) = plt.subplots(
            2, 2, figsize=(12, 9)
        )
        self.fig.suptitle(
            "NaviCore-3D — Telemetría Remota v3 (10 Hz HIL)",
            fontsize=14,
            fontweight="bold",
        )

        self.ax_traj.set_title("Trayectoria (Pos_X vs Pos_Y)")
        self.traj_scatter = self.ax_traj.scatter([], [], s=15, cmap=None)
        (self.route_line,) = self.ax_traj.plot([], [], "k--", alpha=0.35, linewidth=1.0, label="Along-Track")
        (self.recovery_artist,) = self.ax_traj.plot([], [], "b*", markersize=12, label="Hot-Restart")
        self.ax_traj.legend(loc="upper left", fontsize=8)
        self.ax_traj.grid(True)

        self.ax_score.set_title("HealthScore")
        (self.score_line,) = self.ax_score.plot([], [], "k-", label="Score")
        (self.score_dot,) = self.ax_score.plot([], [], "o", color=COLOR_MAP["NOMINAL"])
        self.ax_score.set_ylim(-5, 105)
        self.ax_score.grid(True)

        self.ax_temp.set_title("Temperatura (°C)")
        (self.temp_line,) = self.ax_temp.plot([], [], color="#e67e22", linewidth=2.0)
        (self.temp_dot,) = self.ax_temp.plot([], [], "o", color="#e67e22", markersize=6)
        self.ax_temp.grid(True)

        self.ax_radio.set_title("Paquetes de radio descartados")
        (self.radio_line,) = self.ax_radio.plot([], [], "tab:purple", drawstyle="steps-post")
        self.ax_radio.grid(True)

        self.status_text = self.fig.text(0.5, 0.01, "", ha="center", fontsize=9)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    def update_plot(self, _frame: int):
        self.receiver.drain()

        if not self.receiver.dirty or not self.receiver.samples:
            artists = [
                self.traj_scatter,
                self.route_line,
                self.score_line,
                self.score_dot,
                self.temp_line,
                self.temp_dot,
                self.radio_line,
            ]
            if self.status_text is not None:
                artists.append(self.status_text)
            return artists

        samples = list(self.receiver.samples)
        times = np.fromiter((s.timestamp_s for s in samples), dtype=float, count=len(samples))
        pos_x = np.fromiter((s.x for s in samples), dtype=float, count=len(samples))
        pos_y = np.fromiter((s.y for s in samples), dtype=float, count=len(samples))
        scores = np.fromiter((s.score for s in samples), dtype=float, count=len(samples))
        temps = np.fromiter((s.temperature_c for s in samples), dtype=float, count=len(samples))
        dropped = np.fromiter((s.dropped_packets for s in samples), dtype=float, count=len(samples))
        along = np.fromiter((s.along_track_m for s in samples), dtype=float, count=len(samples))
        colors = [s.color for s in samples]

        offsets = np.column_stack((pos_x, pos_y))
        self.traj_scatter.set_offsets(offsets)
        self.traj_scatter.set_facecolors(colors)

        self.route_line.set_data(along, pos_y)

        if self.receiver.recovery_points:
            rx, ry = zip(*self.receiver.recovery_points)
            self.recovery_artist.set_data(rx, ry)
        else:
            self.recovery_artist.set_data([], [])

        self.ax_traj.set_xlim(float(pos_x.min()) - 5.0, float(pos_x.max()) + 5.0)
        self.ax_traj.set_ylim(float(pos_y.min()) - 5.0, float(pos_y.max()) + 5.0)

        self.score_line.set_data(times, scores)
        self.score_dot.set_data([times[-1]], [scores[-1]])
        self.score_dot.set_color(samples[-1].color)

        self.temp_line.set_data(times, temps)
        self.temp_dot.set_data([times[-1]], [temps[-1]])

        self.radio_line.set_data(times, dropped)

        t_end = float(times[-1])
        t_start = max(0.0, t_end - 20.0)
        for ax in (self.ax_score, self.ax_temp, self.ax_radio):
            ax.set_xlim(t_start, t_end + 2.0)

        self.ax_temp.set_ylim(float(temps.min()) - 2.0, float(temps.max()) + 2.0)
        self.ax_radio.set_ylim(-1.0, max(float(dropped.max()) + 5.0, 10.0))

        last = samples[-1]
        self.fig.suptitle(
            f"NaviCore-3D — {last.scenario_name} | {last.nav_mode_name} | 10 Hz HIL",
            fontsize=14,
            fontweight="bold",
        )
        self.status_text.set_text(
            f"link={self.receiver.link_status()} | seq={last.seq} | "
            f"salud={last.mode}({last.score}) | temp={last.temperature_c:.1f}°C | "
            f"cross={last.cross_track_m:.2f}m along={last.along_track_m:.1f}m | "
            f"evento={self.receiver.latest_event_summary()} | "
            f"invalidos={self.receiver.packets_invalid}"
        )

        self.receiver.clear_dirty()

        return (
            self.traj_scatter,
            self.route_line,
            self.score_line,
            self.score_dot,
            self.temp_line,
            self.temp_dot,
            self.radio_line,
            self.status_text,
        )

    def run(self) -> None:
        self.setup_dashboard()
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False, save_count=100)
        plt.show()
        self.receiver.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="NaviCore-3D Remote Web/Radio Telemetry Visualizer")
    parser.add_argument("--port", type=int, default=TELEMETRY_UDP_DEFAULT_PORT, help="Puerto UDP de escucha")
    args = parser.parse_args()

    print(f"[*] Servidor de telemetría remota v3 escuchando en UDP 0.0.0.0:{args.port}...")
    RemoteVisualizer(port=args.port).run()
