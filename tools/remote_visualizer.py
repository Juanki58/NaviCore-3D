#!/usr/bin/env python3
"""NaviCore-3D — Visualizador remoto UnityTelemetryPacket (100 Hz SIL)."""

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

from telemetry_protocol import COLOR_MAP, UNITY_TELEMETRY_DEFAULT_PORT
from telemetry_receiver import TelemetryReceiver


class RemoteVisualizer:
    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = UNITY_TELEMETRY_DEFAULT_PORT,
        max_points: int = 500,
    ):
        self.receiver = TelemetryReceiver(host=host, port=port, max_samples=max_points)
        self.fig = None
        self.ax_traj = None
        self.ax_score = None
        self.ax_speed = None
        self.ax_att = None
        self.traj_scatter = None
        self.vel_quiver = None
        self.score_line = None
        self.score_dot = None
        self.speed_line = None
        self.speed_dot = None
        self.yaw_line = None
        self.status_text = None
        self.recovery_artist = None

    def setup_dashboard(self) -> None:
        self.fig, ((self.ax_traj, self.ax_score), (self.ax_speed, self.ax_att)) = plt.subplots(
            2, 2, figsize=(12, 9)
        )
        self.fig.suptitle(
            "NaviCore-3D — Telemetría Unity Unificada (0x4E55 @ 100 Hz)",
            fontsize=14,
            fontweight="bold",
        )

        self.ax_traj.set_title("Trayectoria NED (Norte vs Este)")
        self.ax_traj.set_xlabel("Norte [m]")
        self.ax_traj.set_ylabel("Este [m]")
        self.traj_scatter = self.ax_traj.scatter([], [], s=15, cmap=None)
        self.vel_quiver = None
        (self.recovery_artist,) = self.ax_traj.plot([], [], "b*", markersize=12, label="Hot-Restart")
        self.ax_traj.legend(loc="upper left", fontsize=8)
        self.ax_traj.grid(True)
        self.ax_traj.set_aspect("equal", adjustable="datalim")

        self.ax_score.set_title("HealthScore")
        (self.score_line,) = self.ax_score.plot([], [], "k-", label="Score")
        (self.score_dot,) = self.ax_score.plot([], [], "o", color=COLOR_MAP["NOMINAL"])
        self.ax_score.set_ylim(-5, 105)
        self.ax_score.grid(True)

        self.ax_speed.set_title("Velocidad NED |v| [m/s]")
        (self.speed_line,) = self.ax_speed.plot([], [], color="#2980b9", linewidth=2.0)
        (self.speed_dot,) = self.ax_speed.plot([], [], "o", color="#2980b9", markersize=6)
        self.ax_speed.grid(True)

        self.ax_att.set_title("Actitud (Euler desde cuaternión) [°]")
        (self.yaw_line,) = self.ax_att.plot([], [], color="#8e44ad", linewidth=1.8, label="Yaw")
        self.ax_att.grid(True)
        self.ax_att.legend(loc="upper right", fontsize=8)

        self.status_text = self.fig.text(0.5, 0.01, "", ha="center", fontsize=9)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    def update_plot(self, _frame: int):
        self.receiver.drain()

        if not self.receiver.dirty or not self.receiver.samples:
            artists = [
                self.traj_scatter,
                self.score_line,
                self.score_dot,
                self.speed_line,
                self.speed_dot,
                self.yaw_line,
            ]
            if self.status_text is not None:
                artists.append(self.status_text)
            return artists

        samples = list(self.receiver.samples)
        times = np.fromiter((s.timestamp_s for s in samples), dtype=float, count=len(samples))
        pos_n = np.fromiter((s.pos_n_m for s in samples), dtype=float, count=len(samples))
        pos_e = np.fromiter((s.pos_e_m for s in samples), dtype=float, count=len(samples))
        scores = np.fromiter((s.score for s in samples), dtype=float, count=len(samples))
        speeds = np.fromiter((s.speed_mps for s in samples), dtype=float, count=len(samples))
        yaws = np.fromiter((s.yaw_deg for s in samples), dtype=float, count=len(samples))
        colors = [s.color for s in samples]

        offsets = np.column_stack((pos_n, pos_e))
        self.traj_scatter.set_offsets(offsets)
        self.traj_scatter.set_facecolors(colors)

        if self.vel_quiver is not None:
            self.vel_quiver.remove()
            self.vel_quiver = None

        last = samples[-1]
        if abs(last.vel_n_mps) > 1e-3 or abs(last.vel_e_mps) > 1e-3:
            self.vel_quiver = self.ax_traj.quiver(
                last.pos_n_m,
                last.pos_e_m,
                last.vel_n_mps,
                last.vel_e_mps,
                angles="xy",
                scale_units="xy",
                scale=8.0,
                color="crimson",
                width=0.006,
                label="Velocidad",
            )

        if self.receiver.recovery_points:
            rx, ry = zip(*self.receiver.recovery_points)
            self.recovery_artist.set_data(rx, ry)
        else:
            self.recovery_artist.set_data([], [])

        margin = 5.0
        self.ax_traj.set_xlim(float(pos_n.min()) - margin, float(pos_n.max()) + margin)
        self.ax_traj.set_ylim(float(pos_e.min()) - margin, float(pos_e.max()) + margin)

        self.score_line.set_data(times, scores)
        self.score_dot.set_data([times[-1]], [scores[-1]])
        self.score_dot.set_color(last.color)

        self.speed_line.set_data(times, speeds)
        self.speed_dot.set_data([times[-1]], [speeds[-1]])

        self.yaw_line.set_data(times, yaws)

        t_end = float(times[-1])
        t_start = max(0.0, t_end - 20.0)
        for ax in (self.ax_score, self.ax_speed, self.ax_att):
            ax.set_xlim(t_start, t_end + 2.0)

        self.ax_speed.set_ylim(0.0, max(float(speeds.max()) + 1.0, 5.0))
        self.ax_att.set_ylim(float(yaws.min()) - 5.0, float(yaws.max()) + 5.0)

        self.fig.suptitle(
            f"NaviCore-3D — {last.mission_state_name} | {last.nav_mode_name} | Unity 0x4E55",
            fontsize=14,
            fontweight="bold",
        )
        self.status_text.set_text(
            f"link={self.receiver.link_status()} | seq={last.seq} | "
            f"salud={last.mode}({last.score}) | vel=({last.vel_n_mps:.2f},{last.vel_e_mps:.2f},{last.vel_d_mps:.2f}) m/s | "
            f"att=(R{last.roll_deg:.1f}° P{last.pitch_deg:.1f}° Y{last.yaw_deg:.1f}°) | "
            f"pos_d={last.pos_d_m:.2f} m | evento={self.receiver.latest_event_summary()} | "
            f"invalidos={self.receiver.packets_invalid}"
        )

        self.receiver.clear_dirty()

        artists = [
            self.traj_scatter,
            self.score_line,
            self.score_dot,
            self.speed_line,
            self.speed_dot,
            self.yaw_line,
            self.status_text,
        ]
        if self.vel_quiver is not None:
            artists.append(self.vel_quiver)
        return artists

    def run(self) -> None:
        self.setup_dashboard()
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False, save_count=100)
        plt.show()
        self.receiver.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="NaviCore-3D Remote Unity Telemetry Visualizer")
    parser.add_argument(
        "--port",
        type=int,
        default=UNITY_TELEMETRY_DEFAULT_PORT,
        help="Puerto UDP de escucha (unificado: 5556)",
    )
    args = parser.parse_args()

    print(
        f"[*] Receptor UnityTelemetryPacket (0x4E55, 54 B) escuchando en UDP 0.0.0.0:{args.port}..."
    )
    RemoteVisualizer(port=args.port).run()
