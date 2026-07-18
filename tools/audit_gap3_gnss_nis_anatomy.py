#!/usr/bin/env python3
"""GAP-3.1 / 3.2 / 3.3 — Anatomía del NIS GNSS y evolución temporal.

Sin sweep de parámetros. Responde:
  3.1 — innovación, S, NIS descompuesto, gate, corrección aplicada
  3.2 — ||x_pred - GPS|| vs tiempo + aceptación + umbral de rechazo
  3.3 — acoplamiento GNSS→velocidad (pseudo-innovación + dx_vel vs dx_pos)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"

NIS_CSV = BENCH_DIR / "gap3_gnss_nis_anatomy.csv"
REPORT_JSON = BENCH_DIR / "gap3_gnss_nis_anatomy_report.json"
TIMELINE_PNG = BENCH_DIR / "gap3_gnss_predict_error_timeline.png"
NIS_DECOMP_PNG = BENCH_DIR / "gap3_gnss_nis_decomposition.png"
VEL_COUPLING_PNG = BENCH_DIR / "gap3_gnss_velocity_coupling.png"

AUDIT_END_S = 60.0
NIS_THRESHOLD = 11.345

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


@dataclass
class GnssNisRow:
    timestamp_s: float
    gps_index: int
    z_n: float
    z_e: float
    hx_n: float
    hx_e: float
    innov_n: float
    innov_e: float
    innov_d: float
    innov_h: float
    pred_error_3d: float
    vel_pred_h: float
    gps_speed: float
    has_gps_speed: bool
    pseudo_innov_v_h: float
    hph_nn: float
    hph_ee: float
    r_m2: float
    s_nn: float
    s_ee: float
    s_dd: float
    nis_full: float
    nis_horizontal_2d: float
    nis_d_marginal: float
    nis_contrib_n: float
    nis_contrib_e: float
    nis_contrib_d: float
    nis_threshold: float
    accepted: bool
    reject_reason: int
    k_vel_max: float
    dx_pos_h: float
    dx_vel_h: float
    corr_pos_h: float
    corr_vel_h: float
    vel_after_h: float
    dt_since_prev_gnss: float


def load_nis_csv(path: Path) -> list[GnssNisRow]:
    rows: list[GnssNisRow] = []
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        for raw in reader:
            dx_pos_h = math.hypot(float(raw["dx_pos_n_m"]), float(raw["dx_pos_e_m"]))
            dx_vel_h = math.hypot(float(raw["dx_vel_n_mps"]), float(raw["dx_vel_e_mps"]))
            rows.append(
                GnssNisRow(
                    timestamp_s=float(raw["timestamp_s"]),
                    gps_index=int(raw["gps_index"]),
                    z_n=float(raw["z_n_m"]),
                    z_e=float(raw["z_e_m"]),
                    hx_n=float(raw["hx_n_m"]),
                    hx_e=float(raw["hx_e_m"]),
                    innov_n=float(raw["innov_n_m"]),
                    innov_e=float(raw["innov_e_m"]),
                    innov_d=float(raw["innov_d_m"]),
                    innov_h=float(raw["innov_h_m"]),
                    pred_error_3d=float(raw["pred_error_3d_m"]),
                    vel_pred_h=float(raw["vel_pred_h_mps"]),
                    gps_speed=float(raw["gps_speed_mps"]),
                    has_gps_speed=raw["has_gps_speed"] == "1",
                    pseudo_innov_v_h=float(raw["pseudo_innov_v_h_mps"]),
                    hph_nn=float(raw["hph_nn"]),
                    hph_ee=float(raw["hph_ee"]),
                    r_m2=float(raw["r_m2"]),
                    s_nn=float(raw["s_nn"]),
                    s_ee=float(raw["s_ee"]),
                    s_dd=float(raw["s_dd"]),
                    nis_full=float(raw["nis_full"]),
                    nis_horizontal_2d=float(raw["nis_horizontal_2d"]),
                    nis_d_marginal=float(raw["nis_d_marginal"]),
                    nis_contrib_n=float(raw["nis_contrib_n"]),
                    nis_contrib_e=float(raw["nis_contrib_e"]),
                    nis_contrib_d=float(raw["nis_contrib_d"]),
                    nis_threshold=float(raw["nis_threshold"]),
                    accepted=str(raw["accepted"]).strip() in {"1", "1.0", "1.000000"},
                    reject_reason=int(float(raw["reject_reason"])),
                    k_vel_max=float(raw["k_vel_max"]),
                    dx_pos_h=dx_pos_h,
                    dx_vel_h=dx_vel_h,
                    corr_pos_h=float(raw["corr_pos_h_m"]),
                    corr_vel_h=float(raw["corr_vel_h_mps"]),
                    vel_after_h=float(raw["vel_after_h_mps"]),
                    dt_since_prev_gnss=float(raw["dt_since_prev_gnss_s"] or 0.0),
                )
            )
    return rows


def run_replay(replay_exe: Path, replay_csv: Path, calibration: Path, skip_run: bool) -> None:
    if skip_run:
        return
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")
    ensure_calibration(calibration)
    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--h9a-gravity-tilt-init",
        "--output",
        str(BENCH_DIR / "gap3_gnss_nis_replay_output.csv"),
        "--gap3-gnss-nis-audit-csv",
        str(NIS_CSV),
    ]
    print("RUN:", " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)


def find_reject_threshold(rows: list[GnssNisRow]) -> dict:
    accepted = [r for r in rows if r.accepted]
    rejected = [r for r in rows if not r.accepted]
    if not accepted or not rejected:
        return {}
    max_accept_error = max(r.innov_h for r in accepted)
    min_reject_error = min(r.innov_h for r in rejected)
    return {
        "max_innov_h_accepted_m": max_accept_error,
        "min_innov_h_rejected_m": min_reject_error,
        "threshold_band_m": (max_accept_error, min_reject_error),
        "first_reject_timestamp_s": rejected[0].timestamp_s if rejected else None,
        "first_reject_innov_h_m": rejected[0].innov_h if rejected else None,
        "last_accept_timestamp_s": accepted[-1].timestamp_s if accepted else None,
        "last_accept_innov_h_m": accepted[-1].innov_h if accepted else None,
    }


def nis_anatomy_summary(rows: list[GnssNisRow]) -> dict:
    if not rows:
        return {}
    contrib_sum = np.array(
        [
            [r.nis_contrib_n, r.nis_contrib_e, r.nis_contrib_d]
            for r in rows
            if r.nis_full > 0.0
        ],
        dtype=float,
    )
    frac = None
    if contrib_sum.size:
        mean_contrib = np.mean(contrib_sum, axis=0)
        total = float(np.sum(mean_contrib))
        frac = {
            "N": mean_contrib[0] / total if total > 0 else 0.0,
            "E": mean_contrib[1] / total if total > 0 else 0.0,
            "D": mean_contrib[2] / total if total > 0 else 0.0,
        }
    return {
        "mean_innov_h_m": float(np.mean([r.innov_h for r in rows])),
        "mean_nis_full": float(np.mean([r.nis_full for r in rows if r.nis_full > 0])),
        "mean_nis_horizontal_2d": float(np.mean([r.nis_horizontal_2d for r in rows if r.nis_horizontal_2d > 0])),
        "mean_nis_d_marginal": float(np.mean([r.nis_d_marginal for r in rows if r.nis_d_marginal > 0])),
        "mean_sqrt_hph_h_m": float(np.mean([math.sqrt(r.hph_nn + r.r_m2) for r in rows])),
        "mean_sqrt_s_h_m": float(np.mean([math.sqrt(r.s_nn) for r in rows])),
        "nis_contrib_fraction": frac,
        "nis_vs_innov_h_correlation": float(
            np.corrcoef([r.innov_h for r in rows], [r.nis_full for r in rows])[0, 1]
        )
        if len(rows) > 2
        else None,
        "diagnosis": (
            "NIS grande porque la predicción llega desplazada (innov_h >> sqrt(S))"
            if float(np.mean([r.innov_h for r in rows]))
            > 3.0 * float(np.mean([math.sqrt(r.s_nn) for r in rows]))
            else "NIS grande porque S es pequeño (covarianza subestimada)"
        ),
    }


def velocity_coupling_summary(rows: list[GnssNisRow]) -> dict:
    accepted = [r for r in rows if r.accepted]
    with_speed = [r for r in rows if r.has_gps_speed and r.gps_speed > 0.5]
    if not accepted:
        return {"note": "sin aceptaciones para acoplamiento"}
    ratio_vel_pos = [
        r.dx_vel_h / r.dx_pos_h for r in accepted if r.dx_pos_h > 1.0e-4 and r.dx_vel_h > 0.0
    ]
    return {
        "accept_count": len(accepted),
        "mean_pseudo_innov_v_h_mps": float(np.mean([r.pseudo_innov_v_h for r in with_speed]))
        if with_speed
        else None,
        "mean_vel_pred_h_mps": float(np.mean([r.vel_pred_h for r in with_speed])) if with_speed else None,
        "mean_gps_speed_mps": float(np.mean([r.gps_speed for r in with_speed])) if with_speed else None,
        "mean_corr_vel_h_on_accept_mps": float(np.mean([r.corr_vel_h for r in accepted])),
        "mean_corr_pos_h_on_accept_m": float(np.mean([r.corr_pos_h for r in accepted])),
        "mean_k_vel_max_on_accept": float(np.mean([r.k_vel_max for r in accepted])),
        "median_dx_vel_over_dx_pos": float(np.median(ratio_vel_pos)) if ratio_vel_pos else None,
        "vel_after_vs_gps_speed_mean_delta": float(
            np.mean([r.vel_after_h - r.gps_speed for r in with_speed if r.has_gps_speed])
        )
        if with_speed
        else None,
    }


def plot_timeline(rows: list[GnssNisRow], threshold_info: dict, out_png: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    t = [r.timestamp_s for r in rows]
    innov_h = [r.innov_h for r in rows]
    nis = [r.nis_full for r in rows]
    colors = ["#2ca02c" if r.accepted else "#d62728" for r in rows]

    ax0 = axes[0]
    ax0.scatter(t, innov_h, c=colors, s=28, zorder=3)
    ax0.plot(t, innov_h, color="#888888", lw=0.8, alpha=0.5)
    if threshold_info.get("max_innov_h_accepted_m") is not None:
        ax0.axhline(
            threshold_info["max_innov_h_accepted_m"],
            color="#2ca02c",
            ls="--",
            lw=0.9,
            label="max innov_h aceptado",
        )
    if threshold_info.get("min_innov_h_rejected_m") is not None:
        ax0.axhline(
            threshold_info["min_innov_h_rejected_m"],
            color="#d62728",
            ls="--",
            lw=0.9,
            label="min innov_h rechazado",
        )
    ax0.set_ylabel("||x_pred - GPS||_h [m]")
    ax0.set_title("GAP-3.2 — Error horizontal predicho en cada fix GNSS")
    ax0.grid(True, alpha=0.3)
    ax0.legend(fontsize=8)

    ax1 = axes[1]
    ax1.scatter(t, nis, c=colors, s=28, zorder=3)
    ax1.axhline(NIS_THRESHOLD, color="k", ls=":", lw=0.9, label=f"NIS gate={NIS_THRESHOLD:.2f}")
    ax1.set_ylabel("NIS")
    ax1.set_xlabel("t [s]")
    ax1.set_title("NIS por fix (verde=aceptado, rojo=rechazado)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def plot_nis_decomposition(rows: list[GnssNisRow], out_png: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    t = [r.timestamp_s for r in rows]
    ax0 = axes[0]
    ax0.stackplot(
        t,
        [r.nis_contrib_n for r in rows],
        [r.nis_contrib_e for r in rows],
        [r.nis_contrib_d for r in rows],
        labels=["contrib N", "contrib E", "contrib D"],
        alpha=0.85,
    )
    ax0.plot(t, [r.nis_full for r in rows], "k-", lw=1.0, label="NIS total")
    ax0.axhline(NIS_THRESHOLD, color="k", ls=":", lw=0.8)
    ax0.set_title("GAP-3.1 — Contribución por eje a NIS")
    ax0.set_xlabel("t [s]")
    ax0.set_ylabel("NIS")
    ax0.legend(fontsize=8)
    ax0.grid(True, alpha=0.3)

    ax1 = axes[1]
    ax1.scatter(
        [r.innov_h for r in rows],
        [math.sqrt(r.s_nn) for r in rows],
        c=["#2ca02c" if r.accepted else "#d62728" for r in rows],
        s=24,
    )
    lim = max(max(r.innov_h for r in rows), max(math.sqrt(r.s_nn) for r in rows)) * 1.05
    ax1.plot([0, lim], [0, lim], "k--", lw=0.8, label="innov = sigma")
    ax1.set_xlabel("innov_h [m]")
    ax1.set_ylabel("sqrt(S_nn) [m]")
    ax1.set_title("Innovación vs incertidumbre posición")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def plot_velocity_coupling(rows: list[GnssNisRow], out_png: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    with_speed = [r for r in rows if r.has_gps_speed and r.gps_speed > 0.5]
    ax0 = axes[0]
    if with_speed:
        ax0.scatter(
            [r.gps_speed for r in with_speed],
            [r.vel_pred_h for r in with_speed],
            c=["#2ca02c" if r.accepted else "#d62728" for r in with_speed],
            s=24,
        )
        lim = max(max(r.gps_speed for r in with_speed), max(r.vel_pred_h for r in with_speed))
        ax0.plot([0, lim], [0, lim], "k--", lw=0.8)
    ax0.set_xlabel("GPS speed [m/s]")
    ax0.set_ylabel("||v_pred||_h [m/s]")
    ax0.set_title("GAP-3.3 — Velocidad EKF vs GPS")
    ax0.grid(True, alpha=0.3)

    accepted = [r for r in rows if r.accepted and r.dx_pos_h > 1.0e-4]
    ax1 = axes[1]
    if accepted:
        ax1.scatter(
            [r.dx_pos_h for r in accepted],
            [r.dx_vel_h for r in accepted],
            c="#1f77b4",
            s=28,
        )
    ax1.set_xlabel("|dx_pos| hipotético [m]")
    ax1.set_ylabel("|dx_vel| hipotético [m/s]")
    ax1.set_title("Acoplamiento pos→vel en aceptadas (K cross-cov)")
    ax1.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3 GNSS NIS anatomy audit")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-run", action="store_true")
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    run_replay(args.replay_exe, replay_csv, args.calibration, args.skip_run)

    if not NIS_CSV.is_file():
        print(f"Falta {NIS_CSV}", file=sys.stderr)
        return 1

    rows = [r for r in load_nis_csv(NIS_CSV) if r.timestamp_s <= AUDIT_END_S]
    threshold = find_reject_threshold(rows)
    anatomy = nis_anatomy_summary(rows)
    coupling = velocity_coupling_summary(rows)

    accept_frac = sum(1 for r in rows if r.accepted) / len(rows) if rows else 0.0
    report = {
        "audit_end_s": AUDIT_END_S,
        "gnss_fix_count": len(rows),
        "accept_frac": accept_frac,
        "gap3_1_nis_anatomy": anatomy,
        "gap3_2_predict_error_timeline": threshold,
        "gap3_3_velocity_coupling": coupling,
        "interpretation": {
            "innov_h_reference": "innov_h = ||z - h(x_pred)|| horizontal; error del estado EKF respecto al fix GPS en el instante del update",
            "why_nis_large": anatomy.get("diagnosis"),
            "gate_mechanism": (
                f"Rechazo sistemático tras t≈{threshold['first_reject_timestamp_s']:.1f}s "
                f"cuando innov_h>{threshold['max_innov_h_accepted_m']:.1f}m"
                if threshold.get("first_reject_timestamp_s") is not None
                and threshold.get("max_innov_h_accepted_m") is not None
                else "umbral no identificado"
            ),
            "velocity_coupling": (
                "GNSS observa solo posición; velocidad se corrige vía cross-cov en K. "
                "Si |v_pred|<<GPS speed, la siguiente predict re-escapa."
            ),
        },
        "verdict": "PREDICT_ERROR_DRIVES_NIS_REJECTION"
        if threshold.get("min_innov_h_rejected_m", 0.0) > threshold.get("max_innov_h_accepted_m", 999.0)
        else "MIXED_OR_COVARIANCE_DRIVEN",
    }

    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    plot_timeline(rows, threshold, TIMELINE_PNG)
    plot_nis_decomposition(rows, NIS_DECOMP_PNG)
    plot_velocity_coupling(rows, VEL_COUPLING_PNG)
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2))
    print(f"Wrote {NIS_CSV}")
    print(f"Wrote {REPORT_JSON}")
    print(f"Wrote {TIMELINE_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
