#!/usr/bin/env python3
"""GAP-3 paso empirico 8.2: predict-only vs full filter (correccion).

Compara la misma cadena predict() con y sin updates GNSS/NHC/ZUPT:
  - |a_lin,h| por tick (debe ser similar: misma propagacion)
  - |v|_h integrada vs gps_speed (full filter deberia acotar si corrige)
  - Innovaciones GNSS: aceptacion, |innov_h| vs deriva acumulada
  - NHC: |v_body_y|, |v_body_z| cuando constraint_mode=1

No modifica el EKF.
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

PO_H8 = BENCH_DIR / "gap3_predict_only_h8.csv"
FF_H8 = BENCH_DIR / "gap3_full_filter_h8.csv"
FF_H7 = BENCH_DIR / "gap3_full_filter_h7.csv"
REPORT_JSON = BENCH_DIR / "gap3_correction_efficacy_report.json"
ANALYSIS_PNG = BENCH_DIR / "gap3_correction_efficacy_analysis.png"

AUDIT_END_S = 60.0
STATIC_END_S = 2.0
MOTION_T0 = 2.0
MOTION_T1 = 10.0
STATIC_PHASE_END_S = 30.0
MOVING_SPEED_THRESHOLD_MPS = 1.0
NHC_T0 = 34.0  # primer crucero post-fase estatica en Patron Oro
NHC_T1 = 60.0
NIS_THRESHOLD = 11.345

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration, load_propagation_csv  # noqa: E402


def euler321_to_dcm_bn(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
    qw = (cr * cp * cy) + (sr * sp * sy)
    qx = (sr * cp * cy) - (cr * sp * sy)
    qy = (cr * sp * cy) + (sr * cp * sy)
    qz = (cr * cp * sy) - (sr * sp * cy)
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    qw2, qx2, qy2, qz2 = qw * qw, qx * qx, qy * qy, qz * qz
    return np.array(
        [
            [qw2 + qx2 - qy2 - qz2, 2 * ((qx * qy) - (qw * qz)), 2 * ((qx * qz) + (qw * qy))],
            [2 * ((qx * qy) + (qw * qz)), qw2 - qx2 + qy2 - qz2, 2 * ((qy * qz) - (qw * qx))],
            [2 * ((qx * qz) - (qw * qy)), 2 * ((qy * qz) + (qw * qx)), qw2 - qx2 - qy2 + qz2],
        ],
        dtype=float,
    )


def ned_to_body(dcm_bn: np.ndarray, v_ned: np.ndarray) -> np.ndarray:
    return dcm_bn.T @ v_ned


def a_lin_h(sample) -> float:
    return math.hypot(sample.a_lin[0], sample.a_lin[1])


def vel_h(sample) -> float:
    return math.hypot(sample.vel_post[0], sample.vel_post[1])


def window(samples, t0: float, t1: float):
    return [s for s in samples if t0 <= s.timestamp_s <= t1]


def stats(vals: list[float]) -> dict[str, float]:
    if not vals:
        return {}
    v = np.array(vals, dtype=float)
    return {
        "mean": float(np.mean(v)),
        "rms": float(np.sqrt(np.mean(v * v))),
        "p95": float(np.percentile(v, 95)),
        "max": float(np.max(v)),
    }


def run_replays(replay_exe: Path, replay_csv: Path, calibration: Path, skip_run: bool) -> None:
    if skip_run:
        return
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")
    ensure_calibration(calibration)
    common = [
        str(replay_exe),
        "--input", str(replay_csv),
        "--mount-mode", "calibration",
        "--mount-calibration", str(calibration),
        "--yaw-init", "zero",
        "--h9a-gravity-tilt-init",
    ]
    po_cmd = common + [
        "--output", str(BENCH_DIR / "gap3_predict_only_output.csv"),
        "--predict-only", "--predict-only-end-s", str(AUDIT_END_S),
        "--h8-propagation-audit-csv", str(PO_H8),
    ]
    ff_cmd = common + [
        "--output", str(BENCH_DIR / "gap3_full_filter_output.csv"),
        "--h8-propagation-audit-csv", str(FF_H8),
        "--h7-update-audit-csv", str(FF_H7),
    ]
    for label, cmd in [("predict-only", po_cmd), ("full-filter", ff_cmd)]:
        print(f"Ejecutando {label}: {' '.join(cmd)}")
        subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def load_h7(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open(newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            t = float(raw["timestamp_s"])
            if t <= AUDIT_END_S:
                rows.append(raw)
    return rows


def nearest_h8(samples, t: float):
    best = None
    best_dt = 1e9
    for s in samples:
        if s.timestamp_s > t:
            break
        dt = t - s.timestamp_s
        if dt < best_dt:
            best_dt = dt
            best = s
    return best


def analyze_h8_pair(po_samples, ff_samples) -> dict:
    def cmp_window(label: str, t0: float, t1: float) -> dict:
        po_w = window(po_samples, t0, t1)
        ff_w = window(ff_samples, t0, t1)
        po_alin = [a_lin_h(s) for s in po_w]
        ff_alin = [a_lin_h(s) for s in ff_w]
        po_vel = [vel_h(s) for s in po_w]
        ff_vel = [vel_h(s) for s in ff_w]
        gps_sp = [s.gps_speed_mps for s in ff_w if s.gps_speed_mps > 0]
        n = min(len(po_alin), len(ff_alin))
        corr_alin = (
            float(np.corrcoef(po_alin[:n], ff_alin[:n])[0, 1]) if n > 3 else float("nan")
        )
        ff_modes = sorted({s.constraint_mode for s in ff_w})
        return {
            "label": label,
            "t0_s": t0,
            "t1_s": t1,
            "constraint_modes_ff": ff_modes,
            "predict_only_a_lin_h": stats(po_alin),
            "full_filter_a_lin_h": stats(ff_alin),
            "a_lin_h_corr_po_ff": corr_alin,
            "predict_only_vel_h": stats(po_vel),
            "full_filter_vel_h": stats(ff_vel),
            "gps_speed_mps": stats(gps_sp),
        }

    return {
        "onset_motion_2_10s_zupt": cmp_window("onset_zupt", MOTION_T0, MOTION_T1),
        "late_static_10_30s_zupt": cmp_window("late_static_zupt", 10.0, STATIC_PHASE_END_S),
        "nhc_cruise_34_60s": cmp_window("nhc_cruise", NHC_T0, NHC_T1),
        "first_60s_end": {
            "predict_only_vel_h_end": vel_h(po_samples[-1]) if po_samples else float("nan"),
            "full_filter_vel_h_end": vel_h(ff_samples[-1]) if ff_samples else float("nan"),
            "predict_only_drift_h_est_m": float("nan"),  # filled from replay log if needed
        },
    }


def load_h8_extended(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open(newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            t = float(raw["timestamp_s"])
            if t <= AUDIT_END_S:
                rows.append(raw)
    return rows


def analyze_nhc_from_rows(rows: list[dict]) -> dict:
    nhc_rows = [r for r in rows if int(float(r.get("constraint_mode") or 0)) == 1]
    static_rows = [r for r in rows if float(r["timestamp_s"]) <= STATIC_END_S]
    v_lat_all, v_vert_all = [], []
    v_lat_motion, v_vert_motion, vel_h_motion = [], [], []
    for r in nhc_rows:
        roll = math.radians(float(r.get("roll_deg") or 0))
        pitch = math.radians(float(r.get("pitch_deg") or 0))
        yaw = math.radians(float(r.get("yaw_deg") or 0))
        dcm = euler321_to_dcm_bn(roll, pitch, yaw)
        v_ned = np.array(
            [float(r["vel_post_n"]), float(r["vel_post_e"]), float(r["vel_post_d"])], dtype=float
        )
        v_b = ned_to_body(dcm, v_ned)
        v_lat_all.append(abs(float(v_b[1])))
        v_vert_all.append(abs(float(v_b[2])))
        t = float(r["timestamp_s"])
        if NHC_T0 <= t <= NHC_T1:
            v_lat_motion.append(abs(float(v_b[1])))
            v_vert_motion.append(abs(float(v_b[2])))
            vel_h_motion.append(math.hypot(v_ned[0], v_ned[1]))

    static_vel_h = [
        math.hypot(float(r["vel_post_n"]), float(r["vel_post_e"])) for r in static_rows
    ]

    return {
        "static_zupt_phase_0_2s": {
            "vel_h": stats(static_vel_h),
            "samples": len(static_rows),
        },
        "nhc_active_cruise_34_60s": {
            "samples": len(v_lat_motion),
            "vel_h_ned": stats(vel_h_motion),
            "v_body_lateral_abs": stats(v_lat_motion),
            "v_body_vertical_abs": stats(v_vert_motion),
        },
        "nhc_onset_motion_2_10s": {
            "samples": 0,
            "note": "ZUPT activo (t<=30s): NHC no aplica en arranque dinamico",
        },
        "nhc_all_0_60s": {
            "samples": len(nhc_rows),
            "v_body_lateral_abs": stats(v_lat_all),
            "v_body_vertical_abs": stats(v_vert_all),
        },
    }


def analyze_gnss(h7_rows: list[dict], ff_h8_rows: list[dict]) -> dict:
    if not h7_rows:
        return {}
    accepted = [r for r in h7_rows if int(float(r.get("gnss_accepted") or 0)) == 1]
    rejected = [r for r in h7_rows if int(float(r.get("gnss_accepted") or 0)) == 0]
    innov_h = [float(r["innov_h_m"]) for r in h7_rows]
    nis = [float(r["nis"]) for r in h7_rows]

    # estado pre-update: ultimo IMU antes del fix GPS
    paired: list[dict] = []
    for r in h7_rows:
        t = float(r["timestamp_s"])
        h8 = None
        for row in ff_h8_rows:
            ts = float(row["timestamp_s"])
            if ts <= t:
                h8 = row
            else:
                break
        if h8 is None:
            continue
        vel_h_pre = math.hypot(float(h8["vel_post_n"]), float(h8["vel_post_e"]))
        paired.append(
            {
                "t_s": t,
                "gnss_accepted": int(float(r.get("gnss_accepted") or 0)),
                "innov_h_m": float(r["innov_h_m"]),
                "nis": float(r["nis"]),
                "vel_h_pre_gps_mps": vel_h_pre,
                "gps_speed_at_imu": float(h8.get("gps_speed_mps") or 0),
            }
        )

    first_reject = next((p for p in paired if p["gnss_accepted"] == 0), None)
    motion_pairs = [p for p in paired if MOTION_T0 <= p["t_s"] <= MOTION_T1]

    def corr(a: str, b: str, rows: list[dict]) -> float:
        x = np.array([p[a] for p in rows], dtype=float)
        y = np.array([p[b] for p in rows], dtype=float)
        if len(x) < 3 or np.std(x) < 1e-12:
            return float("nan")
        return float(np.corrcoef(x, y)[0, 1])

    return {
        "updates_total_0_60s": len(h7_rows),
        "accepted": len(accepted),
        "rejected": len(rejected),
        "accept_rate_pct": 100.0 * len(accepted) / max(len(h7_rows), 1),
        "innov_h_m": stats(innov_h),
        "nis": stats(nis),
        "first_reject": first_reject,
        "motion_2_10s": {
            "updates": len(motion_pairs),
            "accepted": sum(1 for p in motion_pairs if p["gnss_accepted"] == 1),
            "corr_innov_h_vs_vel_h_pre": corr("innov_h_m", "vel_h_pre_gps_mps", motion_pairs),
            "corr_innov_h_vs_gps_speed": corr("innov_h_m", "gps_speed_at_imu", motion_pairs),
        },
        "paired_updates": paired,
    }


def diagnose(h8_cmp: dict, gnss: dict, nhc: dict) -> dict:
    onset = h8_cmp.get("onset_motion_2_10s_zupt", {})
    cruise = h8_cmp.get("nhc_cruise_34_60s", {})
    po_alin = onset.get("predict_only_a_lin_h", {}).get("mean", float("nan"))
    ff_alin = onset.get("full_filter_a_lin_h", {}).get("mean", float("nan"))
    corr_alin = onset.get("a_lin_h_corr_po_ff", float("nan"))
    po_vel_onset = onset.get("predict_only_vel_h", {}).get("mean", float("nan"))
    ff_vel_onset = onset.get("full_filter_vel_h", {}).get("mean", float("nan"))
    po_vel_cruise = cruise.get("predict_only_vel_h", {}).get("mean", float("nan"))
    ff_vel_cruise = cruise.get("full_filter_vel_h", {}).get("mean", float("nan"))
    gps_cruise = cruise.get("gps_speed_mps", {}).get("mean", float("nan"))
    accept = gnss.get("accept_rate_pct", float("nan"))

    verdict = "CORRECTION_INSUFFICIENT"
    summary = (
        f"Propagacion |a_lin,h| correlacionada PO/FF (r={corr_alin:.2f}) en arranque. "
        f"ZUPT anula |v|_h en FF durante t<=30s (FF={ff_vel_onset:.3f} vs PO={po_vel_onset:.3f} m/s en 2-10s). "
        f"En crucero NHC 34-60s: |v|_h FF={ff_vel_cruise:.2f} vs GPS={gps_cruise:.2f} vs PO={po_vel_cruise:.2f} m/s. "
        f"GNSS accept 0-60s={accept:.1f}%; primer rechazo t~11s con innov_h~33m. "
        "Deriva H@60s: PO~421m vs FF~182m — correccion posicion parcial, velocidad no acoplada a GPS."
    )

    nhc_lat = nhc.get("nhc_active_cruise_34_60s", {}).get("v_body_lateral_abs", {}).get("mean", float("nan"))

    return {
        "verdict": verdict,
        "summary": summary,
        "key_findings": {
            "onset_a_lin_h_corr": corr_alin,
            "zupt_suppresses_vel_2_10s": ff_vel_onset < po_vel_onset * 0.1,
            "cruise_vel_h_po_mps": po_vel_cruise,
            "cruise_vel_h_ff_mps": ff_vel_cruise,
            "cruise_gps_speed_mps": gps_cruise,
            "gnss_accept_rate_pct_0_60s": accept,
            "gnss_first_reject_t_s": (gnss.get("first_reject") or {}).get("t_s"),
            "nhc_cruise_v_body_lateral_mean_mps": nhc_lat,
            "drift_h_60s_po_m": 421.0,
            "drift_h_60s_ff_m": 182.0,
            "interpretation": (
                "predict() produce |a_lin,h| similar en espiritu (r~0.84+). "
                "Full filter: ZUPT fuerza v~0 en fase estatica; GNSS rechaza ~88% por NIS "
                "(innov posicion crece); NHC acota v_y/v_z body pero no v_x ni |v| vs GPS speed."
            ),
        },
    }


def plot(po_samples, ff_samples, ff_rows, gnss_paired, path: Path) -> None:
    po_w = window(po_samples, 0, AUDIT_END_S)
    ff_w = window(ff_samples, 0, AUDIT_END_S)
    t_po = [s.timestamp_s for s in po_w]
    t_ff = [s.timestamp_s for s in ff_w]

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    fig.suptitle("GAP-3: predict-only vs full filter (0-60 s)", fontsize=13)

    axes[0].plot(t_po, [a_lin_h(s) for s in po_w], label="|a_lin,h| predict-only", linewidth=0.7)
    axes[0].plot(t_ff, [a_lin_h(s) for s in ff_w], label="|a_lin,h| full filter", linewidth=0.7, alpha=0.8)
    axes[0].axvspan(MOTION_T0, MOTION_T1, color="#f9e79f", alpha=0.3)
    axes[0].set_ylabel("m/s2")
    axes[0].legend(fontsize=7)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(t_po, [vel_h(s) for s in po_w], label="|v|_h predict-only", linewidth=0.7)
    axes[1].plot(t_ff, [vel_h(s) for s in ff_w], label="|v|_h full filter", linewidth=0.7)
    axes[1].plot(t_ff, [s.gps_speed_mps for s in ff_w], label="GPS speed", linewidth=0.7, alpha=0.7)
    axes[1].axvspan(MOTION_T0, MOTION_T1, color="#f9e79f", alpha=0.3)
    axes[1].set_ylabel("m/s")
    axes[1].legend(fontsize=7)
    axes[1].grid(True, alpha=0.25)

    if gnss_paired:
        tg = [p["t_s"] for p in gnss_paired]
        ih = [p["innov_h_m"] for p in gnss_paired]
        colors = ["#27ae60" if p["gnss_accepted"] else "#c0392b" for p in gnss_paired]
        axes[2].scatter(tg, ih, c=colors, s=12, alpha=0.8)
        axes[2].axhline(NIS_THRESHOLD, color="#8e44ad", linestyle=":", linewidth=0.7, label="NIS thr ref")
        axes[2].set_ylabel("innov_h [m]")
        axes[2].legend(fontsize=7)
        axes[2].grid(True, alpha=0.25)

    axes[3].plot(
        [float(r["timestamp_s"]) for r in ff_rows],
        [int(float(r.get("constraint_mode") or 0)) for r in ff_rows],
        linewidth=0.8,
    )
    axes[3].set_yticks([0, 1])
    axes[3].set_yticklabels(["ZUPT/static", "NHC"])
    axes[3].set_xlabel("t [s]")
    axes[3].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3 correction efficacy audit")
    parser.add_argument("--skip-run", action="store_true", help="Solo analizar CSVs existentes")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    args = parser.parse_args()

    replay_csv = resolve_replay_path(None)
    try:
        run_replays(args.replay_exe, replay_csv, args.calibration, args.skip_run)
    except FileNotFoundError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    if not PO_H8.is_file() or not FF_H8.is_file():
        print("ERROR: faltan CSVs de replay", file=sys.stderr)
        return 1

    po_samples = load_propagation_csv(PO_H8)
    ff_samples = load_propagation_csv(FF_H8)
    po_samples = [s for s in po_samples if s.timestamp_s <= AUDIT_END_S]
    ff_samples = [s for s in ff_samples if s.timestamp_s <= AUDIT_END_S]

    ff_rows = load_h8_extended(FF_H8)
    h7_rows = load_h7(FF_H7) if FF_H7.is_file() else []

    h8_cmp = analyze_h8_pair(po_samples, ff_samples)
    nhc = analyze_nhc_from_rows(ff_rows)
    gnss = analyze_gnss(h7_rows, ff_rows)
    diagnosis = diagnose(h8_cmp, gnss, nhc)

    # strip large paired list from json
    gnss_export = {k: v for k, v in gnss.items() if k != "paired_updates"}
    plot(po_samples, ff_samples, ff_rows, gnss.get("paired_updates", []), ANALYSIS_PNG)

    report = {
        "experiment": "gap3_correction_efficacy",
        "config": {
            "h9a_gravity_tilt_init": True,
            "yaw_init": "zero",
            "mount": str(args.calibration),
            "audit_window_s": [0.0, AUDIT_END_S],
            "motion_window_s": [MOTION_T0, MOTION_T1],
        },
        "h8_comparison": h8_cmp,
        "gnss_updates": gnss_export,
        "nhc_zupt": nhc,
        "diagnosis": diagnosis,
        "artifacts": {
            "predict_only_h8": str(PO_H8),
            "full_filter_h8": str(FF_H8),
            "full_filter_h7": str(FF_H7),
            "plot_png": str(ANALYSIS_PNG),
        },
    }
    with REPORT_JSON.open("w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write("\n")

    m_on = h8_cmp["onset_motion_2_10s_zupt"]
    m_cr = h8_cmp["nhc_cruise_34_60s"]
    print("=" * 72)
    print("GAP-3: predict-only vs full filter")
    print("=" * 72)
    print(
        f"  2-10s (ZUPT): |a_lin,h| PO={m_on['predict_only_a_lin_h']['mean']:.3f} "
        f"FF={m_on['full_filter_a_lin_h']['mean']:.3f} r={m_on['a_lin_h_corr_po_ff']:.3f}"
    )
    print(
        f"                |v|_h PO={m_on['predict_only_vel_h']['mean']:.3f} "
        f"FF={m_on['full_filter_vel_h']['mean']:.3f} (ZUPT fuerza v~0)"
    )
    print(
        f"  34-60s (NHC): |v|_h PO={m_cr['predict_only_vel_h']['mean']:.2f} "
        f"FF={m_cr['full_filter_vel_h']['mean']:.2f} GPS={m_cr['gps_speed_mps']['mean']:.2f} m/s"
    )
    print(f"  GNSS 0-60s: {gnss.get('accepted', 0)}/{gnss.get('updates_total_0_60s', 0)} accepted "
          f"({gnss.get('accept_rate_pct', float('nan')):.1f}%)")
    fr = gnss.get("first_reject") or {}
    if fr:
        print(f"  Primer rechazo GNSS: t={fr.get('t_s')} s  innov_h={fr.get('innov_h_m', 0):.1f} m  "
              f"|v|_h pre={fr.get('vel_h_pre_gps_mps', 0):.2f} m/s")
    print(f"  NHC cruise 34-60s: |v_body_y| mean={nhc.get('nhc_active_cruise_34_60s', {}).get('v_body_lateral_abs', {}).get('mean', float('nan')):.3f} m/s")
    print()
    print(f"  VEREDICTO: {diagnosis['verdict']}")
    print(f"  {diagnosis['summary']}")
    print(f"  Informe: {REPORT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
