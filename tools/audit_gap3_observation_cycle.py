#!/usr/bin/env python3
"""GAP-3 paso empirico 8.3: auditoria de observacion por ciclo.

Instrumenta cada update GNSS / NHC / ZUPT y mide:
  - deriva acumulada desde la ultima correccion (pred_accum)
  - correccion aplicada (corr_*)
  - ratio pred/corr por ciclo

Clasifica mecanismos A/B/C/D:
  A  pocas observaciones
  B  llegan pero se rechazan
  C  se aceptan pero corrigen poco
  D  corrigen pero la deriva vuelve rapido
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

OBS_CSV = BENCH_DIR / "gap3_observation_cycle.csv"
PO_OUTPUT = BENCH_DIR / "gap3_obs_predict_only_output.csv"
FF_OUTPUT = BENCH_DIR / "gap3_obs_full_filter_output.csv"
REPORT_JSON = BENCH_DIR / "gap3_observation_cycle_report.json"
ANALYSIS_PNG = BENCH_DIR / "gap3_observation_cycle_analysis.png"

AUDIT_END_S = 60.0
CORR_EPS_M = 0.05
CORR_EPS_V = 0.02
RATIO_HIGH = 5.0

sys.path.insert(0, str(REPO_ROOT))
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h8_propagation_audit import ensure_calibration  # noqa: E402


@dataclass
class ObsRow:
    timestamp_s: float
    update_type: str
    accepted: bool
    reject_reason: int
    pred_accum_dpos_h: float
    pred_accum_dvel_h: float
    pred_accum_dt_s: float
    innov_h_m: float
    nis: float
    corr_pos_h_m: float
    corr_vel_h_mps: float
    hypo_corr_pos_h_m: float
    hypo_corr_vel_h_mps: float
    pred_over_corr_dpos_ratio: float
    pred_over_corr_dvel_ratio: float
    state_vel_h_mps: float


def load_observation_csv(path: Path) -> list[ObsRow]:
    rows: list[ObsRow] = []
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        for raw in reader:
            rows.append(
                ObsRow(
                    timestamp_s=float(raw["timestamp_s"]),
                    update_type=raw["update_type"],
                    accepted=raw["accepted"] == "1",
                    reject_reason=int(raw["reject_reason"]),
                    pred_accum_dpos_h=float(raw["pred_accum_dpos_h_m"]),
                    pred_accum_dvel_h=float(raw["pred_accum_dvel_h_mps"]),
                    pred_accum_dt_s=float(raw["pred_accum_dt_s"]),
                    innov_h_m=float(raw["innov_h_m"]),
                    nis=float(raw["nis"]),
                    corr_pos_h_m=float(raw["corr_pos_h_m"]),
                    corr_vel_h_mps=float(raw["corr_vel_h_mps"]),
                    hypo_corr_pos_h_m=float(raw["hypo_corr_pos_h_m"]),
                    hypo_corr_vel_h_mps=float(raw["hypo_corr_vel_h_mps"]),
                    pred_over_corr_dpos_ratio=float(raw["pred_over_corr_dpos_ratio"]),
                    pred_over_corr_dvel_ratio=float(raw["pred_over_corr_dvel_ratio"]),
                    state_vel_h_mps=float(raw["state_vel_h_mps"]),
                )
            )
    return rows


def run_replays(replay_exe: Path, replay_csv: Path, calibration: Path, skip_run: bool) -> None:
    if skip_run:
        return
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")
    ensure_calibration(calibration)
    common = [
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
    ]
    po_cmd = common + [
        "--output",
        str(PO_OUTPUT),
        "--predict-only",
        "--predict-only-end-s",
        str(AUDIT_END_S),
    ]
    ff_cmd = common + [
        "--output",
        str(FF_OUTPUT),
        "--gap3-observation-audit-csv",
        str(OBS_CSV),
    ]
    for cmd in (po_cmd, ff_cmd):
        print("RUN:", " ".join(cmd))
        subprocess.run(cmd, check=True, cwd=REPO_ROOT)


def load_last_gps_before(replay_csv: Path, end_s: float) -> tuple[float, float] | None:
    last = None
    with replay_csv.open(newline="", encoding="utf-8-sig") as fp:
        reader = csv.DictReader(fp)
        for raw in reader:
            if raw.get("type") != "GPS":
                continue
            t = float(raw["timestamp_s"])
            if t > end_s:
                break
            if raw.get("pos_n") and raw.get("pos_e"):
                last = (float(raw["pos_n"]), float(raw["pos_e"]))
    return last


def horizontal_drift_from_output(output_csv: Path, replay_csv: Path, end_s: float) -> float | None:
    if not output_csv.is_file():
        return None
    ref_gps = load_last_gps_before(replay_csv, end_s)
    if ref_gps is None:
        return None
    last_imu_state = None
    with output_csv.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        for raw in reader:
            t = float(raw["timestamp_s"])
            if t > end_s:
                break
            if raw.get("row_type") == "IMU":
                last_imu_state = (float(raw["pos_n_m"]), float(raw["pos_e_m"]))
    if last_imu_state is None:
        return None
    dn = last_imu_state[0] - ref_gps[0]
    de = last_imu_state[1] - ref_gps[1]
    return math.hypot(dn, de)


def classify_mechanism(rows: list[ObsRow]) -> str:
    if not rows:
        return "A_FEW_OBSERVATIONS"
    gnss = [r for r in rows if r.update_type == "GNSS"]
    if len(gnss) < 3:
        return "A_FEW_OBSERVATIONS"
    reject_frac = sum(1 for r in gnss if not r.accepted) / len(gnss)
    if reject_frac > 0.5:
        return "B_REJECTED"
    accepted = [r for r in gnss if r.accepted]
    if not accepted:
        return "B_REJECTED"
    mean_corr = float(np.mean([r.corr_pos_h_m for r in accepted]))
    mean_hypo = float(np.mean([r.hypo_corr_pos_h_m for r in accepted]))
    if mean_corr < CORR_EPS_M and mean_hypo < CORR_EPS_M:
        return "C_WEAK_CORRECTION"
    ratios = [
        r.pred_over_corr_dpos_ratio
        for r in accepted
        if r.pred_over_corr_dpos_ratio >= 0.0 and r.corr_pos_h_m > CORR_EPS_M
    ]
    if ratios and float(np.median(ratios)) > RATIO_HIGH:
        return "D_REDRIFT_AFTER_CORRECTION"
    return "MIXED"


def summarize_type(rows: list[ObsRow], update_type: str) -> dict:
    subset = [r for r in rows if r.update_type == update_type]
    if not subset:
        return {"count": 0}
    accepted = [r for r in subset if r.accepted]
    ratios = [
        r.pred_over_corr_dpos_ratio
        for r in accepted
        if r.pred_over_corr_dpos_ratio >= 0.0 and r.corr_pos_h_m > CORR_EPS_M
    ]
    hypo_ratios = [
        r.pred_accum_dpos_h / r.hypo_corr_pos_h_m
        for r in subset
        if r.hypo_corr_pos_h_m > CORR_EPS_M
    ]
    return {
        "count": len(subset),
        "accept_count": len(accepted),
        "accept_frac": len(accepted) / len(subset),
        "mean_pred_dpos_h": float(np.mean([r.pred_accum_dpos_h for r in subset])),
        "mean_corr_pos_h": float(np.mean([r.corr_pos_h_m for r in accepted])) if accepted else 0.0,
        "mean_hypo_corr_pos_h": float(np.mean([r.hypo_corr_pos_h_m for r in subset])),
        "median_pred_over_corr_dpos": float(np.median(ratios)) if ratios else None,
        "median_pred_over_hypo_corr_dpos": float(np.median(hypo_ratios)) if hypo_ratios else None,
        "mean_innov_h": float(np.mean([r.innov_h_m for r in subset])),
        "mean_nis": float(np.mean([r.nis for r in subset if r.nis > 0.0])) if any(r.nis > 0.0 for r in subset) else None,
    }


def plot_analysis(rows: list[ObsRow], po_drift: float | None, ff_drift: float | None, out_png: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    ax0, ax1, ax2, ax3 = axes.ravel()

    gnss = [r for r in rows if r.update_type == "GNSS"]
    if gnss:
        t = [r.timestamp_s for r in gnss]
        ax0.scatter(t, [r.innov_h_m for r in gnss], c=["g" if r.accepted else "r" for r in gnss], s=18)
        ax0.set_title("GNSS innovacion horizontal")
        ax0.set_xlabel("t [s]")
        ax0.set_ylabel("|innov_h| [m]")
        ax0.grid(True, alpha=0.3)

        ax1.plot(t, [r.pred_accum_dpos_h for r in gnss], "C0-o", label="pred drift", ms=3)
        ax1.plot(t, [r.corr_pos_h_m for r in gnss], "C1-s", label="corr aplicada", ms=3)
        ax1.plot(t, [r.hypo_corr_pos_h_m for r in gnss], "C2--^", label="corr hipotetica", ms=3)
        ax1.set_title("GNSS: deriva vs correccion")
        ax1.set_xlabel("t [s]")
        ax1.set_ylabel("[m]")
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.3)

        ratios = [
            (r.timestamp_s, r.pred_over_corr_dpos_ratio)
            for r in gnss
            if r.accepted and r.pred_over_corr_dpos_ratio >= 0.0
        ]
        if ratios:
            ax2.scatter([x[0] for x in ratios], [x[1] for x in ratios], c="C3", s=20)
        ax2.axhline(RATIO_HIGH, color="k", ls="--", lw=0.8)
        ax2.set_title("Ratio pred/corr (GNSS aceptadas)")
        ax2.set_xlabel("t [s]")
        ax2.set_ylabel("||dx_pred|| / ||dx_update||")
        ax2.grid(True, alpha=0.3)

    nhc = [r for r in rows if r.update_type == "NHC"]
    if nhc:
        ax3.plot(
            [r.timestamp_s for r in nhc],
            [r.corr_vel_h_mps for r in nhc],
            ".-",
            label="|corr_vel_h|",
            ms=2,
        )
        ax3.plot(
            [r.timestamp_s for r in nhc],
            [r.pred_accum_dvel_h for r in nhc],
            ".-",
            label="pred_dvel_h",
            ms=2,
        )
        ax3.set_title("NHC: deriva velocidad vs correccion")
        ax3.set_xlabel("t [s]")
        ax3.set_ylabel("[m/s]")
        ax3.legend(fontsize=8)
        ax3.grid(True, alpha=0.3)
    else:
        ax3.text(0.5, 0.5, "Sin updates NHC", ha="center", va="center")
        ax3.set_axis_off()

    if po_drift is not None and ff_drift is not None:
        fig.suptitle(
            f"GAP-3 observacion @ {AUDIT_END_S:.0f}s | deriva PO={po_drift:.0f}m FF={ff_drift:.0f}m",
            fontsize=11,
        )
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="GAP-3 observation cycle audit")
    parser.add_argument("--replay-exe", type=Path, default=DEFAULT_REPLAY_EXE)
    parser.add_argument("--replay-csv", type=Path, default=None)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--skip-run", action="store_true")
    args = parser.parse_args()

    replay_csv = args.replay_csv or resolve_replay_path(None)
    run_replays(args.replay_exe, replay_csv, args.calibration, args.skip_run)

    if not OBS_CSV.is_file():
        print(f"Falta {OBS_CSV}", file=sys.stderr)
        return 1

    rows = [r for r in load_observation_csv(OBS_CSV) if r.timestamp_s <= AUDIT_END_S]
    mechanism = classify_mechanism(rows)
    po_drift = horizontal_drift_from_output(PO_OUTPUT, replay_csv, AUDIT_END_S)
    ff_drift = horizontal_drift_from_output(FF_OUTPUT, replay_csv, AUDIT_END_S)

    report = {
        "audit_end_s": AUDIT_END_S,
        "observation_rows": len(rows),
        "mechanism_class": mechanism,
        "drift_predict_only_m": po_drift,
        "drift_full_filter_m": ff_drift,
        "drift_ratio_po_over_ff": (po_drift / ff_drift) if po_drift and ff_drift and ff_drift > 0 else None,
        "gnss": summarize_type(rows, "GNSS"),
        "nhc": summarize_type(rows, "NHC"),
        "zupt": summarize_type(rows, "ZUPT"),
        "verdict": (
            "CORRECTION_INSUFFICIENT"
            if mechanism in {
                "B_REJECTED",
                "C_WEAK_CORRECTION",
                "D_REDRIFT_AFTER_CORRECTION",
                "MIXED",
            }
            and (po_drift is None or ff_drift is None or ff_drift > 0.3 * po_drift)
            else "CORRECTION_EFFECTIVE_OR_INCONCLUSIVE"
        ),
    }

    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    plot_analysis(rows, po_drift, ff_drift, ANALYSIS_PNG)
    REPORT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2))
    print(f"Wrote {OBS_CSV}")
    print(f"Wrote {REPORT_JSON}")
    print(f"Wrote {ANALYSIS_PNG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
