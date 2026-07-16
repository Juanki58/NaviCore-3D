#!/usr/bin/env python3
"""H7b — Unificar origen NED del EKF con el del parse (1er fix Location.csv).

Ejecuta replay con --gnss-ref-lat/lon/alt y compara update audit vs baseline H7 (Barcelona).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"
DEFAULT_LOCATION = REPO_ROOT / "data" / "real_run" / "Location.csv"

BENCH_DIR = REPO_ROOT / "docs" / "benchmarks"
UPDATE_AUDIT_CSV = BENCH_DIR / "h7b_update_audit.csv"
REPORT_JSON = BENCH_DIR / "h7b_unified_origin_report.json"
ANALYSIS_PNG = BENCH_DIR / "h7b_unified_origin_analysis.png"
H7_BASELINE_REPORT = BENCH_DIR / "h7_gnss_chain_report.json"

ABSURD_INNOV_H_M = 20.0

from parse_mobile_log import load_location_csv  # noqa: E402
from analyze_real_run import resolve_replay_path  # noqa: E402
from run_h7_gnss_chain_audit import (  # noqa: E402
    EKF_REF_ALT_M,
    EKF_REF_LAT_DEG,
    EKF_REF_LON_DEG,
    find_first_absurd_update,
    load_update_audit,
    plot_analysis,
)


def ensure_calibration(path: Path) -> None:
    if path.is_file():
        return
    subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "audit_imu_chain.py"),
            "--export-calibration",
            str(path),
        ],
        cwd=REPO_ROOT,
        check=True,
    )


def run_h7b_replay(
    replay_csv: Path,
    replay_exe: Path,
    calibration: Path,
    ref_lat: float,
    ref_lon: float,
    ref_alt: float,
) -> None:
    if not replay_exe.is_file():
        raise FileNotFoundError(f"No existe {replay_exe}")

    cmd = [
        str(replay_exe),
        "--input",
        str(replay_csv),
        "--output",
        str(BENCH_DIR / "h7b_replay_output.csv"),
        "--mount-mode",
        "calibration",
        "--mount-calibration",
        str(calibration),
        "--yaw-init",
        "zero",
        "--gnss-ref-lat",
        f"{ref_lat:.7f}",
        "--gnss-ref-lon",
        f"{ref_lon:.7f}",
        "--gnss-ref-alt",
        f"{ref_alt:.3f}",
        "--h7-update-audit-csv",
        str(UPDATE_AUDIT_CSV),
    ]
    print(f"Ejecutando: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def summarize_update(rows) -> dict:
    if not rows:
        return {}
    accepted = sum(1 for r in rows if r.gnss_accepted)
    total = len(rows)
    innov_h = [r.innov_h_m for r in rows]
    first_absurd = find_first_absurd_update(rows)
    return {
        "samples": total,
        "gnss_accept_count": accepted,
        "gnss_reject_count": total - accepted,
        "gnss_accept_pct": 100.0 * accepted / total if total else 0.0,
        "innov_h_mean_m": sum(innov_h) / len(innov_h),
        "innov_h_max_m": max(innov_h),
        "first_absurd": None
        if first_absurd is None
        else {
            "timestamp_s": first_absurd.timestamp_s,
            "gps_index": first_absurd.gps_index,
            "innov_h_m": first_absurd.innov_h_m,
            "innov_n_m": first_absurd.innov_n,
            "innov_e_m": first_absurd.innov_e,
            "nis": first_absurd.nis,
            "gnss_accepted": first_absurd.gnss_accepted,
        },
    }


def load_baseline_first_absurd() -> dict | None:
    if not H7_BASELINE_REPORT.is_file():
        return None
    with H7_BASELINE_REPORT.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    return payload.get("update_audit", {}).get("first_absurd")


def main() -> int:
    parser = argparse.ArgumentParser(description="H7b unified NED origin experiment")
    parser.add_argument("--skip-replay", action="store_true")
    parser.add_argument("--location", type=Path, default=DEFAULT_LOCATION)
    args = parser.parse_args()

    try:
        replay_csv = resolve_replay_path(None)
        ensure_calibration(DEFAULT_CALIBRATION)
        BENCH_DIR.mkdir(parents=True, exist_ok=True)

        locations = load_location_csv(args.location)
        if not locations:
            raise ValueError("Location.csv sin fixes validos")
        parse_ref = (locations[0].latitude, locations[0].longitude, locations[0].altitude)

        if not args.skip_replay:
            run_h7b_replay(
                replay_csv,
                DEFAULT_REPLAY_EXE,
                DEFAULT_CALIBRATION,
                parse_ref[0],
                parse_ref[1],
                parse_ref[2],
            )

        if not UPDATE_AUDIT_CSV.is_file():
            raise FileNotFoundError(f"Falta {UPDATE_AUDIT_CSV}")

        update_rows = load_update_audit(UPDATE_AUDIT_CSV)
        if not update_rows:
            raise ValueError("Update audit vacio")

        h7b_stats = summarize_update(update_rows)
        baseline_absurd = load_baseline_first_absurd()
        first_absurd = find_first_absurd_update(update_rows)

        plot_analysis([], update_rows, first_absurd, ANALYSIS_PNG)

        payload = {
            "experiment": "H7b_unified_origin",
            "parse_origin_deg": {
                "lat": parse_ref[0],
                "lon": parse_ref[1],
                "alt_m": parse_ref[2],
            },
            "ekf_seed_ref_deg": {
                "lat": parse_ref[0],
                "lon": parse_ref[1],
                "alt_m": parse_ref[2],
            },
            "baseline_h7_ekf_seed_ref_deg": {
                "lat": EKF_REF_LAT_DEG,
                "lon": EKF_REF_LON_DEG,
                "alt_m": EKF_REF_ALT_M,
            },
            "update_audit": h7b_stats,
            "comparison_vs_h7_baseline": {
                "baseline_first_absurd": baseline_absurd,
                "h7b_first_absurd": h7b_stats.get("first_absurd"),
                "first_absurd_delayed_s": None,
                "first_absurd_eliminated": first_absurd is None,
            },
        }

        if baseline_absurd and h7b_stats.get("first_absurd"):
            payload["comparison_vs_h7_baseline"]["first_absurd_delayed_s"] = (
                h7b_stats["first_absurd"]["timestamp_s"] - baseline_absurd["timestamp_s"]
            )
        elif baseline_absurd and first_absurd is None:
            payload["comparison_vs_h7_baseline"]["first_absurd_eliminated"] = True

        with REPORT_JSON.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
            handle.write("\n")

        print("=" * 78)
        print(" H7b - ORIGEN NED UNIFICADO (parse == EKF seed)")
        print("=" * 78)
        print(
            f"  Origen unificado: lat={parse_ref[0]:.7f} lon={parse_ref[1]:.7f} alt={parse_ref[2]:.1f} m"
        )
        print("-" * 78)
        print(
            f"  GNSS accept: {h7b_stats['gnss_accept_count']}/{h7b_stats['samples']} "
            f"({h7b_stats['gnss_accept_pct']:.1f}%)"
        )
        print(
            f"  innov_h: mean={h7b_stats['innov_h_mean_m']:.2f} m  "
            f"max={h7b_stats['innov_h_max_m']:.2f} m"
        )
        if first_absurd is not None:
            print(
                f"  1er innov absurda (>{ABSURD_INNOV_H_M:.0f} m): t={first_absurd.timestamp_s:.2f} s  "
                f"idx={first_absurd.gps_index}  innov_h={first_absurd.innov_h_m:.1f} m  "
                f"accepted={first_absurd.gnss_accepted}"
            )
        else:
            print(f"  1er innov absurda (>{ABSURD_INNOV_H_M:.0f} m): ninguna")
        print("-" * 78)
        if baseline_absurd:
            print("  Comparacion vs H7 baseline (Barcelona seed):")
            print(
                f"    baseline: t={baseline_absurd['timestamp_s']:.2f} s  "
                f"idx={baseline_absurd['gps_index']}  innov_h={baseline_absurd['innov_h_m']:.1f} m"
            )
            if first_absurd is None:
                print("    H7b: primera innovacion absurda eliminada")
            elif h7b_stats.get("first_absurd"):
                delay = h7b_stats["first_absurd"]["timestamp_s"] - baseline_absurd["timestamp_s"]
                print(
                    f"    H7b: t={h7b_stats['first_absurd']['timestamp_s']:.2f} s  "
                    f"idx={h7b_stats['first_absurd']['gps_index']}  "
                    f"innov_h={h7b_stats['first_absurd']['innov_h_m']:.1f} m  "
                    f"delta_t={delay:+.1f} s"
                )
        else:
            print("  (Sin baseline H7 en h7_gnss_chain_report.json)")
        print("-" * 78)
        print(f"  Reporte: {REPORT_JSON}")
        print(f"  Grafico: {ANALYSIS_PNG}")
        print("=" * 78)
        return 0
    except (FileNotFoundError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
