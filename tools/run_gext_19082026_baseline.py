#!/usr/bin/env python3
"""G-ext — Validación externa run 19082026 (fases A → B → C).

Experimento independiente del brazo GAP-4 G1. Misma shell EKF/config G1;
única variable = recorrido. Artefactos solo en
docs/benchmarks/real_run_19082026_baseline/.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = REPO_ROOT / "docs" / "benchmarks" / "real_run_19082026_baseline"
RAW_DIR = REPO_ROOT / "data" / "real_run" / "19082026"
SOURCE_REPLAY = REPO_ROOT / "docs" / "benchmarks" / "real_run_19082026_replay.csv"
G1_DIR = REPO_ROOT / "docs" / "benchmarks" / "gap4_gnss_velocity" / "G1"
G1_REPORT = G1_DIR / "gap4_g1_report.json"
REPLAY_EXE = REPO_ROOT / "build" / "NaviCore3D_Replay.exe"
CALIBRATION = REPO_ROOT / "calibration" / "imu_mount.json"

sys.path.insert(0, str(REPO_ROOT / "tools"))
from run_gap3_f1_nhc_dose_response import analyze_gap, fix_timestamps  # noqa: E402
from run_gap4_arm import analyze, load_csv, run_replay  # noqa: E402


def _write_json(path: Path, obj: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, allow_nan=False), encoding="utf-8")


def _safe(v: float) -> float | None:
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return None
    return float(v)


# ---------------------------------------------------------------------------
# Phase A
# ---------------------------------------------------------------------------


def phase_a() -> dict:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    if not SOURCE_REPLAY.is_file():
        raise FileNotFoundError(
            f"Missing {SOURCE_REPLAY}. Run parse_mobile_log.py first."
        )

    dest = OUT_DIR / "real_run_replay.csv"
    shutil.copy2(SOURCE_REPLAY, dest)

    df = pd.read_csv(dest)
    imu = df[df["type"] == "IMU"]
    gps = df[df["type"] == "GPS"]

    t0 = float(df["timestamp_s"].iloc[0])
    t1 = float(df["timestamp_s"].iloc[-1])
    dur = t1 - t0

    # Sync: nearest GPS vs IMU dt distribution
    imu_t = imu["timestamp_s"].to_numpy()
    gps_t = gps["timestamp_s"].to_numpy()
    sync_dt = []
    if len(imu_t) and len(gps_t):
        for gt in gps_t:
            i = int(np.searchsorted(imu_t, gt))
            cands = []
            if i > 0:
                cands.append(abs(imu_t[i - 1] - gt))
            if i < len(imu_t):
                cands.append(abs(imu_t[i] - gt))
            sync_dt.append(min(cands) if cands else math.nan)
    sync_dt_arr = np.asarray(sync_dt, dtype=float)

    # Raw Location vs replay GPS count
    loc_n = None
    loc_path = RAW_DIR / "Location.csv"
    if loc_path.is_file():
        loc_n = sum(1 for _ in loc_path.open(encoding="utf-8")) - 1

    # Long clean GNSS stretch from raw Location (hAcc + speed)
    clean_windows: list[dict] = []
    if loc_path.is_file():
        loc = pd.read_csv(loc_path)
        loc["ok"] = (loc["horizontalAccuracy"] <= 5.0) & (loc["speed"] >= 3.0)
        # consecutive True runs
        run_start = None
        for i, ok in enumerate(loc["ok"].tolist()):
            if ok and run_start is None:
                run_start = i
            elif not ok and run_start is not None:
                clean_windows.append((run_start, i - 1))
                run_start = None
        if run_start is not None:
            clean_windows.append((run_start, len(loc) - 1))

        window_stats = []
        for a, b in clean_windows:
            if b - a + 1 < 10:
                continue
            seg = loc.iloc[a : b + 1]
            window_stats.append(
                {
                    "n_fixes": int(b - a + 1),
                    "t_start_s": float(seg["seconds_elapsed"].iloc[0]),
                    "t_end_s": float(seg["seconds_elapsed"].iloc[-1]),
                    "duration_s": float(
                        seg["seconds_elapsed"].iloc[-1] - seg["seconds_elapsed"].iloc[0]
                    ),
                    "hAcc_mean_m": float(seg["horizontalAccuracy"].mean()),
                    "speed_mean_mps": float(seg["speed"].mean()),
                    "speed_max_mps": float(seg["speed"].max()),
                }
            )
        window_stats.sort(key=lambda w: w["duration_s"], reverse=True)
        clean_windows_out = window_stats[:8]
    else:
        clean_windows_out = []

    imu_dt = np.diff(imu_t) if len(imu_t) > 1 else np.array([])
    report = {
        "phase": "A",
        "dataset": "G-ext",
        "source_raw": str(RAW_DIR.relative_to(REPO_ROOT)),
        "replay_csv": str(dest.relative_to(REPO_ROOT)),
        "n_rows_total": int(len(df)),
        "n_imu": int(len(imu)),
        "n_gps": int(len(gps)),
        "n_location_raw": loc_n,
        "timestamp_s_first": t0,
        "timestamp_s_last": t1,
        "duration_s": dur,
        "imu_rate_hz": float(len(imu) / dur) if dur > 0 else None,
        "gps_rate_hz": float(len(gps) / dur) if dur > 0 else None,
        "imu_dt_median_s": float(np.median(imu_dt)) if len(imu_dt) else None,
        "imu_dt_p99_s": float(np.percentile(imu_dt, 99)) if len(imu_dt) else None,
        "gps_to_nearest_imu_dt_median_s": (
            float(np.nanmedian(sync_dt_arr)) if len(sync_dt_arr) else None
        ),
        "gps_to_nearest_imu_dt_p99_s": (
            float(np.nanpercentile(sync_dt_arr, 99)) if len(sync_dt_arr) else None
        ),
        "checks": {
            "duration_ge_670s": bool(dur >= 670.0),
            "imu_near_99hz": bool(90.0 <= (len(imu) / dur) <= 110.0) if dur > 0 else False,
            "gps_near_1hz": bool(0.8 <= (len(gps) / dur) <= 1.2) if dur > 0 else False,
            "gps_count_matches_location": bool(loc_n is not None and loc_n == len(gps)),
            "sync_median_lt_20ms": bool(
                len(sync_dt_arr) and float(np.nanmedian(sync_dt_arr)) < 0.020
            ),
        },
        "clean_gnss_windows_hAcc_le_5m_speed_ge_3mps": clean_windows_out,
        "longest_clean_gnss_window_s": (
            clean_windows_out[0]["duration_s"] if clean_windows_out else 0.0
        ),
    }
    report["phase_a_ok"] = all(report["checks"].values())
    _write_json(OUT_DIR / "phase_a_report.json", report)
    print(json.dumps(report, indent=2))
    print(f"\nWrote {OUT_DIR / 'phase_a_report.json'}")
    return report


# ---------------------------------------------------------------------------
# Phase B — G1-identical passive replay
# ---------------------------------------------------------------------------


def phase_b(*, skip_replay: bool = False) -> dict:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    replay_csv = OUT_DIR / "real_run_replay.csv"
    if not replay_csv.is_file():
        phase_a()

    if not skip_replay:
        if not REPLAY_EXE.is_file():
            raise FileNotFoundError(REPLAY_EXE)
        # Full run — no --replay-end-s
        run_replay(
            REPLAY_EXE,
            replay_csv,
            CALIBRATION,
            OUT_DIR,
            gnss_obs_mode="pos_vel",
            ppv_policy="none",
            replay_end_s=None,
        )

    report = analyze("G1", OUT_DIR, ppv_policy="none")
    report["dataset"] = "G-ext"
    report["dataset_note"] = (
        "External validation on 19082026; same G1 shell; not GAP-4 G2 vel_only"
    )
    report["input_replay_csv"] = str(replay_csv.relative_to(REPO_ROOT))
    report["reference_g1_dir"] = str(G1_DIR.relative_to(REPO_ROOT))

    # Duration from replay output
    out_csv = OUT_DIR / "replay_output.csv"
    if out_csv.is_file():
        out_df = pd.read_csv(out_csv, usecols=["timestamp_s"])
        report["replay_output_duration_s"] = float(
            out_df["timestamp_s"].iloc[-1] - out_df["timestamp_s"].iloc[0]
        )
        report["replay_output_rows"] = int(len(out_df))

    gnss = load_csv(OUT_DIR / "gnss_nis_audit.csv")
    if not gnss.empty:
        report["n_gnss_events"] = int(len(gnss))
        report["n_rejects"] = int((gnss["accepted"] == 0).sum())
        report["accept_rate"] = float(report["accepts"] / len(gnss)) if len(gnss) else None
        report["t_first_gnss_s"] = float(gnss["timestamp_s"].iloc[0])
        report["t_last_gnss_s"] = float(gnss["timestamp_s"].iloc[-1])

    path = OUT_DIR / "gap4_g1_report.json"
    # analyze may produce NaN — convert
    def _sanitize(o):
        if isinstance(o, dict):
            return {k: _sanitize(v) for k, v in o.items()}
        if isinstance(o, list):
            return [_sanitize(v) for v in o]
        if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
            return None
        return o

    report = _sanitize(report)
    _write_json(path, report)
    print(json.dumps({k: report[k] for k in report if k != "gnss_dP_audit"}, indent=2))
    print(f"\nWrote {path}")
    return report


# ---------------------------------------------------------------------------
# Phase C — mechanistic comparison (not RMSE)
# ---------------------------------------------------------------------------


def _north_dominance(gnss: pd.DataFrame) -> dict:
    rej = gnss[gnss["accepted"] == 0].copy()
    if rej.empty:
        return {"n_rejects": 0}
    contrib = 100.0 * rej["nis_contrib_n"] / rej["nis_full"].replace(0, np.nan)
    lam = rej.apply(
        lambda r: abs(float(r["innov_n_m"])) / math.sqrt(float(r["s_nn"]))
        if float(r["s_nn"]) > 0
        else math.nan,
        axis=1,
    )
    # K4-style: |innov_N| largest among N/E (horizontal) on rejects
    innov_n_dom = int(
        (
            rej["innov_n_m"].abs() >= rej["innov_e_m"].abs()
        ).sum()
    )
    axes = rej[["nis_contrib_n", "nis_contrib_e", "nis_contrib_d"]].to_numpy()
    north_axis = (
        int(sum(1 for i in range(len(axes)) if int(np.nanargmax(np.abs(axes[i]))) == 0))
        if len(axes)
        else 0
    )
    return {
        "n_rejects": int(len(rej)),
        "contrib_n_pct_median_rejects": _safe(float(np.nanmedian(contrib))),
        "contrib_n_pct_mean_rejects": _safe(float(np.nanmean(contrib))),
        "Lambda_N_median_rejects": _safe(float(np.nanmedian(lam))),
        "Lambda_N_p90_rejects": _safe(float(np.nanpercentile(lam, 90))),
        "innov_n_ge_innov_e_reject_frac": _safe(innov_n_dom / len(rej)),
        "north_axis_dominant_reject_count": north_axis,
        "north_axis_dominant_reject_frac": _safe(north_axis / len(rej)),
    }


def _ppv_growth(cov: pd.DataFrame) -> dict:
    pre = cov[(cov["update_type"] == "gnss") & (cov["phase"] == "pre")].copy()
    if pre.empty:
        return {}
    pre = pre.sort_values("timestamp_s")
    ratio = pre["P_pv_frob"] / pre["P_vv_frob"].replace(0, np.nan)
    # growth from first to max
    ppv0 = float(pre["P_pv_frob"].iloc[0])
    ppv_max = float(pre["P_pv_frob"].max())
    return {
        "P_pv_frob_first_gnss_pre": _safe(ppv0),
        "P_pv_frob_max_gnss_pre": _safe(ppv_max),
        "P_pv_grew": bool(ppv_max > ppv0 * 1.5 + 1e-9),
        "P_pv_over_P_vv_median_gnss_pre": _safe(float(np.nanmedian(ratio))),
        "P_pv_over_P_vv_max_gnss_pre": _safe(float(np.nanmax(ratio))),
    }


def _pvv_compression(cov: pd.DataFrame, gnss: pd.DataFrame) -> dict:
    nhc = cov[(cov["update_type"] == "nhc") & (cov["phase"] == "post")]
    gnss_pre = cov[(cov["update_type"] == "gnss") & (cov["phase"] == "pre")]
    acc = gnss[gnss["accepted"] == 1].sort_values("gps_index")
    out: dict = {
        "P_vv_frob_min_nhc_post": _safe(float(nhc["P_vv_frob"].min())) if len(nhc) else None,
        "P_vv_frob_median_gnss_pre": (
            _safe(float(gnss_pre["P_vv_frob"].median())) if len(gnss_pre) else None
        ),
        "P_vv_frob_min_gnss_pre": (
            _safe(float(gnss_pre["P_vv_frob"].min())) if len(gnss_pre) else None
        ),
    }
    if len(acc) >= 3:
        # early compression: P_vv at accept#3 vs peak before
        t3 = float(acc.iloc[2]["timestamp_s"])
        early = gnss_pre[gnss_pre["timestamp_s"] <= t3 + 1e-3]
        if len(early):
            out["P_vv_pre_accept3"] = _safe(float(early.iloc[-1]["P_vv_frob"]))
            out["P_vv_peak_before_accept3"] = _safe(float(early["P_vv_frob"].max()))
            peak = early["P_vv_frob"].max()
            at3 = float(early.iloc[-1]["P_vv_frob"])
            out["early_P_vv_compression_ratio"] = _safe(at3 / peak) if peak > 0 else None
    return out


def _nhc_burst_metrics(cov: pd.DataFrame, gnss: pd.DataFrame) -> dict:
    fixes = fix_timestamps(gnss)
    out: dict = {"fix2_fix3_available": fixes is not None}
    if fixes is not None:
        gap = analyze_gap(cov, fixes["t_fix2"], fixes["t_fix3"], nhc_enabled=True)
        out["gap_fix2_fix3"] = {k: _safe(v) if isinstance(v, float) else v for k, v in gap.items()}
    # Also global: among all NHC ticks, top3 share of |ΔP_vv|
    # Approximate Δ from consecutive nhc post vs predict post within imu_seq is heavy;
    # use nhc_block if available
    nhc_block = OUT_DIR / "nhc_block_audit.csv"
    if nhc_block.is_file():
        nb = load_csv(nhc_block)
        if "delta_P_vv_frob" in nb.columns and len(nb):
            abs_d = nb["delta_P_vv_frob"].abs()
            total = float(abs_d.sum())
            top3 = float(abs_d.nlargest(3).sum()) if len(abs_d) >= 3 else float(abs_d.sum())
            out["global_nhc_top3_share_abs_delta_P_vv"] = (
                _safe(top3 / total) if total > 0 else None
            )
            out["global_nhc_erosion_pattern"] = (
                "bursty"
                if out["global_nhc_top3_share_abs_delta_P_vv"] is not None
                and out["global_nhc_top3_share_abs_delta_P_vv"] > 0.5
                else "uniform"
            )
    return out


def _g1_reference_mechanistics() -> dict:
    """Frozen G1 numbers from report + recomputed from G1 audits when needed."""
    g1 = json.loads(G1_REPORT.read_text(encoding="utf-8"))
    gnss = load_csv(G1_DIR / "gnss_nis_audit.csv")
    cov = load_csv(G1_DIR / "cov_step_audit.csv")
    ref = {
        "accepts": g1.get("accepts"),
        "n_gnss_events": int(len(gnss)),
        "n_rejects": int((gnss["accepted"] == 0).sum()),
        "verdict_h1": g1.get("verdict_h1"),
        "Lambda_n_fix8": g1.get("Lambda_n_fix8"),
        "contrib_n_pct_fix8": g1.get("contrib_n_pct_fix8"),
        "P_vv_pre_fix3": g1.get("P_vv_pre_fix3"),
        "north": _north_dominance(gnss),
        "ppv": _ppv_growth(cov),
        "pvv": _pvv_compression(cov, gnss),
        "nhc_burst": _nhc_burst_metrics_dir(G1_DIR, cov, gnss),
    }
    return ref


def _best_inter_gnss_nhc_burst(cov: pd.DataFrame, gnss: pd.DataFrame) -> dict:
    """Scan gaps between consecutive GNSS events; keep max top3 NHC share (F1-style)."""
    times = gnss.sort_values("timestamp_s")["timestamp_s"].to_numpy()
    best: dict = {
        "max_top3_share": None,
        "max_gamma": None,
        "best_gap_t0": None,
        "best_gap_t1": None,
        "best_erosion_pattern": None,
    }
    if len(times) < 2:
        return best
    max_top3 = -1.0
    for i in range(min(len(times) - 1, 80)):  # early/mid gaps enough; bound cost
        t0, t1 = float(times[i]), float(times[i + 1])
        if t1 - t0 < 0.05:
            continue
        gap = analyze_gap(cov, t0, t1, nhc_enabled=True)
        top3 = gap.get("top3_nhc_share_of_abs_drop")
        if top3 is None or (isinstance(top3, float) and math.isnan(top3)):
            continue
        if top3 > max_top3:
            max_top3 = float(top3)
            best = {
                "max_top3_share": _safe(top3),
                "max_gamma": _safe(gap.get("gamma_nhc_over_predict")),
                "best_gap_t0": t0,
                "best_gap_t1": t1,
                "best_erosion_pattern": gap.get("erosion_pattern"),
                "best_sum_abs_delta_P_vv_nhc": _safe(gap.get("sum_abs_delta_P_vv_nhc")),
            }
    return best


def _nhc_burst_metrics_dir(audit_dir: Path, cov: pd.DataFrame, gnss: pd.DataFrame) -> dict:
    fixes = fix_timestamps(gnss)
    out: dict = {"fix2_fix3_available": fixes is not None}
    if fixes is not None:
        gap = analyze_gap(cov, fixes["t_fix2"], fixes["t_fix3"], nhc_enabled=True)
        out["gap_fix2_fix3"] = {k: _safe(v) if isinstance(v, float) else v for k, v in gap.items()}
    out["inter_gnss_scan"] = _best_inter_gnss_nhc_burst(cov, gnss)
    nhc_block = audit_dir / "nhc_block_audit.csv"
    if nhc_block.is_file():
        nb = load_csv(nhc_block)
        if "delta_P_vv_frob" in nb.columns and len(nb):
            abs_d = nb["delta_P_vv_frob"].abs()
            total = float(abs_d.sum())
            top3 = float(abs_d.nlargest(3).sum()) if len(abs_d) >= 3 else float(abs_d.sum())
            share = top3 / total if total > 0 else math.nan
            out["global_nhc_top3_share_abs_delta_P_vv"] = _safe(share)
            out["global_nhc_erosion_pattern"] = (
                "bursty" if (not math.isnan(share) and share > 0.5) else "uniform"
            )
            # Strong NHC floor (mechanism K1/K2), independent of top3 concentration
            out["P_vv_floor_nhc_post"] = _safe(float(nb["P_post_vv_frob"].min()))
            out["nhc_drives_P_vv_to_floor"] = bool(float(nb["P_post_vv_frob"].min()) < 0.1)
    return out


def phase_c() -> dict:
    gext_report = json.loads((OUT_DIR / "gap4_g1_report.json").read_text(encoding="utf-8"))
    gnss = load_csv(OUT_DIR / "gnss_nis_audit.csv")
    cov = load_csv(OUT_DIR / "cov_step_audit.csv")

    gext = {
        "accepts": gext_report.get("accepts"),
        "n_gnss_events": int(len(gnss)),
        "n_rejects": int((gnss["accepted"] == 0).sum()),
        "verdict_h1": gext_report.get("verdict_h1"),
        "Lambda_n_fix8": gext_report.get("Lambda_n_fix8"),
        "contrib_n_pct_fix8": gext_report.get("contrib_n_pct_fix8"),
        "P_vv_pre_fix3": gext_report.get("P_vv_pre_fix3"),
        "north": _north_dominance(gnss),
        "ppv": _ppv_growth(cov),
        "pvv": _pvv_compression(cov, gnss),
        "nhc_burst": _nhc_burst_metrics_dir(OUT_DIR, cov, gnss),
    }
    g1 = _g1_reference_mechanistics()

    def yn(cond: bool | None) -> str:
        if cond is None:
            return "?"
        return "✓" if cond else "✗"

    # --- Operationalizations aligned with STATE_OF_KNOWLEDGE (not RMSE) ---
    # Burst: F1-style top3>0.5 in some inter-GNSS gap, OR NHC drives P_vv to floor
    # (classic F1 burst was pos-only; under G1 shell Joseph@fix#2 changes gap stats).
    scan_g1 = g1["nhc_burst"].get("inter_gnss_scan") or {}
    scan_gext = gext["nhc_burst"].get("inter_gnss_scan") or {}
    burst_g1 = bool(
        (scan_g1.get("max_top3_share") or 0) > 0.5
        or g1["nhc_burst"].get("nhc_drives_P_vv_to_floor")
    )
    burst_gext = bool(
        (scan_gext.get("max_top3_share") or 0) > 0.5
        or gext["nhc_burst"].get("nhc_drives_P_vv_to_floor")
    )

    # P_vv compression: floor after NHC or median pre-GNSS P_vv << init (~1.73)
    def _pvv_compressed(pvv: dict, burst: dict) -> bool:
        floor = burst.get("P_vv_floor_nhc_post")
        med = pvv.get("P_vv_frob_median_gnss_pre")
        ratio = pvv.get("early_P_vv_compression_ratio")
        if ratio is not None and ratio < 0.5:
            return True
        if floor is not None and floor < 0.1:
            return True
        if med is not None and med < 1.0:
            return True
        return False

    pvv_comp_g1 = _pvv_compressed(g1["pvv"], g1["nhc_burst"])
    pvv_comp_gext = _pvv_compressed(gext["pvv"], gext["nhc_burst"])

    # North innov: |innov_N| >= |innov_E| on majority of rejects (K4 horizontal)
    # OR elevated Lambda_N median (gate stress on North)
    def _north_story(north: dict) -> bool:
        frac = north.get("innov_n_ge_innov_e_reject_frac")
        lam = north.get("Lambda_N_median_rejects")
        if frac is not None and frac >= 0.5:
            return True
        if lam is not None and lam >= 2.0:
            return True
        return False

    north_g1 = _north_story(g1["north"])
    north_gext = _north_story(gext["north"])

    lam_story_g1 = (g1["north"].get("Lambda_N_median_rejects") or 0) >= 2.0
    lam_story_gext = (gext["north"].get("Lambda_N_median_rejects") or 0) >= 2.0

    reject_g1 = (g1.get("n_rejects") or 0) > (g1.get("accepts") or 0)
    reject_gext = (gext.get("n_rejects") or 0) > (gext.get("accepts") or 0)

    rows = [
        {
            "property": "Burst NHC / NHC floor",
            "G1": yn(burst_g1),
            "G_ext": yn(burst_gext),
            "same_story": yn(burst_g1 and burst_gext),
            "G1_detail": {
                "inter_gnss_scan": scan_g1,
                "nhc_drives_P_vv_to_floor": g1["nhc_burst"].get("nhc_drives_P_vv_to_floor"),
                "P_vv_floor_nhc_post": g1["nhc_burst"].get("P_vv_floor_nhc_post"),
                "note": "F1 classic burst was pos-only; G1 shell scored via scan+floor",
            },
            "G_ext_detail": {
                "inter_gnss_scan": scan_gext,
                "nhc_drives_P_vv_to_floor": gext["nhc_burst"].get("nhc_drives_P_vv_to_floor"),
                "P_vv_floor_nhc_post": gext["nhc_burst"].get("P_vv_floor_nhc_post"),
            },
        },
        {
            "property": "Compresión P_vv",
            "G1": yn(pvv_comp_g1),
            "G_ext": yn(pvv_comp_gext),
            "same_story": yn(pvv_comp_g1 and pvv_comp_gext),
            "G1_detail": {"P_vv_pre_fix3": g1.get("P_vv_pre_fix3"), **g1["pvv"]},
            "G_ext_detail": {
                "P_vv_pre_fix3": gext.get("P_vv_pre_fix3"),
                **gext["pvv"],
            },
        },
        {
            "property": "Crecimiento P_pv",
            "G1": yn(g1["ppv"].get("P_pv_grew")),
            "G_ext": yn(gext["ppv"].get("P_pv_grew")),
            "same_story": yn(
                bool(g1["ppv"].get("P_pv_grew")) and bool(gext["ppv"].get("P_pv_grew"))
            ),
            "G1_detail": g1["ppv"],
            "G_ext_detail": gext["ppv"],
        },
        {
            "property": "Innovación Norte dominante",
            "G1": yn(north_g1),
            "G_ext": yn(north_gext),
            "same_story": yn(north_g1 and north_gext),
            "G1_detail": g1["north"],
            "G_ext_detail": gext["north"],
        },
        {
            "property": "Evolución Λ_N (rejects elevados)",
            "G1": yn(lam_story_g1),
            "G_ext": yn(lam_story_gext),
            "same_story": yn(lam_story_g1 and lam_story_gext),
            "G1_detail": {
                "Lambda_n_fix8": g1.get("Lambda_n_fix8"),
                **{k: g1["north"].get(k) for k in ("Lambda_N_median_rejects", "Lambda_N_p90_rejects")},
            },
            "G_ext_detail": {
                "Lambda_n_fix8": gext.get("Lambda_n_fix8"),
                **{
                    k: gext["north"].get(k)
                    for k in ("Lambda_N_median_rejects", "Lambda_N_p90_rejects")
                },
            },
        },
        {
            "property": "Rechazos GNSS (mayoría reject)",
            "G1": yn(reject_g1),
            "G_ext": yn(reject_gext),
            "same_story": yn(reject_g1 and reject_gext),
            "G1_detail": {
                "accepts": g1.get("accepts"),
                "rejects": g1.get("n_rejects"),
                "events": g1.get("n_gnss_events"),
            },
            "G_ext_detail": {
                "accepts": gext.get("accepts"),
                "rejects": gext.get("n_rejects"),
                "events": gext.get("n_gnss_events"),
            },
        },
    ]

    n_same = sum(1 for r in rows if r["same_story"] == "✓")
    comparison = {
        "phase": "C",
        "dataset": "G-ext",
        "reference": "GAP-4 G1 (frozen)",
        "note": "Mechanistic properties only — no RMSE / drift scoring",
        "G1": g1,
        "G_ext": gext,
        "table": rows,
        "n_properties_same_story": n_same,
        "n_properties": len(rows),
        "external_validation_strong": n_same >= 5,
    }
    _write_json(OUT_DIR / "phase_c_mechanistic_comparison.json", comparison)

    md = [
        "# G-ext Phase C — Comparación mecanicista vs G1",
        "",
        f"**Same-story count:** {n_same}/{len(rows)}",
        f"**Validacion externa fuerte (>=5/6):** {'SI' if n_same >= 5 else 'NO'}",
        "",
        "| Propiedad | G1 | G-ext | ¿Misma historia? |",
        "|-----------|----|-------|------------------|",
    ]
    for r in rows:
        md.append(
            f"| {r['property']} | {r['G1']} | {r['G_ext']} | {r['same_story']} |"
        )
    md.extend(
        [
            "",
            "## Conteos GNSS",
            "",
            f"- G1: accepts={g1.get('accepts')} rejects={g1.get('n_rejects')} events={g1.get('n_gnss_events')}",
            f"- G-ext: accepts={gext.get('accepts')} rejects={gext.get('n_rejects')} events={gext.get('n_gnss_events')}",
            "",
            "## Detalle clave",
            "",
            f"- G1 P_vv floor NHC: {g1['nhc_burst'].get('P_vv_floor_nhc_post')}",
            f"- G-ext P_vv floor NHC: {gext['nhc_burst'].get('P_vv_floor_nhc_post')}",
            f"- G1 Lambda_N median rejects: {g1['north'].get('Lambda_N_median_rejects')}",
            f"- G-ext Lambda_N median rejects: {gext['north'].get('Lambda_N_median_rejects')}",
            f"- G1 inter-GNSS max top3: {scan_g1.get('max_top3_share')} pattern={scan_g1.get('best_erosion_pattern')}",
            f"- G-ext inter-GNSS max top3: {scan_gext.get('max_top3_share')} pattern={scan_gext.get('best_erosion_pattern')}",
            "",
            "## Nota",
            "",
            "No se evalua RMSE ni deriva. Solo propiedades mecanicistas.",
            "El burst clasico F1 (pos-only, gamma~19.7) no se re-mide aqui;",
            "bajo shell G1 se puntua scan inter-GNSS + floor P_vv por NHC.",
            "",
        ]
    )
    (OUT_DIR / "phase_c_comparison.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    try:
        print("\n".join(md))
    except UnicodeEncodeError:
        print((OUT_DIR / "phase_c_comparison.md").read_text(encoding="utf-8"))
    print(f"\nWrote {OUT_DIR / 'phase_c_mechanistic_comparison.json'}")
    return comparison


def main() -> int:
    parser = argparse.ArgumentParser(description="G-ext 19082026 baseline A→B→C")
    parser.add_argument(
        "--phase",
        choices=["A", "B", "C", "all"],
        default="all",
        help="Run a single phase or all (default)",
    )
    parser.add_argument(
        "--skip-replay",
        action="store_true",
        help="Phase B: reuse existing audit CSVs",
    )
    args = parser.parse_args()

    if args.phase in ("A", "all"):
        a = phase_a()
        if not a.get("phase_a_ok"):
            print("WARNING: Phase A checks not all green — review phase_a_report.json")

    if args.phase in ("B", "all"):
        phase_b(skip_replay=args.skip_replay)

    if args.phase in ("C", "all"):
        if not (OUT_DIR / "gap4_g1_report.json").is_file():
            print("ERROR: Phase B artefacts missing", file=sys.stderr)
            return 1
        phase_c()

    return 0


if __name__ == "__main__":
    sys.exit(main())
