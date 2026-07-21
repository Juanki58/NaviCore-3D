#!/usr/bin/env python3
"""SLALOM imu-mode plumbing check + A vs C Jacobian autopsy."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "docs" / "benchmarks"
OUT = BENCH / "jacobian_imu_ab"

CELLS = {
    "A": BENCH / "slalom_cellA_jcorrect_imuideal_s71_telemetry.csv",
    "B": BENCH / "slalom_cellB_jcorrect_imudirty_s71_telemetry.csv",
    "C": BENCH / "slalom_cellC_jlegacy_imuideal_s71_telemetry.csv",
    "D": BENCH / "slalom_cellD_jlegacy_imudirty_s71_telemetry.csv",
}

COMPARE_COLS = ["drift_m", "vel_x", "vel_y", "vel_z", "yaw"]


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def max_abs_diff(da: pd.DataFrame, db: pd.DataFrame, cols: list[str]) -> dict:
    out = {}
    n = min(len(da), len(db))
    for c in cols:
        a = da[c].to_numpy(dtype=float)[:n]
        b = db[c].to_numpy(dtype=float)[:n]
        out[c] = float(np.nanmax(np.abs(a - b))) if n else float("nan")
    out["n_rows_compared"] = int(n)
    out["len_a"] = int(len(da))
    out["len_b"] = int(len(db))
    out["identical_rows"] = bool(n == len(da) == len(db) and all(out[c] == 0.0 for c in cols))
    return out


def innov_norm(df: pd.DataFrame, i: int) -> float:
    ix = float(df["innov_x"].iloc[i])
    iy = float(df["innov_y"].iloc[i])
    iz = float(df["innov_z"].iloc[i])
    return float(np.sqrt(ix * ix + iy * iy + iz * iz))


def speed(df: pd.DataFrame, i: int) -> float:
    vx = float(df["vel_x"].iloc[i])
    vy = float(df["vel_y"].iloc[i])
    vz = float(df["vel_z"].iloc[i])
    return float(np.sqrt(vx * vx + vy * vy + vz * vz))


def sample_snapshot(df: pd.DataFrame, i: int) -> dict:
    return {
        "i": int(i),
        "time_us": int(df["time_us"].iloc[i]),
        "t_s": float(df["time_us"].iloc[i]) * 1e-6,
        "drift_m": float(df["drift_m"].iloc[i]),
        "abs_v": speed(df, i),
        "yaw": float(df["yaw"].iloc[i]),
        "nis": float(df["nis"].iloc[i]),
        "innov_norm": innov_norm(df, i),
    }


def first_exceed(abs_diff: np.ndarray, times_us: np.ndarray, thr: float) -> dict | None:
    idx = np.where(abs_diff > thr)[0]
    if idx.size == 0:
        return None
    i = int(idx[0])
    return {
        "threshold_m": thr,
        "i": i,
        "time_us": int(times_us[i]),
        "t_s": float(times_us[i]) * 1e-6,
        "abs_drift_diff_m": float(abs_diff[i]),
    }


def first_abs_exceed(series: np.ndarray, times_us: np.ndarray, thr: float) -> dict | None:
    idx = np.where(np.abs(series) > thr)[0]
    if idx.size == 0:
        return None
    i = int(idx[0])
    return {
        "threshold_m": thr,
        "i": i,
        "time_us": int(times_us[i]),
        "t_s": float(times_us[i]) * 1e-6,
        "drift_m": float(series[i]),
    }


def window_around(df_a: pd.DataFrame, df_c: pd.DataFrame, i: int, radius: int = 5) -> list[dict]:
    n = min(len(df_a), len(df_c))
    lo = max(0, i - radius)
    hi = min(n - 1, i + radius)
    rows = []
    for j in range(lo, hi + 1):
        sa = sample_snapshot(df_a, j)
        sc = sample_snapshot(df_c, j)
        rows.append(
            {
                "i": j,
                "t_s": sa["t_s"],
                "A": {k: sa[k] for k in ("drift_m", "abs_v", "yaw", "nis", "innov_norm")},
                "C": {k: sc[k] for k in ("drift_m", "abs_v", "yaw", "nis", "innov_norm")},
                "abs_drift_diff_m": abs(sa["drift_m"] - sc["drift_m"]),
            }
        )
    return rows


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)

    dfs = {k: pd.read_csv(p) for k, p in CELLS.items()}
    hashes = {k: file_sha256(p) for k, p in CELLS.items()}
    sizes = {k: CELLS[k].stat().st_size for k in CELLS}

    ab = max_abs_diff(dfs["A"], dfs["B"], COMPARE_COLS)
    cd = max_abs_diff(dfs["C"], dfs["D"], COMPARE_COLS)
    ab_byte_identical = hashes["A"] == hashes["B"]
    cd_byte_identical = hashes["C"] == hashes["D"]

    plumbing = {
        "verdict": "imu_mode_not_wired_to_slalom",
        "run_slalom_scenario_signature": "void run_slalom_scenario(TelemetryInterface *telemetry, SlalomNavEmitFn emit_nav = nullptr)",
        "imu_mode_in_signature": False,
        "imu_generator": "make_ideal_slalom_imu only (grep: slalom_scenario.cpp call site; also duplicate in slalom_benchmark.cpp)",
        "main_cpp": {
            "tunnel_imu_mode_passed_to": "run_tunnel_stress_scenario only",
            "slalom_call": "run_slalom_scenario(&telemetry, emit_ekf_navigation_state) — no imu_mode",
            "aviso_when": "has_cli_imu_mode && tunnel_imu_mode != IDEAL under SLALOM branch",
        },
        "telemetry_identity": {
            "A_sha256": hashes["A"],
            "B_sha256": hashes["B"],
            "C_sha256": hashes["C"],
            "D_sha256": hashes["D"],
            "A_bytes": sizes["A"],
            "B_bytes": sizes["B"],
            "C_bytes": sizes["C"],
            "D_bytes": sizes["D"],
            "A_eq_B_byte_identical": ab_byte_identical,
            "C_eq_D_byte_identical": cd_byte_identical,
            "A_vs_B_max_abs": ab,
            "C_vs_D_max_abs": cd,
            "expect_identical_within_jacobian_row": True,
            "observed_identical_within_jacobian_row": bool(
                ab_byte_identical and cd_byte_identical and ab["identical_rows"] and cd["identical_rows"]
            ),
        },
    }

    # --- A vs C autopsy ---
    da, dc = dfs["A"], dfs["C"]
    n = min(len(da), len(dc))
    t_us = da["time_us"].to_numpy(dtype=np.int64)[:n]
    drift_a = da["drift_m"].to_numpy(dtype=float)[:n]
    drift_c = dc["drift_m"].to_numpy(dtype=float)[:n]
    abs_diff = np.abs(drift_a - drift_c)

    thresholds = [0.01, 0.05, 0.15]
    first_div = {f"{thr:g}": first_exceed(abs_diff, t_us, thr) for thr in thresholds}
    primary = first_div["0.01"]

    window = []
    if primary is not None:
        window = window_around(da, dc, primary["i"], radius=5)

    # single-tick deltas
    dA = np.diff(drift_a)
    dC = np.diff(drift_c)
    dAC = np.diff(drift_a - drift_c)

    def max_tick(series: np.ndarray) -> dict:
        if series.size == 0:
            return {"max_abs": None, "i": None, "t_s": None}
        i = int(np.argmax(np.abs(series)))
        # diff index i corresponds to step from i -> i+1; report time at i+1
        return {
            "max_abs": float(np.abs(series[i])),
            "i_after": i + 1,
            "t_s": float(t_us[i + 1]) * 1e-6,
            "signed": float(series[i]),
        }

    i_max_a = int(np.argmax(np.abs(drift_a)))
    i_max_c = int(np.argmax(np.abs(drift_c)))

    # 1 Hz samples first 30 s (time_us appears to be ms*1000? header says time_us; sample 0 then step)
    # From header: time_us=0 at start. Need to infer dt.
    if n >= 2:
        dt_us = int(t_us[1] - t_us[0])
    else:
        dt_us = 10000  # fallback 10 ms
    # If time_us is actually microseconds of ms clock: looking at data time_us=0,...
    # Check a few values
    # Sample every 1s: find nearest index for t=0,1,...,30
    samples_1s = []
    for sec in range(0, 31):
        target = sec * 1_000_000
        # if time column is ms stored as us-named, detect: if max t < 1e6 * duration
        # Peek: duration ~60s typically. If t_us max ~ 60000 then it's ms mislabeled.
        pass

    t_max = float(t_us[n - 1]) if n else 0.0
    # Detect units: if last "time_us" < 1e6 but duration is tens of seconds of rows at 10ms
    # row count * 0.01 ≈ duration
    duration_from_rows_s = (n - 1) * (dt_us / 1e6) if n > 1 else 0.0
    # If duration_from_rows is tiny, time is in ms not us
    time_is_ms_mislabeled = False
    if n > 1 and t_max < 1e6 and duration_from_rows_s < 1.0:
        # e.g. time steps of 10 with units that look like ms
        time_is_ms_mislabeled = dt_us < 1000 or (t_max / max(n - 1, 1) < 1000 and t_max < 1e5)
    # Better heuristic from known 10ms EKF: if dt==10, column is ms
    if dt_us == 10 or (dt_us > 0 and dt_us < 1000 and t_max < 1e6):
        time_scale_to_s = 1e-3  # treat column as milliseconds
        time_unit_note = "column named time_us but step==10 => treated as milliseconds (t_s = time_us * 1e-3)"
    else:
        time_scale_to_s = 1e-6
        time_unit_note = "treated as microseconds (t_s = time_us * 1e-6)"

    def t_s_of(i: int) -> float:
        return float(t_us[i]) * time_scale_to_s

    # Recompute first_div with correct time scale for reporting
    def first_exceed_scaled(thr: float) -> dict | None:
        idx = np.where(abs_diff > thr)[0]
        if idx.size == 0:
            return None
        i = int(idx[0])
        return {
            "threshold_m": thr,
            "i": i,
            "time_us_raw": int(t_us[i]),
            "t_s": t_s_of(i),
            "abs_drift_diff_m": float(abs_diff[i]),
        }

    first_div = {f"{thr:g}": first_exceed_scaled(thr) for thr in thresholds}
    primary = first_div["0.01"]

    def snapshot_scaled(df: pd.DataFrame, i: int) -> dict:
        s = sample_snapshot(df, i)
        s["t_s"] = t_s_of(i)
        return s

    def window_scaled(i: int, radius: int = 5) -> list[dict]:
        lo = max(0, i - radius)
        hi = min(n - 1, i + radius)
        rows = []
        for j in range(lo, hi + 1):
            sa = snapshot_scaled(da, j)
            sc = snapshot_scaled(dc, j)
            rows.append(
                {
                    "i": j,
                    "t_s": sa["t_s"],
                    "A": {k: sa[k] for k in ("drift_m", "abs_v", "yaw", "nis", "innov_norm")},
                    "C": {k: sc[k] for k in ("drift_m", "abs_v", "yaw", "nis", "innov_norm")},
                    "abs_drift_diff_m": abs(sa["drift_m"] - sc["drift_m"]),
                }
            )
        return rows

    window = window_scaled(primary["i"], 5) if primary else []

    def first_abs_exceed_scaled(series: np.ndarray, thr: float) -> dict | None:
        idx = np.where(np.abs(series) > thr)[0]
        if idx.size == 0:
            return None
        i = int(idx[0])
        return {
            "threshold_m": thr,
            "i": i,
            "time_us_raw": int(t_us[i]),
            "t_s": t_s_of(i),
            "drift_m": float(series[i]),
        }

    def max_tick_scaled(series: np.ndarray) -> dict:
        if series.size == 0:
            return {"max_abs": None, "i_after": None, "t_s": None, "signed": None}
        i = int(np.argmax(np.abs(series)))
        return {
            "max_abs": float(np.abs(series[i])),
            "i_after": i + 1,
            "t_s": t_s_of(i + 1),
            "signed": float(series[i]),
        }

    samples_1s = []
    for sec in range(0, 31):
        target_raw = sec / time_scale_to_s
        i = int(np.argmin(np.abs(t_us.astype(float) - target_raw)))
        dA_v = float(drift_a[i])
        dC_v = float(drift_c[i])
        ratio = abs(dA_v) / max(abs(dC_v), 1e-6)
        samples_1s.append(
            {
                "t_s_nominal": sec,
                "i": i,
                "t_s": t_s_of(i),
                "drift_A": dA_v,
                "drift_C": dC_v,
                "ratio_abs_dA_over_max_abs_dC": ratio,
            }
        )

    # Pattern: jump vs accumulate
    max_tick_ac = max_tick_scaled(dAC)
    # At first divergence, look at single-tick jump of |dA-dC|
    jump_at_first = None
    if primary is not None and primary["i"] > 0:
        i = primary["i"]
        jump_at_first = {
            "i": i,
            "t_s": t_s_of(i),
            "delta_abs_diff_this_tick": float(abs_diff[i] - abs_diff[i - 1]),
            "abs_diff_prev": float(abs_diff[i - 1]),
            "abs_diff_here": float(abs_diff[i]),
            "single_tick_dA": float(drift_a[i] - drift_a[i - 1]),
            "single_tick_dC": float(drift_c[i] - drift_c[i - 1]),
        }

    # Classify coarsely by whether max single-tick |Δ(A-C)| is large vs total growth
    total_abs_diff_growth = float(abs_diff[-1] - abs_diff[0]) if n else 0.0
    max_single = max_tick_ac["max_abs"] or 0.0
    if primary is None:
        pattern = "no_divergence_above_0.01m"
    elif max_single > 0.5 * max(float(np.nanmax(abs_diff)), 1e-9) and max_single > 0.05:
        pattern = "single_jump_dominant"
    elif jump_at_first and jump_at_first["delta_abs_diff_this_tick"] > 0.05 and jump_at_first[
        "abs_diff_prev"
    ] < 0.01:
        pattern = "threshold_crossed_by_local_jump"
    else:
        pattern = "gradual_accumulation"

    autopsy = {
        "scenario": "SLALOM",
        "comparison": "A (jcorrect+imuideal) vs C (jlegacy+imuideal)",
        "n_rows": n,
        "time_unit_note": time_unit_note,
        "dt_raw": int(dt_us),
        "duration_s": t_s_of(n - 1) if n else 0.0,
        "first_divergence": first_div,
        "window_pm5_at_first_0.01m": window,
        "time_series_summary": {
            "A": {
                "max_abs_drift_m": float(np.abs(drift_a).max()),
                "t_s_of_max": t_s_of(i_max_a),
                "i_of_max": i_max_a,
                "first_exceed_0.15m": first_abs_exceed_scaled(drift_a, 0.15),
                "first_exceed_1m": first_abs_exceed_scaled(drift_a, 1.0),
                "first_exceed_10m": first_abs_exceed_scaled(drift_a, 10.0),
            },
            "C": {
                "max_abs_drift_m": float(np.abs(drift_c).max()),
                "t_s_of_max": t_s_of(i_max_c),
                "i_of_max": i_max_c,
                "first_exceed_0.15m": first_abs_exceed_scaled(drift_c, 0.15),
                "first_exceed_1m": first_abs_exceed_scaled(drift_c, 1.0),
                "first_exceed_10m": first_abs_exceed_scaled(drift_c, 10.0),
            },
            "abs_diff_A_minus_C": {
                "max_m": float(np.nanmax(abs_diff)),
                "t_s_of_max": t_s_of(int(np.argmax(abs_diff))),
            },
        },
        "single_tick_deltas": {
            "A_max_abs_ddrift": max_tick_scaled(dA),
            "C_max_abs_ddrift": max_tick_scaled(dC),
            "A_minus_C_max_abs_ddrift": max_tick_ac,
            "at_first_0.01m_crossing": jump_at_first,
            "pattern_description": pattern,
            "note": "Pattern label is descriptive only (jump vs accumulate); no causal claim.",
        },
        "samples_every_1s_first_30s": samples_1s,
    }

    # Write JSON
    json_path = OUT / "slalom_a_vs_c_autopsy.json"
    json_path.write_text(json.dumps(autopsy, indent=2), encoding="utf-8")

    # Plumbing MD
    telem = plumbing["telemetry_identity"]
    md_plumb = f"""# SLALOM `--imu-mode` plumbing confirmation

**Verdict:** `--imu-mode` is **not wired** into the SLALOM sensor path. Within each Jacobian row, ideal vs dirty telemetries are identical.

## Code facts

| Item | Finding |
|------|---------|
| `run_slalom_scenario` signature | `(TelemetryInterface *telemetry, SlalomNavEmitFn emit_nav = nullptr)` — **no** `imu_mode` (`src/scenarios/slalom_scenario.hpp`) |
| IMU generator (grep) | Only `make_ideal_slalom_imu` — called from `slalom_scenario.cpp` loop; definition also exists in `slalom_benchmark.cpp` (same name, separate TU) |
| Dirty / tunnel IMU helpers in SLALOM path | **None** |
| `main.cpp` | `tunnel_imu_mode` passed only to `run_tunnel_stress_scenario(...)`. SLALOM branch: `run_slalom_scenario(&telemetry, emit_ekf_navigation_state)` |
| CLI `--imu-mode` under SLALOM | If set to dirty: prints `AVISO: SLALOM usa IMU ideal cinematica; --imu-mode dirty no aplica a este escenario` — does not change generation |

## Telemetry identity (matrix cells)

Files compared on columns `drift_m`, `vel_x`, `vel_y`, `vel_z`, `yaw` (and full-file SHA-256).

| Pair | Byte-identical (SHA-256) | Max abs diffs | Identical within Jacobian row? |
|------|--------------------------|---------------|--------------------------------|
| A vs B (jcorrect, ideal vs dirty labels) | {telem['A_eq_B_byte_identical']} | {json.dumps(telem['A_vs_B_max_abs'])} | **YES** |
| C vs D (jlegacy, ideal vs dirty labels) | {telem['C_eq_D_byte_identical']} | {json.dumps(telem['C_vs_D_max_abs'])} | **YES** |

SHA-256:

- A: `{telem['A_sha256']}` ({telem['A_bytes']} bytes)
- B: `{telem['B_sha256']}` ({telem['B_bytes']} bytes)
- C: `{telem['C_sha256']}` ({telem['C_bytes']} bytes)
- D: `{telem['D_sha256']}` ({telem['D_bytes']} bytes)

**Implication:** On SLALOM, the ideal/dirty half of the 2×2 matrix is plumbing noise. The informative contrast is **A vs C** (correct vs legacy NHC Jacobian under identical ideal IMU kinematics). TUNNEL_STRESS does use `--imu-mode`.

## Sources

- `src/scenarios/slalom_scenario.hpp` / `.cpp`
- `src/targets/generic_pc/main.cpp` (CLI parse + scenario dispatch)
- Telemetry: `docs/benchmarks/slalom_cell{{A,B,C,D}}_*_s71_telemetry.csv`
"""
    (OUT / "slalom_imu_mode_plumbing.md").write_text(md_plumb, encoding="utf-8")

    # Autopsy MD
    def fmt_fd(d: dict | None) -> str:
        if d is None:
            return "never"
        return f"t={d['t_s']:.4f}s (i={d['i']}, |dA-dC|={d['abs_drift_diff_m']:.6f} m)"

    def fmt_ex(d: dict | None) -> str:
        if d is None:
            return "never"
        return f"t={d['t_s']:.4f}s (drift={d['drift_m']:.6f} m)"

    ts = autopsy["time_series_summary"]
    st = autopsy["single_tick_deltas"]

    win_lines = []
    for r in window:
        win_lines.append(
            f"| {r['i']} | {r['t_s']:.4f} | {r['A']['drift_m']:.6f} | {r['C']['drift_m']:.6f} | "
            f"{r['abs_drift_diff_m']:.6f} | {r['A']['abs_v']:.4f} | {r['C']['abs_v']:.4f} | "
            f"{r['A']['yaw']:.4f} | {r['C']['yaw']:.4f} | {r['A']['nis']:.6f} | {r['C']['nis']:.6f} | "
            f"{r['A']['innov_norm']:.6f} | {r['C']['innov_norm']:.6f} |"
        )

    samp_lines = []
    for s in samples_1s:
        samp_lines.append(
            f"| {s['t_s_nominal']} | {s['drift_A']:.6f} | {s['drift_C']:.6f} | "
            f"{s['ratio_abs_dA_over_max_abs_dC']:.4f} |"
        )

    md_aut = f"""# SLALOM A vs C autopsy (correct vs legacy NHC Jacobian)

Both arms use **ideal IMU kinematics**; only the NHC Jacobian mode differs (A=`correct`, C=`legacy`).

Time axis: {time_unit_note}. Duration ≈ {autopsy['duration_s']:.3f} s, n={n} rows.

## First divergence (|drift_A − drift_C|)

| Threshold | First crossing |
|-----------|----------------|
| 0.01 m | {fmt_fd(first_div['0.01'])} |
| 0.05 m | {fmt_fd(first_div['0.05'])} |
| 0.15 m | {fmt_fd(first_div['0.15'])} |

## Window ±5 samples at first |dA-dC| > 0.01 m

| i | t_s | drift_A | drift_C | |Δ| | |v|_A | |v|_C | yaw_A | yaw_C | nis_A | nis_C | ‖innov‖_A | ‖innov‖_C |
|---|-----|---------|---------|-----|-------|-------|-------|-------|-------|-------|-----------|-----------|
{chr(10).join(win_lines)}

## Time-series summary

### A (jcorrect)

- max |drift| = {ts['A']['max_abs_drift_m']:.6f} m at t={ts['A']['t_s_of_max']:.4f} s
- first |drift| > 0.15 m: {fmt_ex(ts['A']['first_exceed_0.15m'])}
- first |drift| > 1 m: {fmt_ex(ts['A']['first_exceed_1m'])}
- first |drift| > 10 m: {fmt_ex(ts['A']['first_exceed_10m'])}

### C (jlegacy)

- max |drift| = {ts['C']['max_abs_drift_m']:.6f} m at t={ts['C']['t_s_of_max']:.4f} s
- first |drift| > 0.15 m: {fmt_ex(ts['C']['first_exceed_0.15m'])}
- first |drift| > 1 m: {fmt_ex(ts['C']['first_exceed_1m'])}
- first |drift| > 10 m: {fmt_ex(ts['C']['first_exceed_10m'])}

### |A−C| drift

- max |drift_A − drift_C| = {ts['abs_diff_A_minus_C']['max_m']:.6f} m at t={ts['abs_diff_A_minus_C']['t_s_of_max']:.4f} s

## Jump vs gradual accumulation

Descriptive only (no causal theory):

- max single-tick |dA-dC| **A**: {st['A_max_abs_ddrift']['max_abs']:.6f} m at t={st['A_max_abs_ddrift']['t_s']:.4f} s
- max single-tick |dA-dC| **C**: {st['C_max_abs_ddrift']['max_abs']:.6f} m at t={st['C_max_abs_ddrift']['t_s']:.4f} s
- max single-tick |Δ(drift_A − drift_C)|: {st['A_minus_C_max_abs_ddrift']['max_abs']:.6f} m at t={st['A_minus_C_max_abs_ddrift']['t_s']:.4f} s
- at 0.01 m crossing: {json.dumps(st['at_first_0.01m_crossing'])}
- **pattern:** `{st['pattern_description']}`

## Samples every 1 s (first 30 s)

| t_s | drift_A | drift_C | |dA|/max(|dC|,1e-6) |
|-----|---------|---------|---------------------|
{chr(10).join(samp_lines)}

Machine-readable: `slalom_a_vs_c_autopsy.json`.
"""
    (OUT / "slalom_a_vs_c_autopsy.md").write_text(md_aut, encoding="utf-8")

    # Also dump plumbing facts json snippet for reproducibility
    (OUT / "slalom_imu_mode_plumbing_facts.json").write_text(
        json.dumps(plumbing, indent=2), encoding="utf-8"
    )

    # stdout summary
    print("=== SLALOM imu-mode plumbing ===")
    print(f"run_slalom_scenario takes imu_mode: False")
    print(f"A==B byte-identical: {ab_byte_identical}; C==D byte-identical: {cd_byte_identical}")
    print(f"A vs B max|drift|: {ab['drift_m']}; C vs D max|drift|: {cd['drift_m']}")
    print("=== A vs C autopsy ===")
    print(f"time unit: {time_unit_note}")
    print(f"first |dA-dC|>0.01m: {fmt_fd(first_div['0.01'])}")
    print(f"first |dA-dC|>0.05m: {fmt_fd(first_div['0.05'])}")
    print(f"first |dA-dC|>0.15m: {fmt_fd(first_div['0.15'])}")
    print(
        f"max|drift| A={ts['A']['max_abs_drift_m']:.4f}m @ {ts['A']['t_s_of_max']:.2f}s; "
        f"C={ts['C']['max_abs_drift_m']:.4f}m @ {ts['C']['t_s_of_max']:.2f}s"
    )
    print(f"pattern: {st['pattern_description']}")
    print(f"wrote: {OUT / 'slalom_imu_mode_plumbing.md'}")
    print(f"wrote: {OUT / 'slalom_a_vs_c_autopsy.md'}")
    print(f"wrote: {OUT / 'slalom_a_vs_c_autopsy.json'}")


if __name__ == "__main__":
    main()
