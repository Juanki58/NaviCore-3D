#!/usr/bin/env python3
"""GAP-5 v2 / H6 — Caracterización de observables (passive / audit only).

Implementa el script previsto en docs/diagnostics/16-gap5-v2-observable-selection.md §10.
Bindings: docs/benchmarks/gap5_v2_observable_selection/{h6_series_binding,c7_labeling_binding}.md

Prohibido: ranking, score global, controlador, retune, RMSE como criterio.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "docs/benchmarks/gap5_v2_observable_selection"
CONFIGS = {
    "C-F1": REPO / "docs/benchmarks/gap5_adaptive_nhc/p0_passive_f1_bridge",
    "C-PoC": REPO / "docs/benchmarks/gap5_adaptive_nhc/p0_passive_validation",
}

PASO0_PRED = {
    "O1": {"R0": "bajo", "R1": "pico", "R2": "bajo", "R3": "unknown", "R4": "meseta"},
    "O2": {"R0": "bajo", "R1": "unknown", "R2": "unknown", "R3": "unknown", "R4": "meseta"},
    "O3": {"R0": "medio", "R1": "unknown", "R2": "alto", "R3": "pico", "R4": "bajo"},
    "O4": {"R0": "bajo", "R1": "bajo", "R2": "alto", "R3": "alto", "R4": "meseta"},
    "O5": {"R0": "bajo", "R1": "alto", "R2": "pico", "R3": "unknown", "R4": "bajo"},
}


def load_csv(path: Path) -> pd.DataFrame:
    if not path.is_file():
        return pd.DataFrame()
    df = pd.read_csv(path)
    for c in df.columns:
        if c in ("update_type", "phase", "reject_reason", "row_type"):
            continue
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def regime_windows(gnss: pd.DataFrame) -> dict[str, tuple[float, float] | None]:
    if gnss.empty or "accepted" not in gnss.columns:
        return {r: None for r in ("R0", "R1", "R2", "R3", "R4")}
    acc = gnss[gnss["accepted"] == 1].sort_values("gps_index")
    t_end = float(gnss["timestamp_s"].max()) if len(gnss) else 0.0
    if len(acc) < 2:
        return {r: None for r in ("R0", "R1", "R2", "R3", "R4")}
    t2 = float(acc.iloc[1]["timestamp_s"])
    t3 = float(acc.iloc[2]["timestamp_s"]) if len(acc) >= 3 else None
    t4 = float(acc.iloc[3]["timestamp_s"]) if len(acc) >= 4 else None
    t0 = float(gnss["timestamp_s"].min())
    windows: dict[str, tuple[float, float] | None] = {
        "R0": (t0, t2),
        "R1": (t2, t3) if t3 is not None else None,
        "R2": None,
        "R3": None,
        "R4": None,
    }
    if t3 is not None:
        if t4 is not None:
            windows["R2"] = (t3, t4)
            windows["R3"] = (t4, min(t4 + 10.0, t_end))
        else:
            windows["R2"] = (t3, min(t3 + 15.0, t_end))
            windows["R3"] = None
    r4_start = max(t2, 30.0)
    windows["R4"] = (r4_start, t_end) if (t_end - r4_start) >= 5.0 else None
    return windows


def ewma(series: pd.Series, t: pd.Series, tau: float = 1.0) -> np.ndarray:
    x = series.to_numpy(dtype=float)
    tt = t.to_numpy(dtype=float)
    out = np.full_like(x, np.nan)
    if len(x) == 0:
        return out
    out[0] = x[0]
    for i in range(1, len(x)):
        dt = max(tt[i] - tt[i - 1], 1e-6)
        a = dt / (tau + dt)
        prev = out[i - 1] if not math.isnan(out[i - 1]) else x[i]
        xi = x[i] if not math.isnan(x[i]) else prev
        out[i] = a * xi + (1 - a) * prev
    return out


def build_o1_o2(cov: pd.DataFrame, ctrl: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    if not ctrl.empty and "gamma_raw" in ctrl.columns:
        o1 = ctrl[["timestamp_s", "gamma_raw"]].rename(columns={"gamma_raw": "value"}).dropna()
        if "gamma_filtered" in ctrl.columns:
            o2 = ctrl[["timestamp_s", "gamma_filtered"]].rename(columns={"gamma_filtered": "value"}).dropna()
        else:
            o2 = o1.copy()
            o2["value"] = ewma(o1["value"], o1["timestamp_s"], 1.0)
        return o1, o2
    # reconstruct from cov_step
    rows = []
    if cov.empty:
        return pd.DataFrame(columns=["timestamp_s", "value"]), pd.DataFrame(columns=["timestamp_s", "value"])
    for imu_seq, tick in cov.groupby("imu_seq"):
        pred_pre = tick[(tick["update_type"] == "predict") & (tick["phase"] == "pre")]
        pred_post = tick[(tick["update_type"] == "predict") & (tick["phase"] == "post")]
        nhc_post = tick[(tick["update_type"] == "nhc") & (tick["phase"] == "post")]
        if pred_pre.empty:
            continue
        t = float(pred_pre.iloc[0]["timestamp_s"])
        p0 = float(pred_pre.iloc[0]["P_vv_frob"])
        d_pred = float(pred_post.iloc[0]["P_vv_frob"]) - p0 if not pred_post.empty else 0.0
        if not nhc_post.empty:
            p_ap = float(pred_post.iloc[0]["P_vv_frob"]) if not pred_post.empty else p0
            d_nhc = float(nhc_post.iloc[0]["P_vv_frob"]) - p_ap
        else:
            d_nhc = 0.0
        g = abs(d_nhc) / max(abs(d_pred), 1e-12)
        rows.append({"timestamp_s": t, "value": g})
    o1 = pd.DataFrame(rows)
    if o1.empty:
        return o1, o1.copy()
    o2 = o1.copy()
    o2["value"] = ewma(o1["value"], o1["timestamp_s"], 1.0)
    return o1, o2


def build_o3(cov: pd.DataFrame) -> pd.DataFrame:
    if cov.empty:
        return pd.DataFrame(columns=["timestamp_s", "value"])
    df = cov[(cov["P_vv_frob"] > 0)].copy()
    df["value"] = df["P_pv_frob"] / df["P_vv_frob"]
    return df[["timestamp_s", "value"]].dropna()


def build_o4_o5(gnss: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    if gnss.empty:
        empty = pd.DataFrame(columns=["timestamp_s", "value"])
        return empty, empty
    g = gnss.sort_values("timestamp_s").copy()
    g["value"] = g["innov_n_m"].abs() / np.sqrt(np.maximum(g["s_nn"].to_numpy(dtype=float), 1e-12))
    o4 = g[["timestamp_s", "value"]].dropna()
    if len(o4) < 2:
        return o4, pd.DataFrame(columns=["timestamp_s", "value"])
    dt = np.diff(o4["timestamp_s"].to_numpy(dtype=float))
    dlam = np.diff(o4["value"].to_numpy(dtype=float)) / np.maximum(dt, 1e-3)
    o5 = pd.DataFrame({"timestamp_s": o4["timestamp_s"].to_numpy()[1:], "value": dlam})
    return o4, o5


def slice_reg(df: pd.DataFrame, win: tuple[float, float] | None) -> pd.DataFrame:
    if win is None or df.empty:
        return pd.DataFrame(columns=df.columns)
    a, b = win
    return df[(df["timestamp_s"] >= a) & (df["timestamp_s"] < b)]


def regime_stats(df: pd.DataFrame, win: tuple[float, float] | None, b0: float) -> dict:
    if win is None:
        return {"n_samples": 0, "ordinal": "N/A"}
    seg = slice_reg(df, win)
    if seg.empty:
        return {"n_samples": 0, "median": None, "max": None, "p95": None, "t_at_max": None, "iqr": None, "baseline_median_R0": b0, "ordinal": "N/A"}
    v = seg["value"].to_numpy(dtype=float)
    t = seg["timestamp_s"].to_numpy(dtype=float)
    i_max = int(np.nanargmax(v))
    dur = win[1] - win[0]
    t_at = float(t[i_max] - win[0]) if dur > 0 else 0.0
    m = float(np.nanmedian(v))
    M = float(np.nanmax(v))
    p95 = float(np.nanpercentile(v, 95))
    iqr = float(np.nanpercentile(v, 75) - np.nanpercentile(v, 25))
    return {
        "n_samples": int(len(v)),
        "median": m,
        "max": M,
        "p95": p95,
        "t_at_max": t_at,
        "iqr": iqr,
        "baseline_median_R0": b0,
        "duration_s": float(dur),
    }


def assign_ordinal(st: dict, is_r0: bool) -> str:
    if st.get("n_samples", 0) == 0 or st.get("ordinal") == "N/A":
        return "N/A"
    m = st["median"]
    M = st["max"]
    b0 = st["baseline_median_R0"]
    iqr = st["iqr"]
    dur = st.get("duration_s") or 1.0
    t_at = st["t_at_max"]
    if b0 == 0 or b0 is None:
        b0 = max(1e-12, st.get("p95") or 1e-12)
    frac = t_at / dur if dur > 0 else 0.5
    in_central = 0.2 <= frac <= 0.8
    if M >= 2.0 * b0 and M >= 1.5 * m and in_central:
        return "pico"
    if m >= 1.5 * b0:
        return "alto"
    if iqr <= 0.35 * max(m, 1e-12) and (
        (0.7 * b0 <= m <= 1.3 * b0) or ((not is_r0) and (0.8 * m <= M <= 1.25 * m))
    ):
        return "meseta"
    if m <= 1.25 * b0:
        return "bajo"
    return "bajo" if m <= b0 else "alto"


def paso0_contrast(obs: str, reg: str, ordinal: str) -> str:
    pred = PASO0_PRED.get(obs, {}).get(reg, "unknown")
    if ordinal == "N/A" or pred in ("unknown", "?", "medio"):
        return "unknown"
    if pred == ordinal:
        return "match"
    if pred == "pico" and ordinal != "pico":
        return "flat_vs_peak"
    if pred != "pico" and ordinal == "pico":
        return "peak_vs_flat"
    order = {"bajo": 0, "meseta": 1, "alto": 2, "pico": 3}
    if pred in order and ordinal in order:
        return "higher" if order[ordinal] > order[pred] else "lower"
    return "unknown"


def characterize_config(name: str, path: Path) -> dict:
    cov = load_csv(path / "cov_step_audit.csv")
    gnss = load_csv(path / "gnss_nis_audit.csv")
    ctrl = load_csv(path / "controller_audit.csv")
    wins = regime_windows(gnss)
    o1, o2 = build_o1_o2(cov, ctrl)
    o3 = build_o3(cov)
    o4, o5 = build_o4_o5(gnss)
    series = {"O1": o1, "O2": o2, "O3": o3, "O4": o4, "O5": o5}
    out = {"config": name, "path": str(path.relative_to(REPO)), "regimes": {}, "observables": {}}
    # R0 baseline medians per O
    b0s = {}
    for oid, df in series.items():
        st0 = regime_stats(df, wins["R0"], b0=1.0)
        b0s[oid] = st0["median"] if st0.get("median") is not None else 1e-12
    for oid, df in series.items():
        o_block = {"c7": {}, "c7_ordinal": {}, "paso0_contrast": {}}
        for r, win in wins.items():
            st = regime_stats(df, win, b0s[oid])
            if st.get("n_samples", 0) == 0:
                st["ordinal"] = "N/A"
            else:
                st["ordinal"] = assign_ordinal(st, is_r0=(r == "R0"))
            o_block["c7"][r] = {k: v for k, v in st.items() if k != "ordinal"}
            o_block["c7_ordinal"][r] = st["ordinal"]
            o_block["paso0_contrast"][r] = paso0_contrast(oid, r, st["ordinal"])
        # C1–C6
        m_r0 = o_block["c7"].get("R0", {}).get("median") or b0s[oid]
        m_r1 = o_block["c7"].get("R1", {}).get("max")
        c1 = bool(m_r1 is not None and m_r0 is not None and m_r1 >= 1.5 * m_r0)
        ord_r2 = o_block["c7_ordinal"].get("R2", "N/A")
        pred_r2 = PASO0_PRED.get(oid, {}).get("R2", "unknown")
        c2 = ord_r2 != "N/A" and (
            pred_r2 in ("unknown", "?")
            or (pred_r2 in ("alto", "pico") and ord_r2 in ("alto", "pico"))
            or (pred_r2 == "bajo" and ord_r2 == "bajo")
            or (pred_r2 == ord_r2)
        )
        o_block["C"] = {
            "C1_R1_distinguishable": c1,
            "C2_R2_coherent_paso0": c2,
            "C4_memory_issue_vs_R1": None,
            "C5_causal": True,
            "C6_local": True,
        }
        if oid == "O2":
            max_o2 = o_block["c7"].get("R1", {}).get("max")
            max_o1 = None
            # filled later
            o_block["_max_o2_r1"] = max_o2
        out["observables"][oid] = o_block
    # C4 for O2 needs O1
    if "O2" in out["observables"] and "O1" in out["observables"]:
        max_o2 = out["observables"]["O2"]["c7"].get("R1", {}).get("max")
        max_o1 = out["observables"]["O1"]["c7"].get("R1", {}).get("max")
        if max_o1 and max_o2 is not None:
            out["observables"]["O2"]["C"]["C4_memory_issue_vs_R1"] = bool(max_o2 < 0.5 * max_o1)
        for oid in ("O1", "O3", "O4", "O5"):
            out["observables"][oid]["C"]["C4_memory_issue_vs_R1"] = False
    out["regimes"] = {
        r: {"t0": w[0], "t1": w[1]} if w else None for r, w in wins.items()
    }
    return out


def c3_invariance(cf1: dict, cpoc: dict) -> dict:
    res = {}
    for oid in ("O1", "O2", "O3", "O4", "O5"):
        a = cf1["observables"][oid]["c7_ordinal"]
        b = cpoc["observables"][oid]["c7_ordinal"]
        same = 0
        compared = 0
        for r in ("R1", "R2", "R3", "R4"):
            if a.get(r, "N/A") == "N/A" or b.get(r, "N/A") == "N/A":
                continue
            compared += 1
            if a[r] == b[r]:
                same += 1
        res[oid] = {
            "C3_meaning_preserved": bool(compared >= 3 and same >= 3),
            "same_regimes": same,
            "compared_regimes": compared,
        }
        cf1["observables"][oid]["C"]["C3_meaning_preserved"] = res[oid]["C3_meaning_preserved"]
        cpoc["observables"][oid]["C"]["C3_meaning_preserved"] = res[oid]["C3_meaning_preserved"]
    return res


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    results = {}
    for name, path in CONFIGS.items():
        if not path.is_dir():
            raise FileNotFoundError(path)
        results[name] = characterize_config(name, path)
    inv = c3_invariance(results["C-F1"], results["C-PoC"])
    report = {
        "phase": "GAP-5 v2 / H6 characterization",
        "protocol": "gap5-v2-observable-preregistration-v1.2",
        "bindings": {
            "series": "h6_series_binding.md",
            "c7_labels": "c7_labeling_binding.md",
        },
        "note": "Numeric stats first; ordinals via frozen binding. No ranking/score. Synthesis deferred to regime_model.md after review.",
        "configs": results,
        "C3_invariance_summary": inv,
        "prohibited_during_run": [
            "ranking",
            "global_score",
            "controller",
            "threshold_retune",
            "RMSE_as_selection",
            "new_observables",
            "axis_remapping",
        ],
    }

    def sanitize(o):
        if isinstance(o, dict):
            return {k: sanitize(v) for k, v in o.items() if not str(k).startswith("_")}
        if isinstance(o, list):
            return [sanitize(v) for v in o]
        if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
            return None
        return o

    report = sanitize(report)
    out_json = OUT / "observable_characterization.json"
    out_json.write_text(json.dumps(report, indent=2), encoding="utf-8")

    # Minimal md tables without declaring a winner
    lines = [
        "# Observable characterization (H6) — artefactos numéricos",
        "",
        "**Síntesis / modelo de régimen: diferida** — no hay ganador en este archivo.",
        "",
        "Protocolo v1.2 · bindings D18 · script `tools/audit_gap5_v2_observable_selection.py`",
        "",
    ]
    for cfg in ("C-F1", "C-PoC"):
        lines.append(f"## {cfg}")
        lines.append("")
        lines.append("| Oi | R0 | R1 | R2 | R3 | R4 | C1 | C3 |")
        lines.append("|----|----|----|----|----|----|----|----|")
        block = results[cfg]["observables"]
        for oid in ("O1", "O2", "O3", "O4", "O5"):
            ord_ = block[oid]["c7_ordinal"]
            c = block[oid]["C"]
            lines.append(
                f"| {oid} | {ord_.get('R0')} | {ord_.get('R1')} | {ord_.get('R2')} | "
                f"{ord_.get('R3')} | {ord_.get('R4')} | {c.get('C1_R1_distinguishable')} | "
                f"{c.get('C3_meaning_preserved')} |"
            )
        lines.append("")
    lines.append("## C3 invariance summary")
    lines.append("")
    lines.append("```json")
    lines.append(json.dumps(inv, indent=2))
    lines.append("```")
    lines.append("")
    lines.append("Parking lot ideas: `IDEAS_DURING_H6.md`.")
    lines.append("")
    (OUT / "observable_characterization.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {out_json}")
    print(f"Wrote {OUT / 'observable_characterization.md'}")
    print("STOP: do not write regime_model.md until characterization reviewed (protocol §9).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
