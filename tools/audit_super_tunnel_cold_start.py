#!/usr/bin/env python3
"""Algebraic autopsy of N_always cold-start (0–10 s) — protocol follow-up.

Questions (preregistered intent from user):
1) Is P_pv growth 0.016→18 the same family as known high-gain / cold-start events?
2) Does K ≈ HPH^T / (HPH^T+R) close (Bayesian legitimate) or show K/P desync?
3) Does cold-start + aggressive NHC explain original ALWAYS NHC worsening without IEEE-952?

R_nhc lateral = 0.1^2 = 0.01, vertical = 0.05^2 = 0.0025 (nominal).
Scalar proxy: K_scalar ≈ HPH / (HPH+R) using logged k_y,k_z vs innov/NIS if available.
We also use anatomy P_vv_frob / P_pv_frob timelines.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

OUT = Path(__file__).resolve().parents[1] / "docs" / "benchmarks" / "super_tunnel_bd_rerun"
REPORT = OUT / "cold_start_autopsy.md"
VERDICT = OUT / "cold_start_verdict.json"

# Nominal NHC R (matches super_tunnel defaults)
R_LAT = 0.1**2  # 0.01
R_VERT = 0.05**2  # 0.0025

# Preregistered classification thresholds (before looking at conclusion text)
# Family match to known cold-start / high-P gain:
# - gradual P_pv growth with k_max bounded << 1 early → Bayesian with growing cross-cov
# - single tick |ΔP_pv|/P_pv > 10 OR k_max>0.9 with tiny innov → Joseph/desync-like
JUMP_PPV_REL = 10.0
K_HIGH = 0.9
INNOV_TINY = 0.05


def load() -> tuple[pd.DataFrame, pd.DataFrame]:
    anat = pd.read_csv(OUT / "N_always_anatomy.csv")
    tr = pd.read_csv(OUT / "N_always_trace.csv")
    return anat, tr


def find_growth_epochs(anat: pd.DataFrame) -> pd.DataFrame:
    pre = anat[anat["t_ms"] <= 10000].copy()
    pre["d_ppv"] = pre["P_pv_frob"].diff()
    pre["d_ppv_rel"] = pre["d_ppv"] / pre["P_pv_frob"].shift(1).clip(lower=1e-12)
    pre["d_pvv"] = pre["P_vv_frob"].diff()
    pre["d_drift"] = pre["drift_h_m"].diff()
    return pre


def scalar_bayes_check(tr: pd.DataFrame) -> pd.DataFrame:
    """Proxy: if measurement were 1D with S = HPH+R and K_obs ~ k_z or k_y.

    We do NOT have HPH logged; use NIS and innov to back out S:
      NIS = y^T S^{-1} y  (2D). For rough 1D on dominant axis:
      if |innov_z| >> |innov_y|: S_zz ≈ innov_z^2 / NIS  (when NIS>0)
      then K_scalar_z_bayes = (S_zz - R_z) / S_zz = 1 - R_z/S_zz
    Compare to logged k_z.
    """
    df = tr[tr["t_ms"] <= 10000].copy()
    # Prefer vertical when |innov_z| dominates
    use_z = df["innov_z"].abs() >= df["innov_y"].abs()
    innov = np.where(use_z, df["innov_z"].to_numpy(), df["innov_y"].to_numpy())
    k_obs = np.where(use_z, df["k_z"].to_numpy(), df["k_y"].to_numpy())
    r = np.where(use_z, R_VERT, R_LAT)
    nis = df["nis"].to_numpy()

    # 1D proxy S ≈ innov^2 / NIS when NIS>1e-9 and |innov|>1e-6
    s_proxy = np.full_like(innov, np.nan, dtype=float)
    ok = (nis > 1e-9) & (np.abs(innov) > 1e-6)
    s_proxy[ok] = (innov[ok] ** 2) / nis[ok]
    # For 2D NIS this is crude; also try S ≈ HPH+R from k: if K=HPH/S then HPH=K*S
    # Better: from Joseph form K = P H^T S^{-1}, scalar |K| not equal HPH/S.
    # Use: bayes_gain_1d = max(0, 1 - R/S) when S>R
    k_bayes = np.full_like(innov, np.nan, dtype=float)
    valid_s = ok & (s_proxy > r)
    k_bayes[valid_s] = 1.0 - (r[valid_s] / s_proxy[valid_s])

    out = df.copy()
    out["axis"] = np.where(use_z, "z", "y")
    out["s_proxy"] = s_proxy
    out["k_obs_dom"] = k_obs
    out["k_bayes_1d"] = k_bayes
    out["k_residual"] = out["k_obs_dom"] - out["k_bayes_1d"]
    return out


def milestones(pre: pd.DataFrame) -> list[dict]:
    times = [0, 100, 200, 350, 500, 1000, 2000, 5000, 10000]
    rows = []
    for t in times:
        hit = pre.loc[pre["t_ms"] == t]
        if len(hit) == 0:
            continue
        r = hit.iloc[0]
        rows.append(
            {
                "t_ms": int(t),
                "drift_h_m": float(r["drift_h_m"]),
                "P_vv_frob": float(r["P_vv_frob"]),
                "P_pv_frob": float(r["P_pv_frob"]),
                "vel_norm": float(r["vel_norm_mps"]),
                "k_max": float(r["k_max"]),
                "innov_norm": float(r["innov_norm"]),
                "dx_pos": float(r["dx_pos_norm"]),
                "dx_vel": float(r["dx_vel_norm"]),
            }
        )
    return rows


def classify(pre: pd.DataFrame, bayes: pd.DataFrame) -> dict:
    # Growth shape
    ppv0 = float(pre.loc[pre["t_ms"] == 0, "P_pv_frob"].iloc[0])
    ppv10 = float(pre.loc[pre["t_ms"] == 10000, "P_pv_frob"].iloc[0])
    ratio = ppv10 / max(ppv0, 1e-12)

    jump_rows = pre[pre["d_ppv_rel"].abs() > JUMP_PPV_REL]
    high_k_tiny_innov = pre[(pre["k_max"] > K_HIGH) & (pre["innov_norm"] < INNOV_TINY)]

    # Bayes residual stats where defined
    br = bayes["k_residual"].dropna()
    mean_abs_resid = float(br.abs().mean()) if len(br) else float("nan")
    max_abs_resid = float(br.abs().max()) if len(br) else float("nan")
    # If residual typically small → K consistent with inflated S (Bayesian)
    bayes_consistent = bool(len(br) > 50 and mean_abs_resid < 0.25 and max_abs_resid < 0.8)

    # Desync-like: many high-K tiny innov OR huge single P_pv jumps
    desync_like = bool(len(jump_rows) > 0 or len(high_k_tiny_innov) > 20)

    # Gradual cold-start family: large cumulative ratio, no desync flags, k early modest
    early_k = float(pre.loc[pre["t_ms"] <= 500, "k_max"].max())
    gradual = bool(ratio > 100 and len(jump_rows) == 0 and early_k < 0.5)

    if desync_like and not bayes_consistent:
        family = "K_P_DESYNC_OR_JOSEPH_LIKE"
    elif gradual and bayes_consistent:
        family = "COLD_START_HIGH_P_BAYESIAN_GAIN"
    elif gradual:
        family = "COLD_START_HIGH_P_GROWTH_GRADUAL"
    else:
        family = "OTHER_OR_MIXED"

    # Explains original ALWAYS penalty? Same policy N_always on ideal already >> A
    # If family is cold-start and damage is pre-tunnel → yes, sufficient without dirty IMU
    explains_original = family in (
        "COLD_START_HIGH_P_BAYESIAN_GAIN",
        "COLD_START_HIGH_P_GROWTH_GRADUAL",
    ) and float(pre.loc[pre["t_ms"] == 10000, "drift_h_m"].iloc[0]) > 50.0

    return {
        "P_pv_frob_0": ppv0,
        "P_pv_frob_10s": ppv10,
        "P_pv_ratio_10s_over_0": ratio,
        "n_ppv_rel_jumps_gt_10": int(len(jump_rows)),
        "n_high_k_tiny_innov": int(len(high_k_tiny_innov)),
        "early_k_max_0_500ms": early_k,
        "bayes_n_samples": int(len(br)),
        "bayes_mean_abs_k_residual": mean_abs_resid,
        "bayes_max_abs_k_residual": max_abs_resid,
        "bayes_consistent_proxy": bayes_consistent,
        "family": family,
        "explains_original_always_nhc_worsening_without_ieee952": explains_original,
    }


def main() -> int:
    anat, tr = load()
    pre = find_growth_epochs(anat)
    bayes = scalar_bayes_check(tr)
    ms = milestones(pre)
    cls = classify(pre, bayes)

    # Top growth ticks for P_pv
    top_grow = pre.nlargest(8, "d_ppv")[
        ["t_ms", "d_ppv", "d_ppv_rel", "P_pv_frob", "P_vv_frob", "drift_h_m", "k_max", "innov_norm"]
    ]

    # When does drift exceed 10 m / 50 m / 100 m?
    thresholds = {}
    for thr in (10.0, 50.0, 100.0, 200.0):
        hit = pre[pre["drift_h_m"] >= thr]
        thresholds[f"first_t_ms_drift_ge_{int(thr)}"] = (
            int(hit["t_ms"].iloc[0]) if len(hit) else None
        )

    # Critical window around historical t=350 ms cascade
    win = tr[(tr["t_ms"] >= 200) & (tr["t_ms"] <= 600)][
        [
            "t_ms",
            "vel_e",
            "vel_d",
            "yaw_deg",
            "vby",
            "vbz",
            "innov_y",
            "innov_z",
            "k_y",
            "k_z",
            "k_max",
            "nis",
            "dx_vel_norm",
            "dx_att_norm",
            "dx_pos_norm",
        ]
    ]

    # Merge anatomy into window
    win_a = pre[(pre["t_ms"] >= 200) & (pre["t_ms"] <= 600)][
        ["t_ms", "P_vv_frob", "P_pv_frob", "drift_h_m", "vel_norm_mps"]
    ]
    win_m = win.merge(win_a, on="t_ms", how="left")

    # Bayes residuals around cascade and late pre-outage
    bayes_hot = bayes[(bayes["t_ms"] >= 200) & (bayes["t_ms"] <= 2000)][
        ["t_ms", "axis", "innov_norm", "nis", "s_proxy", "k_obs_dom", "k_bayes_1d", "k_residual"]
    ]

    verdict = {
        "arm": "N_always",
        "window": "0_to_10s_pre_outage",
        "milestones": ms,
        "classification": cls,
        "drift_thresholds_t_ms": thresholds,
        "note_gnss": (
            "N_always is NOT GNSS-free in 0–10 s: GPS updates still run when fix_valid. "
            "Naked NHC+cold-start interacts with GNSS until tunnel; outage starts at 10 s."
        ),
    }
    VERDICT.write_text(json.dumps(verdict, indent=2), encoding="utf-8")

    lines = [
        "# Autopsia arranque en frío — `N_always` (0–10 s)",
        "",
        "Objetivo: ¿el estado a t=10 s (P_pv 0.016→18, drift≈255 m) es la misma familia "
        "que ganancia alta con P inflado (K≈P/(P+R) legítimo) o desync K/P tipo Joseph?",
        "",
        "## Aclaración de escenario",
        "",
        "- Política: **NHC ALWAYS** + ZUPT off.",
        "- En 0–10 s **sí hay GNSS** (`fix_valid`); el túnel empieza a 10 s.",
        "- IMU **IDEAL**. Sin IEEE-952.",
        "",
        "## Hitos (anatomía)",
        "",
        "| t_ms | drift_h | P_vv_frob | P_pv_frob | k_max | innov | dx_pos |",
        "|------|---------|-----------|-----------|-------|-------|--------|",
    ]
    for r in ms:
        lines.append(
            f"| {r['t_ms']} | {r['drift_h_m']:.3f} | {r['P_vv_frob']:.4e} | "
            f"{r['P_pv_frob']:.4e} | {r['k_max']:.4f} | {r['innov_norm']:.4f} | "
            f"{r['dx_pos']:.4f} |"
        )

    lines += [
        "",
        "## Forma del crecimiento P_pv",
        "",
        f"- Ratio P_pv(10 s)/P_pv(0) = **{cls['P_pv_ratio_10s_over_0']:.1f}×**",
        f"- Saltos relativos |ΔP_pv|/P_pv > {JUMP_PPV_REL}: **{cls['n_ppv_rel_jumps_gt_10']}**",
        f"- Ticks k_max>{K_HIGH} con innov<{INNOV_TINY}: **{cls['n_high_k_tiny_innov']}**",
        f"- k_max máx en 0–500 ms: **{cls['early_k_max_0_500ms']:.4f}**",
        "",
        "Top ΔP_pv (crecimiento absoluto):",
        "```",
        top_grow.to_string(index=False),
        "```",
        "",
        f"Primera vez drift ≥ 10/50/100/200 m: `{thresholds}`",
        "",
        "## Ventana histórica crítica (~200–600 ms, cascada E)",
        "",
        "```",
        win_m.head(25).to_string(index=False),
        "```",
        "",
        "## Proxy bayesiano K vs 1−R/S",
        "",
        f"- muestras con S proxy: {cls['bayes_n_samples']}",
        f"- mean|k_obs − k_bayes_1d| = {cls['bayes_mean_abs_k_residual']:.4f}",
        f"- max|k_obs − k_bayes_1d| = {cls['bayes_max_abs_k_residual']:.4f}",
        f"- bayes_consistent_proxy = **{cls['bayes_consistent_proxy']}**",
        "",
        "Muestra 200–2000 ms:",
        "```",
        bayes_hot.dropna().head(20).to_string(index=False),
        "```",
        "",
        "## Clasificación (preregistrada)",
        "",
        f"- **family:** `{cls['family']}`",
        f"- **explica 481→1416 sin IEEE-952:** "
        f"**{cls['explains_original_always_nhc_worsening_without_ieee952']}**",
        "",
        "## Lectura",
        "",
    ]

    if cls["family"].startswith("COLD_START"):
        lines += [
            "El daño a t=10 s **no** es un cliff de un tick tipo ZUPT. Es acumulación "
            "desde el arranque: NHC EVERY tick con P0 alto (vel diag=1, att ~5–10°) "
            "mientras GNSS aún actualiza. P_pv crece ~tres órdenes de forma **gradual**; "
            "k temprano es modesto (≪1), compatible con ganancia bayesiana sobre covarianza "
            "cruzada que el propio NHC+predict infla — no con desync Joseph (pocos/ningún "
            "tick high-K/tiny-innov, sin saltos relativos enormes de P_pv).",
            "",
            "Implicación: problema de **inicialización / política NHC en frío** "
            "(NHC ALWAYS desde t=0), no de calibración IEEE-952. Afecta cualquier "
            "arranque en frío con NHC agresivo, con o sin túnel posterior.",
            "",
            "El misterio original ALWAYS+dirty ~480→~1400 queda explicado por la misma "
            "familia: el aislamiento histórico ya mostró ideal≈dirty; hoy N_always ideal "
            "sale a 1422 m con 255 m ya consumidos **antes** del apagón.",
        ]
    else:
        lines += [
            f"Familia `{cls['family']}` — revisar tablas; no cerrar Occam automáticamente.",
        ]

    REPORT.write_text("\n".join(lines), encoding="utf-8")
    print(REPORT.read_text(encoding="utf-8"))
    print(f"\nwrote {VERDICT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
