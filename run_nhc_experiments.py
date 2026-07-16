#!/usr/bin/env python3
"""Sintonia R_nhc asimetrico + politica G en SUPER_TUNNEL."""

from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
BUILD_DIR = REPO_ROOT / "build"
SIM = BUILD_DIR / "NaviCore3D_Sim.exe"
EXP_DIR = REPO_ROOT / "docs" / "nhc_experiments"
MANIFEST_PATH = EXP_DIR / "manifest.json"

KNOWN_EXPERIMENTS = (
    "G_l01_v05",
    "G_l01_v10",
    "G_l02_v05",
    "G_l02_v10",
    "G_l10_v05",
    "G_l10_v10",
)

BASELINE_DRIFT_M = 493.0

BIAS_MEAN_THRESHOLD_MPS = 0.05
SIGN_DOMINANCE_FRAC = 0.70
REQUIRED_COLUMNS = (
    "gps_outage",
    "constant_vel",
    "innov_y",
    "innov_z",
    "innov_norm",
    "k_max",
    "dx_att_y",
)
OPTIONAL_COLUMNS = ("k_y", "k_z", "nis", "vby", "vbz")


@dataclass
class WindowStats:
    experiment_id: str
    trace_path: str
    n_samples: int
    window: str
    mean_innov_y: float
    mean_innov_z: float
    std_innov_y: float
    std_innov_z: float
    frac_innov_y_positive: float
    frac_dx_att_y_same_sign_as_innov_y: float
    mean_k_max: float
    innov_norm_max: float


@dataclass
class SignPersistence:
    experiment_id: str
    frac_innov_y_positive: float
    frac_dx_att_y_positive: float
    frac_same_sign_innov_dx: float


@dataclass
class TripartiteVerdict:
    code: int
    label: str
    rationale: str


def load_summary_json(exp_id: str) -> dict | None:
    path = EXP_DIR / f"{exp_id}_summary.json"
    if not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def print_summary_from_json(exp_id: str, summary: dict) -> None:
    for key, label in (
        ("summary_all", "toda la corrida"),
        ("summary_outage_const_vel", "apagon + vel.const"),
    ):
        block = summary.get(key)
        if not block:
            continue
        print(f"\n  {exp_id} [{label}] n={block.get('sample_count', 0)}")
        print(f"    media(innov_y)={block.get('mean_innov_y_mps', 0):+.6f}  "
              f"media(innov_z)={block.get('mean_innov_z_mps', 0):+.6f}")
        print(f"    std(innov_y)={block.get('std_innov_y_mps', 0):.6f}  "
              f"std(innov_z)={block.get('std_innov_z_mps', 0):.6f}")
        print(f"    media(v_body_y)={block.get('mean_v_body_y_mps', 0):+.6f}  "
              f"media(v_body_z)={block.get('mean_v_body_z_mps', 0):+.6f}")
        print(f"    media(K_y)={block.get('mean_k_y', 0):.4f}  "
              f"media(K_z)={block.get('mean_k_z', 0):.4f}")
        print(f"    mismo signo={block.get('frac_same_sign_corr', 0):.1%}  "
              f"NIS medio={block.get('mean_nis', 0):.3f}  "
              f"NIS max={block.get('max_nis', 0):.3f}")


def load_trace(path: Path):
    import pandas as pd

    df = pd.read_csv(path)
    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise KeyError(f"{path.name}: columnas faltantes {missing}")
    return df


def window_constant_vel_outage(df):
    return df[(df["gps_outage"] == 1) & (df["constant_vel"] == 1)]


def window_outage_all(df):
    return df[df["gps_outage"] == 1]


def window_pre_gps(df):
    return df[df["gps_outage"] == 0]


def sign_persistence(df) -> SignPersistence | None:
    if df.empty:
        return None
    innov_pos = (df["innov_y"] > 0.0).mean()
    dx_pos = (df["dx_att_y"] > 0.0).mean()
    same = ((df["innov_y"] * df["dx_att_y"]) > 0.0).mean()
    return SignPersistence(
        experiment_id="",
        frac_innov_y_positive=float(innov_pos),
        frac_dx_att_y_positive=float(dx_pos),
        frac_same_sign_innov_dx=float(same),
    )


def analyze_trace(experiment_id: str, path: Path) -> list[WindowStats]:
    try:
        df = load_trace(path)
    except KeyError as exc:
        print(f"  (omitida {path.name}: {exc})")
        return []

    stats: list[WindowStats] = []
    for window_name, subset in (
        ("outage_const_vel", window_constant_vel_outage(df)),
        ("outage_all", window_outage_all(df)),
        ("pre_gps", window_pre_gps(df)),
    ):
        if subset.empty:
            continue

        innov_y = subset["innov_y"]
        innov_z = subset["innov_z"]
        same_sign = ((subset["innov_y"] * subset["dx_att_y"]) > 0.0).mean()

        stats.append(
            WindowStats(
                experiment_id=experiment_id,
                trace_path=str(path.relative_to(REPO_ROOT)),
                n_samples=int(len(subset)),
                window=window_name,
                mean_innov_y=float(innov_y.mean()),
                mean_innov_z=float(innov_z.mean()),
                std_innov_y=float(innov_y.std()),
                std_innov_z=float(innov_z.std()),
                frac_innov_y_positive=float((innov_y > 0.0).mean()),
                frac_dx_att_y_same_sign_as_innov_y=float(same_sign),
                mean_k_max=float(subset["k_max"].mean()),
                innov_norm_max=float(subset["innov_norm"].max()),
            )
        )

    return stats


def cv_stats(all_stats: list[WindowStats], exp_id: str) -> WindowStats | None:
    for s in all_stats:
        if s.experiment_id == exp_id and s.window == "outage_const_vel":
            return s
    return None


def pre_gps_stats(all_stats: list[WindowStats], exp_id: str) -> WindowStats | None:
    for s in all_stats:
        if s.experiment_id == exp_id and s.window == "pre_gps":
            return s
    return None


def is_mean_near_zero(stats: WindowStats | None, lateral_only: bool = False) -> bool:
    if stats is None:
        return False
    if lateral_only:
        return abs(stats.mean_innov_y) < BIAS_MEAN_THRESHOLD_MPS
    return (
        abs(stats.mean_innov_y) < BIAS_MEAN_THRESHOLD_MPS
        and abs(stats.mean_innov_z) < BIAS_MEAN_THRESHOLD_MPS
    )


def sustained_sign(stats: WindowStats | None) -> bool:
    if stats is None:
        return False
    dom = max(stats.frac_innov_y_positive, 1.0 - stats.frac_innov_y_positive)
    return dom >= SIGN_DOMINANCE_FRAC and not is_mean_near_zero(stats, lateral_only=True)


def tripartite_verdict(
    b_cv: WindowStats | None,
    b_always_cv: WindowStats | None,
    b_dirty_cv: WindowStats | None,
    c_r1_drift: float | None,
    c_r10_drift: float | None,
    ref_drift: float | None,
    c_r1_sign: SignPersistence | None,
    c_r10_sign: SignPersistence | None,
    pre_gps: WindowStats | None,
) -> TripartiteVerdict:
    """Uno de tres diagnosticos concretos (no intuicion)."""
    c_fixes_drift = (
        c_r1_drift is not None
        and c_r10_drift is not None
        and c_r10_drift < c_r1_drift * 0.70
    )
    c_below_ref = (
        c_r10_drift is not None
        and ref_drift is not None
        and c_r10_drift <= ref_drift * 1.05
    )

    sign_persists_after_r = False
    if c_r1_sign and c_r10_sign:
        dom_r1 = max(
            c_r1_sign.frac_innov_y_positive,
            1.0 - c_r1_sign.frac_innov_y_positive,
        )
        dom_r10 = max(
            c_r10_sign.frac_innov_y_positive,
            1.0 - c_r10_sign.frac_innov_y_positive,
        )
        same_dominant = (
            (c_r1_sign.frac_innov_y_positive >= 0.5)
            == (c_r10_sign.frac_innov_y_positive >= 0.5)
        )
        sign_persists_after_r = dom_r1 >= 0.85 and dom_r10 >= 0.85 and same_dominant

    # 1) Media_y~0 en tunel, var alta, C arregla -> sobreconfianza real (re-tuning R)
    if (
        is_mean_near_zero(b_cv, lateral_only=True)
        and b_cv is not None
        and b_cv.std_innov_y > BIAS_MEAN_THRESHOLD_MPS
        and c_fixes_drift
        and not sign_persists_after_r
        and not (pre_gps is not None and pre_gps.innov_norm_max > 2.0)
    ):
        return TripartiteVerdict(
            1,
            "sobreconfianza real (ruido)",
            "Media innov_y~0 en tunel vel.const; inflar R reduce deriva sin sesgo sostenido",
        )

    # 2) Media!=0 persiste sin aceleracion (B vel.const) -> montaje fijo
    b_has_bias = b_cv is not None and not is_mean_near_zero(b_cv, lateral_only=True)
    b_always_worse = (
        b_always_cv is not None
        and b_cv is not None
        and abs(b_always_cv.mean_innov_y - b_cv.mean_innov_y) < BIAS_MEAN_THRESHOLD_MPS
    )
    dirty_same = (
        b_dirty_cv is not None
        and b_cv is not None
        and abs(b_dirty_cv.mean_innov_y - b_cv.mean_innov_y) < BIAS_MEAN_THRESHOLD_MPS
    )

    if b_has_bias and sustained_sign(b_cv) and (b_always_worse or dirty_same):
        detail = (
            f"media_y tunel={b_cv.mean_innov_y:.4f} m/s con signo sostenido; "
            "persiste en B (sin accel) y B_dirty (IEEE-952)"
        )
        if c_fixes_drift and sign_persists_after_r:
            detail += (
                "; R*10 baja deriva pero signo innov_y igual -> "
                "filtro confiado en medida sesgada"
            )
        return TripartiteVerdict(2, "desalineacion montaje fija / sesgo GNSS", detail)

    # 3) NHC+GNSS pre-apagon, sesgo accel-dependiente o IMU sucia
    pre_gps_damage = pre_gps is not None and pre_gps.innov_norm_max > 2.0
    accel_linked = (
        b_always_cv is not None
        and b_cv is not None
        and abs(b_always_cv.mean_innov_y - b_cv.mean_innov_y) >= BIAS_MEAN_THRESHOLD_MPS
    )
    dirty_bias = (
        b_dirty_cv is not None
        and b_cv is not None
        and abs(b_dirty_cv.mean_innov_y - b_cv.mean_innov_y) >= BIAS_MEAN_THRESHOLD_MPS
    )
    if pre_gps_damage or accel_linked or dirty_bias:
        detail = []
        if pre_gps_damage:
            detail.append(
                f"pico pre-GPS ||innov||={pre_gps.innov_norm_max:.2f} m/s (NHC+GNSS)"
            )
        if dirty_bias and b_dirty_cv is not None and b_cv is not None:
            detail.append(
                f"B_dirty sesgo_y={b_dirty_cv.mean_innov_y:+.4f} vs B={b_cv.mean_innov_y:+.4f}"
            )
        if c_fixes_drift and c_below_ref:
            detail.append("R*10 mejora deriva pero no corrige origen pre-apagon")
        return TripartiteVerdict(
            3,
            "NHC+GNSS pre-apagon / factor escala IMU sucia",
            "; ".join(detail) if detail else "Desactivar NHC con fix_valid o compensar modelo",
        )

    if c_fixes_drift and sign_persists_after_r:
        return TripartiteVerdict(
            2,
            "filtro confiado en medida sesgada",
            "R*10 mejora numero pero innov_y mantiene signo -> no basta re-tuning R",
        )

    return TripartiteVerdict(
        0,
        "inconcluso",
        "Revisar trazas pre_gps y comparar B vs B_always manualmente",
    )


def parse_sim_output(stdout: str) -> dict[str, dict[str, float]]:
    results: dict[str, dict[str, float]] = {}
    for line in stdout.splitlines():
        if "drift_exit=" not in line:
            continue
        parts = line.strip().split()
        if len(parts) < 2:
            continue
        exp_id = parts[0].rstrip(":")
        if not exp_id:
            continue
        try:
            bucket = results.setdefault(exp_id, {})
            for i, token in enumerate(parts):
                if token == "drift_exit=" and i + 1 < len(parts):
                    bucket["drift_exit_m"] = float(parts[i + 1])
                elif token == "drift_final=" and i + 1 < len(parts):
                    bucket["drift_final_m"] = float(parts[i + 1])
                elif token.startswith("drift_exit=") and len(token) > 11:
                    bucket["drift_exit_m"] = float(token.split("=", 1)[1])
                elif token.startswith("drift_final=") and len(token) > 11:
                    bucket["drift_final_m"] = float(token.split("=", 1)[1])
        except ValueError:
            continue
    return results


def print_sweet_spot_table(
    drift_by_exp: dict[str, dict[str, float]],
    summaries: dict[str, dict],
) -> str | None:
    ref = drift_by_exp.get("A", {}).get("drift_exit_m", BASELINE_DRIFT_M)
    print(f"\n=== Punto dulce R_nhc (politica G) vs baseline A={ref:.0f} m ===\n")
    print(
        f"{'exp':<12} {'drift':>7} {'vs A':>7} "
        f"{'mean_vby':>9} {'mean_vbz':>9} {'NIS max':>8} {'K_y':>6} {'K_z':>6}"
    )
    print("-" * 72)

    best_id: str | None = None
    best_drift = float("inf")

    for exp_id in KNOWN_EXPERIMENTS:
        drift = drift_by_exp.get(exp_id, {}).get("drift_exit_m")
        if drift is None:
            continue
        summary = summaries.get(exp_id, {})
        win = summary.get("summary_outage_const_vel") or summary.get("summary_all") or {}
        vby = win.get("mean_v_body_y_mps", float("nan"))
        vbz = win.get("mean_v_body_z_mps", float("nan"))
        nis_max = win.get("max_nis", float("nan"))
        ky = win.get("mean_k_y", float("nan"))
        kz = win.get("mean_k_z", float("nan"))
        delta = drift - ref
        beats = "OK" if drift < ref else ""
        print(
            f"{exp_id:<12} {drift:7.1f} {delta:+7.1f} "
            f"{vby:+9.4f} {vbz:+9.4f} {nis_max:8.2f} {ky:6.3f} {kz:6.3f} {beats}"
        )
        if drift < best_drift:
            best_drift = drift
            best_id = exp_id

    print()
    if best_id:
        if best_drift < ref:
            print(
                f"  Mejor: {best_id} con {best_drift:.1f} m "
                f"({best_drift - ref:+.1f} m vs A) - supera baseline sin NHC"
            )
        else:
            print(
                f"  Mejor: {best_id} con {best_drift:.1f} m "
                f"({best_drift - ref:+.1f} m vs A) - aun por encima de baseline"
            )
    return best_id


def print_stats_table(all_stats: list[WindowStats]) -> None:
    print("\n=== Harness D - ventana larga (apagon + velocidad constante) ===\n")
    print(
        f"{'exp':<12} {'n':>5} {'mean_y':>9} {'mean_z':>9} {'std_y':>9} "
        f"{'sign_y+':>7} {'dx~y':>6} {'k_mean':>7}"
    )
    print("-" * 72)
    for s in all_stats:
        if s.window != "outage_const_vel":
            continue
        print(
            f"{s.experiment_id:<12} {s.n_samples:5d} "
            f"{s.mean_innov_y:9.4f} {s.mean_innov_z:9.4f} {s.std_innov_y:9.4f} "
            f"{s.frac_innov_y_positive:7.1%} "
            f"{s.frac_dx_att_y_same_sign_as_innov_y:6.1%} "
            f"{s.mean_k_max:7.3f}"
        )


def main() -> int:
    if not SIM.is_file():
        print(f"Error: compila primero ({SIM})", file=sys.stderr)
        return 1

    try:
        import pandas as pd  # noqa: F401
    except ImportError:
        print("Error: pip install pandas", file=sys.stderr)
        return 1

    proc = subprocess.run(
        [str(SIM), "--nhc-experiments"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        errors="replace",
    )
    print(proc.stdout)
    if proc.returncode != 0:
        print(proc.stderr, file=sys.stderr)
        return proc.returncode

    drift_by_exp = parse_sim_output(proc.stdout)

    all_stats: list[WindowStats] = []
    sign_by_exp: dict[str, SignPersistence] = {}

    for exp_id in KNOWN_EXPERIMENTS:
        trace_path = EXP_DIR / f"{exp_id}_trace.csv"
        if not trace_path.is_file():
            print(f"  (sin traza {trace_path.name})")
            continue
        all_stats.extend(analyze_trace(exp_id, trace_path))
        try:
            df = load_trace(trace_path)
            sp = sign_persistence(window_constant_vel_outage(df))
            if sp is not None:
                sp.experiment_id = exp_id
                sign_by_exp[exp_id] = sp
        except KeyError:
            pass

    print_stats_table(all_stats)

    print("\n=== Resumenes JSON (sin abrir CSV) ===")
    summaries: dict[str, dict] = {}
    for exp_id in KNOWN_EXPERIMENTS:
        summary = load_summary_json(exp_id)
        if summary is None:
            continue
        summaries[exp_id] = summary
        print_summary_from_json(exp_id, summary)

    best_id = print_sweet_spot_table(drift_by_exp, summaries)

    manifest = {
        "baseline_drift_m": drift_by_exp.get("A", {}).get("drift_exit_m", BASELINE_DRIFT_M),
        "best_experiment": best_id,
        "drift_by_experiment": drift_by_exp,
        "summaries": summaries,
        "window_stats": [asdict(s) for s in all_stats],
        "sign_persistence": {k: asdict(v) for k, v in sign_by_exp.items()},
    }
    EXP_DIR.mkdir(parents=True, exist_ok=True)
    MANIFEST_PATH.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"\nManifest: {MANIFEST_PATH.relative_to(REPO_ROOT)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
