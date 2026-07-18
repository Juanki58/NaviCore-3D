# G-ext — Validación externa (run 19082026)

**Estado:** ejecutado (A/B/C) — 2026-07-18  
**Etiqueta:** `G-ext` (no es GAP-4 G2 / vel_only)  
**Dataset:** `data/real_run/19082026` (Metadata: `2026-07-17_19-19-30`)  
**Predecesor de referencia (no mezclar artefactos):** GAP-4 G1 en `docs/benchmarks/gap4_gnss_velocity/G1/`

### Resultado rápido

| Fase | Outcome |
|------|---------|
| A | OK — 677.12 s, IMU 66939 @ ~99 Hz, GPS 681, sync median ~2.6 ms; ventana GNSS limpia ~**506 s** |
| B | OK — shell G1 completa; **1 accept / 680 rejects** |
| C | Ver **[INTERPRETATION.md](INTERPRETATION.md)** (lectura conservadora normativa) |

**Claim sólido:** G-ext reproduce el **mecanismo de bloqueo** bajo recorrido independiente; **no** reproduce toda la secuencia causal de G1 (p.ej. no alcanza la región de bifurcación fix#4).

---

## Objetivo

Validar si el **modelo mecanicista** congelado en GAP-3/4 (NHC burst, compresión P_vv, crecimiento P_pv, innovación Norte / Λ_N, rechazos GNSS) reaparece en un recorrido **independiente**, con el **mismo EKF y la misma configuración G1**, sin controlador adaptativo ni intervención P_pv.

**No** es el benchmark GAP-5 v2.  
**No** se usa RMSE / drift como criterio de esta fase.

---

## Reglas

1. Artefactos **solo** bajo `docs/benchmarks/real_run_19082026_baseline/`.
2. No escribir en `gap4_gnss_velocity/G1/` ni reutilizar sus CSV como entrada.
3. Única variable deliberada: el **recorrido** (CSV de replay).
4. Configuración = shell G1 (§11.1 / `tools/run_gap4_arm.py --arm G1`):
   - `--gnss-obs-mode pos_vel`
   - `--p-pv-policy none`
   - `--constraint-policy disabled` (ZUPT OFF)
   - `--nhc-policy enabled` (N=1 default)
   - `--mount-mode calibration` + `calibration/imu_mount.json`
   - `--yaw-init zero` + `--h9a-gravity-tilt-init`
5. Instrumentación idéntica: audits GAP-3 GNSS NIS / NHC block / cov step / constraint pipeline / k-block JSONL.

---

## Fases

| Fase | Qué | Criterio de cierre |
|------|-----|-------------------|
| **A** | Smoke técnico: formato, conteos IMU/GNSS, timestamps/sync, replay ~677 s | `phase_a_report.json` OK |
| **B** | Replay pasivo completo G1-shell | audits + `gap4_g1_report.json` (etiqueta arm=G1, dataset=G-ext) |
| **C** | Comparación mecanicista vs G1 congelado | `phase_c_mechanistic_comparison.json` + tabla |

---

## Orquestador

```powershell
python tools\run_gext_19082026_baseline.py
# fases sueltas:
python tools\run_gext_19082026_baseline.py --phase A
python tools\run_gext_19082026_baseline.py --phase B
python tools\run_gext_19082026_baseline.py --phase C
```

---

## Entregables

| Archivo | Fase |
|---------|------|
| `PROTOCOL.md` | — |
| `real_run_replay.csv` | A (copia/enlace del parse) |
| `phase_a_report.json` | A |
| `replay_output.csv` | B |
| `gnss_nis_audit.csv` | B |
| `nhc_block_audit.csv` | B |
| `cov_step_audit.csv` | B |
| `constraint_pipeline_audit.csv` | B |
| `gnss_k_block.jsonl` | B |
| `gap4_g1_report.json` | B (mismo schema que G1; campo `dataset=G-ext`) |
| `phase_c_mechanistic_comparison.json` | C |
| `phase_c_comparison.md` | C |
