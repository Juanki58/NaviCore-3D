# Catálogo de experimentos

> **Proveniencia ZUPT:** Full-filter con replay por defecto (H3–H8, GAP-3.1–3.7) usó política `forced_time` hasta GAP-3.8. Esos resultados están **condicionados** — ver [11-replay-zupt-provenance.md](11-replay-zupt-provenance.md). Re-ejecutar con `--constraint-policy imu_stationary`.

Configuración común para bloque H9+: **predict-only 60 s**, `--h9a-gravity-tilt-init`, `--mount-calibration calibration/imu_mount.json`.

## Fase A — Montaje y trayectoria

### H0 / H1 — Montaje sensor → body

| | H0 | H1 |
|---|----|----|
| **Modo** | Euler legacy | `imu_mount.json` (Rodrigues) |
| **Script** | `run_mount_experiment.py` | idem |
| **RMSE horizontal** | 2311 m | 2351 m |
| **NIS medio** | 179 961 | 197 011 |
| **RMSE roll/pitch/yaw** | 144° / 12° / 98° | **6.6° / 4.4° / 102°** |

**Veredicto:** H1 corrige roll/pitch; no resuelve deriva horizontal ni yaw.

**Artefactos:** `docs/benchmarks/mount_experiment_report.json`, `real_run_output_h0.csv`, `real_run_output_h1.csv`

---

### H2 / H3 — Yaw init dinámico y comparativa

| Script | Propósito |
|--------|-----------|
| `run_yaw_init_experiment.py` | Inicialización yaw desde GPS bearing |
| `run_h3_experiment.py` | H0 vs H2 vs H3 con montaje Rodrigues |
| `analyze_yaw_init_timing.py` | Timing del primer fix GPS vs init |

**Veredicto H3:** Mejora marginal en NIS; RMSE horizontal ~2300 m en las tres variantes.

**Artefactos:** `h3_experiment_report.json`, `h3_trajectory_comparison.png`, `yaw_init_experiment_report.json`

---

## Fase B — Consistencia estadística del filtro

### H4 — NEES / NIS y barrido P0

| Script | `run_h4_experiment.py`, `run_h4_sweep.py` |
|--------|------------------------------------------|
| **Métrica clave** | `error/σ_h` medio ≈ **611**; NIS medio ≈ **197 000** |
| **NEES** | >99% muestras con NEES_n > 11 |

**Veredicto:** Sobreconfianza extrema del EKF; escalar P0 (2×–100×) no corrige la deriva de forma estructural.

**Artefactos:** `h4_experiment_report.json`, `h4_sweep_report.json`, `h4_consistency_*.csv`

---

### H5 — Sincronización y grid Q/R

| Script | Propósito |
|--------|-----------|
| `run_h5_sync_analysis.py` | Histograma Δt GPS–IMU |
| `run_h5_grid_search.py` | Grid ruido proceso Q × ruido NHC R |

**Veredicto:** Desincronización presente pero no explica sola km de deriva; grid no encuentra combinación Q/R que estabilice el filtro.

**Artefactos:** `h5_sync_audit.csv`, `h5_grid_report.json`

---

### H6 — Auditoría temporal completa

| Script | `run_h6_sync_audit.py` |
|--------|------------------------|
| **Muestras** | 331 GPS; 304 en fase móvil |

**Veredicto:** Refuerza H5; ratio innovación/velocidad coherente con problema de modelo, no solo de timestamp.

**Artefactos:** `h6_sync_audit_report.json`, `h6_sync_audit_analysis.png`

---

## Fase C — Geodesia y GNSS

### H7 — Cadena GNSS y primer fix

| Script | `run_h7_gnss_audit.py`, `run_h7_gnss_chain_audit.py` |
|--------|------------------------------------------------------|
| **Hallazgo** | Origen parse vs seed EKF (Barcelona placeholder) separados ~**58 km** |
| **Error geodésico** | Hasta ~13.6 m vs conversión WGS84 independiente (pre-fix) |

**Veredicto:** Error de datum/origen refutado como causa única tras geodesy estricta; primer rechazo GNSS ~t=13 s, innovación ~20 m.

**Artefactos:** `h7_gnss_chain_report.json`, `h7_gnss_audit_report.json`

---

### H7b — Origen NED unificado

| Script | `run_h7b_unified_origin.py` |
|--------|-------------------------------|
| **Acción** | Alinear origen EKF con primer fix `Location.csv` |

**Veredicto:** Elimina salto inicial por origen; no corrige leak de propagación.

**Artefactos:** `h7b_unified_origin_report.json`

---

## Fase D — Propagación inercial

### H8 — Auditoría magnitudes internas

| Script | `run_h8_propagation_audit.py` |
|--------|-------------------------------|
| **Fase estática ZUPT** | `a_lin,h` ≈ **0.52 m/s²** (con updates GNSS activos) |
| **Crucero** | `a_lin,h` ≈ **0.99 m/s²** |

**Veredicto:** Leak horizontal significativo incluso con cadena completa; motiva aislamiento predict-only.

**Artefactos:** `h8_propagation_audit_report.json`, `h8_propagation_audit.csv`

---

### H9 — Predict-only (60 s)

| Script | `run_h9_predict_only_isolation.py` |
|--------|-------------------------------------|
| **Config** | Sin GNSS, ZUPT, NHC |
| **`a_lin,h` medio** | **0.94 m/s²** |
| **Tilt implícito** | ~5.5° |

**Veredicto:** El leak **nace en predict** (R_bn, gravedad, montaje o bias en propagación).

**Artefactos:** `h9_predict_only_report.json`, `h9_predict_only_audit.csv`

---

## Fase E — Actitud (H9a–H9d)

Ver documento dedicado: [05-attitude-investigation.md](05-attitude-investigation.md).

| ID | Script | Pregunta |
|----|--------|----------|
| **H9a** | `run_h9a_gravity_init.py`, `run_h9a_gravity_alignment_audit.py` | ¿Init roll/pitch desde gravedad corrige tilt? |
| **H9.1** | `run_h9_1_tilt_diagnostic.py` | Diagnóstico observacional inclinación |
| **H9b** | `run_h9b_attitude_propagation_audit.py` | ¿Salto por integración giro vs paso gravedad? |
| **H9c** | `run_h9c_orientation_ref_audit.py` | ¿EKF vs Orientation o solo accel? |
| **H9d** | `run_h9d_gravity_subtraction_audit.py` | ¿Error antes o después de restar g? |
| **Chain** | `run_propagation_chain_audit.py` | Cadena completa a_raw → a_lin |
| **Heading** | `run_rb_forward_heading_audit.py` | `R_bn·e_x` vs GPS bearing |
| **Triada** | `run_gravity_triad_audit.py` | pred / ref / meas en body |
| **Referencias** | `audit_reference_chain.py` | Eslabones Sensor→EKF |
| **Convenciones** | `audit_attitude_conventions.py` | Coherencia quat/DCM + cadena |

### Flags replay relevantes (`NaviCore3D_Replay`)

```
--predict-only
--predict-only-end-s 60
--h9a-gravity-tilt-init
--mount-calibration calibration/imu_mount.json
--h9b-attitude-propagation-audit-csv <path>
--h9d-gravity-subtraction-audit-csv <path>
--propagation-chain-audit-csv <path>
--h9a-gravity-alignment-audit-csv <path>
```

---

## Fase F — Análisis transversal

| Script | Propósito |
|--------|-----------|
| `analyze_real_run.py` | Análisis GPS vs filtro, actitud vs Orientation |
| `audit_imu_chain.py` | Cadena IMU sin EKF; exporta `imu_mount.json` |
| `parse_mobile_log.py` | CSV Sensor Logger → replay unificado |
| `geodesy.py` | LLA↔NED WGS84 (réplica Python) |
| `run_gnss_innovation_audit.py` | Innovaciones GNSS detalladas |

---

## Mapa script → informe JSON

| Script | Informe principal |
|--------|---------------------|
| `run_h3_experiment.py` | `h3_experiment_report.json` |
| `run_h4_experiment.py` | `h4_experiment_report.json` |
| `run_h5_grid_search.py` | `h5_grid_report.json` |
| `run_h6_sync_audit.py` | `h6_sync_audit_report.json` |
| `run_h7_gnss_chain_audit.py` | `h7_gnss_chain_report.json` |
| `run_h8_propagation_audit.py` | `h8_propagation_audit_report.json` |
| `run_h9_predict_only_isolation.py` | `h9_predict_only_report.json` |
| `run_h9b_attitude_propagation_audit.py` | `h9b_attitude_propagation_report.json` |
| `run_h9c_orientation_ref_audit.py` | `h9c_orientation_ref_report.json` |
| `run_h9d_gravity_subtraction_audit.py` | `h9d_gravity_subtraction_report.json` |
| `run_propagation_chain_audit.py` | `propagation_chain_audit_report.json` |
| `run_rb_forward_heading_audit.py` | `rb_forward_heading_report.json` |
| `run_gravity_triad_audit.py` | `gravity_triad_report.json` |
| `audit_reference_chain.py` | `reference_chain_audit.json` |
| `audit_attitude_conventions.py` | `attitude_convention_audit.json` |
