# Proveniencia experimental — política ZUPT del replay (H9 → GAP-3.7)

**Estado:** vigente  
**Fecha:** 2026-07-18  
**Relacionado:** [10-gap3-ins-model-audit.md](10-gap3-ins-model-audit.md) §8.9–8.10, GAP-3.8 matriz A–E

---

## 1. Hecho descubierto

Entre las revisiones **H9** y **GAP-3.7**, el replay (`real_run_replay.cpp`) aplicaba ZUPT con política **`forced_time`**:

```
ZUPT armado  ⇔  (t ≤ static_phase_end_s, default 30 s)  OR  (gps_speed ≤ 0.1 m/s)
```

**No** usaba criterio IMU (‖a‖≈g, ‖ω‖ bajo). Durante arranque con GPS 2–6 m/s, ZUPT seguía activo **747 ticks IMU** (GAP-3.7).

Esto **anula o comprime** `v_nominal`, colapsa `P_vv`, y condiciona NIS/innovación GNSS, aceptaciones y cualquier conclusión sobre acoplamiento posición→velocidad.

---

## 2. Regla de validez (obligatoria al citar resultados)

> **Todos los experimentos full-filter ejecutados con la política legacy `forced_time` deben considerarse *condicionados* por ese mecanismo** hasta repetirse con estacionariedad basada **exclusivamente en IMU** (`--constraint-policy imu_stationary`).

No invalida automáticamente la *existencia* del hallazgo (p. ej. que ZUPT espurio ocurría), pero **sí** invalida interpretar métricas de esos runs como comportamiento del vehículo o del filtro bajo constraints físicas.

---

## 3. Clasificación de campañas

### 3.1 Condicionados — re-ejecutar con `imu_stationary`

| Campaña | Artefactos típicos | Notas |
|---------|-------------------|--------|
| **H3–H8** full filter | `h*_*.csv`, `h*_report.json` | Replay sin flag de política → `forced_time` implícito |
| **H9** comparativas full filter | p. ej. `gap3_full_filter_*.csv` | Contraste vs predict-only en GAP-3 sigue siendo útil, pero rama FF condicionada |
| **GAP-3.1 – GAP-3.7** | `gap3_*` (NIS, K-block, cov, Pregunta A) | Incluye veredictos sobre `v_nominal`, `P_vv`, NIS |
| **GAP-3.8 Exp A** | `constraint_matrix/A/` | Reproducción explícita del baseline legacy |
| Cualquier replay **sin** `--constraint-policy` antes de GAP-3.8 | `real_run_output*.csv`, `docs/benchmarks/*` | Default histórico = `forced_time` |
| Invocaciones actuales **sin** el flag | N/A | Fallan en CLI (flag obligatorio desde higiene Jul 2026) |

### 3.2 No condicionados por ZUPT legacy (o exentos)

| Campaña | Motivo |
|---------|--------|
| **H9 predict-only** (`--predict-only`) | Sin GNSS update, sin ZUPT/NHC |
| **GAP-2 / pruebas A–D** kinemáticas | No pasan por ciclo constraints replay |
| **GAP-3.8 Exp B–E** | Política declarada (`disabled`, `gps_stop`, `imu_stationary`) |
| **Tunnel / slalom sintéticos** | Lógica de constraints propia del escenario |

### 3.3 Parcialmente válidos (interpretación acotada)

| Campaña | Uso permitido | Uso no permitido |
|---------|---------------|------------------|
| GAP-3.7 / 3.8 | Demostrar **mecanismo** ZUPT espurio y causalidad A vs B | Cuantificar rendimiento operacional del filtro |
| GAP-3.5 / 3.6 | Estructura algebraica de F, H, Joseph bajo ciclo observado | Generalizar a producción sin re-run IMU |

---

## 4. Política de re-ejecución

**Baseline obligatorio para repetir campaña condicionada:**

```bash
NaviCore3D_Replay.exe ... \
  --constraint-policy imu_stationary \
  --nhc-policy enabled \
  --imu-stationary-accel-dev-mps2 0.5 \
  --imu-stationary-gyro-radps 0.05
```

Ajustar umbrales IMU solo con registro explícito en el informe.

**No usar** `forced_time` ni `auto` (alias legacy) para nuevos experimentos estructurales (GNSS+velocidad, sweeps P/Q/R, etc.).

**CLI (higiene, Jul 2026):** `NaviCore3D_Replay` **exige** `--constraint-policy`. No hay default: omitir el flag o pasar `auto` termina con error. `forced_time` solo si se pide por nombre (reproducción legacy intencional).

**Arquitectura (ZUPT + NHC):** ver [17-conditional-constraints-architecture.md](17-conditional-constraints-architecture.md) — un solo principio: disparo por estado del sistema, no por reloj ni ALWAYS.

---

## 5. Metadatos en informes

Al generar o actualizar un `*_report.json` de campaña condicionada, incluir:

```json
"experiment_provenance": {
  "zupt_policy": "forced_time",
  "validity": "CONDITIONED",
  "rerun_required_policy": "imu_stationary",
  "note": "Replay legacy t<=30s OR gps_speed<=0.1 m/s; ver docs/diagnostics/11-replay-zupt-provenance.md"
}
```

Los informes GAP-3.8 ya distinguen `constraint_policy` por experimento.

---

## 6. Lectura post GAP-3.8 (matriz A–E)

| Comparación | Qué demuestra |
|-------------|---------------|
| **A → B** | ZUPT `forced_time` causa `v_nominal` baja (causalidad controlada) |
| **B → E** | NHC causa colapso aceptación GNSS (7 → 56); ZUPT off **no** basta |
| **C vs D** | Detectores distintos → innov distinta; requiere log temporal antes de elegir baseline |

**Ranking de prioridades (Jul 2026):**

1. ~~Retirar `forced_time`~~ ✓  
2. **Auditar bloque NHC** (GAP-3.9) — misma profundidad que GNSS  
3. Re-run campañas condicionadas con `imu_stationary`  
4. GNSS+velocidad — **después** de entender NHC  

---

## 7. Historial

| Versión | Fecha | Notas |
|---------|-------|-------|
| 1.0 | 2026-07-18 | Regla de validez post GAP-3.7; clasificación H9→GAP-3.7 |
| 1.1 | 2026-07-18 | Post GAP-3.8: NHC domina GNSS accepts; prioridad GAP-3.9 |
| 1.2 | 2026-07-18 | CLI: `--constraint-policy` obligatorio; `auto` retirado |
