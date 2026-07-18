# Auditoría de conformidad — predict() vs Body Frame Contract

**Referencia normativa:** [08-body-frame-contract.md](08-body-frame-contract.md)  
**Dataset:** Patrón Oro (`data/real_run/`)  
**Informe machine-readable:** `docs/benchmarks/body_frame_conformance_audit.json`  
**Herramienta:** `tools/audit_body_frame_conformance.py`

---

## 1. Veredicto ejecutivo

| Régimen | ¿Conforme con §6? | Evidencia |
|---------|:-----------------:|---------|
| **Estático (0–2 s)** | **Sí** | I1–I3 PASS; mount Z alineado |
| **Dinámico (2–10 s+)** | **No** | I2–I3 FAIL: `\|a_nav_pre_g\|_h` median ≈ 0.43–0.74 m/s² |
| **Sintaxis de código** | **Sí** | Orden, marcos y signos coherentes con §4 |
| **Calibración vs contrato** | **Conforme** | `R_mount` cumple Z↓; **+X forward verificado** con yaw coherente (GAP-1 cerrado) |

**Conclusión:** la implementación **respeta el contrato en reposo** y **viola invariantes I2/I3 en dinámica**. No se detecta bug de “matriz al revés” ni doble mount. La brecha principal entre **contrato normativo** (body = vehículo FRD) y **calibración actual** (Rodrigues → `body Z+ = g`) queda documentada en §5.

---

## 2. Auditoría `real_run_replay.cpp` → `predict()`

### 2.1 Replay (pre-predict)

| Línea / bloque | Variable | Contrato §4 | Conforme | Notas |
|----------------|----------|-------------|:--------:|-------|
| `parse_mobile_log` → CSV | `row.accel` | Device, specific force | ✅ | `AccelerometerUncalibrated.csv` |
| `mat3_vec3_mul(mount, row.accel, aligned_accel)` | `aligned_accel` | Body, specific force | ⚠️ | **Una** aplicación de `R_mount`; Z alineado; X/Y no auditados vs vehículo |
| idem gyro | `aligned_gyro` | Body, ω | ⚠️ | Mismo marco que accel |
| `filter->predict(dt, aligned_accel, aligned_gyro)` | entrada EKF | Body | ✅ | Sin re-mount en EKF |

### 2.2 `InsEkfFilter::predict()` — línea a línea

| Paso | Código | Variable | Marco | Magnitud | Conforme |
|------|--------|----------|-------|----------|:--------:|
| 1 | `vec3_sub(accel, bias_a, a_corr)` | `a_corr` | B | specific force − bias | ✅ |
| 2 | `vec3_sub(gyro, bias_g, w_corr)` | `w_corr` | B | ω − bias | ✅ |
| 3 | `quat_integrate_first_order(q, w_corr, dt)` | `q_att` | B→N | integración actitud | ✅ |
| 4 | `quat_to_dcm_bn(q, dcm_bn)` | `R_bn` | B→N | DCM | ✅ |
| 5 | `body_to_ned(dcm, a_corr, a_n)` | `a_nav_pre_g` | N | `R_bn · a_corr` | ✅ sintaxis |
| 6 | `a_n[i] -= kGravityNed[i]` | `a_lin` | N | − g **después** de rotar | ✅ |
| 7 | `vel += a_lin * dt` | `vel` | N | integración | ✅ |

**Prohibiciones del contrato verificadas:**

- ❌ No se resta gravedad en body antes de rotar.
- ❌ No se usa `R_nb` en lugar de `R_bn` (`body_to_ned` = filas de `R_bn` · v).
- ❌ No hay segunda aplicación de `R_mount` dentro de `predict()`.

### 2.3 H9a gravity tilt init (interfaz replay → EKF)

```1581:1582:src/core/ins_ekf.cpp
const float roll = atan2f(ay, az);
const float pitch = atan2f(-ax, sqrtf(horiz_sq));
```

| Elemento | Conforme | Notas |
|----------|:--------:|-------|
| Fórmula roll/pitch desde gravedad | ✅ | **Asume body FRD**, +Z down (`az` dominante ≈ +g) |
| Bias accel post-init | ✅ | `bias_a = mean_a_body − R_bnᵀ · g_NED` |
| Yaw preservado | ✅ | Coherente con M6 (actitud vehículo, yaw init separado) |

### 2.4 NHC (`update_nhc`)

| Elemento | Contrato | Conforme |
|----------|----------|:--------:|
| `v_body = R_bnᵀ · v_NED` | I7 — mismo body que `a_corr` | ✅ |
| Observación `y = [-v_y, -v_z]` | +X forward libre; lateral/vertical ≈ 0 | ✅ FRD |

---

## 3. Resultados de invariantes (Patrón Oro)

Generado por `tools/audit_body_frame_conformance.py` sobre `h9d_gravity_subtraction.csv`.

### 3.1 Montaje estático (`R_mount`)

| Magnitud | Valor | Límite | I# |
|----------|------:|--------|-----|
| Mediana `aligned_accel` body | (−0.002, −0.002, **+9.796**) m/s² | ≈ (0,0,+g) | I1 |
| Error ‖[0,0,g] − mediana‖ | **0.011 m/s²** | < 0.05 | I1 |

### 3.2 Por régimen (post-predict, H9d)

| Régimen | `\|f_B\|` mean | `\|a_nav_pre_g\|_h` median | `a_lin_h` median | I2 | I3 |
|---------|---------------:|----------------------------:|-----------------:|:--:|:--:|
| Estático 0–2 s | 9.80 m/s² | **0.017** m/s² | **0.017** m/s² | ✅ | ✅ |
| Marcha 2–10 s | — | **0.428** m/s² | **0.428** m/s² | ❌ | ❌ |
| Crucero 11–25 s | — | **~0.52** m/s² | **~0.52** m/s² | ❌ | ❌ |

**Observación H9d:** `a_lin_h` = `\|a_nav_pre_g\|_h` exactamente (corr = 1.0) — el leak es **previo** a `−g`.

### 3.3 Identidad Android (I6)

| Test | Resultado |
|------|-----------|
| `Uncal ≈ Gravity + Linear` (m/s²) | median residual **0.001 m/s²** ✅ |
| `Uncal == TotalAcceleration` | median **0** ✅ |

---

## 4. Tabla de variables — conformidad §4

| Variable | §4 OK | Observable audit | Estático | Dinámico |
|----------|:-----:|------------------|:--------:|:--------:|
| `row.accel` | ✅ | CSV directo | ✅ | ✅ |
| `aligned_accel` | ⚠️ | mount + I1 | ✅ | ✅ (magnitud) |
| `a_corr` | ✅ | H9d | ✅ | ✅ |
| `a_nav_pre_g` | ✅ sintaxis | H9d | ✅ | **❌ I2** |
| `a_lin` | ✅ sintaxis | H9d | ✅ | **❌ I3** |
| `R_bn` | ✅ | convenciones PASS | ✅ tilt | **❌ proyección** |

---

## 5. Brechas contrato ↔ implementación

### GAP-1 — Calibración `R_mount` vs body vehículo FRD — **CERRADO (Jul 2026)**

**Criterio de éxito (constancia, no coincidencia):**  
`delta_psi = heading(R_bn·e_x) − bearing_GPS` — ¿es **constante** o **varía**?

**Script principal:** `tools/audit_gap1_delta_psi_constancy.py` → `gap1_delta_psi_constancy_report.json`

| Ventana | delta_psi media | std circular | p95 desv | ¿Constante? |
|---------|-----------------|--------------|----------|-------------|
| Estático 0–2 s | **321°** (≡ −39°) | 8.6° | 13.9° | No |
| **Crucero recto 12–24 s** | **225°** (≡ −135°) | **2.7°** | **4.4°** | **Sí** |
| Giro 2–10 s | 168° | **123°** | 164° | **No** |

**Correlaciones crucero recto:** corr(delta_psi, delta_pitch) = **−0.02**; corr(delta_pitch, ax) = 0.14 → tilt **no** arrastra delta_psi.

**Interpretación:**

1. En recto, delta_psi **es constante** → offset fijo calibrable por régimen.
2. Estático (−39°) vs crucero (−135°) difiere ~96° → el offset **sigue al bearing** porque `forward_heading ≈ 0°` (yaw=0): es **yaw misinit**, no yaw de montaje Rodrigues independiente.
3. En giro 2–10 s, std 123° → delta **cambia dinámicamente** (giro + yaw=0).
4. Complemento: `audit_gap1_body_forward_axis.py` — con yaw=GPS, residual forward = 0°.

**Veredicto:** `GAP-1_CLOSED_YAW_INIT_REQUIRED` — Rodrigues + **yaw_init H2** cierran el contrato FRD. Sin segundo ajuste mecánico de mount.

### GAP-2 — Dinámica: transformación medida → NED (prioridad alta)

**Prueba tick a tick (GAP-2 paso 1):** `tools/audit_gap2_gravity_identity_tick.py`

Identidad: `|a_lin,h| ≈ g·sin(δ_tilt)` **en cada muestra** (2–10 s), no solo media.

| δ_tilt | RMS | NRMSE | r | % muestras err<15% | ¿Identidad tick? |
|--------|-----|-------|---|---------------------|------------------|
| **gravity_align** (interno EKF) | 0.030 m/s² | **4.1%** | **0.999** | **98.9%** | **Sí** |
| Orientation `delta_tilt` | 0.332 m/s² | 44.8% | 0.917 | 27.5% | No |
| `delta_pitch` | 0.335 m/s² | 45.3% | 0.916 | 24.9% | No |

H9d reconfirmado: `a_lin,h ≡ |a_nav_pre_g|_h` (max diff ≈ 1e-5).

**Veredicto paso 2:** **BREAK_AT_SPECIFIC_FORCE_CONTAMINATION** — primera ruptura E4 @ t≈3.7 s; `pred_tilt` permanece bajo (~1°) mientras crece `meas_tilt` / `f_horizontal`.

**Paso 3 — A/B/C/D:** `tools/audit_gap2_specific_force_decomposition.py`

| Prueba | `\|a_lin,h\|` mean 2–10 s | Lectura |
|--------|--------------------------:|---------|
| A `R_bn·a_corr` | 0.741 m/s² | Actual |
| B `R_bn·[0,0,az]` | 0.180 m/s² | Sin horizontal body |
| C `R_bn·‖a‖·e_z` | 0.181 m/s² | ≈ B (módulo irrelevante) |
| D `R_bn·g_body_pred` | 0.000 | Identidad algebraica |

Descomposición: **76 %** A−B (f_horizontal); **24 %** residuo B (~1°, mismo orden que `pred_tilt`). D demuestra coherencia interna quat/DCM, **no** actitud física perfecta.

**Veredicto GAP-2:** mecanismo causal validado; causa raíz ya no es “primera transformación rota” sino **uso strapdown de f completa en dinámica** + corrección insuficiente (→ GAP-3).

**Nota:** Orientation correlaciona (r≈0.92) pero **no** satisface la identidad muestra a muestra; el ángulo operativo del leak es el interno EKF (g_pred↔g_meas), no el tilt vectorial vs Android.

**Hecho:** I2/I3 fallan con leak horizontal ~0.4–0.74 m/s² en `a_nav_pre_g`.

**No demostrado:** cuál de Casos A–E (contrato §7 comentarios) es la causa.

**Demostrado:**

- No es la resta de gravedad.
- No es doble mount.
- No es incoherencia Android CSV (I6 PASS).
- Entrada **es** specific force (M1 OK en dataset).

**Hipótesis acotadas compatibles con auditoría:**

1. **Representación dinámica de `R_bn · f_B`** — inclinación en propagación no conserva “gravedad solo en D” bajo aceleración específica real.
2. ~~**GAP-1** — marco body no alineado con vehículo~~ **Descartado** (ver §5 GAP-1 cerrado).
3. **Bias / init** — menos probable en estático (I3 OK post-H9a).

### GAP-3 — Modelo INS: propagación vs corrección (prioridad alta)

**Documento:** [10-gap3-ins-model-audit.md](10-gap3-ins-model-audit.md)

**Pregunta (conceptual, pre-código):** ¿Qué pretende calcular `predict()`? ¿Qué parte de `f_B` entra en propagación de velocidad vs actitud?

**Veredicto provisional:**

- `predict()` implementa **v̇ = R_bn·f_B − g** (INS strapdown estándar, Modelo A).
- **No hay update accel-based** en runtime → H2 reformulada (“accel corrompe actitud”) no aplica literalmente.
- Prueba D: coherencia **algebraica** quat/DCM/proyección — **no** valida actitud física.
- A−B (76 %): leak dominado por **f_horizontal** física; residuo B (~0.18 m/s² ≈ 1°) **abierto**.
- Deriva global probablemente en **capacidad de corrección** (GNSS ~2 %, NHC no observa longitudinal), no en bug de una línea.

**Próximo empírico:** comparar predict-only vs full filter (innovaciones GNSS/NHC).

### GAP-4 — Documentación legacy (prioridad baja)

| Ubicación | Texto legacy | Contrato 08 |
|-----------|--------------|-------------|
| `imu_mount.json` | `body Z+ (EKF down)` | Body = vehículo FRD completo |
| `02-data-and-frames.md` | Actualizado | Alineado |

---

## 6. Checklist de conformidad para futuros cambios

Antes de merge en `predict()` / replay / mount:

- [ ] §4: cada variable nueva tiene fila (marco, magnitud, gravedad, observable)
- [ ] §6: I1–I3 PASS en estático Patrón Oro
- [ ] §6: I2–I3 en dinámica — objetivo **medir mejora**, no solo “no empeorar”
- [ ] §8: regresión cuádruple (tilt, `a_lin_h`, GPS %, RMSE)
- [ ] `tools/audit_body_frame_conformance.py` ejecutado
- [ ] Sin nueva hipótesis H*n* — solo cierre de GAP-*

---

## 7. Próximo paso técnico acotado

1. ~~**Validar +X body**~~ **Hecho** — GAP-1 cerrado.
2. ~~**GAP-2 A/B/C/D**~~ **Hecho** — descomposición f_horizontal vs residuo B.
3. **GAP-3 conceptual** — [10-gap3-ins-model-audit.md](10-gap3-ins-model-audit.md): confirmar Modelo A vs B; auditoría predict-only vs full filter **sin tocar `predict()`**.
4. Mantener regresión Patrón Oro (tilt, `a_lin,h`, GPS %, RMSE) en cualquier cambio futuro.

---

## 8. Historial

| Versión | Fecha | Notas |
|---------|-------|-------|
| 1.0 | 2026-07-18 | Primera auditoría post Body Frame Contract v1.0 |
| 1.2 | 2026-07-18 | GAP-1: criterio constancia delta_psi (`audit_gap1_delta_psi_constancy.py`) |
