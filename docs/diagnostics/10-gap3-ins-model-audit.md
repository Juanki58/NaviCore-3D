# GAP-3 — Auditoría conceptual del modelo INS

**Estado:** **cerrado** — síntesis congelada en **[12-gap3-synthesis.md](12-gap3-synthesis.md)** (2026-07-18)  
**Prerequisitos:** GAP-1 cerrado, GAP-2 pasos 1–3  
**Referencias:** [08-body-frame-contract.md](08-body-frame-contract.md), [09-predict-conformance-audit.md](09-predict-conformance-audit.md), **[11-replay-zupt-provenance.md](11-replay-zupt-provenance.md)**, **[12-gap3-synthesis.md](12-gap3-synthesis.md)**  
**Artefactos GAP-2 paso 3:** `tools/audit_gap2_specific_force_decomposition.py` → `gap2_specific_force_decomposition_report.json`

> **⚠ Validez experimental (H9 → GAP-3.7):** Los runs full-filter con política legacy `forced_time` están **condicionados** por ZUPT espurio. Baseline científico = ZUPT OFF. Ver [11-replay-zupt-provenance.md](11-replay-zupt-provenance.md).

---

## 1. Pregunta central (antes de tocar `predict()`)

> ¿Qué pretende calcular exactamente `predict()`?  
> ¿Qué parte de la fuerza específica debe entrar en la **propagación de velocidad** y qué parte debe usarse **solo para estimar actitud**?

Esta pregunta es distinta de “¿hay un bug en una línea?”. Si el código implementa el INS strapdown estándar, la propagación puede ser **correcta** y la deriva observada puede ser un problema de **observabilidad / corrección**, no de proyección.

---

## 2. Qué implementa `predict()` hoy

Cadena en `InsEkfFilter::predict()` (`ins_ekf.cpp` ~640–744):

| Paso | Ecuación | Rol |
|------|----------|-----|
| Bias | `a_corr = f_B − b_a`, `ω_corr = ω_B − b_g` | Corrección sensor |
| Actitud | `q ← integrate(q, ω_corr, dt)` | **Solo giroscopio** |
| Proyección | `f_N = R_bn · a_corr` | Specific force → NED |
| Gravedad | `a_N = f_N − g_N` | Aceleración lineal cinemática |
| Integración | `v += a_N·dt`, `p += v·dt` | Propagación estado nominal |

**Ecuación de propagación:**

```
v̇_N = R_bn · f_B − g_N
```

Con `f_B = a_corr` (specific force medida, incluye gravedad aparente + aceleración del vehículo).

**Conforme con contrato §4 y hipótesis M1** (acelerómetro mide specific force, no aceleración cinemática pura).

---

## 3. Qué NO hace el EKF con el acelerómetro en runtime

| Uso | ¿Existe? | Dónde |
|-----|:--------:|-------|
| Propagación velocidad | **Sí** | `predict()` |
| Propagación actitud (giro) | **Sí** | `quat_integrate` |
| **Update actitud desde accel** | **No** | — |
| Complementary / tilt aiding dinámico | **No** | — |
| Gravity update en movimiento | **No** | — |

Init estático H9a usa accel para roll/pitch inicial y bias; **después solo gyro** integra actitud.

**Implicación:** `gravity_align` en dinámica mide discrepancia entre `normalize(a_corr)` y `R_bnᵀ·g_N`, pero **el filtro no corrige actitud con esa señal**. No es H2 en el sentido “EKF confunde accel con gravedad para corregir actitud” — simplemente **no hay corrección accel-based**.

---

## 4. Qué observaciones corrigen el estado

| Update | Observa | Estados corregidos | Limitación relevante |
|--------|---------|-------------------|----------------------|
| **GNSS** | Posición NED | pos (+ acoplamiento vía P) | ~2 % aceptación Patrón Oro; sin velocidad GNSS directa |
| **NHC** | `v_y ≈ 0`, `v_z ≈ 0` body | vel (+ acoplamiento actitud) | **`v_x` libre** — no observa aceleración longitudinal |
| **ZUPT** | `v_NED = 0` | vel | Solo parado |

**NHC no restringe aceleración longitudinal.** Si `predict()` integra `a_x` real del vehículo en body → componente forward en NED, eso es **físicamente coherente** y NHC no lo anula.

---

## 5. Resultados A/B/C/D — lectura matizada

Experimento: `tools/audit_gap2_specific_force_decomposition.py` (Patrón Oro, **predict-only**, 2–10 s).

| Prueba | `\|a_lin,h\|` mean | Lectura |
|--------|------------------:|---------|
| **A** `R_bn·a_corr` | 0.741 m/s² | Cadena actual |
| **B** `R_bn·[0,0,a_z]` | 0.180 m/s² | Sin horizontal body |
| **C** `R_bn·‖a‖·e_z` | 0.181 m/s² | Módulo conservado (≈ B) |
| **D** `R_bn·g_body_pred` | **0.000** | Control algebraico |

### 5.1 Prueba D — qué demuestra y qué no

**Demuestra (sólido):**

> El leak **no** proviene de incoherencia interna entre cuaternión, DCM y `body_to_ned`.

`g_body_pred = R_bnᵀ·g_NED` ⇒ `R_bn·g_body_pred = g_NED` es identidad algebraica. D=0 valida la implementación, no la verdad física de `R_bn`.

**No demuestra:**

- Que `R_bn` represente la orientación física correcta del vehículo.
- Que H2 (“actitud inclina gravedad por misinterpretar accel”) esté descartada.

H2 como “error de estimador de actitud” **no aplica literalmente**: no hay update accel-based. H2 reformulado sería: **error de actitud por integración gyro-only** — eso sigue abierto y no lo resuelve D.

### 5.2 Descomposición A−B — lo más útil

| Componente | Magnitud | Fracción de A |
|------------|----------|:-------------:|
| **A − B** (f_horizontal) | **0.56 m/s²** | **~76 %** |
| **Residuo B** | **0.18 m/s²** | **~24 %** |

- **76 %:** fuerza específica horizontal medida → coherente con `f = g + a_vehículo`.
- **24 %:** persiste anulando horizontal. Posibles fuentes (no mutuamente excluyentes):
  - inclinación real pitch/roll (~0.18/9.81 → **arcsin ≈ 1.0°**, mismo orden que `pred_tilt ≈ 1°` en GAP-2 paso 2)
  - usar `[0,0,a_z]` ≠ vector `g_body_pred` bajo pitch/roll
  - ruido accel, cuantización, calibración residual

**No decir “descartado”** para el residuo B; decir **“acotado y pendiente de atribución”**.

### 5.3 Instrumentación `f_residual = a_corr − g_body_pred`

| Régimen | `\|f_residual\|_h` | Interpretación |
|---------|--------------------:|----------------|
| Estático 0–2 s | ~0.016 m/s² | ≈ 0 en reposo ✓ |
| Dinámica 2–10 s | ~0.740 m/s² | ≈ `\|a_corr\|_h` (r≈0.996) |

En dinámica, `f_residual` **es** la componente de specific force que el modelo trata como “no gravedad según actitud EKF”. En un acelerón longitudinal puro sería ≈ `[a_x, 0, 0]`. En 2–10 s (giro) domina componente **lateral** — coherente con maniobra, no con bug de signos.

---

## 6. Confusión de invariantes (importante)

Contrato §6 define **I2/I3 para reposo** (“vehículo parado”).

En **dinámica**, `|a_lin,h| > 0` bajo ecuación strapdown **no es violación** si el vehículo acelera. El audit `09` marca I2/I3 FAIL en 2–10 s — eso mide **magnitud de aceleración integrada**, no necesariamente error de implementación.

**Reformulación:**

| Régimen | Pregunta correcta |
|---------|------------------|
| Estático | ¿I2/I3 PASS? (sí, post-H9a) |
| Dinámica predict-only | ¿`|a_lin,h|` es coherente con `R_bn·f_B − g` dado f medido? (GAP-2: sí, tick a tick con `gravity_align`) |
| Dinámica full filter | ¿Las **updates** contienen la deriva que la propagación introduce? (**GAP-3 empírico pendiente**) |

---

## 7. Dos modelos mentales — cuál asume NaviCore

### Modelo A — INS strapdown clásico (lo implementado)

- `f_B` entra completo en `v̇ = R_bn·f_B − g`.
- Aceleración del vehículo **debe** aparecer en `a_lin` antes de correcciones.
- Actitud: gyro; posición/vel: GNSS + NHC + ZUPT.

**Si este es el modelo deseado:** no parchear `predict()` para “quitar horizontal”. Preguntar por qué GNSS/NHC no contienen.

### Modelo B — Separación explícita gravedad / aceleración dinámica

- Parte de `f_B` solo informa actitud; solo componente “cinemática conocida” entra en `v̇`.
- Requiere **política** (detección maniobra, modelo vehículo, etc.).

**Si este es el modelo deseado:** hay gap entre contrato §4 actual e implementación deseada — es cambio de **especificación**, no bugfix.

**GAP-3 concluye provisionalmente:** el código implementa **Modelo A**. La auditoría debe verificar si Modelo A es el INS que NaviCore **pretende** implementar (respuesta: contrato 08 dice que sí).

---

## 8. GAP-3 empírico propuesto (sin modificar `predict()`)

Antes de cualquier parche, responder:

### 8.1 Coherencia física de A (predict-only)

Comparar en 2–10 s y crucero 12–24 s:

```
|a_lin,h|_A   vs   |d(GPS speed)/dt|
f_residual     vs   aceleración referencia (GPS / Android Linear)
```

Si correlacionan → propagación reproduce dinámica medida; “leak” es aceleración real integrada.

### 8.2 Eficacia de corrección (full filter, no predict-only) — **EJECUTADO**

Script: `tools/audit_gap3_correction_efficacy.py`  
Artefactos: `gap3_correction_efficacy_report.json`, `gap3_predict_only_h8.csv`, `gap3_full_filter_h8.csv`, `gap3_full_filter_h7.csv`

Config común: H9a, mount Rodrigues, yaw=0, Patrón Oro.

| Ventana | Métrica | Predict-only | Full filter | Notas |
|---------|---------|-------------:|------------:|-------|
| 2–10 s | `\|a_lin,h\|` mean | 0.741 | 0.401 | r=0.84 — misma física, estado EKF diverge |
| 2–10 s | `\|v\|_h` mean | 1.37 | **0.02** | **ZUPT** activo (t≤30 s) |
| 34–60 s | `\|v\|_h` mean | 10.25 | 1.66 | GPS speed ≈ **6.47** m/s |
| 34–60 s | `\|v_body_y\|` mean | — | 0.38 | NHC acota lateral body parcialmente |
| 0–60 s | Deriva horizontal | **421 m** | **182 m** | GNSS parcialmente efectivo en posición |
| 0–60 s | GNSS aceptadas | 0/57 | **7/57 (12.3 %)** | Primer rechazo t≈11.3 s, innov_h≈33 m |

**Veredicto 8.2:** `CORRECTION_INSUFFICIENT` — `predict()` produce leak coherente; full filter **no acopla velocidad a GPS** (|v|_h FF ≪ GPS speed en crucero). GNSS rechaza ~88 % por NIS creciente (innov posición, no velocidad). NHC no observa v_x longitudinal. Corrección de **posición** parcial (deriva 421→182 m @60 s); **velocidad** no contenida.

**Implicación:** el gap no está en `predict()` sino en observabilidad/corrección (GNSS pos-only + NHC lateral + ZUPT estático).

### 8.3 Auditoría de observación por ciclo — **EJECUTADO**

Pregunta: ¿las correcciones que recibe el filtro son suficientes para controlar una propagación físicamente coherente?

Instrumentación (EKF + replay):

| Campo | GNSS | NHC | ZUPT |
|-------|------|-----|------|
| timestamp, accepted, reject_reason | ✓ | ✓ (aceptado) | ✓ |
| innovación posición / velocidad | ✓ Δp NED | lateral + vertical body | — (solo corr. estado) |
| NIS, ganancia K, Δx hipotético | ✓ (incluso si NIS rechaza) | ✓ | — |
| `pred_accum_*` desde última corrección | ✓ | ✓ | ✓ |
| ratio `\|Δx_predict\| / \|Δx_update\|` | ✓ | ✓ | ✓ |

CLI replay: `--gap3-observation-audit-csv <csv>`

Script: `tools/audit_gap3_observation_cycle.py`  
Artefactos: `gap3_observation_cycle.csv`, `gap3_observation_cycle_report.json`, `gap3_observation_cycle_analysis.png`

Clasificación de mecanismos por ciclo:

- **A** — pocas observaciones
- **B** — llegan pero se rechazan (NIS, etc.)
- **C** — se aceptan pero corrigen poco (`corr ≪ pred`)
- **D** — corrigen pero la deriva se reacumula de inmediato (ratio pred/corr alto)

| Métrica @60 s | Valor |
|---------------|------:|
| Deriva predict-only | **421 m** |
| Deriva full filter | **182 m** |
| GNSS intentos / aceptadas | 57 / **7 (12.3 %)** |
| Mecanismo dominante | **B_REJECTED** |
| `\|innov_h\|` GNSS mean | 132 m |
| NIS GNSS mean | 613 |
| `corr_pos_h` GNSS (aceptadas) mean | 0.49 m |
| `hypo_corr_pos_h` GNSS mean | **12.4 m** |
| Updates NHC / ZUPT | 5798 / 2897 |

**Veredicto 8.3:** `CORRECTION_INSUFFICIENT` — el filtro **recibe** observaciones (8752 ciclos NHC+ZUPT+GNSS en 60 s), pero GNSS **rechaza el 88 %** por NIS. Cuando acepta, la corrección aplicada es **≪** la corrección hipotética del Kalman (ratio mediano pred/hypo ≈ 0.01). NHC/ZUPT operan cada tick pero corrigen velocidad lateral (~0.01 m/s/tick), no la deriva posicional acumulada. La mitad inferior del diagrama (observación) no recupera el estado: deriva FF sigue siendo **182 m** vs **421 m** PO — mejora parcial, no control.

Comparación obligatoria PO vs FF confirma: propagación del **estado nominal** coherente con el modelo strapdown implementado + observación insuficiente. **No** equivaler esto a «predict() está bien» — ver §8.7.

### 8.4 GAP-3.1 / 3.2 / 3.3 — Anatomía NIS (sin sweep de parámetros)

**Principio:** no tocar P₀/Q/R hasta entender *por qué* el gate rechaza.

| Paso | Pregunta | Instrumentación |
|------|----------|-----------------|
| **3.1** | ¿NIS grande por S malo o por predicción ya desplazada? | `z`, `h(x)`, `S`, NIS total + contrib. N/E/D + bloque horizontal 2D |
| **3.2** | ¿132 m respecto a qué? | `innov_h = ‖x_pred − GPS‖` vs tiempo + ✓/✗ + NIS |
| **3.3** | ¿GNSS→velocidad acopla? | pseudo-innovación `v_pred − v_gps`, `dx_vel` vs `dx_pos` en aceptadas |

CLI: `--gap3-gnss-nis-audit-csv`  
Script: `tools/audit_gap3_gnss_nis_anatomy.py`  
Artefactos: `gap3_gnss_nis_anatomy.csv`, `gap3_gnss_nis_anatomy_report.json`, gráficos timeline / NIS / acoplamiento

**Nota:** el update GNSS observa **solo posición** (3D). No hay innovación de velocidad en el modelo; la pseudo-innovación de velocidad compara `v_EKF` con `speed/course` GPS para GAP-3.3.

**Resultados @60 s (Patrón Oro):**

| GAP | Hallazgo |
|-----|----------|
| **3.1** | NIS dominado por **posición horizontal** (contrib N≈80 %, E≈18 %, D≈2 %). `innov_h` mean=133 m vs `sqrt(S)`≈9.6 m → **predicción desplazada**, no S pequeño |
| **3.2** | Umbral claro: última aceptación t≈10.6 s (`innov_h`≈25 m); **primer rechazo t≈11.3 s** (`innov_h`≈33 m). Banda 31–33 m |
| **3.3** | `\|v_pred\|_h`≈0.89 m/s vs GPS speed≈5.6 m/s; `corr_vel` en aceptadas ≈0; acoplamiento pos→vel vía K **existe pero es negligible** |

**Veredicto:** `PREDICT_ERROR_DRIVES_NIS_REJECTION` — el gate no es “malo”; el EKF **llega al GPS ya desplazado ~30 m** y tras el primer rechazo la deriva explota. No tocar P₀/Q/R hasta confirmar si un update de velocidad explícito cambia el mecanismo.

### 8.6 GAP-3.4 — Bloque K_vel,pos (fix único aceptado)

**Pregunta:** ¿Qué observa realmente el update GNSS? ¿Cuánto acopla `K_vel,pos` una innovación de posición a velocidad?

**Respuesta en código:** solo posición. `update_gnss()` forma `y = z − pos_` (3D), `S = P[pos,pos] + R`, `K = P[:,pos] · S⁻¹`. No hay filas de `H` sobre velocidad; `GpsSample` incluye `speed_mps` / `course_deg` pero el EKF **no los usa** en la corrección (solo posición LLA→NED).

CLI: `--gap3-gnss-k-block-audit-json`  
Script: `tools/audit_gap3_gnss_k_block.py`  
Artefactos: `gap3_gnss_k_block_audit.json`, `gap3_gnss_k_block_report.json`

**Fix auditado:** última GNSS **aceptada** antes del primer rechazo, **t ≈ 10.55 s** (NIS=8.6, `innov_h`≈25 m).

| Magnitud | Valor |
|----------|------:|
| `‖P_vel,pos‖` (Frobenius) | 3.4×10⁻⁵ |
| `max |K_vel,pos|` | **1.8×10⁻⁷** |
| `max |K_pos,pos|` (diagonal) | 0.028 / 0.055 (D) |
| `‖Δx_pos‖` aplicado | **1.42 m** |
| `‖Δx_vel‖` aplicado | **4.6×10⁻⁶ m/s** |
| `Δx_vel / Δx_pos` | **3.3×10⁻⁶** |
| `‖K_vel,pos · innov‖` | 4.6×10⁻⁶ (= `‖Δx_vel‖`, coherente) |
| `v_h` prior / post | 0.012 / 0.012 m/s |
| GPS speed (log, no usado) | 2.64 m/s |

**Desglose Δx (fix t≈10.55 s):**

| Bloque | ‖Δx‖ |
|--------|-----:|
| posición | 1.42 m |
| velocidad | 4.6×10⁻⁶ m/s |
| actitud | 3.9×10⁻⁶ rad |
| bias accel / gyro | 7.1×10⁻⁶ / 1.1×10⁻⁶ |

**Mecanismo (las tres hipótesis del usuario):**

1. **H no conecta posición→velocidad** — **confirmado por diseño.** `H` implícito = identidad solo en errores de posición.
2. **Modelo deliberadamente posición-only** — **confirmado.** Velocidad GPS disponible en log, no en `z`.
3. **P_vel,pos no crece** — **confirmado como causa numérica de K pequeño.** `P_vel,pos ≈ 10⁻⁵ m²/s` → `K_vel,pos = P_vel,pos · S⁻¹ ≈ 10⁻⁷` con `S ≈ 138 m²`. ZUPT/NHC mantienen `v ≈ 0` en nominal y covarianza; la correlación pos–vel no se alimenta.

**Veredicto 8.6:** `DESIGN_LIMITATION` — no es un bug de signos ni de gate. El GPS **mueve ~1.4 m de posición** pero **≈0 m/s de velocidad** pese a innovación ~25 m y speed GPS ~2.6 m/s. La variable que gobierna la siguiente deriva (`v`) queda sin observabilidad directa ni acoplamiento indirecto material vía Kalman.

### 8.7 GAP-3.5 — Propagación de covarianza (F, Φ, Q, P_pv)

**Pregunta:** ¿Por qué `P_vel,pos ≈ 10⁻⁵` y `K_vel,pos ≈ 10⁻⁷`? ¿Bug en F/Φ o consecuencia del ciclo de updates?

**Estructura de F (discretización sparse en `ins_ekf_propagate_covariance_sparse`):**

| Bloque | Implementación |
|--------|----------------|
| **∂p/∂v** | `Phi[pos,vel] = dt · I` (fila pos: `P_pos,* += dt · P_vel,*`) |
| **∂v/∂att** | `Phi[vel,att] = -R_bn · [a_body]× · dt` (`f_va`) |
| **∂v/∂bias_a** | `Phi[vel,bias_a] = -R_bn · dt` (`f_vba`) |
| **Q** | **solo diagonal**; sin términos cruzados |
| **P₀** | **solo diagonal** — bloques cruzados inicializan en cero |

CLI: `--gap3-cov-propagation-audit-csv`  
Script: `tools/audit_gap3_cov_propagation.py`  
Artefactos: `gap3_cov_propagation_audit.csv`, `gap3_cov_propagation_report.json`

**Matiz:** «implementación del predict consistente con strapdown» ≠ «modelo suficiente para el problema».

**Resultados @332 s (Patrón Oro):**

| Evento | `‖P_vel,pos‖` mean | `‖P_vel,vel‖` mean | `v_h` EKF mean | GPS speed mean |
|--------|------------------:|-------------------:|---------------:|---------------:|
| `init` | 0 | 1.73 | — | — |
| `predict_1hz` | **0.069** | 0.033 | 1.17 m/s | 15.0 m/s |
| `gnss_pre` (todos) | **0.069** | 0.038 | 1.17 m/s | 15.1 m/s |
| `gnss_pre` último accept (t≈10.55 s) | **3.4×10⁻⁵** | **4.2×10⁻⁴** | 0.012 m/s | 2.6 m/s |
| `gnss_post` (7 accepts) | **2.8×10⁻⁵** | 0.248 | 0.027 m/s | 0.77 m/s |
| `gnss_reject` | 0.071 | 0.034 | 1.20 m/s | 15.4 m/s |

**Lectura:**

1. **F no está ausente** — en t≈10.55 s: `F_dt≈0.011`, `‖F_va‖≈0.15 (∂v/∂att activo).
2. **P_pv sí crece entre GNSS** — media `predict_1hz`/`gnss_reject` ≈ **0.07**; el acoplamiento Φ no está muerto globalmente.
3. **En el último accept, P_pv ya era 10⁻⁵ antes del update** — coincide con **P_vel ≪ 1** (NHC/ZUPT han comprimido incertidumbre en velocidad); con P_vel pequeño, P_pos,vel no puede crecer → K_vel,pos ≈ 10⁻⁷.
4. **GNSS post-Joseph destruye P_pv** (mean 2.8×10⁻⁵) — update pos-only no reintroduce correlación pos–vel.

**Veredicto 8.7:** `P_VP_SUPPRESSED_BY_CONSTRAINT_CYCLE` — no es bug obvio en F/Φ; es consecuencia del **ciclo NHC cada IMU + GNSS pos-only + P₀/Q diagonales**. El gate NIS es síntoma: estado nominal escapa porque la cadena no corrige `v`.

### 8.8 GAP-3.6 — Ciclo pre/post por update (¿quién destruye P_pv?)

CLI: `--gap3-cov-step-audit-csv`  
Script: `tools/audit_gap3_cov_step_cycle.py`  
Artefactos: `gap3_cov_step_audit.csv` (~138k filas), `gap3_cov_step_audit_report.json`, gráficos ventana / P_vv ruta / reducción por update

**Serie temporal confirmada (mediana Δ post−pre por IMU):**

| Update | Δ‖P_pv‖ | Δ‖P_vv‖ | ΔP_vv body fwd |
|--------|--------:|--------:|---------------:|
| **predict** | **+3.2×10⁻⁴** | +1.6×10⁻⁴ | +1.1×10⁻⁴ |
| **NHC** | **−3.2×10⁻⁴** | −1.6×10⁻⁴ | −1.1×10⁻⁴ |
| **ZUPT** (fase estática) | −3.6×10⁻⁶ | **−4.2×10⁻⁵** | **−4.0×10⁻⁴** |
| **GNSS reject** | ≈0 | ≈0 | ≈0 |

**Cascada temprana diag(P_vv)** (primeros IMU, fase ZUPT): 1.0 → 0.15 → 0.022 → 0.0035 → … → **~4×10⁻⁴** en t≈10.55 s (σ_vel≈**0.015 m/s**).

**Último GNSS accept (t≈10.55 s, pre-update):** `‖P_pv‖=3.4×10⁻⁵`, `‖P_vv‖=4.2×10⁻⁴` — **ya aniquilado antes del Joseph GNSS**, no en el salto instantáneo desde 0.07 (ese 0.07 corresponde a `gnss_reject` en fases posteriores con P_vv repuesta).

**NHC y v_x:** NHC observa solo v_body_y, v_body_z, pero Joseph `(I−KH)P(I−KH)ᵀ` reduce **también** P_vv_body_forward (ratio reducción fwd/lat ≈ **0.92**). No es solo lateral/vertical — el acoplamiento NED↔body arrastra v_x.

**Veredicto 8.8:** `VELOCITY_OVERCONSTRAINED` — predict+NHC en equilibrio neto en P_pv; ZUPT colapsa P_vv al inicio; el filtro llega al GNSS con **v≈0 y σ_v≈0.015 m/s**. K_vel,pos≈10⁻⁷ es **consecuencia**, no misterio.

### 8.9 GAP-3.7 — Pregunta A: ¿por qué v_nominal ≈ 0?

**Separación causal (dos preguntas distintas):**

| Pregunta | Enunciado | Estado |
|----------|-----------|--------|
| **A** | ¿Por qué `v_nominal ≈ 0.01 m/s` pese a GPS 2–6 m/s? | **Cerrada** (esta sección) |
| **B** | ¿Por qué GNSS no corrige `v_nominal`? | **Cerrada** (8.5–8.8: P_vv/P_pv + H pos-only → K≈10⁻⁷) |

GAP-3.5 responde B; GAP-3.7 responde A instrumentando **quién escribe `vel_`**, no quién podría hacerlo.

CLI: `--gap3-vel-source-audit-csv`, `--gap3-imu-constraint-audit-csv`  
Script: `tools/audit_gap3_question_a.py`  
Artefactos: `gap3_vel_source_audit.csv`, `gap3_imu_constraint_audit.csv`, `gap3_question_a_report.json`, gráficos `gap3_vel_accumulation_by_source.png`, `gap3_zupt_vs_gps_speed.png`, `gap3_bias_ax_vs_abody_x.png`

#### 8.9.1 Σ‖Δv‖ acumulado por fuente (Patrón Oro, 332 s)

| Fuente | N writes | Σ‖Δv‖ [m/s] | Nota |
|--------|--------:|------------:|------|
| **predict** | 32 845 | **359.6** | Integración strapdown activa |
| **NHC** | 32 845 | **207.9** | Cada IMU (incl. fase estática con σ ZUPT) |
| **ZUPT** | 2 906 | **12.9** | Solo fase `t ≤ 30 s` (replay) |
| **GNSS** | 7 | **≈0** | Coherente con K_vel≈0 (Pregunta B) |

**Lectura:** predict **sí** aporta velocidad, pero NHC (cada tick) y ZUPT (fase estática) **compiten** con la misma magnitud de orden. Σ‖Δv‖ no es velocidad neta — es suma de magnitudes de escritura — pero elimina la duda de que GNSS sea el actor que anula `v`.

**Componente longitudinal (Σ|Δv_N|):** predict 203 m/s · tick⁻¹ acumulado, NHC 126, ZUPT 6 — NHC **no** es solo lateral: el acoplamiento NED↔body arrastra también v_N en nominal (coherente con 8.8 en P).

#### 8.9.2 Disparo ZUPT vs GPS speed (Hipótesis 1 — falsada en positivo)

Replay (`real_run_replay.cpp`): ZUPT cuando `t ≤ static_phase_end (30 s)` **OR** `gps_speed ≤ 0.1 m/s`. **No hay criterio accel/gyro** — solo reloj + último speed GPS.

Instrumentación por tick IMU: `zupt_armed`, `zupt_applied`, `nhc_applied`, `gps_speed_mps`, `accel_norm`, `gyro_norm`, `bias_a*`, `a_body_*`.

| Métrica | Valor |
|---------|------:|
| IMU ticks ZUPT **aplicado** con GPS 2–6 m/s | **747** |
| Idem antes de `static_phase_end` | **747** (100 %) |
| Primer instante espurio | **t ≈ 10.56 s** |
| Max GPS speed durante ZUPT apply | **6.61 m/s** |
| ZUPT armed pero no applied (GPS 2–6 m/s) | 0 |

**Respuesta a la pregunta simple:** **Sí** — ZUPT activo con GPS 2–6 m/s. **747 veces** en la ventana de arranque.

**Veredicto Hipótesis 1:** `ZUPT_ACTIVE_WHILE_GPS_MOVING` — el replay fuerza ZUPT durante los primeros 30 s **independientemente** de la velocidad GPS. Cualquier trabajo sobre GNSS+velocidad antes de corregir esto sería ambiguo.

#### 8.9.3 Bias acelerómetro longitudinal (Hipótesis 2 — señal secundaria)

| Métrica | Valor |
|---------|------:|
| corr(`bias_ax`, `a_body_x`) en t ∈ [1, 20] s | **−0.89** |
| `bias_ax` @ t≈10 s | **−1.02 m/s²** |

Correlación fuerte negativa durante arranque: el bias **absorbe** aceleración body-x mientras el vehículo acelera. Contribución secundaria respecto a ZUPT (que resetea v directamente), pero **barata de verificar** y coherente con P bias_a observable vía Φ.

#### 8.9.4 NHC — matriz H real vs nominal

En **97.9 %** de updates NHC, `|H_vN| > 0.05` — la restricción lateral/vertical en body **no** deja v_N libre en la Jacobiana. En nominal, Σ|Δv_N| ≈ Σ|Δv_E| (126 vs 133 m/s acumulados).

#### 8.9.5 Veredicto 8.9 (Pregunta A)

`ZUPT_ACTIVE_WHILE_GPS_MOVING` + `VELOCITY_OVERCONSTRAINED` (NHC cada IMU).

**Cadena causal propuesta:**

```
Replay: t ≤ 30 s → ZUPT cada IMU (+ NHC interno en predict)
        ↓
v_nominal → 0 (reset periódico)
        ↓
P_vv → 10⁻⁴, P_pv → 10⁻⁵
        ↓
GNSS pos-only: K_vel,pos ≈ 10⁻⁷  (Pregunta B, ya cerrada)
```

**No introducir velocidad GNSS** hasta corregir la política de constraints en replay (o aislar predict-only / NHC-only / ZUPT-only).

### 8.10 GAP-3.8 — Matriz de políticas de restricciones (A–E)

CLI: `--constraint-policy`, `--nhc-policy`, `--gap3-constraint-pipeline-audit-csv`  
Script: `tools/run_gap3_constraint_matrix.py`  
Artefactos: `docs/benchmarks/constraint_matrix/{A..E}/`, `gap3_constraint_matrix_report.json`

**Políticas ZUPT (`--constraint-policy`):**

| Valor | Comportamiento |
|-------|----------------|
| `auto` | Alias de `forced_time` (calibración/replay legacy) |
| `forced_time` | `t ≤ static_phase_end` OR `gps_speed ≤ threshold` (baseline) |
| `gps_stop` | Solo `gps_speed ≤ threshold` |
| `imu_stationary` | `\|‖a‖ − g\| ≤ dev` AND `\|ω\| ≤ threshold` |
| `disabled` | Nunca armar ZUPT |

**NHC:** `--nhc-policy enabled|disabled` independiente de ZUPT.

**Pipeline por tick IMU** (`constraint_pipeline_audit.csv`):

```
vel_before → predict (Δv, v_after_pred) → NHC (Δv, v_after_nhc) → ZUPT (Δv, v_after_zupt)
```

**Matriz mínima (Patrón Oro, 332 s):**

| Exp | ZUPT | NHC | `vel_h` @ t∈[30,35] s | GNSS accept | `innov_h` accept mean |
|-----|------|-----|----------------------:|------------:|----------------------:|
| **A** baseline | forced_time | ON | **1.09** m/s | 7 | — |
| **B** ZUPT OFF | disabled | ON | **10.22** m/s | 7 | — |
| **C** GPS stop | gps_stop | ON | **2.80** m/s | 8 | 45.8 m |
| **D** IMU stat. | imu_stationary | ON | **0.07** m/s | 7 | 27.9 m |
| **E** libre | OFF | OFF | **6.23** m/s | **56** | **7.7** m |

**Veredicto causal (Exp B vs A):** quitar ZUPT forzado sube `v_nominal` de ~0.01→1.2 m/s (t≈10 s) y ~1.1→10.2 m/s (t≈30–35 s) con **misma IMU, EKF, GNSS y NHC** — causalidad **demostrada** para retirar `forced_time` como baseline científico.

**Exp B vs E (dato clave):** ZUPT OFF + NHC ON → **7** accepts; ZUPT OFF + NHC OFF → **56** accepts, `innov_h` accept ~**7.7 m**. Eliminar ZUPT **no** restaura observabilidad GNSS. El salto 7→56 lo produce **NHC**, no ZUPT.

**Exp C vs D (comparación útil, con cautela):** `gps_stop` → innov accept ~46 m; `imu_stationary` → ~28 m. Antes de declarar D “mejor”, falta instrumentar **cuándo**, **cuánto tiempo** y **por qué** dispara cada detector (reason, window, confidence).

**Exp E (referencia libre):** 56 accepts, NIS bajo, `innov_h` accept ~7.7 m — techo sin constraints; deriva ~23 km (no operacional).

#### 8.10.1 Tres conclusiones sólidas (post-matriz)

| # | Conclusión | Evidencia |
|---|------------|-----------|
| **1** | `forced_time` **contaminaba** el replay | A→B: única diferencia ZUPT; `v_nominal` cambia radicalmente |
| **2** | Pipeline por tick era la instrumentación que faltaba | `constraint_pipeline_audit.csv`: predict→NHC→ZUPT **por muestra**, no solo Σ‖Δv‖ |
| **3** | El problema **no** era exclusivamente ZUPT | B: v≈10 m/s pero **7** accepts; E: **56** accepts sin NHC |

#### 8.10.2 Árbol causal revisado (dos actores)

```
                    ┌─ ZUPT (legacy forced_time)
                    │       ↓
                    │   v_nominal ≈ 0        ← demostrado A→B
                    │
Política replay ────┤
                    │
                    └─ NHC (cada IMU)
                            ↓
                    estado / P comprimidos
                            ↓
                    GNSS rechaza (7 vs 56)  ← demostrado B→E
```

**No afirmar** “el cuello es ZUPT” para el ciclo GNSS completo. **Sí afirmar:** ZUPT explica **velocidad nominal** en legacy; NHC explica **pérdida masiva de aceptaciones GNSS** en Patrón Oro con ZUPT ya off.

Hipótesis abiertas sobre NHC (no excluyentes): σ demasiado fuerte, P infravalorada, aplicación inválida en este escenario, acoplamiento marco/referencia, defecto de implementación.

**Veredicto 8.10:** `FORCED_TIME_RETIRED` + `NHC_DOMINATES_GNSS_ACCEPTANCE` — baseline científico = `imu_stationary` (ZUPT) + auditoría NHC antes de GNSS+velocidad.

### 8.11 GAP-3.9 — Auditoría de bloque NHC

**Prioridad #1** antes de GNSS+velocidad. **No tocar** Q/R/gains/predict hasta cerrar causalidad.

CLI: `--gap3-nhc-block-audit-csv`  
Script: `tools/audit_gap3_nhc_block.py`  
Artefactos: `docs/benchmarks/gap3_nhc_block/{B_nhc_on,E_nhc_off}/`, `gap3_nhc_block_report.json`, plots timeline/NIS/v_body/ΔP-vs-Δx

**Diseño:** ZUPT OFF (`--constraint-policy disabled`), comparar NHC ON (B) vs OFF (E).

**Por cada update NHC** (mismo nivel que GAP-3.4 GNSS):

| Campo | Contenido |
|-------|-----------|
| v_body | before / after + Δv_body (detectar acoplamiento vx) |
| innov | y, z, ‖y‖ |
| S / NIS | S, S⁻¹, NIS_total, **NIS_contrib_y**, **NIS_contrib_z** |
| K | k_y, k_z, k_vel/pos/att/bias_max |
| Δx | pos, vel, att, **bias** |
| P | pre/post **P_pp, P_vv, P_pv, P_vp, P_aa**, P_vv_body, **ΔP_*** |
| gate | `accepted=1` (NHC siempre aplica si enabled) |

**Hipótesis a comprobar (no “NHC mal”, sino mecanismo):**

```
predict → estado OK → NHC → Δx≈0 pero ΔP_vv/P_pv≪0
→ GNSS k_vel,pos≈0 → pos corrige, vel no → predict escapa
```

**Resultados (Patrón Oro, 332 s):**

| Caso | NHC updates | GNSS accepts | innov_h (accepts) |
|------|-------------|--------------|-------------------|
| B (NHC ON) | 32 845 | **7** | 27.2 m |
| E (NHC OFF) | 0 | **56** | 7.7 m |

**Verdict:** `NHC_DOMINATES_GNSS_ACCEPTANCE` — re-ejecutar `audit_gap3_nhc_block.py` tras ampliar CSV para métricas ΔP/v_body/NIS por componente.

**Detector estacionariedad (fase 2):** reason / window / confidence para C vs D — pendiente.

### 8.12 GAP-3.10 — Autopsia de los 7 fixes GNSS aceptados

**Sin más escenarios.** Sólo exp B (ZUPT OFF, NHC ON), ventana **[-2 s, +0.5 s]** por fix.

Script: `tools/audit_gap3_gnss_accepted_autopsy.py`  
Artefactos: `docs/benchmarks/gap3_gnss_accepted_autopsy/`

**Por fix:** innov, S, cond(S), NIS, gate, P pre/post GNSS, K, Δx, v before/after, Σ|ΔP|_NHC y ΣΔv (predict/NHC/ZUPT) en -2 s.

**Cautela causal:** no afirmar `ΔP_pv → K_vel` globalmente hasta descartar innov/S/linealización por fix.

### 8.11 GAP-3.11 — Comparativa fix #2 vs #7

Script: `tools/audit_gap3_fix2_vs_fix7.py`  
Artefactos: `docs/benchmarks/gap3_fix2_vs_fix7/`

Dos estados del mismo sistema (no mejor/peor): innov ~ mismo orden, cond(S)≈1, k_vel cae ~10×.

Métricas: **P_vv(t), P_pv(t), P_aa(t)** escalera pre/post NHC; **ΔP/P**; **Σ|K·innov|** vs **Σ|innov|**; **k_vel/P_vv**.

### 8.12 GAP-3.12 — Exp F: NHC cada N ticks

CLI: `--nhc-every-n-ticks N`  
Script: `tools/run_gap3_nhc_decimation.py`

### 8.13 GAP-3.13 — Autoconsumo Joseph fix #2 vs erosión inter-fix (prioridad sobre #2 vs #7)

Script: `tools/audit_gap3_fix2_fix3_autoconsume.py`  
Artefactos: `docs/benchmarks/gap3_fix2_fix3_autoconsume/`

**Comprobación de una línea (post#2 vs pre#3):**

| Evento | P_vv (frob) |
|--------|------------:|
| fix#2 pre-GNSS | **89.6** |
| fix#2 post-GNSS (Joseph) | **62.0** (−31 %) |
| fix#3 pre-GNSS | **2.5** |
| Ratio post#2 / pre#3 | **24.8×** |

La identidad literal post#2 ≈ pre#3 **no se cumple**. Joseph en fix#2 consume P_vv pero no las ~2 órdenes de magnitud completas.

**Mecanismo híbrido (cerrado con cov_step existente):**

1. **Joseph fix#2** (k_vel=0.197): 89.6 → 62.0 — único accept con palanca real; autoconsume parcial vía K·H·P.
2. **Ventana inter-fix 0.39 s** (76 ticks NHC, predict no reinfla: max P_vv≈62.5): 62.0 → 2.5 — **último NHC post = pre#3 exactamente**.
3. Σ|ΔP_vv| NHC en [-2 s] sigue siendo ~64 para #2 y #3 (ventanas solapadas); la erosión decisiva ocurre **después** del accept #2, dentro del gap que #3 hereda como pre-GNSS.

**Veredicto:** no es erosión gradual uniforme de 7 pasos ni autoconsumo Joseph puro en un solo update — es **un paso Joseph agresivo + compresión NHC ultrarrápida** antes del accept #3. Los accepts #3–#7 quedan con k_vel 0.001–0.023 independientemente del NHC entre medias.

**K_vel algebra (6 accepts con JSONL; #1 sin bloque / k_vel=0):**  
`K_vel_pos = P_vel_pos · S⁻¹` cierra con residual ~1e−8; `k_vel_max` CSV = max abs predicho en todos los fixes (#2: 0.197, #3: 0.008, …). La cadena correlación→ganancia **cierra algebraicamente**; la ambigüedad causal PENDING queda acotada a *por qué* P_vel_pos/P_vv tienen esos valores en cada instante, no a la fórmula de K.

**Hallazgo independiente — innov_h plano:** en los 7 accepts (~10 s GPS, no túnel), innov_h permanece **20–32 m** (media 27 m, σ≈4 m) sin convergencia monótona. La posición se corrige parcialmente cada accept pero el error horizontal se reacumula antes del siguiente — evidencia de ciclo persistente, no bug puntual.

**Implicación remedio (sin implementar aún):** si el cuello es el primer accept con k_vel alto, el fix tipo `ZUPT_MAX_GAIN` aplicado al bloque velocidad del Joseph GNSS es más directo que retocar R_nhc/decimación solos. GAP-3.11 (#2 vs #7) pasa a confirmación narrativa, no herramienta de descubrimiento.

### 8.14 GAP-3.14 — Reconstrucción tick-a-tick fix#2→#3 + Joseph fix#2

Script: `tools/audit_gap3_fix2_fix3_tick_reconstruction.py`  
Artefactos: `docs/benchmarks/gap3_fix2_fix3_tick_reconstruction/`

**Joseph fix#2 (89.6→62.0):** reconstrucción `P+_vv` con bloques pos-vel de JSONL + `R` escalar → post predicho **58.7** vs obs **62.0** (Δ≈3.3 frob, ~5 %). Drop predicho 30.9 vs obs 27.6 (~12 %). **Sin bug Joseph evidente** — residual compatible con acoplamiento att/bias no exportado en k-block JSONL.

**38 ticks IMU** (5.664→6.054 s), cada uno: predict (+ΔP_vv ~+0.05) → NHC (−ΔP_vv):

| Métrica | Valor |
|---------|------:|
| P_vv inicio (post-fix#2) | 62.0 |
| P_vv fin (pre-fix#3) | 2.5 |
| ΣΔP_vv predict | **+3.2** (+5 % del drop neto) |
| ΣΔP_vv NHC | **−62.7** |
| Patrón | **bursty** — top-3 ticks = 74 % de \|ΔP_vv\| NHC |

**Top cliffs NHC:** tick 3 (imu 413): ΔP_vv=**−28.0**; tick 2 (imu 412): **−9.9** con |Δv| NHC=5.66 m/s post-corrección GNSS agresiva.

**Diagnóstico emergente:** predict regenera P_vv lentamente; NHC la consume mucho más deprisa — **orden temporal propagación/restricciones**, no GNSS como actor dominante post-fix#2. F1 (decimar NHC) antes que F2 (GNSS_MAX_GAIN) como intervención limpia.

### 8.15 GAP-3.15 — F1 NHC dose-response (falsificación)

Script: `tools/run_gap3_f1_nhc_dose_response.py`  
Artefactos: `docs/benchmarks/gap3_f1_nhc_dose_response/`

Curva N = 1, 2, 5, 10, 20, OFF (∞). ZUPT OFF. Métricas mecanísticas (no RMSE).

| Policy | N | accepts | P_vv pre#3 | k_vel#3 | Γ | top3 share |
|--------|--:|--------:|-----------:|--------:|--:|-----------:|
| baseline | 1 | 7 | 2.5 | 0.008 | **19.7** | 74% |
| F1c | 10 | 7 | **22.0** | **0.092** | **3.4** | 96% |
| F1d | 20 | 5 | 77.6 | 0.243 | 0.3 | 100% |
| OFF | ∞ | **56** | 123.6 | 0.256 | — | — |

**Predicciones falsables (N↑):**

| Predicción | N=10 vs N=1 | Veredicto |
|------------|-------------|-----------|
| P_vv pre#3 ↑ | 2.5 → 22.0 | ✅ |
| k_vel ↑ | mean 0.036 → 0.097; fix#3 0.008 → 0.092 | ✅ |
| Γ ↓ | 19.7 → 3.4 | ✅ |
| accepts ↑ | 7 → 7 | ❌ (hasta N=10) |
| innov_h ↓ | 27.2 → 31.4 | ❌ (media; OFF sí baja a 7.7) |
| cliff suaviza | top3 share ↑ 74→96% | ❌ sigue bursty |

**Veredicto:** `FREQUENCY_MECHANISM_CONFIRMED_GATE_UNCHANGED` — la sobre-observación NHC explica compresión P_vv y pérdida de k_vel; el gate de 7 accepts persiste hasta N=20 (5) u OFF (56). **No tocar GNSS_MAX_GAIN antes de política NHC.**

### 8.16 GAP-3.16 — Mecanismo del cliff NHC (4 comprobaciones)

Script: `tools/audit_gap3_nhc_cliff_mechanism.py`  
Artefactos: `docs/benchmarks/gap3_nhc_cliff_mechanism/`

**1. ¿K≈1?** No. Ticks 2–4: K_scalar_z=HPH/(HPH+R) ∈ [0.36, 0.55]; k_vel_max 2.3–3.5. Cliff tick 3: NIS≈0.03, ||ΔP||/||Δx||≈**47** → covarianza se consume con innovación pequeña. **Geometría multivariable**, no saturación escalar.

**2. predict +3.2:** ΣΔP_vv=+3.2 vs Q_blanco frob≈0.0003 (**~12000×**). ΣΔP_pv=+5.0 en el gap. Crecimiento vía **F·P** (att/bias→vel), no ruido blanco.

**3. ¿2.5 suelo?** Últimos ticks del gap: slope≈0 → **equilibrio predict↔NHC**; 2.5 no es caída interrumpida.

**4. F1 N=10 vs N=1:** ticks 1–9 sin NHC; cliff pasa a tick 26 pero max |ΔP|≈37 (≥ baseline 28). **Frecuencia elimina cliffs tempranos; el evento bursty persiste.**

### 8.17 GAP-3.17 — F1.1 Anatomía del gate NIS (cuello de botella)

Script: `tools/audit_gap3_f1_nis_gate_anatomy.py`  
Artefactos: `docs/benchmarks/gap3_f1_nis_gate_anatomy/` (`rejected_fixes_nis_anatomy.csv`, report JSON/MD, plots)

**Cadena dividida por F1:**

| Eslabón | Estado |
|---------|--------|
| NHC → P_vv | ✅ confirmado |
| P_vv → k_vel | ✅ confirmado (dosis-respuesta) |
| k_vel → accepts | ❌ **no confirmado** |

**Veredicto F1.1:** el gate está limitado por **innovación / estado nominal** (r y S acoplados), no por K solo. Restaurar k_vel vía decimación NHC no reabre accepts.

**Transición último accept → primer reject:**

| Policy | acc# | rej# | NIS | innov_h (m) | \|Λ_N\| | S_NN | k_vel | eje dom. |
|--------|-----:|-----:|----:|------------:|--------:|-----:|------:|----------|
| N=1 | 7 | 8 | 6.4→9.2 | 20→28 | 1.60→2.34 | 149→137 | 0.023→0.024 | N |
| N=10 | 7 | 8 | 7.3→9.4 | 25→30 | 1.73→2.28 | 205→173 | 0.130→0.149 | N |
| N=20 | 5 | 6 | 8.3→9.2 | 38→34 | 2.28→2.25 | 269→218 | 0.043→0.062 | N |
| OFF | 56 | 57 | 9.4→12.2 | 19→22 | 2.89→3.27 | 43→42 | 0.106→0.091 | N |

En los cuatro casos el primer reject **domina el eje N** (contrib_N ≫ contrib_E). El salto no es homogéneo: crece \|innov_n\| mientras S_NN también cae → Λ_N sube por **numerador y denominador**.

**Rejects gps_index 8–14 (N=1):**

| fix | NIS | contrib_N | contrib_E | contrib_D | \|Λ_N\| | innov_h |
|-----|----:|----------:|----------:|----------:|--------:|--------:|
| 8 | 9 | 5.5 | 0.1 | 3.6 | 2.34 | 27.8 |
| 9 | 25 | 15.1 | 2.0 | 8.3 | 3.91 | 32.1 |
| 10 | 51 | 29.3 | 7.4 | 13.9 | 5.46 | 37.4 |
| 11 | 66 | 37.3 | 14.0 | 15.1 | 6.21 | 43.8 |
| 12 | 83 | 46.0 | 21.0 | 16.2 | 7.00 | 51.8 |
| 13 | 115 | 64.0 | 31.4 | 19.1 | 8.25 | 62.6 |
| 14 | 139 | 75.0 | 44.5 | 19.6 | 9.00 | 71.6 |

**Paradoja N=10 vs N=20:** N=20 tiene P_vv pre#3≈78 y k_vel≈0.24 (vs 22 y 0.09 en N=10), pero accepts=5<7 y first_reject=#6 (no #8). Menos NHC → más K, pero simultáneamente estado nominal peor → innovación mayor. El gate solo ve rᵀS⁻¹r; no hay relación monotónica P→accepts.

**Métrica nueva:** Λ = r/√S por componente (no NIS total). CSV incluye innov_h/long/lat/vert, S_ii, contrib por eje, margen vs threshold.

**Próximo:** F1.2 — anatomía del cliff (tick a tick, K real NHC, ΔP, burst vs estado).

### 8.18 GAP-3.18 — F1.2 Anatomía del cliff NHC

Script: `tools/audit_gap3_f1_cliff_anatomy.py`  
Artefactos: `docs/benchmarks/gap3_f1_cliff_anatomy/` (`gap_ticks_all_policies.csv`, `nhc_events_state_conditioned.csv`, plots)

**Pregunta:** ¿el burst depende del **estado** (P_pre al disparar NHC) o solo de la **frecuencia**?

**Veredicto:** `STATE_CONDITIONED_BURST` — decimar NHC cambia *cuándo* dispara, pero |ΔP| depende de P_pre (corr N=1 = **0.70**). El cliff persiste bursty (top3 share 74%→**96%**); no es convergencia Riccati suave.

| Policy | N | nhc/gap | cliff tick | \|ΔP\| cliff | P_pre cliff | top3 share | 1er NHC tick | \|ΔP\| 1er NHC |
|--------|--:|--------:|-----------:|-------------:|------------:|-----------:|-------------:|---------------:|
| N=1 | 1 | 38 | **3** | 28.0 | 51.9 | 74% | 1 | 1.6 |
| N=10 | 10 | 4 | **26** | 36.9 | 66.5 | **96%** | 6 | 2.5 |
| N=20 | 20 | 2 | **26** | 4.0 | 74.6 | 100% | 6 | 2.4 |

**K real (N=1, ticks 2–4):** K_scalar_z ∈ [0.36, 0.55], k_vel_max ∈ [2.3, 3.5] — no saturación escalar ~0.99; cliff tick 3 tiene NIS≈0.03 pero \|ΔP\|=28 (geometría multivariante Joseph).

**Distribución ΔP por bins de P_pre:**

| P bin | n events | mean \|ΔP\| | mean \|ΔP\|/P |
|-------|--------:|------------:|--------------:|
| (0, 10] | 32 | 0.26 | 0.05 |
| (40, 70] | 7 | 12.7 | 0.21 |
| (70, 200] | 1 | 4.0 | 0.05 |

**Implicación conjunta F1 + F1.1 + F1.2:**

1. **Observabilidad** (NHC→P_vv→k_vel): caracterizada; restaurar K no reabre accepts.
2. **Innovación nominal** (r, S, Λ_N): limitante del gate GNSS.
3. **Cliff bursty**: condicionado por estado al disparar NHC, no eliminable solo bajando frecuencia.

Barridos de R_NHC o GNSS velocity quedan para después — atacarían mecanismos distintos.

### 8.5 Residuo B (24 %)

Comparar `|a_lin,h|_B` con:

- `pred_tilt` / pitch EKF
- `|a_lin,h|_D` (= 0) vs B−D
- segmento **solo longitudinal** (si existe en Patrón Oro)

---

## 9. Cierre GAP-3 — ver [12-gap3-synthesis.md](12-gap3-synthesis.md)

**GAP-3 cerrado.** El informe de síntesis congelada incluye:

1. **Cronología** de hipótesis (Jacobiano → ZUPT → P/K → NIS → burst)
2. **Nivel A** — demostrado · **Nivel B** — descartado · **Nivel C** — abierto
3. **Tres mecanismos separados:** observabilidad, innovación nominal, burst NHC
4. **Diagrama causal** con codificación verde/rojo/naranja
5. **Puerta GAP-4** — pregunta de intervención propuesta (no iniciada)

### Resumen ejecutivo

| Nivel | Contenido clave |
|-------|-----------------|
| **A Demostrado** | ZUPT→v (A→B); NHC→P_vv→k_vel (F1); k_vel↛accepts (F1.1); gate→Λ_N (F1.1); burst estado-condicionado (F1.2) |
| **B Descartado** | solo ZUPT; solo k_vel; cliff frecuencial puro; K≈1; bug Joseph; F roto |
| **C Abierto** | **¿por qué innov_N diverge más rápido que mejora K?** — NHC restrictivo, actitud–vel, sesgos, sin vel GNSS |

**Cuello de botella actual del gate:** mecanismo 2 (estado nominal / innovación), no mecanismo 1 (P/K).

---

## 10. Próximo paso — GAP-4 (fuera de alcance GAP-3)

Ver §9 de [12-gap3-synthesis.md](12-gap3-synthesis.md) y protocolo **[13-gap4-gnss-velocity-protocol.md](13-gap4-gnss-velocity-protocol.md)** (preregistrado, no implementado).

Tareas legacy GAP-3 (opcionales, no bloquean cierre):

1. Residuo B predict-only (~24 %) — atribución pitch vs artefacto
2. Re-run `super_tunnel` con `--constraint-policy imu_stationary`
3. Instrumentar detector estacionariedad (reason/window/confidence) para Exp C vs D

---

## 11. Historial

| Versión | Fecha | Notas |
|---------|-------|-------|
| 1.0 | 2026-07-18 | GAP-3 conceptual post A/B/C/D; matización prueba D; reformulación I2/I3 dinámico |
| 1.1 | 2026-07-18 | GAP-3.7 Pregunta A: ZUPT espurio + Σ‖Δv‖ por fuente; separación causal A/B |
| 1.2 | 2026-07-18 | GAP-3.8 matriz políticas A–E + pipeline vel por tick |
| 1.3 | 2026-07-18 | Regla proveniencia: H9→GAP-3.7 condicionados; re-run `imu_stationary` — doc 11 |
| 1.4 | 2026-07-18 | 8.10.2 árbol causal dual ZUPT→v, NHC→GNSS accepts; GAP-3.9 NHC block audit plan |
| 1.5 | 2026-07-18 | GAP-3.9 implementado: `--gap3-nhc-block-audit-csv`, `audit_gap3_nhc_block.py` |
| 1.6 | 2026-07-18 | GAP-3.10 autopsia 7 fixes GNSS; cond(S) GNSS/NHC; K-block JSONL |
| 1.7 | 2026-07-18 | GAP-3.11 #2 vs #7; GAP-3.12 Exp F `--nhc-every-n-ticks` |
| 1.8 | 2026-07-18 | GAP-3.13 post#2 vs pre#3; híbrido Joseph+NHC; K_vel algebra; innov_h plano |
| 1.9 | 2026-07-18 | GAP-3.14 tick-a-tick 38 IMU; Joseph fix#2 OK; erosión NHC bursty |
| 2.0 | 2026-07-18 | GAP-3.15 F1 dose-response N=1..20,OFF; Γ, P_vv, k_vel confirman frecuencia |
| 2.1 | 2026-07-18 | GAP-3.17 F1.1 NIS gate anatomy; GAP-3.18 F1.2 cliff anatomy |
| **3.0** | **2026-07-18** | **GAP-3 cerrado; síntesis [12-gap3-synthesis.md](12-gap3-synthesis.md)** |
