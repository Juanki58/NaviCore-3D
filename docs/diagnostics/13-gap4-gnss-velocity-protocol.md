# GAP-4 — Protocolo de intervención: observación velocidad GNSS

**Estado:** G0 ejecutado (2026-07-18) — **§11.7 H1e ejecutado → FAIL** (veredicto en `arm_1e_innov_h/`)
  
**Tipo:** intervención (contrasta con GAP-3 autopsia)  
**Prerequisito:** [12-gap3-synthesis.md](12-gap3-synthesis.md) (GAP-3 cerrado)  
**Baseline experimental:** Patrón Oro, `data/real_run/`, ~332 s, `--constraint-policy disabled`, `--nhc-policy enabled`, `--nhc-every-n-ticks 1`

---

## 0. Resumen ejecutivo

| Campo | Valor |
|-------|-------|
| **Hipótesis primaria (H1)** | Añadir observación de velocidad GNSS reduce el crecimiento de **Λ_N** en rejects sin destruir estabilidad del filtro |
| **Mecanismo atacado** | **M2** — estado nominal / innovación (eje N) |
| **Mecanismos NO atacados directamente** | M1 (P/K vía NHC), M3 (burst NHC) |
| **Endpoints primarios** | \|Λ_N\| @ primer reject; accepts GNSS |
| **Endpoints secundarios** | contrib_N/NIS, \|innov_n\| #8–14, P_vv/k_vel @ fix#3, estabilidad |
| **Arms** | G0 pos-only · G1 pos+vel · G2 vel-only · G3 OFF (referencia) |

---

## 1. Evidencia previa (GAP-3) — tablas con datos

Esta sección justifica **por qué M2 es el candidato principal** y no M1 ni M3.

### 1.0 Resultado ancla — k_vel no discrimina el gate

Cohorte **N=1, Exp B** (`gnss_nis_audit.csv`):

| cohorte | n | **k_vel mean** | \|Λ_N\| mean | innov_h mean |
|---------|--:|---------------:|-------------:|-------------:|
| accepts | 7 | **0.0359** | 2.69 | 27.2 m |
| rejects | 324 | **0.0356** | 29.65 | 1938 m |

**k_vel medio idéntico (0.036 vs 0.036):** la ganancia velocidad **no discrimina el gate**. Rechazo = **estado nominal** (Λ_N, innov_n), no falta de K. Justificación directa de GAP-4 sobre **M2**, no M1.

### 1.1 F1 — ¿La frecuencia NHC controla P_vv, Γ y k_vel? (Mecanismo 1)

Gap fix#2→#3 (38 ticks IMU, 0.389 s). ZUPT OFF en todos los casos.

| Policy | N | accepts | P_vv pre#3 | k_vel#2 | k_vel#3 | k_vel mean | innov_h acc | Σ\|ΔP\| NHC | Γ | n_nhc | top3 raw |
|--------|--:|--------:|-----------:|--------:|--------:|-----------:|------------:|----------:|--:|------:|---------:|
| baseline | 1 | **7** | **2.50** | 0.1970 | **0.0078** | 0.0359 | 27.2 | 62.7 | **19.7** | 38 | 74.0% |
| F1a | 2 | **7** | 4.41 | 0.1984 | 0.0172 | 0.0457 | 27.7 | 63.5 | 11.7 | 19 | 76.1% |
| F1b | 5 | **7** | 10.99 | 0.1993 | 0.0471 | 0.0673 | 29.3 | 61.4 | 6.4 | 8 | 80.6% |
| F1c | 10 | **7** | **22.03** | 0.1996 | **0.0923** | **0.0965** | 31.4 | 57.7 | **3.4** | 4 | 95.7% |
| *(F1d)* | *(20)* | *(5)* | *(77.58)* | *(0.1998)* | *(0.2426)* | *(0.1096)* | *(32.2)* | *(6.5)* | *(0.31)* | *(2)* | *(100%)* |
| OFF | ∞ | **56** | 123.58 | 0.2008 | 0.2565 | 0.2064 | **7.7** | 0.0 | — | 0 | — |

**⚠ Fila F1d entre paréntesis — no concluyente:** `n_nhc = 2` en el gap (misma muestra insuficiente que invalida top3/B en N=20). La caída accepts 7→**5** **no** se interpreta como tendencia de “decimar demasiado empeora el gate”; puede ser ruido de muestra pequeña. **No citar F1d en curvas dosis-respuesta** al mismo nivel que N=1 o N=10.

**Comparación N=1 vs N=10 (criterio de parada F1):**

| Magnitud | N=1 | N=10 | Δ | ¿Predicción “más observabilidad → más accepts”? |
|----------|----:|-----:|--:|------------------------------------------------|
| P_vv pre#3 | 2.50 | 22.03 | **+19.5** | — |
| k_vel @ fix#3 | 0.0078 | 0.0923 | **×11.8** | — |
| Γ | 19.7 | 3.4 | −16.3 | — |
| **accepts** | 7 | 7 | **0** | **❌** |
| innov_h mean (accepts) | 27.2 | 31.4 | +4.2 m | **❌ empeora** |

**Conclusión M1:** NHC frecuencia → equilibrio P_vv / k_vel **confirmado numéricamente**. Eslabón **k_vel → accepts no confirmado**.

---

### 1.2 F1.1 — ¿El gate está dominado por innovación eje N (Λ_N)? (Mecanismo 2)

Λ_N = r_N / √S_NN. Gate: NIS = rᵀS⁻¹r, threshold ≈ 11.345 (3 DoF @ 99 %).

**Transición último accept → primer reject:**

| Policy | acc→rej | NIS | innov_h (m) | \|Λ_N\| | S_NN | k_vel | contrib_N | contrib_E | dom |
|--------|---------|-----|-------------|---------|------|-------|-----------|-----------|-----|
| N=1 | 7→8 | 6.41→**9.22** | 20.1→27.8 | 1.60→**2.34** | 149→137 | 0.023→0.024 | **5.46** | 0.14 | n |
| N=10 | 7→8 | 7.28→9.42 | 25.3→30.4 | 1.73→**2.28** | 205→173 | **0.130→0.149** | **5.35** | 0.17 | n |
| *(N=20)* | *(5→6)* | *…* | *…* | *2.25* | *…* | *0.062* | *5.23* | *…* | n |

*(N=20: first_reject #6, accepts=5 — **no concluyente**, n_nhc=2 en gap; no usar en comparativas de gate.)*
| OFF | 56→57 | 9.42→12.18 | 19.3→22.0 | 2.89→3.27 | 43→42 | 0.106→0.091 | **10.80** | 0.99 | n |

En N=10, k_vel en el reject es **6×** mayor que en N=1; accepts y \|Λ_N\|@reject **iguales**.

**Rejects #8–14 (N=1) — serie que motiva GAP-4:**

| fix | NIS | innov_n (m) | innov_long (m) | innov_h (m) | S_NN | \|Λ_N\| | contrib_N | contrib_E | k_vel |
|-----|----:|------------:|---------------:|------------:|-----:|--------:|----------:|----------:|------:|
| 8 | 9.2 | −27.4 | −26.9 | 27.8 | 137 | **2.34** | 5.5 (59%) | 0.1 | 0.024 |
| 9 | 25.5 | −30.4 | −31.2 | 32.1 | 61 | 3.91 | 15.1 (59%) | 2.0 | 0.028 |
| 10 | 50.5 | −34.2 | −37.4 | 37.4 | 39 | 5.46 | 29.3 (58%) | 7.4 | 0.019 |
| 11 | 66.4 | −38.7 | −43.6 | 43.8 | 39 | 6.21 | 37.3 (56%) | 14.0 | 0.025 |
| 12 | 83.5 | −44.8 | −51.3 | 51.8 | 41 | 7.00 | 46.2 (55%) | 21.0 | 0.038 |
| 13 | 115 | −53.7 | −62.5 | 62.6 | 42 | 8.25 | 64.0 (56%) | 31.4 | 0.037 |
| 14 | 139 | −59.6 | −71.6 | 71.6 | 44 | **9.00** | 74.9 (54%) | 44.5 | 0.042 |

**Cohorte accepts vs rejects (N=1)** — ver **§1.0** (ancla). Resumen: \|Λ_N\| 2.69 vs 29.65; k_vel **0.036 vs 0.036**.

**Precisión de marco:** el gate domina **eje N (NED)**. `innov_long` acoplado pero diverge de `innov_n` en #11+.

**Conclusión M2:** limitante actual del gate = **residuo nominal en N** (r_N, Λ_N), no palanca k_vel. GAP-4 ataca directamente esta cadena.

---

### 1.3 F1.2 / índice B — ¿Burst NHC (M3) obliga a otra intervención primero? (Mecanismo 3)

Top3 share crudo **no** comparar entre N sin normalizar. Baseline uniforme: `B_uniform = min(3, n_nhc) / n_nhc`.

| Policy | N | n_nhc | top3 raw | B_uniform | **top3 / B_uniform** |
|--------|--:|------:|---------:|----------:|---------------------:|
| N=1 | 1 | 38 | 74.0% | 7.9% | **9.37×** |
| F1c | 10 | 4 | 95.7% | 75.0% | **1.28×** |
| F1d | 20 | 2 | 100% | 100% | **1.00×** (n=2, no informa) |

Burst **sólido en N=1**; no bloquea GAP-4 porque F1 demostró que restaurar P/k_vel **no** mueve accepts. M3 queda como **confounder a monitorizar** (P_vv, Γ, top3_excess), no como hipótesis primaria.

---

### 1.4 Por qué M2 y no M1/M3 — resumen

| Mecanismo | Evidencia numérica clave | ¿Intervención GAP-4? |
|-----------|-------------------------|----------------------|
| **M1** P/K | k_vel#3: 0.008→0.092 (N=10); accepts=7 | No — ya falsificado |
| **M2** innov_N | **k_vel flat 0.036 acc/rej (§1.0)**; \|Λ_N\| #8→#14: 2.34→9.00 | **Sí — hipótesis primaria** |
| **M3** burst | top3/B_uni=9.4× (N=1) | Monitorizar; no primario |

---

## 2. Hipótesis y predicciones

### H1 (primaria)

> Incorporar observación de velocidad GNSS reduce \|Λ_N\| en el primer reject **y** aumenta accepts, **sin** signos de colapso puntual de covarianza (patrón Joseph/ZUPT).

La confirmación **fuerte** exige **ambos** endpoints (M2 atacado + gate abierto). Un solo endpoint es **parcial** y debe etiquetarse (evita “vía paralela” vs “M2 directo”).

### Predicciones falsables (direccional)

| ID | Si H1… | Métrica |
|----|--------|---------|
| P1 | \|Λ_N\|@first_reject **baja** ≥20 % vs G0 (≤ **1.87**) | Λ_N — **M2** |
| P2 | accepts **≥ 8** (+1 vs G0) | gate abierto |
| P3 | contrib_N / NIS@fix#8 **baja** ≥10 pp (59%→<49%) | descomposición (secundario) |
| P4 | P_vv@fix#3 **no cae** >50 % vs G0; k_vel@fix#3 **no cae** >50 % | M1/M3 guardrail |
| P5 | Sin abort §3.3 | estabilidad |

---

## 3. Umbrales de falsación (explícitos)

Baselines G0 = Exp B actual (N=1, pos-only): **\|Λ_N\|@reject#8 = 2.34**, **accepts = 7**, **contrib_N/NIS@#8 = 59%**, **k_vel#3 = 0.0078**, **P_vv pre#3 = 2.50**.

### 3.1 Veredicto de tres vías (pre-registrado)

Evaluar **G1 vs G0**. P1 = |Λ_N| ≤ **1.87** (−20 % vs 2.34); P2 = accepts ≥ **8**; P5 = sin abort §3.3.

| Veredicto | Condición | Lectura |
|-----------|-----------|---------|
| **H1 CONFIRMADA** | P1 **∧** P2 **∧** P5 | M2 **y** gate — hipótesis primaria |
| **H1 PARCIAL — vía M2** | P1 **∧** P5, ¬P2 | Λ_N mejora; gate no |
| **H1 PARCIAL — vía gate** | P2 **∧** P5, ¬P1 | Accepts suben; Λ_N no — **vía paralela** |
| **H1 REFUTADA** | ¬P1 **∧** accepts≤7 **∧** P5 | Sin efecto |
| **ABORT** | §3.3 | No interpretable |

**Nunca** etiquetar “confirmada” con solo P1 o solo P2.

Comparación cross-arm: |Λ_N| en **mismo gps_index** (#8) si timing difiere.

### 3.2 REFUTADA (equivalente)

|Λ_N|@reject ≥ **2.22** (<5 % mejora) **y** accepts ≤ **7** **y** P5.

### 3.3 Abort — inestabilidad / colapso patológico (no confundir con K bayesiano alto)

**Contexto G2:** k_vel≈0,975 con P_vv≈89 y R_vel=2,25 es **K=P/(P+R) legítimo** (arranque post-gap), no patrón Joseph/fix#2. El abort **no** debe disparar por ganancia alta sola.

| Condición | Regla | Notas |
|-----------|-------|-------|
| **k_vel patológico** | k_vel_max **> 0,50** **∧** err_post **>** err_pre **en componente observada de velocidad** (Δ\|v−v_GPS\| > ε_v) | vel-only / pos+vel: **no** mirar posición |
| **k_pos patológico** | k_pos_max **> 0,50** **∧** err_post **>** err_pre en posición (Δ\|pos−GPS\|_H > ε_pos) | pos / pos+vel |
| **ΔP/P patológico** | \|ΔP_vv\|/P_pre **> 0,50** **∧** misma regla de empeoramiento en componente observada | Joseph legítimo si mejora verdad |
| \|ΔP_vv\| absoluto | **> 30** sin mejora en componente observada | GAP-3.13 |
| P_vv pre#3 | **< 1,0** sin mejora Λ_N | erosión tipo NHC |
| NaN / P no PD | cualquiera | divergencia |

ε_v = **0,01 m/s**, ε_pos = **0,5 m**. Monitor secundario (no abort duro v1.1): patrón **sostenido** ≥2 ticks consecutivos.

**Descartado para GAP-4:** subir σ_v global solo para moderar arranque — sacrificaría sensibilidad M2 donde P_vv alto es legítimo.

---

## 4. Diseño experimental

### 4.1 Brazos

| Arm | GNSS update | Propósito |
|-----|-------------|-----------|
| **G0** | Posición 3D (actual) | Control — rerun instrumentado |
| **G1** | Posición 3D + velocidad 2D/3D NED | **Intervención principal** |
| **G2** | **Velocidad sola** (sin posición en el mismo update) | Aislar palanca vel vs acoplamiento S pos–vel |
| **G3** | NHC OFF, pos-only | Techo (56 accepts); no intervención vel |

Todos G0–G2: ZUPT OFF, NHC ON, N=1. G3 solo referencia M1/M3.

### 4.2 Orden de ejecución

1. G0 (reproducibilidad GAP-3)
2. G2 (vel-only — diagnóstico palanca)
3. G1 (pos+vel — intervención)
4. G3 (opcional, si G0/G1 completos)

### 4.3 Ruido de medida de velocidad (decisión de diseño)

**Fuente de medida (Patrón Oro):**

- `Location.csv`: `speed` (m/s), `bearing` (deg), `speedAccuracy` (m/s, típico **1.5**)
- **No** es raw Doppler en replay; es solución filtrada Android (fused GNSS). Documentar como **pseudo-velocidad horizontal**.

**Construcción:**

```
v_meas_N = speed × cos(bearing)
v_meas_E = speed × sin(bearing)
v_meas_D = no observado (G1/G2) o 0 con R_D grande
```

**Matriz R_vel (default preregistrada):**

| Componente | σ default | R_ii = σ² | Fuente |
|------------|----------:|----------:|--------|
| v_N, v_E | **1.5 m/s** | **2.25 m²/s²** | mediana `speedAccuracy` en `Location.csv` |
| v_D (si usado) | 5.0 m/s | 25 | no observado — varianza alta |

**Sweep secundario (post confirmación G1 default):** σ_v ∈ {0.5, 1.0, 1.5, 2.0, 3.0} m/s — una pasada, no barrido libre.

**R_pos (sin cambiar G0):** `NAVICORE_INS_EKF_GNSS_POS_VAR_M2 = 6.0` m² diagonal (σ_pos ≈ 2.45 m por eje, coherente con audit actual).

**Tasa:** 1 Hz — misma fila `Location.csv` que posición (no interpolación).

**Correlaciones:** fase 1 diagonal R_vel; cross speed–course no modelado (documentar como limitación).

**G2 vel-only:** H = [0 I_vel 0 …] (solo filas velocidad); **misma R_vel**. Sin filas posición en ese update → S no incluye acoplamiento pos–vel del update conjunto.

**G1 pos+vel:** H apilado 3+2 o 3+3; S = HPHᵀ + R_block; R block-diagonal diag(R_pos, R_vel).

---

## 5. Instrumentación obligatoria (desde diseño, no post-hoc)

Patrón GAP-3.10 — **todo arm desde el primer replay**:

| Artefacto | CLI / path | Contenido mínimo |
|-----------|------------|------------------|
| NIS audit | `--gap3-gnss-nis-audit-csv` | innov_n/e/d, S_ii, Λ_i, contrib_i, k_vel_max, accepted |
| K-block JSONL | `--gap3-gnss-k-block-audit-json` | **cada fix** accept+reject: K_vel_pos, K_pos_pos, S⁻¹, P_vel_pos |
| Cov steps | `--gap3-cov-step-audit-csv` | P_vv, P_pv pre/post GNSS → derivar **ΔP_vv/P_vv_pre** @ GNSS |

**Columnas derivadas (post-proceso, no hardware nuevo):**

| Columna | Fuente | Uso |
|---------|--------|-----|
| `dP_vv_over_P_pre` | cov pre/post GNSS | abort §3.3 (colapso puntual) |
| `k_vel_max` | NIS audit / K JSONL | abort §3.3 |
| `Lambda_n` | NIS audit | P1, veredicto §3.1 |

**Gap conocido G0:** K JSONL actual solo en **accepts** (`real_run_replay.cpp`); GAP-4.1 extiende a rejects.

---

## 6. Métricas y tabla de reporte

| Métrica | Definición | G0 baseline | **G0 rerun** |
|---------|------------|------------:|-------------:|
| accepts | fixes accepted | **7** | **7** ✓ |
| \|Λ_N\| @ fix#8 | \|innov_n\|/√S_NN | **2.34** | **2.34** ✓ |
| contrib_N/NIS @ #8 | — | **59%** | **59.3%** ✓ |
| \|innov_n\| @ #8 | m | **27.4** | **27.4** ✓ |
| innov_h accept mean | m | **27.2** | **27.2** ✓ |
| P_vv pre#3 | frob | **2.50** | **2.50** ✓ |
| k_vel#3 | max\|K_vel\| | **0.0078** | **0.0078** ✓ |
| k_vel mean acc / rej | cohorte | **0.036 / 0.036** | **0.036 / 0.036** ✓ |
| max k_vel @ accept | abort guard | <0.5 | **0.197** ✓ |
| max \|ΔP_vv\|/P_pre @ accept | abort guard | <0.5 | **0.308** (fix#2) ✓ |
| Γ gap | Σ\|ΔP\|_NHC / ΣΔP_predict | **19.7** | — |
| top3/B_uniform | burst guardrail | **9.37×** | — |

**G0 PASS** — artefactos: `docs/benchmarks/gap4_gnss_velocity/G0/`, reporte `gap4_g0_baseline_report.json`, script `tools/run_gap4_g0_baseline.py`.

Reporte JSON: comparativa G0 vs G1 vs G2 con veredicto H1/P1–P5.

---

## 7. Qué NO se hace en GAP-4

- Barrido R_NHC
- Cambios Q process noise
- GNSS_MAX_GAIN / NHC_MAX_GAIN (salvo abort §3.3 condicional)
- Decimación NHC (ya caracterizada en F1)
- RMSE como endpoint primario
- **G1b sweep R_pos/R_vel** sin decisión §10.5
- **Subir σ_v global** para moderar k_vel en arranque

---

## 8. Implementación (fases)

| Fase | Entregable | Criterio salida |
|------|------------|-----------------|
| **4.0** | `--gnss-obs-mode pos\|pos_vel\|vel_only` + R_vel | compila; G0 reproduce 7 accepts ±0 — **G0 ✓ 2026-07-18** |
| **4.1** | Instrumentación vel en NIS + K JSONL | G0 numéricamente idéntico a §1 |
| **4.2** | Run G2 → G1 | reporte vs §3 umbrales — **G2 ✓ / G1 ABORT ✓ 2026-07-18** |
| **4.3** | (opcional) σ_v sweep | **descartado** — ver §10; G1b R-sweep **no autorizado** hasta decisión §10.5 |

---

## 10. Cierre fase diagnóstica (2026-07-18)

### 10.1 Guardarraíl condicional (opción 1) — operativo

| Brazo | Abort §3.3 | Veredicto §3.1 | Notas |
|-------|------------|----------------|-------|
| **G0** | no | PASS (7 accepts) | regresión GAP-3 intacta |
| **G2** vel-only | no | H1 PARCIAL — vía gate | P2✓ (33 accepts); k≈0,975 legítimo (K=P/(P+R)); legacy flags informativos |
| **G1** pos+vel | **sí** (fix#2) | ABORT | P2✓ (8 accepts); fix#2 empeora Δ\|v−v_GPS\| (+0,034 m/s) |

**Descartado:** subir σ_v global (opción 2) — moderaría arranque legítimo en G2 sin resolver acoplamiento G1.

Artefactos: `docs/benchmarks/gap4_gnss_velocity/{G0,G2,G1}/`, script `tools/gap4_abort_guardrail.py`.

### 10.2 Anatomía k_vel G2 — dos de dos LEGITIMATE_HIGH_GAIN

| Evento | k_vel | P_vv_n pre | K=P/(P+R) | Δ\|v−v_GPS\| | Veredicto |
|--------|-------|------------|-----------|--------------|-----------|
| fix#2 | 0,975 | 89,6 m²/s² | ✓ | −0,018 m/s | LEGITIMATE_HIGH_GAIN |
| fix#3 | 0,561 | 2,88 m²/s² | ✓ | −0,059 m/s | LEGITIMATE_HIGH_GAIN |

Informe: `G2/gap4_g2_kvel_anatomy_report.json`, script `tools/audit_gap4_g2_kvel_anatomy.py`.

### 10.3 G1 fix#2 — verificación álgebra 5D (bug vs acoplamiento)

Tres chequeos sobre update H apilado + Joseph N-dimensional en fix#2 (`t=5,664 s`):

1. **S_pos,vel = P_pos,vel** — ‖error‖_F = 0; P_pv≈140 m²/s coincide con GAP-3.10; signo positivo (coherente con Φ pos→vel).
2. **K 5D + Joseph** — K_vel,vel[N,N] reconstruido 0,963 vs log 0,965; P_vv post Joseph error **0,24%**.
3. **K_vel,vel G1 vs G2** — G1 0,963 ≤ G2 vel-only 0,975 (mismo signo).

**Veredicto:** `ALGEBRA_CONSISTENT` — no hay bug de implementación en H/S/K/Joseph.

**Mecanismo (descompuesto):** Δv = K_vel,pos·y_pos + K_vel,vel·y_vel. En fix#2 el término cruzado domina (~**69%** de ‖Δv‖) porque y_pos≈39,6 m e P_pv≈140.

Informe: `G1/gap4_g1_fix2_5d_algebra_report.json` (fix#2), script `tools/audit_gap4_g1_fix2_5d_algebra.py`.

### 10.4 Tres puntos — colinealidad tiempo vs ‖y_pos‖ (n=3, tendencia)

**Limitaciones honestas:**

- G1 pos+vel: solo **8 accepts**; fix#7 (`t≈10,6 s`) no es régimen profundo, solo “menos transitorio que fix#2”.
- Con **n=2** (fix#2 vs #7) hay tendencia, no curva — no distingue tiempo desde gap vs ‖y_pos‖ (covarían).
- **Tercer punto** para romper colinealidad: **G2 fix#56** @ `t≈58 s` (33 accepts, vel-only, no abort) — **pos+vel contrafactual 5D** (misma álgebra/P/y, sin corrida pos+vel real).

| Punto | Brazo | t (s) | ‖y_pos‖_3D (m) | P_pv pre | Fracc. cruzada | Cruzada domina | Δ\|v−v_GPS\| |
|-------|-------|------:|---------------:|---------:|--------------:|:-------------:|-------------:|
| fix#2 | G1 actual | 5,7 | 39,6 | 140,1 | **69%** | **sí** | **+0,034** (empeora) |
| fix#7 | G1 actual | 10,6 | 32,7 | 0,72 | 39% | no | −0,066 (mejora) |
| fix#56 | G2 **contrafactual** 5D | 58,3 | **214,2** | 1,11 | **98%** | **sí** | −0,171 (mejora*) |

\*Contrafactual: vel post = vel_pred + Δv_5D; no es post real del filtro (G2 corrió vel-only).

**Paradoja dominancia vs outcome:** fix#56 tiene cruzada al **98%** pero **mejora** vel (−0,171); fix#2 domina menos (69%) pero **empeora** (+0,034). La fracción cruzada **no** predice el signo del efecto.

### 10.4b Alineación direccional — cos(dv_pos, err_pre) (n=3)

**err_pre** = v_pred − v_GPS (NE). **dv_pos** = K_vel,pos·y_pos (componente cruzada).

| Punto | Fracc. cruzada | cos(dv_pos, err_pre) | proj(dv_pos → −err̂) | Δ\|v−v_GPS\| |
|-------|---------------:|---------------------:|---------------------:|-------------:|
| fix#2 | 69% | **+0,28** (tira con el error) | **−0,067** | **+0,034** empeora |
| fix#7 | 39% | +0,48 (cruzada sola mal) | −0,045 | −0,066 mejora* |
| fix#56 cf. | 98% | **−0,36** (opone al error) | **+0,80** | −0,171 mejora |

\*fix#7: Δv total logueado alineado con corrección (cos≈**+0,94**); el canal vel rescata aunque la cruzada sola empuje con el error.

**Veredicto §10.4 (revisado):** `ALIGNMENT_PRIMARY` (n=3) — lo que determina ayuda/perjuicio no es ‖y_pos‖ ni tiempo ni P_pv solos, sino si **P_pv en ese instante proyecta la cruzada en la dirección de corrección** (−err). fix#2: P_pv inflado post-gap con orientación **contaminada** (cos>0). fix#56: ‖y_pos‖ enorme pero orientación **útil** (cos<0). Correlación cos vs Δerr: **r≈0,74** (n=3) — **no estadísticamente informativo**; ver §10.4c.

`INNOVATION_MAGNITUDE_PRIMARY` queda **deprecado** — Huber vs ‖y_pos‖ penalizaría fix#56 (caso bueno).

Informe n=3: `G1/gap4_coupling_collinearity_report.json`.

### 10.4c Barrido completo — cos(dv_pos, err_pre) en todos los accepts k_block (n=45 / G2 n=33)

**Script:** `tools/audit_gap4_alignment_sweep.py`  
**Informe:** `G1/gap4_alignment_sweep_report.json`  
**Cobertura:** accepts con `gnss_k_block.jsonl` + speed (G0 6/7, G1 6/8, G2 33/33). Contrafactual 5D pos+vel en todos los brazos. **Pool primario: G2 n=33** (evita triple-conteo del mismo fix físico en G0/G1/G2).

| Métrica | G2 (n=33) | Pool 45 (G0+G1+G2) |
|---------|----------:|-------------------:|
| cos media | −0,06 | +0,03 |
| cos mediana | +0,13 | +0,28 |
| frac. cruzada ayuda (cos<0) | **48%** | 42% |
| corr(cos, Δerr_cf) | −0,07 | −0,10 |
| corr(cos, dt_accept) | −0,22 | −0,22 |
| corr(cos, **‖y_pos‖**) | **−0,35** | −0,35 |
| corr(cos, P_pv) | +0,02 | +0,06 |

**Contingencia signo → outcome (G2, contrafactual 5D):**

| cos(dv_pos, err) | n | frac. mejora vel cf. |
|------------------|--:|---------------------:|
| **> 0** (empuja con error) | 17 | **0%** |
| **< 0** (opone al error) | 16 | **50%** |

→ El patrón n=3 **sí generaliza** en dirección: cos<0 predice mejora; cos>0 nunca mejora en este barrido. No es geometría de tres puntos.

**Buckets por gap (G2, `effective_gap` = dt desde último accept; primer accept → dt desde último fix GNSS):**

| Bucket | n | cos media | frac. cos<0 (ayuda) |
|--------|--:|----------:|--------------------:|
| gap ≤ 1 s | 20 | **+0,13** | 35% |
| gap > 1 s | 13 | **−0,40** | 75% |
| gap > 4 s | 6 | −0,29 | 67% |
| gap > 10 s | 3 | −0,69 | 100% |

→ **Invertido respecto al reset post-gap ingenuo:** los gaps **largos** tienden a cos<0 (cruzada útil); los **cortos** a cos>0 (cruzada perjudicial). fix#2 (gap≈4,6 s, cos≈+0,28) es outlier dentro del bucket largo, no la regla.

**Implicación para §10.5:** la hipótesis #1 original (“reset P_pv post-gap largo”) queda **falsada en dirección** por este barrido. Un reset incondicional **destruiría** señal útil en ~48% de accepts (y más en gaps largos). La bifurcación preregistrada razonable es **reset/downweight condicionado a gap corto** (brazo experimental) **vs reset incondicional** (control negativo), no al revés.

**Caveats:** n=33 sigue siendo modesto; buckets gap>10 s tienen n=3; contrafactual 5D ≠ post real G1 pos+vel; G0/G1 pierden 1–2 accepts sin k_block.

### 10.4d Discriminación gap vs ‖y_pos‖ — ¿qué condicionar en 1a? (G2 n=33)

**Script:** `tools/audit_gap4_threshold_discrimination.py`  
**Informe:** `G1/gap4_threshold_discrimination_report.json`

| Criterio | ‖y_pos‖ | gap (`effective_gap`) |
|----------|---------|------------------------|
| Pearson r(cos, ·) | **−0,35** | −0,22 |
| Spearman ρ(cos, ·) | +0,06 (no monótona) | **−0,28** |
| Mejor umbral (Youden J) | y≤211 m → J=0,25 | **gap≤1 s → J=0,33** |
| Sens. malos (cos>0) | 94% | 76% |
| Spec. buenos (cos<0) | 31% | **56%** |

→ **Pearson favorece ‖y_pos‖; poder discriminativo de umbral favorece gap≤1 s.** La relación cos–‖y_pos‖ **no es monótona** (Spearman≈0): fix#56-like (‖y‖ enorme, cos<0) convive con fix#2-like (‖y‖ moderado, cos>0). **Ninguna frontera es limpia** (solapamiento total en ambas variables).

**Dentro de gap≤1 s (n=20):** mediana cos=**+0,44** vs media +0,13 (dispersión alta). r(cos,‖y_pos‖)=−0,46 dentro del bucket, pero split por mediana ‖y_pos‖: bajo ‖y‖ 70% cos>0, alto ‖y‖ **60% cos>0** y **40% cos<0** — accepts **#62, #84, #238, #239** (gap corto, ‖y_pos‖ alto, cos≪0) serían **víctimas colaterales** de un reset solo por gap.

**Reglas combinadas (replay diagnóstico, no causal):**

| Regla | intervenir | sens. malos | spec. buenos |
|-------|----------:|------------:|-------------:|
| gap≤1 s | 20 | 76% | 56% |
| gap≤1 s ∧ ‖y‖≤197 m | 10 | 41% | **81%** |
| ‖y‖≤50 m | 6 | 18% | 81% |

**Implicación §10.5:** preregistrar **1a gap≤1 s** (mejor Youden que ‖y_pos‖ entre proxies) **+ 1b incondicional** + **1d gate cos directo** (§10.4e). **No incluir 1c** (sobreajuste dual proxy).

### 10.4e Gate causal directo — simulación exploratoria H1d vs proxies (G2 n=33)

**Script:** `tools/audit_gap4_direct_cos_gate.py`  
**Informe:** `G1/gap4_direct_cos_gate_report.json`

**Variable causal en tiempo real (sin look-ahead):** cos(dv_pos, err_pre) con err_pre = v_pred − v_GPS (medida entrante); dv_pos = K_vel,pos·y_pos disponible pre-corrección.

**Outcome simulado (exploratorio):** `zero_cross_helps` := err(vel-only) < err(5D completo) — álgebra contrafactual G2, no replay G1.

| Regla | intervenir | sens. (zero_cross ayuda) | spec. (conservar cruzada buena) | fp |
|-------|----------:|-------------------------:|--------------------------------:|---:|
| baseline 5D | 0 | 0% | 100% | 0 |
| H1d cos(dv_pos,err)>0 | 17 | 81% | 100% | 0 |
| H1a gap≤1 s | 20 | 71% | 58% | 5 |
| H1b incondicional | 33 | 100% | 0% | 12 |

**⚠ Reetiquetado metodológico (obligatorio):** esta tabla es **diagnóstico exploratorio post-hoc**, no validación de H1d. H1d usa la **misma** cantidad (cos) que definió `ALIGNMENT_PRIMARY` en estos mismos 33 puntos G2; H1a/H1b usan variables independientes. Sens/spec favorables a H1d **no deben leerse** como “H1d domina a los proxies” — miden coherencia interna de la hipótesis de alineación sobre la muestra que la generó. **La prueba decisiva es replay G1 fuera de muestra** (§11).

**Motivación mecanicista (no confirmada):** gap y ‖y_pos‖ eran proxies débiles (Youden J≈0,33) de una cantidad computable en el tick; H1d ataca orientación directamente.

**H1d′** cos(Δv_total, err)>0: hipótesis separada — el canal vel puede “rescatar” el neto cuando la cruzada sola está mal alineada (fix#7 G1). Brazo **obligatorio** en §11, no opcional condicionado a frecuencia.

**Nota implementación:** gatear ≠ mirar outcome posterior; usa v_GPS entrante + K pre-aplicación.

### 10.5 Cierre fase diagnóstica — **CONGELADA** (2026-07-18)

**Estado:** **CERRADA** — no más instrumentación diagnóstica salvo regresión.  
**Tag Git:** `gap4-diagnostic-complete`  
**Fase siguiente:** §11 intervención (experimento independiente, hipótesis preregistrada).

**Modelo explicativo (cuatro frases):**

1. NHC comprime la covarianza de velocidad mucho más rápido de lo que `predict()` la regenera.
2. La actualización GNSS usa P_pv para trasladar parte de la innovación de posición a velocidad; no es un bug, es el funcionamiento normal del EKF.
3. La decisión de modificar P_pv en fix#4 cambia la trayectoria completa del filtro; desde ese punto ya no se comparan políticas sobre el mismo estado, sino dos EKF distintos.
4. Los criterios `cos_pos` y `cos_tot` difieren precisamente en ese punto de bifurcación; después el propio estado hace que los cosenos evolucionen de forma distinta.

**Figuras de síntesis** (`docs/benchmarks/gap4_gnss_velocity/G1_intervention/`):

| Fig | Archivo | Contenido |
|-----|---------|-----------|
| 1 | `fig1_divergence_tree.png` | Árbol fix#4 — bifurcación 1d / 1d′ |
| 2 | `fig2_causal_chain.png` | Cadena mecanicista predict → P → gate → trayectoria |
| 3 | `fig3_ppv_ratio_vs_cos_scatter.png` | \|P_pv\|/P_vv vs cos (post-bifurcación) |

Regenerar: `python tools/render_gap4_diagnostic_synthesis.py` (requiere `ppv_divergence_tree.json`).

**Criterio de éxito post-diagnóstico:** la intervención debe ser coherente con este modelo mecanicista — no basta «acepta más GNSS» o «baja RMSE».

### 10.6 Regla metodológica — cos / gate P_pv solo desde logging del filtro

**Estado:** vigente desde autopsia gps#32 (2026-07-18).

Durante la autopsia mecanicista (§10, pre-§11) se demostró que **reconstrucciones offline de `cos(dv, err_pre)` no sustituyen al valor evaluado en el tick del filtro**:

| Fuente | gps#32 (1d′) | Gate `ppv_triggered` |
|--------|--------------|----------------------|
| Reconstrucción post-hoc (K bloque pos) | cos ≈ **+0.87** | inferido 1 — **falso** |
| Logging directo (`gnss_nis_audit.csv`) | cos_tot ≈ **−0.87** | **0** (correcto) |

La divergencia no es un bug de implementación del zero de P_pv: el gate **no dispara** porque en el estado degradado real `cos_tot < 0`. La reconstrucción usaba un estado nominal distinto al del tick.

**Regla obligatoria (caracterización y §11):**

1. Toda hipótesis sobre alineación, disparo del gate o dominio de validez de `cos_pos` / `cos_tot` debe apoyarse en campos logueados al final de cada fila de `gnss_nis_audit.csv`: `ppv_policy`, `ppv_triggered`, `cos_dv_pos_err_pre`, `cos_dv_tot_err_pre`, `ppv_frob_pre`, `ppv_frob_post`.
2. Scripts de autopsia que recomputan K o cos desde CSV de replay son **exploratorios**; no pueden contradecir ni sustituir el log del filtro.
3. Comparaciones 1d vs 1d′ (truth table fix#2–7, gps#32) usan replays con logging directo (`arm_1d_cos_pos_0_40s_logged`, `arm_1d_prime_cos_tot_0_40s_logged`); agregación: `tools/audit_gap4_ppv_truth_table.py`.

**Implicación:** antes de modificar 1d′ o ampliar el gate, caracterizar el **dominio de validez** de `cos_tot` con valores logueados — no introducir heurísticas ad hoc en estado degradado.

### 10.7 Modelo de bifurcación — árbol fix#4 (cierre autopsia mecanicista)

**Estado:** derivado de replays logueados 0–40 s (2026-07-18).

Hasta fix#3, los brazos 1d y 1d′ son **un solo EKF** (`estado_A ≈ estado_B`). En fix#4 comparten el mismo pre-update; la política diverge **solo** en post:

| | 1d (`cos_pos>0`) | 1d′ (`cos_tot>0`) |
|--|------------------|-------------------|
| pre fix#4 | \|P_pv\|=5.41, cos_pos=+0.64, **cos_tot=−0.67** | idéntico |
| post fix#4 | P_pv→0 (trigger=1) | P_pv→1.58 (trigger=0) |

A partir de ahí existen **dos instancias** del filtro. Comparar `cos` en fix#6–7 **no** es comparar políticas sobre el mismo estado; es comparar trayectorias disto.

**Artefactos:**

- Truth table: `ppv_truth_table_1d_vs_1dprime.json` — `tools/audit_gap4_ppv_truth_table.py`
- Árbol de divergencia: `ppv_divergence_tree.json`, diagrama Mermaid `ppv_divergence_tree.mmd` — `tools/audit_gap4_ppv_divergence_tree.py`
- Magnitudes por nodo: \|P_pv\|, P_vv (Frobenius), ratio \|P_pv\|/P_vv, cos_pos, cos_tot, NIS, accept/reject

**Hipótesis causal pendiente (no refutada):** `cos` puede ser **observable** de la geometría de P (bloque P_pv vs P_vv), no la variable de diseño primaria. Tras fix#4, la rama 1d′ acumula \|P_pv\| y ratio \|P_pv\|/P_vv más altos; el signo de `cos` en fix#6–7 se invierte respecto a 1d **junto con** el cambio de nominal, no como efecto aislado del ángulo.

**Pregunta de diseño (post-autopsia):** no «¿qué gate es mejor?» sino «¿cómo cambia el espacio de estados cuando zero/no-zero P_pv en fix#4?» — §11 evalúa outcome; §10.7 fija el mecanismo.

**G1b no autorizado.** Intervención preregistrada en §11; **no ejecutar** hasta nueva fase explícita post-tag.

---

## 11. Preregistración — intervención P_pv / gate de alineación (GAP-4 fase 4.1)

**Estado:** ejecutado (familia 1a/1d/1e → **FAIL** / ABORT @ fix#2; H1e omisión PASS)  
**Fecha:** 2026-07-18  
**Prerequisito:** §10 diagnóstico cerrado; guardrail §3.3 implementado (`tools/gap4_abort_guardrail.py`)

### 11.0 Pregunta e hipótesis

> ¿Anular o atenuar el arrastre pos→vel vía P_pv mejora el outcome de velocidad en G1 pos+vel sin destruir casos donde la cruzada está bien orientada?

| ID | Intervención | Mecanismo |
|----|-------------|-----------|
| **H1a** | P_pv←0 (zero cruzada) si `effective_gap ≤ 1.0 s` | Proxy temporal (mejor Youden entre proxies en §10.4d) |
| **H1b** | P_pv←0 **incondicional** en todo accept GNSS | Control negativo — coste de destruir señal útil |
| **H1d** | P_pv←0 si **cos(dv_pos, err_pre) > 0** | Gate causal directo (`ALIGNMENT_PRIMARY`) |
| **H1d′** | P_pv←0 si **cos(Δv_total, err_pre) > 0** | Neto pos+vel mal alineado; canal vel no rescata gate |

**Definiciones (pre-corrección, sin look-ahead):**

- `err_pre` = **v_pred − v_GPS** (NE), con v_GPS de la medida entrante (speed + course).
- `dv_pos` = **K_vel,pos · y_pos** (2D NE), del bloque K del update conjunto pos+vel.
- `Δv_total` = **K_vel,· · y** (componente velocidad NE del gain completo 5D).
- `effective_gap` = Δt desde último accept GNSS; primer accept → Δt desde fix GNSS anterior (§10.4c).

**Descartados:** reset post-gap largo (§10.4c dirección falsada); 1c gap∧‖y_pos‖ (sobreajuste); G1b R-sweep; σ_v global.

### 11.1 Diseño experimental

| Parámetro | Valor fijado |
|-----------|--------------|
| Dataset | `data/real_run/` (Patrón Oro) |
| Modo GNSS | **pos+vel** (`--gnss-obs-mode pos_vel`) |
| R_vel | σ = 1.5 m/s (sin barrido) |
| Constraint / NHC | `--constraint-policy disabled`, `--nhc-policy enabled`, `--nhc-every-n-ticks 1` |
| Guardrail | §3.3 condicional activo (abort si k>0.5 **∧** Δerr_vel empeora) |
| Brazos | **1a, 1b, 1d, 1d′** — cuatro replays independientes, mismo seed/config salvo política P_pv |
| Orden ejecución | Aleatorio o fijo en script; **no** mirar resultados entre brazos antes de completar los cuatro |

**Cohorte primaria:** replay **G1** (8 accepts con speed en baseline abortado).

**Cohorte secundaria (ancla fix#56):** replay **G2 pos+vel** con la **misma** política P_pv por brazo — fix#56 ∉ accepts G1 pero es ancla causal de §10.4; evaluación preregistrada, no post-hoc.

**Baseline de referencia (ya corrido, no re-ejecutar salvo regresión):**

- G0 pos-only: 7 accepts — techo conservador pos/estabilidad.
- G1 pos+vel sin intervención: **ABORT** @ fix#2 (Δerr_vel = **+0,034 m/s**, guardrail).

### 11.2 Endpoints

**Primarios (por brazo, cohorte G1):**

1. **P1 — No abort:** guardrail §3.3 no dispara en ningún accept.
2. **P2 — RMSE velocidad:** RMSE de |v−v_GPS| sobre accepts G1 con speed ≤ **G0** (pos-only no tiene vel obs; usar RMSE post-update vs GPS speed en mismos timestamps accept).
3. **P3 — Superioridad vs proxy:** RMSE vel del brazo **≤ RMSE vel de 1a** (criterio decisivo para H1d vs proxy).

**Secundarios agregados:**

4. **S1 — Accepts:** n_accepts ≥ 8 (≥ G1 baseline).
5. **S2 — Estabilidad pos:** innov_h mean accepts ≤ G1 baseline + 10 %.
6. **S3 — P_vv @ fix#3:** no cae > 50 % vs G0 (guardrail M1, §1).

**Anclas nominales (FAIL si cualquiera falla, aunque agregados pasen):**

**Constante numérica preregistrada:** `ε_vel = 0,02 m/s` — tolerancia para comparaciones pairwise y desempates (mismo orden que criterios agregados).

| Ancla | Cohorte | Métrica | Baseline G1 (sin intervención) | PASS |
|-------|---------|---------|----------------------------------|------|
| **fix#2** | G1 | Δ\|v−v_GPS\| @ accept | **+0,034 m/s** (empeora; abort) | Δerr_vel **≤ −ε_vel** (mejora clara; no empate numérico) |
| **fix#7** | G1 | Δ\|v−v_GPS\| @ accept | **−0,066 m/s** (mejora) | Δerr_vel **≤** baseline G1 + ε_vel |
| **fix#56** | G2 suplementario | Δ\|v−v_GPS\| @ accept | cf. §10.4 | Δerr_vel **≤** G2 ref + ε_vel **y** no peor que 1b + ε_vel |

*fix#56:* comparar contra replay G2 pos+vel **sin** política P_pv (corrida única de referencia preregistrada al inicio de fase 4.1).

### 11.3 Criterios PASS / FAIL por brazo

**PASS H1d (hipótesis principal):** P1 ∧ P2 ∧ P3 ∧ **anclas fix#2, fix#7, fix#56** ∧ S1.

**PASS H1d′:** mismos criterios que H1d; además en **fix#7:**  
`Δerr_vel(1d′) < Δerr_vel(1d) − ε_vel` (mejora **clara** vs 1d, no empate por ruido float).

**PASS H1a:** P1 ∧ P2 ∧ anclas fix#2, fix#56 (fix#7 hereda baseline); suelo comparativo para P3 de 1d.

**PASS H1b:** solo para contraste — se espera **FAIL** en fix#56 y/o P3; PASS parcial en fix#2 no invalida H1d si 1d pasa.

**FAIL global del experimento:** ningún brazo cumple P1 (fix#2 sigue abortando) → revisar implementación antes de reinterpretar hipótesis.

### 11.4 Instrumentación obligatoria

Por accept GNSS (log JSONL + CSV existentes):

- `effective_gap_s`, `y_pos_norm_3d_m`, `P_pv_frob_pre/post`
- `cos_dv_pos_err_pre`, `cos_dv_tot_err_pre` (post-hoc audit OK; **gate usa solo pre-corrección**)
- `dv_pos`, `dv_vel`, `delta_err_vel_mps`, flags guardrail
- Política activa: `{arm: 1a|1b|1d|1d_prime, triggered: bool}`

### 11.5 Disciplina y límites

1. **No** ajustar umbrales (gap 1 s, cos>0) post-hoc tras ver RMSE.
2. **No** autorizar G1b R-sweep ni barrido σ_v en esta fase.
3. Tabla §10.4e **no** cuenta como evidencia de superioridad de H1d — solo motivación.
4. **Regla de desempate (preescrita, antes de ver resultados):** entre brazos {1a, 1d, 1d′} que cumplan PASS completo:
   - **(a)** gana el **menor RMSE vel** (P3);
   - **(b)** si |RMSE_a − RMSE_b| ≤ ε_vel → preferir **simplicidad:** **1d > 1d′ > 1a** (nunca implementar 1b salvo contraste);
   - **(c)** si 1d y 1d′ empatan en RMSE dentro de ε **y** 1d′ cumple fix#7 vs 1d → implementar **1d′**; si 1d′ falla fix#7 vs 1d → **1d**;
   - **(d)** si ninguno pasa P1 → FAIL global (§11.3); no reinterpretar hipótesis sin nueva preregistración.

### 11.6 Artefactos esperados

```
docs/benchmarks/gap4_gnss_velocity/G1_intervention/
  arm_1a_gap/
  arm_1b_unconditional/
  arm_1d_cos_pos/
  arm_1d_prime_cos_tot/
  G2_reference_posvel/          # fix#56 baseline sin gate
  gap4_intervention_report.json
```

Script: `tools/run_gap4_arm.py` (`--ppv-policy`), orquestador `tools/run_gap4_intervention.py` (`--run-all`, `--g2-reference`).  
Replay: `--p-pv-policy {none,gap_le_1s,zero,cos_pos,cos_tot,innov_h}` en `NaviCore3D_Replay.exe`.

---

## 11.7 Reformulación — criterio independiente de zona degradada (H1e)

**Estado:** ejecutado 2026-07-18 — **FAIL**  
**Motivo de la reformulación:** 1d/1d′ fallan por **omisión** en zona degradada (`cos≤0` cuando `innov_h` grande; 0 casos `trig=0 ∧ cos_pos>0` en 331 ticks). gps#20 muestra que cuando el gate sí dispara en degradada, **mejora** vel y pos — no es falso positivo.

### Hipótesis H1e (primaria tras reformulación)

> Anular P_pv cuando la innovación horizontal de posición supera un umbral **independiente de cos** identifica la zona degradada donde el arrastre pos→vel es dañino, sin depender del signo de alineación (que colapsa precisamente cuando más hace falta intervenir).

| ID | Intervención | Trigger (pre-corrección) |
|----|--------------|--------------------------|
| **H1e** | P_pv←0 | **`innov_h ≥ T_innov_h`** |

**Umbral (fijado aquí, antes de ejecutar H1e):**

```
T_innov_h = 50.0 m
```

**Justificación (solo baseline G1 sin intervención P_pv, congelado):**

| Cohorte G1 | innov_h |
|------------|---------|
| Accepts tempranos gps#1–7 | 22–31 m |
| Accept gps#37 | 148 m |
| Rejects p10 | ~138 m |

50 m queda **por encima** de accepts “sanos” tempranos y **por debajo** del cuerpo de rejects / accept tardío degradado — sin usar cos ni ajustar post-hoc tras ver RMSE de H1e.

**NIS:** se **loguea** (`nis_full`) para anatomía; **no** entra en el trigger (evita ambigüedad y/o y dependencia P→S). Criterio = solo `innov_h`.

### Relación con brazos previos

| Brazo | Rol tras reformulación |
|-------|------------------------|
| 1a, 1b | Controles (sin cambio) |
| 1d, 1d′ | Exploratorios / contraste — omisión conocida en degradada |
| **1e** | **Hipótesis primaria** |

### PASS H1e (preregistrado)

Mismos P1–P3 y anclas §11.2–11.3 que H1d, con:

- **P3′:** RMSE vel(1e) ≤ RMSE vel(1a) (vs proxy temporal).  
- **Omisión:** en ticks con `innov_h ≥ T` ∧ `cos_pos ≤ 0`, fracción `ppv_triggered=1` debe ser **≥ 0.95** (el punto de la reformulación).  
- **gps#20-class:** si existe accept con `innov_h ≥ T` ∧ `cos_pos > 0`, Δerr_vel y Δerr_pos deben **IMPROVE** (misma definición `gap4_abort_guardrail`).

**Desempate §11.5** ampliado: entre {1a, 1d, 1d′, 1e} que cumplan PASS completo → (a) menor RMSE vel; (b) empate ε → preferir **1e > 1d > 1d′ > 1a**.

### Artefacto adicional

```
docs/benchmarks/gap4_gnss_velocity/G1_intervention/arm_1e_innov_h/
```

CLI: `--p-pv-policy innov_h` (umbral compilado `NAVICORE_INS_EKF_PPV_INNOV_H_THRESHOLD_M`, default 50).

### Resultado H1e (2026-07-18)

| Criterio | Resultado |
|----------|-----------|
| **PASS global §11.7** | **FAIL** |
| P1 / P5 (no abort) | **FAIL** — `verdict_h1=ABORT` @ fix#2 (`k_vel≈0.965`, Δerr_vel≈+0.034 m/s) |
| Gate activo en fix#2 | **No** — `innov_h≈29.3 m < T=50`, `ppv_triggered=0` |
| Omisión (`innov_h≥T ∧ cos≤0` → trig) | **PASS** — 322/322 = **1.0** |
| Accepts | 13 (S1 numérico OK; no salva P1) |
| gps#20-class | N/A (0 accepts con `innov_h≥T ∧ cos_pos>0`) |
| Familia 1a / 1d | También **ABORT** en el mismo abort @ fix#2 |

**Lectura causal (no post-hoc de umbral):** la reformulación **sí** cierra la omisión en zona degradada. El abort primario de G1 ocurre en la cohorte “sana” temprana (`innov_h` 22–31 m), **fuera del dominio** del trigger `innov_h≥50`. H1e no podía prevenir fix#2 sin bajar T por debajo de los accepts tempranos — eso reabriría el dominio y **no** está autorizado sin nueva preregistración (§11.5).

**Artefactos:** `gap4_g1_innov_h_report.json`, `h1e_section11_verdict.json`.

---

## 11.8 Cierre GAP-4 — lectura agregada (cinco brazos)

**Fecha:** 2026-07-18  
**Estado:** **cerrado** — conclusión de investigación, no solo veredicto de H1e

### Patrón agregado

Las cinco variantes de gating P_pv preregistradas fallan en el **mismo** evento:

| Brazo | Trigger | Abort locus |
|-------|---------|-------------|
| 1a | `gap ≤ 1 s` | **fix#2** |
| 1b | incondicional | **fix#2** |
| 1d | `cos_pos > 0` | **fix#2** |
| 1d′ | `cos_tot > 0` | **fix#2** |
| 1e | `innov_h ≥ 50 m` | **fix#2** (gate **inactivo**: innov_h≈29 m) |

No son cinco modos de fallo distintos. Es **un único evento** ya caracterizado como `LEGITIMATE_HIGH_GAIN`: Kalman bayesianamente correcto, `cos>0`, la corrección de posición arrastra velocidad en dirección equivocada vía acoplamiento P_pv. Ninguna estrategia de gating — tiempo, alineación, ni magnitud de innovación — lo resuelve, porque el propio fix está **por debajo de cualquier umbral razonable de “sospechoso”** (`innov_h≈29 m`) y aun así produce corrección dañina por la geometría específica de ese instante.

### Respuesta a la pregunta original de GAP-4

> ¿Existe un criterio de gating que evite el arrastre dañino sin sacrificar los casos donde ayuda?

**Evidencia actual: no** — al menos no con ninguna variable derivada de cos / gap / magnitud entre las probadas bajo preregistración §11 / §11.7.

Eso es más fuerte que «OQ7 closed FAIL»: cierra la **formulación por gating** de la intervención P_pv, no solo el brazo H1e.

### Qué queda abierto (problema distinto, más acotado)

Tratar **fix#2** con una **intervención directa sobre el evento**, no con un gate general — mismo tipo de arreglo que `ZUPT_MAX_GAIN`: clamp de ganancia del término cruzado pos→vel cuando `k_vel` se acerca a 1, **independiente** del signo de cos.

| | Gate general (cerrado) | Clamp de ganancia cruzada (futuro) |
|--|------------------------|-------------------------------------|
| Disparo | cos / gap / ‖innov‖ | geometría de K (`k_vel` alto) |
| Dominio | “zona sospechosa” | evento de alta ganancia legítima |
| Analogía | — | `NAVICORE_INS_EKF_ZUPT_MAX_GAIN` |

**No implementar** en esta sesión. Candidata de trabajo futuro si se retoma OQ7 / GAP-4 intervención; preregistrar umbral y anclas antes de tocar código.

### Implicación de sesión

§11 cerrado. Siguiente paso formal: regresión E2E TUNNEL_STRESS/SLALOM con `p_pv_policy=none` (ninguna intervención experimental) y constraints del escenario Sim (no replay `forced_time`).

---

## 12. Historial

| Versión | Fecha | Notas |
|---------|-------|-------|
| 1.0 | 2026-07-18 | Protocolo preregistrado; evidencia F1/F1.1 inline; falsación explícita; G2 vel-only; R_vel; instrumentación obligatoria |
| 1.1 | 2026-07-18 | §1.0 ancla k_vel; F1d caveat n=2; §3.1 veredicto tres vías; ΔP/P abort complementario |
| 1.2 | 2026-07-18 | **G0 ejecutado** — 7 accepts, métricas §1/§6 reproducidas; `run_gap4_g0_baseline.py` |
| 1.3 | 2026-07-18 | **`--gnss-obs-mode`** (pos/pos_vel/vel_only) + R_vel σ=1.5 m/s; **G2 ejecutado** |
| 1.4 | 2026-07-18 | Guardarraíl §3.3 condicional (k∧Δerr componente observada); G2/G1 corridos; anatomía k_vel G2 |
| 1.5 | 2026-07-18 | Cierre diagnóstico §10 — álgebra 5D ALGEBRA_CONSISTENT; comparativa fix#2/#7 (n=2) |
| 1.6 | 2026-07-18 | Tercer punto G2 fix#56; colinealidad; veredicto alineación `ALIGNMENT_PRIMARY` (n=3); §10.5 candidata #1 P_pv reset |
| 1.7 | 2026-07-18 | §10.4c barrido n=33 G2; cos<0→helps generaliza; reset post-gap largo falsado en dirección; §10.5 → 1a short-gap vs 1b incondicional |
| 1.8 | 2026-07-18 | §10.4d discriminación gap vs ‖y_pos‖; gap≤1 s gana Youden; T=1 s fijado; 1c dual opcional |
| 1.9 | 2026-07-18 | §10.4e H1d gate cos directo; diseño 1a+1b+1d; 1c descartado |
| 2.0 | 2026-07-18 | §10.4e reetiquetado exploratorio; **§11 preregistración formal** 1a+1b+1d+1d′; anclas fix#2/#7/#56 |
| 2.1 | 2026-07-18 | §11 ε_vel=0,02 m/s; desempate preescrito; `--p-pv-policy` implementado |
| 2.2 | 2026-07-18 | **§11.7 H1e** — `innov_h ≥ 50 m`; primaria tras omisión 1d; NIS solo log |
| 2.3 | 2026-07-18 | **H1e ejecutado → FAIL**; omisión PASS; abort fix#2 fuera de dominio del gate |
| 2.4 | 2026-07-18 | **§11.8 cierre GAP-4**: gating falsificado (5/5 @ fix#2); candidata clamp tipo `ZUPT_MAX_GAIN` |