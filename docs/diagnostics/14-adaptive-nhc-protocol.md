# GAP-5 — Política NHC adaptativa (preregistración)

**Estado:** **CONGELADA** (v1.0) — tag `gap5-preregistration-frozen`; hook replay autorizado post-tag  
**Fecha congelación:** 2026-07-18  
**Prerequisito:** tag `gap4-diagnostic-complete`; [13-gap4-gnss-velocity-protocol.md](13-gap4-gnss-velocity-protocol.md) §10 congelado; [12-gap3-synthesis.md](12-gap3-synthesis.md)

---

## 0. Qué sabemos y qué no

**Sabemos qué NO hacer** (evidencia acumulada F1, F1.1, F1.2, autopsia P_pv):

| Enfoque | Veredicto |
|---------|-----------|
| Retocar Q hasta que funcione | ❌ descartado |
| Retocar R_GNSS | ❌ descartado |
| GNSS_MAX_GAIN / clamp K | ❌ descartado |
| Heurísticas ad hoc sobre NIS | ❌ descartado |
| Parchear 1d′ / ampliar gate P_pv | ❌ descartado (dominio no caracterizado para diseño) |

**No sabemos** con el mismo nivel de confianza cuál es la **solución definitiva**. La transición correcta es de autopsia mecanicista → **política de observación** derivada del mecanismo, no otro barrido de parámetros.

---

## 1. Modelo consolidado (cuatro problemas confirmados)

1. **NHC domina la evolución de P** — curva dosis-respuesta F1 (Γ, P_vv, accepts).
2. **El gate GNSS no falla porque K sea pequeño** — restaurar K no restaura accepts (F1.1).
3. **El problema terminal es el estado nominal** — innovación N crece, S_N cae, Λ_N explota (F1.2, autopsia NIS).
4. **P_pv no es un bug** — es acoplamiento EKF legítimo; modificarlo es **decisión de diseño**, no corrección (GAP-4).

**Consecuencia:** no diseñar un parche; diseñar una **política adaptativa** sobre cuándo ciertas observaciones entran con plena autoridad.

---

## 2. Intervención propuesta (única para PoC / PR)

**No tocar:** formulación EKF, Joseph, GNSS (R, gate), política P_pv (1d / 1d′ congelada).

**Tocar solo:** autoridad del **NHC** en función del estado interno del filtro.

Concepto:

```
if (estado_normal)  → NHC completo (cada tick, R nominal)
else                → NHC debilitado (frecuencia ↓ y/o R ↑)
```

### 2.1 Señal de régimen — elección PoC y limitación explícita

**Γ se emplea como señal de régimen por ser la magnitud mecanicista mejor caracterizada en GAP-3/GAP-4 (F1, dosis-respuesta). No se asume que sea la variable óptima de control; esa hipótesis forma parte del experimento.**

F1 usó Γ para **explicar** el equilibrio predict() vs NHC. Eso **no implica** que Γ sea la mejor señal para **controlar** la política. Contraejemplo conceptual:

| Caso | Γ | Λ_N / nominal | ¿Actuar? |
|------|---|---------------|----------|
| A | 18 (alto) | excelente | ¿Sí? — no obvio |
| B | 9 (moderado) | Λ_N creciendo rápido | ¿No? — tampoco obvio |

El PoC con Γ prueba: *«¿modular NHC por desequilibrio de covarianza mejora outcomes sin romper accepts/RMSE?»*  
**No prueba:** *«Γ es el controlador óptimo.»*

Candidatos reservados para **fase posterior** (no PoC):

| Señal | Rol |
|-------|-----|
| **‖P_pv‖ / P_vv** | ratio cruzado (GAP-4) |
| **dΛ_N/dt** o innov_N trend | proxy estado nominal (F1.2) |
| **NIS** reciente | consistencia update GNSS |
| **consistencia predict()** | fase 2 |

### 2.2 Modos de NHC debilitado (orden de preferencia post-PoC)

| Opción | Mecanismo | Notas |
|--------|-----------|-------|
| **A** | Frecuencia dinámica `every N(P)` | F1 demostró que frecuencia importa; **PoC usa solo esta** |
| **B** | Inflar R_NHC temporalmente | Limpio matemáticamente; fase posterior |
| **C** | `R = R₀ · f(Γ̄)` o `R₀ · f(‖P_pv‖/P_vv)` | Continuo; fase posterior |

---

## 3. Lazo cerrado — restricciones anti-confusión

El controlador propuesto cierra un lazo:

```
Γ̄  →  N (frecuencia NHC)  →  compresión P  →  Γ̄
```

Aunque sea solo replay, se imponen **desde el PoC** dos restricciones para no medir el controlador en lugar del filtro:

### 3.1 Suavizado de señal (no Γ por tick)

F1 mostró fenómenos **bursty**; un burst aislado no debe cambiar N al instante.

**Definición preregistrada:**

- **Γ̄** = media móvil de **Γ instantáneo** en ventana **T_w = 1.0 s** (tiempo de reloj IMU), **o** EWMA equivalente con τ = 1.0 s.
- **Γ instantáneo** (misma definición GAP-3 §8.16): en ventana deslizante corta interna,  
  `Γ_inst = Σ|ΔP_vv|_NHC / Σ|ΔP_vv|_predict`  
  (sumas acumuladas desde el último reset de ventana de 1 s; si denominador = 0 → no actualizar política).

**Prohibido en PoC:** umbrales evaluados sobre Γ de un solo tick o de un solo update NHC.

### 3.2 Histeresis (mecanismo del controlador, no hipótesis)

Evitar oscilación N=1 ↔ N=5 ↔ N=10 en el borde de umbral. Las transiciones usan **umbrales distintos al subir y al bajar** (ver §4.1 instancia inicial) más **dwell** (§3.3).

**Prohibido en PoC:** evaluar política sin histeresis (chatter invalidaría la comparación con B0).

### 3.3 Permanencia mínima (dwell)

- **T_dwell = 1.0 s** mínimo en un nivel N antes de permitir otra transición.
- El temporizador de dwell se reinicia **solo** al cambiar N efectivo.
- Durante dwell, Γ̄ se sigue acumulando pero **no** se aplican nuevas transiciones.

---

## 4. Experimento PoC (único pre-implementación)

**Objetivo:** prueba de concepto de **control adaptativo por régimen** — no calibración final de umbrales.

### 4.0 Hipótesis (científica)

> **H5-PoC:** Existe una política adaptativa de NHC, accionada por una señal mecanicista interna (Γ̄), que **mejora simultáneamente** los objetivos obligatorios (O1–O3) respecto a B0 y mejora al menos una métrica deseable (D*).

Equivalente operativo:

> *Adaptive NHC driven by Γ̄ improves the estimator* (sin romper accepts ni RMSE).

**No es parte de la hipótesis:**

- que **12** o **22** sean los umbrales correctos;
- que **Γ** sea la señal óptima de control (solo la señal PoC);
- la forma exacta de la tabla de transiciones.

Cambiar 22 → 20 en un futuro PoC **no cambia H5-PoC**; cambia la **instancia del controlador** (v2).

### 4.1 Política P0 — instancia inicial del controlador (no hipótesis)

Estado del controlador: **N ∈ {1, 5, 10}** (`--nhc-every-n-ticks`). Estado inicial: **N = 1**.

**Bandas objetivo** (después de histeresis + dwell sobre Γ̄ §3.1):

| Régimen Γ̄ | N efectivo |
|------------|------------|
| Γ̄ **<** 8 | 1 |
| 12 **≤** Γ̄ **<** 22 | 5 |
| Γ̄ **≥** 22 | 10 |

*(Banda 8–12: zona muerta de histeresis — no forzar transición.)*

**Transiciones preregistradas** (condición sostenida durante **T_dwell** §3.3):

| Transición | Condición sobre Γ̄ |
|------------|-------------------|
| 1 → 5 | Γ̄ **>** 12 |
| 5 → 10 | Γ̄ **≥** 22 |
| 10 → 5 | Γ̄ **<** 18 |
| 5 → 1 | Γ̄ **<** 8 |
| 10 → 1 | *(no directo)* — vía 5 |

Estos números son **Initial controller parameters** para el PoC v1.0. No optimizar post-hoc tras ver P0.

| N | Significado |
|---|-------------|
| 1 | NHC cada tick (autoridad plena) |
| 5 | NHC cada 5 ticks |
| 10 | NHC cada 10 ticks |

### 4.2 Baselines comparativos

| Brazo | Descripción |
|-------|-------------|
| **B0** | NHC cada tick (N=1 fijo) — referencia G1 |
| **B1** | NHC cada 5 ticks (N=5 fijo) — cota F1 estática |
| **P0** | Política adaptativa §4.1 |

**Config común:** `data/real_run/`, `--constraint-policy disabled`, `--nhc-policy enabled`, `--ppv-policy none`, GNSS pos+vel, sin intervención P_pv.

**Orden de ejecución:** B0 → B1 → P0 (fijo en script; no mirar P0 antes de completar B0/B1).

### 4.3 Criterios de éxito — jerarquía explícita

#### Nivel 1 — Obligatorios (PoC **falla** si cualquiera falla)

Evaluados P0 vs B0 en cohorte G1 (primeros 8 accepts con speed, o ventana preregistrada GAP-4):

| ID | Criterio | Operacionalización |
|----|----------|-------------------|
| **O1** | No empeorar RMSE | RMSE pos horizontal final ≤ B0 + **0.5 m** (margen mínimo anti-ruido) |
| **O2** | No reducir accepts | `count(accept) ≥ count(B0)` en misma ventana temporal |
| **O3** | No acelerar colapso P_vv | P_vv_frob pre fix#3 ≥ 0.9 × B0 (misma definición cov_step gnss/pre) |

**Veredicto PoC positivo** ⟺ O1 ∧ O2 ∧ O3 ∧ **al menos una** métrica deseable D* estrictamente mejor que B0.

#### Nivel 2 — Deseables (evidencia a favor del mecanismo)

| ID | Métrica | Comparación P0 vs B0 |
|----|---------|----------------------|
| **D1** | Γ̄ medio (replay) | menor |
| **D2** | Compresión P_vv | menor \|ΔP_vv\| acumulado pre fix#3 |
| **D3** | Deriva nominal | menor innov_h medio en accepts; menor max Λ_N en audit |
| **D4** | n_NHC | coherente con N efectivo (sanity) |

**Interpretación mixta prohibida:** si D1 mejora 80 % pero O2 falla → **PoC fallido**, no «parcialmente prometedor».

#### Nivel 3 — Diagnóstico del controlador (no criterio de éxito)

Registrar siempre en P0 (`controller_audit.csv` + agregados en informe JSON):

| Métrica | Uso |
|---------|-----|
| **Tiempo en N=1 / N=5 / N=10** (% replay) | ¿Política casi estática o activa? |
| **Número de transiciones** | ¿Chatter pese a histeresis/dwell? |
| **Duración media por estado N** | ¿Permanencia en régimen debilitado? |

Dos PoC pueden cumplir O1–O3 con historias distintas:

| Caso | RMSE | Actividad controlador |
|------|------|------------------------|
| A | OK | N=10 el **95 %** del tiempo |
| B | OK | N=1 **55 %**, N=5 **35 %**, N=10 **10 %** |

Ambos pasan endpoints obligatorios; el diagnóstico distingue *«casi siempre debilitado»* vs *«modulación fina»*.

### 4.4 Implementación permitida (post-freeze)

- **Solo capa replay:** hook que actualiza `ins_ekf_set_nhc_every_n_ticks()` según máquina de estados §3–§4.
- **Logging obligatorio del controlador:** por tick o cada 100 ms: `Gamma_inst`, `Gamma_bar`, `N_current`, `N_pending`, `dwell_s`, `transition_reason`.
- **Agregados post-run (P0):** `time_frac_N1`, `time_frac_N5`, `time_frac_N10`, `n_transitions`, `mean_dwell_N1_s`, `mean_dwell_N5_s`, `mean_dwell_N10_s`.
- **Prohibido:** cambios `ins_ekf.cpp` (Joseph/GNSS/update); P_pv; Q/R globales; retocar umbrales post-hoc tras ver P0.

### 4.5 Artefactos

```
docs/benchmarks/gap5_adaptive_nhc/
  baseline_n1/          # B0
  baseline_n5/          # B1
  poc_adaptive/           # P0
  gap5_adaptive_nhc_poc_report.json
  controller_audit.csv    # lazo cerrado — trazabilidad
```

Script previsto: `tools/run_gap5_adaptive_nhc_poc.py` (implementar post-tag).

---

## 5. Criterio de éxito de fase (post-PoC)

| Resultado | Acción |
|-----------|--------|
| O1–O3 OK + ≥1 D* mejor | Preregistrar PR producción (Opción A refinada o C); §11 P_pv sigue congelado |
| O* falla, D* mejora | **Rechazar** PoC; probar señal alternativa (‖P_pv‖/P_vv, dΛ_N/dt) en **nuevo** protocolo |
| O* OK, ningún D* | Mecanismo no confirmado; no PR producción |
| Mejora solo vs B1, no vs B0 | Insuficiente — B0 es referencia |

---

## 6. Trazabilidad y secuencia de trabajo

```
gap4-diagnostic-complete
        │
        ▼
  14-adaptive-nhc-protocol.md (v1.0 CONGELADA)
        │
        ▼
  tag gap5-preregistration-frozen
        │
        ▼
  hook replay + run_gap5_adaptive_nhc_poc.py
        │
        ▼
  tag gap5-poc-complete (solo tras informe JSON)
        │
        ▼
  §11 P_pv (separado; no mezclar brazos)
```

**Regla:** una hipótesis (§4.0), una palanca (NHC frecuencia), un experimento, una interpretación.

**Regla post-tag:** no modificar este protocolo v1.0. Retune de umbrales (p.ej. 22→20) = **controlador v2** + documento v1.1 + nuevo tag; **H5-PoC no cambia**.

---

## 7. Congelación

| Campo | Valor |
|-------|-------|
| Versión | **1.0** |
| Estado | **CONGELADA** — tag `gap5-preregistration-frozen` |
| Hipótesis congelada | H5-PoC (§4.0) + criterios O/D |
| Instancia congelada | Initial controller (§4.1) — parámetros numéricos, no la hipótesis |
| Cambios permitidos sin v1.1 | typos |
| Cambios que requieren v1.1 + nuevo tag | señal de control, T_w, T_dwell, bandas/transiciones, criterios O/D |

---

## Changelog

| Versión | Fecha | Notas |
|---------|-------|-------|
| 0.1 | 2026-07-18 | Borrador inicial post-freeze GAP-4 |
| **1.0** | **2026-07-18** | Congelada: disclaimer Γ, Γ̄ 1 s, histeresis, dwell, jerarquía O/D, hipótesis vs controlador, diagnóstico actividad |
