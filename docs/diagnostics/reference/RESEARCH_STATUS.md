# Research Status — Cadena explicativa (post G-ext)

**Tipo:** documentación de referencia — **mapa de fase del proyecto**, no protocolo experimental.  
**Fecha:** 2026-07-19  
**Estado:** **Stage I CERRADA (D21).** **Pausa D22.** Hito metodológico Stage II: método reproducible (no centrado en una implementación). Una página: [RESEARCH_PROGRAM.md](../../ekf_explorer/RESEARCH_PROGRAM.md). Regla docs: no crear documento sin pregunta nueva. GAP-5 v3 / OQ8 / EKF v2 **no** abiertos.

| Línea | Estado |
|-------|--------|
| EKF / regímenes | Cerrada — modelo parcial del filtro |
| Jacobiano / H-ATT-d | Acotada: espacio de instrumentos para contrastar H-ATT-d delimitado; hipótesis abierta; cand1 dominio-caracterizado |

---

## Cadena de revisión (2026-07-18)

| Doc | Rol |
|-----|-----|
| [EVIDENCE_REVIEW.md](EVIDENCE_REVIEW.md) | K×G1×G-ext |
| [CRITICAL_EVIDENCE_REVIEW.md](CRITICAL_EVIDENCE_REVIEW.md) | Ataque adversario |
| [EVIDENCE_STRENGTH_AUDIT.md](EVIDENCE_STRENGTH_AUDIT.md) | Peso por K + supervivencia OQ + **frase criterio H6** |

**Frase justificación H6 (D17):**  
*No existe evidencia suficiente para discriminar entre los observables candidatos mediante los experimentos ya realizados.*

| Pregunta | Veredicto |
|----------|-----------|
| ¿OQ1 abierta? | **Sí** |
| ¿OQ3/OQ5? | Acotadas (Γ / Γ̄ v1); no cerradas |
| ¿H6 por inercia? | **No** |
| ¿Pausa cerrada? | **Sí** (D17) |
| ¿OQ1? | **Parcialmente abierta** (heredada a Stage II) |
| ¿Stage I? | **Cerrada (D21)** |
| ¿Siguiente? | **Stage II (lab):** campañas V1–V5 — no rediseñar EKF. Puerta: onset+variables en ~5 min ([STAGE2_VALIDATION](../../ekf_explorer/STAGE2_VALIDATION.md)). GAP-5 v3 / EKF v2 solo con decisión explícita + evidencia |
| ¿EKF v2? | **No ahora (~10–20% justificado).** Evidencia para *seguir investigando*, no para tirar el filtro. Stage II: lab días→minutos hasta onset; luego bifurcar corrección local vs v2. Ver [STAGE2_VALIDATION](../../ekf_explorer/STAGE2_VALIDATION.md). |

---

## Qué ha cambiado

El trabajo ya no busca «hacer que el filtro funcione mejor» en cada iteración.  
Construye un **modelo explicativo** del EKF y comprueba qué partes **sobreviven** al cambiar de condiciones.

Secuencia normativa (no saltar pasos):

```
mecanismo → generalidad (recorrido independiente) → observable → controlador
```

Historia que debe poder leerse dentro de meses:

> Primero entendimos el mecanismo. Después comprobamos qué partes eran generales mediante un recorrido independiente. Solo entonces buscamos un observable que preservara ese mecanismo entre configuraciones. Y únicamente después diseñamos un controlador.

El valor del trabajo es ese modelo mecanicista validado paso a paso — no el % de mejora de un controlador futuro.

---

## Cadena cerrada → pregunta actual

| Fase | Pregunta | Estado |
|------|----------|--------|
| GAP-3 | ¿Qué mecanismo degrada el EKF? | **Respondida** |
| GAP-4 | ¿`P_pv` es bug o mecanismo? | **Respondida** (mecanismo) |
| GAP-5 v1 | ¿Γ̄ sirve como observable operacional? | **Respondida** (negativa / operacionalización) |
| G-ext | ¿El mecanismo de bloqueo reaparece fuera de G1? | **Respondida** (parcialmente positiva — K14/K15) |
| **GAP-5 v2** | ¿Qué observable interno permanece coherente cuando la calidad externa del GNSS deja de explicar el régimen? | **Preregistrada — benchmark no abierto** |

H6-OBS ya no nace solo de necesidad interna: nace también de evidencia (dos recorridos; GNSS externo ≠ régimen interno).

---

## Distinción obligatoria

### A. Conocimiento consolidado

Solo lo listado en [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md). Resumen operativo:

| Tema | Ancla |
|------|-------|
| Mecanismo NHC / compresión `P_vv` / gate | K1–K7 |
| `P_pv` legítimo (no bug); bifurcación en región fix#4 | K8–K10 |
| Fracaso operacional de Γ̄ / invarianza Γ | K11–K13 |
| Validación externa del **bloqueo** (no secuencia G1 completa) | **K14** |
| GNSS limpio externo + reject interno continuo | **K15** |

**Robusto (G1 ∩ G-ext):** compresión `P_vv`, crecimiento `P_pv`, estrés del gate (Λ elevada), pérdida de reenganche.

**Todavía dependiente del escenario (no consolidar como universal):** secuencia concreta G1 (fix#4 / bifurcación — G-ext no entra en esa región); qué componente de la innovación domina.

### B. Preguntas abiertas

Solo lo listado en [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md):

| ID | Tema |
|----|------|
| OQ1 | Observable / propiedad de régimen (H6-OBS) |
| OQ2 | Conjunto mínimo (H7-MIN) |
| OQ3 | Invarianza C-F1 ↔ C-PoC |
| OQ4 | Modelo de régimen explícito |
| OQ5–OQ6 | Controlador (GAP-5 v3+, **no ahora**) |
| OQ7 | Intervención `P_pv` §11 | **Cerrada** — gating falsificado (§11.8); candidata futura = clamp tipo ZUPT_MAX_GAIN |
| OQ8 | Jacobiano NHC × IMU E2E | **Abierta — en pausa D22** · mecanismo early cerrado · H-ATT-d intacta · cand1 no generaliza ([CAND1 review](../../benchmarks/jacobian_imu_ab/CAND1_GENERALIZATION_REVIEW.md)) · reformulación científica diferida: *¿existe onset invariante entre dominios?* — **no** abrir experimento ahora |
| OQ9 | Régimen tardío SLALOM | **Abierta** — no fusionar con early vía bias (artefacto latch §12.9) |

**Regla:** no reabrir A dentro de B. Si alguien propone retune Q/R/K, “arreglar Γ̄ subiendo umbral”, “subir T₂ de cand1”, o “Norte → longitudinal” como hecho — rechazar y apuntar a D2 / K12 / D12 / D22 / [patrón operacionalización](OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md).

---

## Decisión estratégica actual

**Stage I cerrada (D21).** No hay próximo experimento urgente. No abrir GAP-5 v3 hasta preregistro explícito de Stage II.

Baseline: [`STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md`](STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md).

Cualquier v3 futuro asume el modelo parcial H6 y **no** reabre identificación de régimen.

**Contribución D20:** *La contribución principal de H6 no es identificar un controlador, sino transformar un conjunto de candidatos independientes en un modelo parcial del espacio de regímenes del filtro.*

---

## Enlaces

| Rol | Archivo |
|-----|---------|
| **Artículo estado** | [CURRENT_STATE_OF_THE_RESEARCH.md](CURRENT_STATE_OF_THE_RESEARCH.md) |
| Cronología científica | [SCIENTIFIC_CHRONOLOGY.md](SCIENTIFIC_CHRONOLOGY.md) |
| **Métricas X/Y/Z + V/P/R** | [RESEARCH_METRICS.md](RESEARCH_METRICS.md) |
| Dependencias | [DEPENDENCY_MAP.md](DEPENDENCY_MAP.md) |
| Confirmado | [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md) |
| Abierto | [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) |
| Decisiones | [DECISION_LOG.md](DECISION_LOG.md) |
| Fases | [RESEARCH_MAP.md](RESEARCH_MAP.md) |
| G-ext | [INTERPRETATION.md](../../benchmarks/real_run_19082026_baseline/INTERPRETATION.md) |
| Preflight H6 | [H6_PREFLIGHT.md](H6_PREFLIGHT.md) |
| Consistencia artefactos H6 (D19) | [H6_ARTIFACT_CONSISTENCY_REVIEW.md](../../benchmarks/gap5_v2_observable_selection/H6_ARTIFACT_CONSISTENCY_REVIEW.md) |
| Modelo de régimen (D20) | [regime_model.md](../../benchmarks/gap5_v2_observable_selection/regime_model.md) |
| **Cierre Stage I (D21)** | [STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md](STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md) |
| Artefactos H6 (numéricos) | [observable_characterization.md](../../benchmarks/gap5_v2_observable_selection/observable_characterization.md) |
| Protocolo v2 (congelado v1.2) | [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) |
