# Stage I — Cierre: identificación del régimen del EKF

**Tipo:** baseline científico de etapa (no protocolo experimental, no conocimiento nuevo).  
**Decisión:** D21  
**Fecha:** 2026-07-18  
**Estado:** **ETAPA CERRADA**  
**Tag sugerido (cuando se versionen estos artefactos):** `stage-I-regime-identification-closed`

> **Fin de la etapa de identificación del régimen del EKF.**

Este documento congela la **etapa completa**, no solo archivos sueltos. Cualquier trabajo futuro (incluido GAP-5 v3) debe tratarlo como baseline asumido o registrar una excepción explícita en [DECISION_LOG.md](DECISION_LOG.md).

---

## 1. Alcance de la etapa

**Dentro:**

- Mecanismo de deterioro / bloqueo del filtro en real-run (GAP-3 / F1\*).  
- Papel estructural de `P_pv` (GAP-4).  
- Falsación de Γ̄ (EWMA τ=1 s) como operacionalización de control (GAP-5 v1).  
- Validación externa del núcleo de bloqueo (G-ext).  
- Caracterización de observables candidatos y modelo **parcial** del espacio de regímenes (H6 / D18–D20).

**Fuera (explícitamente no es esta etapa):**

- Diseño o evaluación de políticas de control / NHC adaptativo (GAP-5 v3).  
- Intervención `P_pv` como política (§11 / OQ7) mezclada con selección de observable.  
- Remapeo Norte→longitudinal como hecho consolidado.  
- Optimización de umbrales, Q/R/Joseph, o “hacer que el filtro acepte más”.

---

## 2. Narrativa de la etapa (preguntas y estado)

| Fase | Pregunta | Estado al cierre |
|------|----------|------------------|
| GAP-3 | ¿Qué mecanismo provoca el deterioro? | **Cerrada** |
| GAP-4 | ¿Qué papel juega `P_pv`? | **Cerrada** (en su dominio) |
| GAP-5 v1 | ¿Γ̄ sirve como operacionalización? | **Refutada** (ops correcta) |
| G-ext | ¿El mecanismo reaparece fuera de G1? | **Validado parcialmente** (núcleo sí; secuencia causal completa no) |
| H6 | ¿Qué observable representa el régimen? | **Modelo parcial** obtenido; OQ1 parcialmente abierta |

Cadena conceptual congelada:

```
Fenómeno
    ↓
Mecanismo (GAP-3)
    ↓
Estructura (GAP-4)
    ↓
Operacionalización (GAP-5 v1)
    ↓
Generalidad (G-ext)
    ↓
Modelo parcial de régimen (H6)
    ↓
—— fin Stage I ——
    ↓
Controlador (GAP-5 v3)  ← Stage II (aún no abierta)
```

**Contribución de cierre (D20):**  
*La contribución principal de H6 no es identificar un controlador, sino transformar un conjunto de candidatos independientes en un modelo parcial del espacio de regímenes del filtro.*

---

## 3. Preguntas respondidas (no reabrir el framing)

Fuente: [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md) (K1–K15), [RESEARCH_MAP.md](RESEARCH_MAP.md), síntesis H6.

Entre lo que **sí** puede darse por respondido en esta etapa:

- NHC modula compresión de covarianza / `k_vel` y domina accepts vs ZUPT solo (K1–K2).  
- Restaurar K no restaura accepts (K3); Joseph no explica todo el drop (K7).  
- `P_pv` es estructura legítima, no bug; en región multi-accept bifurca trayectorias (K8–K9).  
- Γ_inst sigue a Γ offline en F1-equivalente; Γ̄ v1 **no** es detector operacional del burst (K11–K12).  
- Γ offline no es invariante bajo config PoC (K13).  
- Núcleo de bloqueo reaparece en recorrido independiente; GNSS “limpio” puede coexistir con reject interno (K14–K15).  
- H6 discrimina candidatos (B1); O1≡O2 en C7 es colinealidad metodológica (B2).  
- Mapa parcial: **R1←O1**; **R3←O3** provisional; **R2←hueco**; **O2 fuera** como eje distinto.

Lista ampliada de framing refutados: [CURRENT_STATE_OF_THE_RESEARCH.md](CURRENT_STATE_OF_THE_RESEARCH.md) §3.

---

## 4. Preguntas abiertas al cierre (heredan a Stage II+ sin reabrir Stage I)

Fuente: [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md).

| ID | Estado | Nota |
|----|--------|------|
| **OQ1** | Parcialmente abierta | Mapa parcial ≠ propiedad única / conjunto mínimo |
| **OQ2** | Abierta | H7-MIN: evidencia no obliga “uno basta” ni “hace falta conjunto” |
| **OQ3** | Parcialmente acotada | C3 ordinal OK en brazo H6; modelo completo R1–R4 no |
| **OQ4** | Parcialmente respondida | [`regime_model.md`](../../benchmarks/gap5_v2_observable_selection/regime_model.md) |
| **OQ5–OQ6** | No abiertas | Solo Stage II (control) |
| **OQ7** | Separada | No mezclar con H6 / v3 de control sin decisión explícita |

Hueco explícito de representación: **R2** (consistencia nominal) sin ancla clara bajo densidad GNSS actual.

---

## 5. Qué puede asumir cualquier trabajo futuro

Un documento, experimento o preregistro posterior a Stage I **puede asumir**:

1. El mecanismo de bloqueo / deterioro descrito en K1–K7 (con los límites de escenario ya documentados).  
2. `P_pv` como estructura, no como bug (K8–K10).  
3. Que Γ̄ v1 (EWMA τ=1 s) **no** es la operacionalización de control del régimen (K12).  
4. Que el núcleo de bloqueo no es un artefacto exclusivo del trayecto G1 (K14), sin exigir reproducción de toda la secuencia G1 (INTERPRETATION G-ext).  
5. El **modelo parcial** de régimen H6/D20 como baseline de representación:  
   `R1 ← O1`; `R3 ← O3` (provisional); `R2 ← hueco`; `O2` no es eje distinto bajo C1–C7.  
6. Vocabulario Paso 0: Evaluada / Refutada / No refutada / Inconclusa (nunca «confirmada» por una marca suelta).  
7. La cadena metodológica: no saltar a controlador sin citar este baseline o una excepción en DECISION_LOG.

Artefactos canónicos de asunción:

| Rol | Path |
|-----|------|
| Cierre de etapa | este archivo |
| Modelo de régimen | `docs/benchmarks/gap5_v2_observable_selection/regime_model.md` |
| Consistencia pre-síntesis | `.../H6_ARTIFACT_CONSISTENCY_REVIEW.md` |
| Conocimiento K | `STATE_OF_KNOWLEDGE.md` |
| Decisiones | `DECISION_LOG.md` (hasta D21) |

---

## 6. Qué ya no debe volver a discutirse salvo nueva evidencia

Sin experimento/prerregistro nuevo que falsifique o extienda el dominio:

- “Menos NHC → más accepts” / “restaurar K restaura accepts”.  
- “`P_pv` es un bug”.  
- “Arreglar retuneando solo R/Q/K”.  
- “Γ̄ v1 detecta el régimen para control” / “subir el umbral arregla v1”.  
- “G-ext confirma G1 de extremo a extremo”.  
- “O2 (Γ̄) es un eje de régimen independiente de O1 bajo el benchmark H6”.  
- Reabrir el **catálogo H6** (O1–O5, C1–C7, falsación) como si no hubiera corrido — o reinterpretar ordinales “a ojo” contra el binding D18.  
- Tratar OQ1 como **cerrada** o H7-MIN como **confirmada** sin nueva evidencia.  
- Empezar un controlador “porque hace falta controlar algo” sin anclarse al mapa parcial §5.

---

## 7. Handoff a Stage II (GAP-5 v3) — no abierta aún

**No se abre GAP-5 v3 en D21.**

Cuando se abra, la **primera frase** de su preregistración debe ser del estilo:

> GAP-5 v3 no investiga el régimen del filtro. Asume como baseline el modelo parcial obtenido en H6 (Stage I / D20–D21) y estudia exclusivamente políticas de control compatibles con dicho modelo.

Eso impide que v3 reabra debates de H6. Cualquier necesidad de revisar el régimen exige **nueva etapa / nuevo prerregistro**, no un lado en el protocolo de control.

---

## 8. Niveles de conocimiento en el repositorio

Tras este cierre, orientar el árbol documental así (contenido sin moverse a la fuerza; el índice manda):

| Nivel | Qué es | Dónde |
|-------|--------|-------|
| **1. Baselines** | Congelados, asumibles | Este cierre · `regime_model.md` · tags de prerregistro · INTERPRETATION G-ext |
| **2. Programa** | K, OQ, GAP, D, mapas | `docs/diagnostics/reference/` |
| **3. En curso** | Experimentos activos | Vacío al cierre Stage I; futuro v3 bajo `docs/benchmarks/` + protocolo nuevo |

Índice: [README.md](README.md).

---

## 9. Firma de cierre

| Campo | Valor |
|-------|-------|
| Etapa | Stage I — Identificación del régimen del EKF |
| Cierre | D21 |
| Síntesis H6 | D20 |
| OQ1 | Parcialmente abierta (intencional) |
| Siguiente etapa | GAP-5 v3 / control — **solo** tras preregistro con la frase de §7 |

No añade conocimiento nuevo. Protege el adquirido y convierte GAP-5 v3 en el **inicio** de una etapa nueva, no en la continuación difusa de esta.

---

## 10. Cómo citar esta etapa

**Baseline científico:** `stage-I-regime-identification-closed`

Cualquier trabajo posterior (GAP-5 v3, GAP-6 o experimentos independientes) debe referenciar este baseline cuando utilice alguno de sus resultados como hipótesis de partida.

Ejemplo de referencia en un prerregistro futuro:

> Baseline = Stage I (`stage-I-regime-identification-closed`). Se asumen K1–K15 y el modelo parcial de régimen (D20) según [STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md](STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md). No se reabre la identificación del régimen salvo nueva evidencia.
