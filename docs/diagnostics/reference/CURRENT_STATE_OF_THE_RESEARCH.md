# Current State of the Research

**Documento:** artículo interno de consolidación (no README, no protocolo).  
**Rol del autor de esta pasada:** editor científico — ordenar lo existente, no generar conocimiento nuevo.  
**Fecha:** 2026-07-18  
**Métricas:** [RESEARCH_METRICS.md](RESEARCH_METRICS.md) — X=15, Y=11, Z=7, V=2, P=3, R=4  
**Fase actual:** **Stage I CERRADA (D21)** — baseline de identificación de régimen congelado; GAP-5 v3 no abierta  

---

## 1. Qué es este programa ahora

NaviCore-3D (eje EKF real-run) opera como **programa de investigación con trazabilidad explícita**: cada fase responde a una pregunta distinta y solo habilita la siguiente cuando la evidencia lo justifica.

**Cierre de etapa:** [STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md](STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md).

```
Observación → Hipótesis → Prerregistro → Instrumentación
→ Falsación → Congelación → Nueva pregunta
```

Cadena conceptual consolidada (D20–D21):

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
Controlador (GAP-5 v3)  ← Stage II (aún no)
```

El software (replay, audits, shell G1) es la **herramienta**.  
Las decisiones futuras deben citar un `OQ*` o un `K*` — no “probar algo”.

Cronología: [SCIENTIFIC_CHRONOLOGY.md](SCIENTIFIC_CHRONOLOGY.md).  
Dependencias: [DEPENDENCY_MAP.md](DEPENDENCY_MAP.md).  
Síntesis H6: [regime_model.md](../../benchmarks/gap5_v2_observable_selection/regime_model.md).

---

## 2. Qué sabemos (conocimiento consolidado)

Fuente normativa: [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md) (K1–K15).

### 2.1 Núcleo mecanicista (GAP-3 / F1\*)

- NHC modula compresión de `P_vv` y `k_vel` (K1).  
- NHC ON vs OFF domina el conteo de accepts (no ZUPT solo) (K2).  
- Restaurar `k_vel` **no** restaura accepts (K3).  
- Rechazos de gate asociados a innovación / Λ (formulación K4 en STATE).  
- Cliff NHC bursty y state-conditioned (K5); Γ offline NHC≫predict en gap F1 (K6); Joseph explica solo parte del drop (K7).

### 2.2 Acoplamiento `P_pv` (GAP-4)

- `P_pv` es covarianza cruzada legítima, no bug (K8).  
- En la región de accepts múltiples, política `P_pv` en fix#4 bifurca trayectorias (K9).  
- `cos` / gate: valores del log del filtro son autoridad (K10).

### 2.3 Observable Γ̄ (GAP-5 v1)

- Γ_inst sigue a Γ offline en config F1-equivalente (K11).  
- Γ̄ (EWMA) **no** preserva el burst para control — fallo de operacionalización (K12).  
- Γ offline **no** es invariante bajo config PoC pos_vel (K13).

### 2.4 Validación externa (G-ext)

- El **núcleo de bloqueo** reaparece en recorrido independiente bajo shell G1 (K14).  
- Puede coexistir GNSS externo “limpio” (~506 s) con reject interno continuo (K15).

**Formulación sólida (INTERPRETATION):**  
G-ext reproduce el mecanismo de bloqueo bajo un recorrido independiente, **aunque no** reproduce toda la secuencia causal de G1.

### 2.5 Representación / régimen (H6 / D20)

No es un `K*` nuevo todavía; es el entregable de síntesis:

- **B1:** el protocolo discrimina candidatos (escenario A descartado).  
- **B2:** colinealidad parcial O1–O2 en C7 (resultado metodológico).  
- **Mapa parcial:** R1←O1; R3←O3 provisional; R2←hueco; O2 fuera como eje distinto.  
- **Cardinalidad:** indeterminada (ni mono-Oi suficiente ni H7-MIN necesaria demostradas).

**Contribución de fase (D20):**  
*La contribución principal de H6 no es identificar un controlador, sino transformar un conjunto de candidatos independientes en un modelo parcial del espacio de regímenes del filtro.*

---

## 3. Qué quedó refutado (no reabrir el mismo framing)

Fuente: [RESEARCH_MAP.md](RESEARCH_MAP.md) + lista Y en [RESEARCH_METRICS.md](RESEARCH_METRICS.md).

Entre otras:

- Menos NHC → más accepts.  
- Restaurar K → restaurar accepts.  
- Decimación elimina el burst.  
- Dominancia solo-ZUPT / solo-Joseph / cliff=solo frecuencia.  
- `P_pv` es un bug.  
- Arreglar retuneando solo R/Q/K.  
- Γ̄ v1 como detector operacional del régimen (H-ops).  
- “Subir umbral Γ̄” como lectura primaria del fallo v1.  
- “G-ext confirma G1 de extremo a extremo”.  
- O2 como eje de régimen distinto de O1 bajo C1–C7 (H6 / D19–D20).

---

## 4. Qué no sabemos (preguntas abiertas)

Fuente: [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md).

| ID | Pregunta (resumen) | Estado |
|----|--------------------|--------|
| OQ1 | Propiedad / representación del régimen | **Parcialmente abierta** (mapa parcial ≠ cierre) |
| OQ2 | ¿Conjunto mínimo (H7-MIN)? | **Abierta** (evidencia no obliga sí ni no) |
| OQ3 | Invarianza C-F1 ↔ C-PoC del modelo completo | Parcialmente acotada |
| OQ4 | Modelo de régimen explícito | **Parcialmente respondida** (`regime_model.md`) |
| OQ5–OQ6 | Operacionalización / política adaptativa | v3+ — **no abiertas** |
| OQ7 | Intervención `P_pv` §11 | Separado; no mezclar con v2 |

**No adoptar todavía:** “Norte → longitudinal” (D12).

---

## 5. Qué sigue dependiente del escenario (límites explícitos)

- Secuencia concreta G1 (multi-accept, fix#4 / bifurcación): G-ext **no entra** → no valida ni invalida K9.  
- Dominancia del eje Norte como en G1: **no se reproduce** limpiamente en G-ext — no consolidar como universal ni como refutación de K4.  
- Hueco **R2** en representación H6 (densidad GNSS / O4): limitación de datos + mapa incompleto.

---

## 6. Decisiones que cerraron la fase de revisión → H6 → síntesis

| Decisión | Rol |
|----------|-----|
| **D13–D17** | Evidence reviews → pausa cerrada; frase justificación H6 |
| **D18** | Preflight + bindings; caracterización |
| **D19** | Consistencia de artefactos (pre-síntesis) |
| **D20** | `regime_model.md`; OQ1 parcialmente abierta; contribución de fase |
| **D21** | **Stage I cerrada** — baseline científico de etapa |

GAP-5 v3 **no** parte de una intuición sobre qué controlar, sino de un modelo explícito —aunque incompleto— del régimen interno. Eso reduce el riesgo de volver a ajustes empíricos de umbrales sin base mecanicista.

---

## 7. Dónde está cada tipo de verdad

| Pregunta del lector | Documento |
|---------------------|-----------|
| ¿Qué está confirmado? | STATE_OF_KNOWLEDGE |
| ¿Qué está abierto? | OPEN_QUESTIONS |
| ¿Modelo de régimen? | `regime_model.md` |
| ¿Cierre de etapa? | STAGE_I_REGIME_IDENTIFICATION_CLOSURE |
| ¿Consistencia H6? | `H6_ARTIFACT_CONSISTENCY_REVIEW.md` |
| ¿Qué fase dio qué outcome? | RESEARCH_MAP / SCIENTIFIC_CHRONOLOGY |
| ¿Estado operativo? | RESEARCH_STATUS |
| ¿Números del programa? | RESEARCH_METRICS |
| ¿G-ext? | `docs/benchmarks/real_run_19082026_baseline/INTERPRETATION.md` |
| ¿Protocolo v2? | `16-gap5-v2-observable-selection.md` |

---

## 8. Historia que este documento debe preservar

> Primero entendimos el mecanismo. Después comprobamos qué partes eran generales. Luego falsamos una operacionalización concreta. Solo entonces caracterizamos candidatos y obtuvimos un **modelo parcial** del espacio de regímenes — no un ganador. El controlador viene después, si la evidencia lo habilita.

Cualquier intervención futura que salte un paso de esa secuencia debe quedar registrada como excepción en DECISION_LOG — no como atajo silencioso.

---

## 9. Fuera de alcance de esta consolidación

- Abrir GAP-5 v3 / controlador (requiere Stage II + frase de handoff).  
- Forzar cierre completo de OQ1.  
- Combinar Oi post-hoc sin O6 preregistrado.  
- Borrar artefactos listados en REDUNDANCY_INVENTORY.  
- Rediscutir framing de Stage I §6 sin nueva evidencia.
