# Evidence Review — K1–K15 × G1 × G-ext

**Tipo:** revisión sistemática interna. **No genera datos nuevos.**  
**Fecha:** 2026-07-18  
**Fuentes:** STATE_OF_KNOWLEDGE, RESEARCH_MAP, INTERPRETATION G-ext, FOUR_QUESTIONS, docs 12/13/15.  
**Pregunta única de cierre de esta revisión:** ¿OQ1 sigue siendo una pregunta abierta de verdad? → §3.

**Leyenda cobertura**

| Símbolo | Significado |
|---------|-------------|
| **G1** | Evidencia primaria en recorrido/referencia GAP-3–5 / `gap4…/G1` |
| **G-ext** | Evidencia en `real_run_19082026_baseline` (shell G1) |
| **—** | No re-testeado / no aplicable en ese dataset |
| **N/A región** | Experimento no entra en la región de estado requerida |

---

## 1. Tabla por K

| K | Afirmación (corta) | Experimento / ancla que lo soporta | ¿G1? | ¿G-ext? | Evidencia en contra / límite | OQ relacionada |
|---|-------------------|-------------------------------------|:----:|:-------:|------------------------------|----------------|
| **K1** | Frecuencia NHC modula compresión `P_vv` / `k_vel` | F1 dose–response | **Sí** | **No** (dose). Floor NHC es consistencia hermana, no soporte de K1 — ver crítica | — | — (cerrado) |
| **K2** | NHC ON vs OFF domina accepts (no ZUPT solo) | Constraint matrix A–E | **Sí** | **—** | — | — |
| **K3** | Restaurar `k_vel` no restaura accepts | F1, F1.1 | **Sí** | **—** | — | Cerrado (también lista OPEN closed) |
| **K4** | Rechazo gate dominado por innovación / eje Norte (`contrib_N`, `Λ_N`) | F1.1 | **Sí** (G1; scope note STATE) | **Tensión** — `Λ` elevada sí; dominancia **Norte** no reproducida | No leer K4 como Norte universal | Matiz para H6; **no** cierra OQ1 |
| **K5** | Cliff NHC bursty, state-conditioned | F1.2, autopsia fix#2→#3 | **Sí** | **No** (bursty). Floor ≠ burst (crítica) | Coherente con K13 | — |
| **K6** | En gap F1, erosión NHC ≫ predict (Γ≈19.7) | F1 baseline cov_step | **Sí** (pos-only) | **—** / no comparable directo en pos_vel | Bajo PoC, Γ≈0.13 (K13) | OQ3 (invarianza) |
| **K7** | Joseph explica solo parte del drop @ fix#2 | GAP-3.14 | **Sí** | **—** (sin cadena multi-accept análoga) | — | — |
| **K8** | `P_pv` legítimo, no bug | GAP-4 autopsy | **Sí** | **Consistente** (crece `P_pv`); no re-prueba «no bug» (crítica) | — | OQ7 (intervención, no validez de K8) |
| **K9** | fix#4 política `P_pv` bifurca trayectorias | GAP-4 divergence / truth table | **Sí** | **N/A región** — sin 2º accept; no valida ni invalida | Explicit non-claim INTERPRETATION | OQ7 |
| **K10** | cos/gate: log del filtro es autoridad | Autopsy gps#32 | **Sí** (metodológico) | **—** | — | — |
| **K11** | Γ_inst ≈ Γ offline en config F1-eq | GAP-5 v1 `p0_passive_f1_bridge` | **Sí** | **—** | — | — |
| **K12** | Γ̄ no preserva burst para control (H-ops) | GAP-5 v1 passive | **Sí** | **—** | Refuta **esa** operacionalización, no “todo observable interno” | Cierra un candidato; **abre** necesidad de OQ1 |
| **K13** | Γ offline no invariante bajo PoC pos_vel | GAP-5 v1 f1-bridge vs PoC | **Sí** (dos configs, mismo CSV G1) | **—** como segundo CSV | — | **OQ3** |
| **K14** | Núcleo de bloqueo reaparece en trayecto independiente | G-ext Phase B + INTERPRETATION | **Comparando con** G1 | **Sí** (definición) | No afirma secuencia G1 completa | Motiva OQ1; no la responde |
| **K15** | GNSS limpio externo + reject interno continuo | G-ext Phase A+B | **Débil / no equivalente** en G1 (calidad y régimen covariaban más) | **Sí** | — | **Motiva fuerte OQ1**; no elige observable |

---

## 2. Síntesis de cobertura (sin reinterpretar)

| Clase | Ks | Lectura |
|-------|----|---------|
| Solo G1 (o metodológico G1) | K2, K3, K6*, K7, K9†, K10, K11, K12, K13 | Siguen válidos como conocimiento congelado; G-ext no los re-prueba |
| Consistencia / tensión en G-ext (no réplica del K) | K4 (tensión eje), K8 (consistencia P_pv) | Ver [CRITICAL_EVIDENCE_REVIEW.md](CRITICAL_EVIDENCE_REVIEW.md) |
| Propios / fuertes en G-ext | K14, K15 | Validación externa + desacople externo/interno |
| No interrogable en G-ext | K9 | Región de estado ausente |

\*K6 es específico de config F1 pos-only.  
†K9: N/A región, no refutación.

**Generalidad ya ganada (núcleo INTERPRETATION, no cada K1–K7):** `P_vv`↓, `P_pv`↑, gate stress (Λ), no-reenganche — K14.  
**Generalidad no ganada:** K1 dose, K5 burst, K4 Norte, K9 fix#4, K6 Γ F1.

**Revisión adversaria:** [CRITICAL_EVIDENCE_REVIEW.md](CRITICAL_EVIDENCE_REVIEW.md) — OQ1 **sigue abierta**; pausa de revisión **terminada**.

---

## 3. ¿OQ1 sigue abierta de verdad?

### Qué pregunta OQ1 / H6-OBS (formal, doc 16 §2)

> Existe al menos una **propiedad interna** del EKF cuya observación permite **identificar** cambios de régimen ya caracterizados, con **significado estable** bajo las configs del experimento (C-F1 ↔ C-PoC).

Criterio de cierre (doc 16): caracterización C1–C7 + invarianza + entregables `observable_characterization.json` + `regime_model.md`.

### Qué ya responde la evidencia acumulada (sin benchmark nuevo)

| Pregunta implícita | ¿Respondida? | Por |
|--------------------|--------------|-----|
| ¿El bloqueo es solo un artefacto de un CSV? | **Sí (no lo es)** | K14, G-ext |
| ¿La calidad GNSS externa basta para explicar el régimen interno? | **Sí (no basta)** | K15 |
| ¿Γ̄ v1 es esa propiedad/operacionalización? | **Sí (no lo es)** | K12 |
| ¿Γ tiene significado invariante entre configs? | **Sí (no siempre)** | K13 → alimenta OQ3 |
| **¿Qué propiedad/observable(s) cumplen H6-OBS?** | **No** | No hay caracterización O1–O5 ni C7 ni regime_model |
| ¿H7-MIN (conjunto mínimo)? | **No** | OQ2 |

### Veredicto

| Campo | Valor |
|-------|-------|
| **OQ1 sigue abierta** | **Sí** |
| Motivo | G-ext y K12–K15 **justifican y afilan** la pregunta; **no** seleccionan ni caracterizan la propiedad |
| Lo que G-ext cambió sin nuevo benchmark | Nivel de evidencia del bloqueo + desacople externo/interno + motivación OQ1 |
| Lo que G-ext **no** sustituye | Benchmark preregistrado v1.2 (propiedad → observable → caracterización) |
| Siguiente movimiento si se acepta este veredicto | Ejecutar **solo** ese benchmark (D15 abajo), sin controlador / RMSE / NHC adaptativo / retune P_pv·Q·R·Joseph |

**No cerrar OQ1 por lectura sola de K1–K15.** Haría falta afirmar un ganador entre candidatos sin C1–C7 — eso violaría el prerregistro.

---

## 4. Qué queda explícitamente fuera hasta después de H6

Γ̄ / τ / umbrales · controlador adaptativo · intervención P_pv · Joseph tuning · Q · R · “arreglar NHC” como fase — nivel siguiente (OQ5–OQ7 / v3+), ya separado en OPEN_QUESTIONS y D2/D12/D13.

---

## 5. Enlaces

| Doc | Rol |
|-----|-----|
| [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md) | Texto normativo de cada K |
| [INTERPRETATION.md](../../benchmarks/real_run_19082026_baseline/INTERPRETATION.md) | Lectura G-ext |
| [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) | OQ1–OQ7 |
| [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) | Protocolo H6 (sin modificar) |
| [CURRENT_STATE_OF_THE_RESEARCH.md](CURRENT_STATE_OF_THE_RESEARCH.md) | Artículo de estado |
