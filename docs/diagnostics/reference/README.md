# Reference documentation index

**Purpose:** freeze **state of knowledge** — not invent new science.

## Tres niveles de conocimiento

| Nivel | Rol | Entrada |
|-------|-----|---------|
| **1. Baselines** | Congelados; cualquier trabajo futuro los **asume** | [STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md](STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md) · [regime_model.md](../../benchmarks/gap5_v2_observable_selection/regime_model.md) |
| **2. Programa** | K, OQ, GAP, D, mapas, métricas | este directorio (`reference/`) |
| **3. En curso** | Experimentos activos | *(vacío — D22 pausa tras cierre cand1)* |

**Start here (post Stage I):**  
→ [STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md](STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md)  
→ [OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md](OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md) ← patrón transversal Γ̄ / cand1  
→ [CURRENT_STATE_OF_THE_RESEARCH.md](CURRENT_STATE_OF_THE_RESEARCH.md)  
→ [RESEARCH_STATUS.md](RESEARCH_STATUS.md)

---

## 1. Baselines (congelados)

| Documento | Rol |
|-----------|-----|
| **[Stage I closure](STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md)** | Cierre de etapa: alcance, asumibles, no reabrir |
| [regime_model.md](../../benchmarks/gap5_v2_observable_selection/regime_model.md) | Modelo parcial de régimen (D20) |
| [H6_ARTIFACT_CONSISTENCY_REVIEW.md](../../benchmarks/gap5_v2_observable_selection/H6_ARTIFACT_CONSISTENCY_REVIEW.md) | Consistencia pre-síntesis (D19) |
| [INTERPRETATION G-ext](../../benchmarks/real_run_19082026_baseline/INTERPRETATION.md) | Lectura congelada validación externa |
| Protocolo H6 v1.2 | [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) |

Tag sugerido: `stage-I-regime-identification-closed` (al versionar).

---

## 2. Programa de investigación

| Layer | File |
|-------|------|
| **Artículo interno (estado)** | [CURRENT_STATE_OF_THE_RESEARCH.md](CURRENT_STATE_OF_THE_RESEARCH.md) |
| Project phase / status | [RESEARCH_STATUS.md](RESEARCH_STATUS.md) |
| **Consolidated knowledge (K)** | [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md) |
| **Open questions (OQ)** | [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) |
| Design decisions (D) | [DECISION_LOG.md](DECISION_LOG.md) |
| Phase map (GAP / tags) | [RESEARCH_MAP.md](RESEARCH_MAP.md) |
| Scientific chronology | [SCIENTIFIC_CHRONOLOGY.md](SCIENTIFIC_CHRONOLOGY.md) |
| Program metrics X/Y/Z + V/P/R | [RESEARCH_METRICS.md](RESEARCH_METRICS.md) |
| **Operationalization failures (pattern)** | [OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md](OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md) |
| Dependency map | [DEPENDENCY_MAP.md](DEPENDENCY_MAP.md) |
| Evidence Strength Audit | [EVIDENCE_STRENGTH_AUDIT.md](EVIDENCE_STRENGTH_AUDIT.md) |
| Critical Evidence Review | [CRITICAL_EVIDENCE_REVIEW.md](CRITICAL_EVIDENCE_REVIEW.md) |
| Evidence Review (K×G1×G-ext) | [EVIDENCE_REVIEW.md](EVIDENCE_REVIEW.md) |
| Consistency audit | [CONSISTENCY_AUDIT.md](CONSISTENCY_AUDIT.md) |
| Redundancy inventory | [REDUNDANCY_INVENTORY.md](REDUNDANCY_INVENTORY.md) |
| H6 preflight | [H6_PREFLIGHT.md](H6_PREFLIGHT.md) |

## Rules

1. Use only hypotheses and verdicts already frozen (git tags, congelados docs).
2. Do not mix categories in one entry.
3. STATE / OPEN stay non-narrative; CURRENT_STATE / STATUS may frame the chain.
4. Refuted hypotheses appear in **RESEARCH_MAP** outcomes, not as open questions.
5. Update when a phase closes — not before.
6. Never move consolidated ↔ open without a new frozen experiment.
7. Editor mode: consolidate and make traceable; do **not** invent hypotheses or controllers.
8. **Stage I closed (D21):** do not reopen regime-identification framing inside a future control protocol; see Stage I §6–§7.
9. **Operationalization ≠ hypothesis (pattern doc):** Γ̄ and cand1 are Case A/B; do not retune thresholds to “save” a failed operationalization — see D22.
10. **D22 pause:** no immediate OQ8 experiment / cand2 / GAP-5 v3; stabilize docs first.

## Protocol / phase docs (detail)

`docs/diagnostics/10–18-*.md` (incl. [18-jacobian-imu-ab-protocol.md](../18-jacobian-imu-ab-protocol.md)), G-ext: `docs/benchmarks/real_run_19082026_baseline/INTERPRETATION.md`.  
Cand1 instrument close: `docs/benchmarks/jacobian_imu_ab/CAND1_GENERALIZATION_REVIEW.md`.
