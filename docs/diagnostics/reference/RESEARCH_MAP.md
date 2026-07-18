# Research Map

**Tipo:** documentación de referencia — fases, preguntas, outcomes, tags.  
**Regla:** sin cronología narrativa; solo nodos congelados.  
**Última actualización:** 2026-07-18

---

## Flow

```mermaid
flowchart TD
  G3[GAP-3 mechanism autopsy]
  F1[F1 NHC dose-response]
  F11[F1.1 NIS / Lambda_N anatomy]
  F12[F1.2 decimation / burst]
  G4[GAP-4 P_pv diagnostic]
  G5v1[GAP-5 v1 Gamma_bar controller]
  G5v2[GAP-5 v2 observable selection]

  G3 --> F1
  F1 --> F11
  F11 --> F12
  F12 --> G4
  G4 --> G5v1
  G5v1 --> G5v2
```

---

## Nodes

### GAP-3 — INS/EKF mechanism autopsy

| Campo | Valor |
|-------|-------|
| Question | What compresses covariance and breaks GNSS acceptance? |
| Hypothesis | NHC / predict / Joseph / ZUPT / algebra (sequential partition) |
| Outcome | **Mechanistic model closed** — NHC dominates P; gate nominal-driven |
| Tag | — (synthesis: [12-gap3-synthesis.md](../12-gap3-synthesis.md)) |
| Status | **Closed** |

**Refuted (dominant explanation):** ZUPT-only, k_vel-only, Joseph-only, pure frequency cliff.

---

### F1 — NHC dose–response

| Campo | Valor |
|-------|-------|
| Question | Does less NHC restore P_vv and accepts? |
| Hypothesis | Lower NHC frequency → higher P_vv → more accepts |
| Outcome | **Refuted** — P_vv and k_vel rise; accepts unchanged |
| Evidence | `docs/benchmarks/gap3_f1_nhc_dose_response/` |
| Status | **Closed** |

---

### F1.1 — Gate anatomy

| Campo | Valor |
|-------|-------|
| Question | Is rejection due to low K only? |
| Hypothesis | Restoring K restores accepts |
| Outcome | **Refuted** — North innovation / Λ_N dominates |
| Status | **Closed** |

---

### F1.2 — Decimation / burst persistence

| Campo | Valor |
|-------|-------|
| Question | Is cliff purely high NHC frequency? |
| Hypothesis | Decimation removes burst |
| Outcome | **Refuted** — timing shifts; burst persists; state-conditioned |
| Status | **Closed** |

---

### GAP-4 — P_pv / GNSS velocity diagnostic

| Campo | Valor |
|-------|-------|
| Question | What role does P_pv play at fix#4 and beyond? |
| Hypothesis | P_pv bug vs legitimate coupling vs policy bifurcation |
| Outcome | **Confirmed** — legitimate coupling; fix#4 bifurcates EKF; cos from logs only |
| Tag | `gap4-diagnostic-complete` |
| Doc | [13-gap4-gnss-velocity-protocol.md](../13-gap4-gnss-velocity-protocol.md) |
| Status | **Closed** (diagnostic); §11 intervention **not executed** |

---

### GAP-5 v1 — Adaptive NHC via Γ̄

| Campo | Valor |
|-------|-------|
| Question | Can Γ̄ (EWMA 1 s) drive NHC regime control? |
| Hypothesis | H-ops: preregistered operationalization detects F1 burst |
| Outcome | **Refuted (operationalization)** — Γ̄ inactive; burst not preserved; not threshold tuning issue |
| Tag | `gap5-preregistration-frozen` (prereg); outcome [15-gap5-passive-outcome.md](../15-gap5-passive-outcome.md) |
| PoC active | **Not run** (by design) |
| Status | **Closed** |

**Partial confirm:** Γ_inst ≈ offline Γ in C-F1 (implementation OK).

---

### GAP-5 v2 — Observable / property selection

| Campo | Valor |
|-------|-------|
| Question | What filter property defines regime? |
| Hypothesis | H6-OBS; exploratory H7-MIN (minimal set) |
| Outcome | **Not executed** |
| Tag | `gap5-v2-observable-preregistration-v1.2` |
| Doc | [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) |
| Closure deliverables | `observable_characterization.json` + `regime_model.md` |
| Status | **Preregistered — benchmark pending** |

---

## Legend

| Outcome label | Meaning |
|---------------|---------|
| **Confirmed** | Mechanism or rule in [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md) |
| **Refuted** | Hypothesis falsified; do not retry without new preregistration |
| **Refuted (operationalization)** | Methodological — property may exist, this signal/control path does not |
| **Not executed** | Frozen preregistration only |

---

## Related reference docs

- [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md)
- [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md)
- [DECISION_LOG.md](DECISION_LOG.md)
