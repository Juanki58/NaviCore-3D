# Open Questions

**Tipo:** documentación de referencia — solo preguntas **abiertas**.  
**Regla:** no reabrir preguntas cerradas en [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md).  
**Última actualización:** 2026-07-18

---

### OQ1 — What internal observable stays coherent when external GNSS quality no longer explains filter behaviour?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v2 |
| Hypothesis | H6-OBS |
| Status | **Partially open** |
| Protocol | [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) |
| Tag | `gap5-v2-observable-preregistration-v1.2` |
| Motivation | G-ext (K14/K15): ~506 s clean GNSS + continuous reject decouples external quality from internal regime — [INTERPRETATION.md](../../benchmarks/real_run_19082026_baseline/INTERPRETATION.md) |
| Note | Formal H6-OBS unchanged. Do **not** pre-adopt “North → longitudinal”. |
| H6 closure | **2026-07-18** — [regime_model.md](../../benchmarks/gap5_v2_observable_selection/regime_model.md) §9: mapa parcial (R1←O1; R3←O3 provisional; R2 hueco); **no** propiedad única ni H7-MIN demostrados → OQ1 **no cerrada**. |

---

### OQ2 — Is there a minimal observable set (not a single winner)?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v2 (exploratory) |
| Hypothesis | H7-MIN |
| Status | **Open** |
| Note | D19/regime_model: results do **not** allow concluding that one Oi is sufficient **nor** that a minimal set is necessary. H7-MIN neither weakened nor artificially strengthened. |

---

### OQ3 — Which observable preserves meaning across C-F1 and C-PoC?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v2 |
| Status | **Open** (parcialmente acotada) |
| Context | Γ failed invariance (K13); invariance is selection criterion C3/C7 |
| Evidence Strength Audit | **(a) Cerrado para Γ̄ v1 / K13.** **(b)** H6: C3 ordinal OK for O1–O5 on this arm; full R1–R4 model invariance still open ([regime_model.md](../../benchmarks/gap5_v2_observable_selection/regime_model.md) §2.5). |

---

### OQ4 — What is the explicit regime model (which observables feed which R0–R4)?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v2 closure (§9.2) |
| Status | **Partially answered** |
| Note | [regime_model.md](../../benchmarks/gap5_v2_observable_selection/regime_model.md) §2.4–§4: provisional feed map + indeterminate cardinality — not a frozen controller architecture |

---

### OQ5 — What causal operacionalization preserves the chosen property online?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v3+ (not v2) |
| Hypothesis | H5-PoC |
| Status | **Open** — **not tested** (cota negativa parcial) |
| Prerequisite | Observable/property frozen; controller preregistration |
| Evidence Strength Audit | **Cota:** instancia Γ̄ v1 **no** es esa operacionalización (K12). OQ5 completa espera propiedad elegida por H6. No re-probar Γ̄ v1 como respuesta. |

---

### OQ6 — Does an adaptive NHC policy improve O1–O3 vs B0?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v3+ (not v2) |
| Hypothesis | H5-PoC |
| Status | **Open** — **not tested** |
| Prerequisite | Regime model frozen; controller preregistration |

---

### OQ7 — Valid domain and intervention design for P_pv policy (§11 GAP-4)

| Campo | Valor |
|-------|-------|
| Owner | GAP-4 §11 (separate from GAP-5) |
| Status | **Open** — preregistered, not executed |
| Prerequisite | Do not mix with GAP-5 v2 arms |

---

## Explicitly closed (do not reopen without new evidence)

- «Fix by tuning R_GNSS / Q / K alone» — refuted as primary path ([14-adaptive-nhc-protocol.md](../14-adaptive-nhc-protocol.md) §0)
- «P_pv is a bug» — refuted (K8)
- «Γ̄ v1 thresholds need retune 12→9» — refuted as primary reading ([15-gap5-passive-outcome.md](../15-gap5-passive-outcome.md))
- «Restore k_vel → restore accepts» — refuted (K3)
