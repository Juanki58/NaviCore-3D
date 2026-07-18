# Open Questions

**Tipo:** documentación de referencia — solo preguntas **abiertas**.  
**Regla:** no reabrir preguntas cerradas en [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md).  
**Última actualización:** 2026-07-18

---

### OQ1 — What filter property defines regime?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v2 |
| Hypothesis | H6-OBS |
| Status | **Open** |
| Protocol | [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) |
| Tag | `gap5-v2-observable-preregistration-v1.1` |

---

### OQ2 — Is there a minimal observable set (not a single winner)?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v2 (exploratory) |
| Hypothesis | H7-MIN |
| Status | **Open** |
| Note | Outcomes: one suffices / two / three / none of O1–O5 |

---

### OQ3 — Which observable preserves meaning across C-F1 and C-PoC?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v2 |
| Status | **Open** |
| Context | Γ failed invariance (K13); invariance is selection criterion C3/C7 |

---

### OQ4 — What is the explicit regime model (which observables feed which R0–R4)?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v2 closure (§9.2) |
| Status | **Open** |
| Note | C7 provides evidence; `regime_model.md` is the deliverable — may be vector (H7-MIN) |

---

### OQ5 — What causal operacionalization preserves the chosen property online?

| Campo | Valor |
|-------|-------|
| Owner | GAP-5 v3+ (not v2) |
| Hypothesis | H5-PoC |
| Status | **Open** — **not tested** |
| Prerequisite | Observable/property frozen; controller preregistration |

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
