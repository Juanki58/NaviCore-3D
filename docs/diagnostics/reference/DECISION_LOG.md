# Decision Log

**Tipo:** documentación de referencia — **decisiones de diseño**, no resultados experimentales.  
**Regla:** cada entrada enlazable cuando alguien proponga revertir la decisión.  
**Última actualización:** 2026-07-18

---

### D1 — Scientific baseline uses `--constraint-policy disabled` (ZUPT OFF)

| Campo | Valor |
|-------|-------|
| Decision | Do not use legacy ZUPT `forced_time` as scientific baseline |
| Reason | Contaminates nominal velocity; invalidates full-filter comparisons |
| Date | 2026-07 (GAP-3.7 / §8.10) |
| Evidence | Constraint matrix A→B, [11-replay-zupt-provenance.md](../11-replay-zupt-provenance.md) |

---

### D2 — Do not tune R_GNSS / Q / K before identifying mechanism

| Campo | Valor |
|-------|-------|
| Decision | Reject parameter sweeps on R, Q, GNSS_MAX_GAIN as primary diagnosis |
| Reason | Would confound mechanism identification |
| Date | 2026-07-18 |
| Evidence | F1, F1.1, [14-adaptive-nhc-protocol.md](../14-adaptive-nhc-protocol.md) §0 |

---

### D3 — Mechanistic autopsy before intervention (GAP-3 → GAP-4 order)

| Campo | Valor |
|-------|-------|
| Decision | Close GAP-3/GAP-4 diagnostic phases before adaptive policy PoC |
| Reason | Interventions must align with causal model, not isolated metrics |
| Date | 2026-07-18 |
| Evidence | [12-gap3-synthesis.md](../12-gap3-synthesis.md), `gap4-diagnostic-complete` |

---

### D4 — Gate / cos / P_pv hypotheses only from filter logging

| Campo | Valor |
|-------|-------|
| Decision | Offline cos/K reconstruction is exploratory only; cannot override `gnss_nis_audit.csv` |
| Reason | Demonstrated false gate inference at gps#32 |
| Date | 2026-07-18 |
| Evidence | [13-gap4-gnss-velocity-protocol.md](../13-gap4-gnss-velocity-protocol.md) §10.6 |

---

### D5 — Do not patch 1d′ / widen P_pv gate without domain characterization

| Campo | Valor |
|-------|-------|
| Decision | P_pv modification is design choice, not bugfix; §11 separate experiment |
| Reason | GAP-4 bifurcation model; gate domain not characterized for design |
| Date | 2026-07-18 |
| Evidence | GAP-4 §10.5, §11 frozen not executed |

---

### D6 — Freeze GAP-5 v1; do not lower Γ̄ thresholds (12→9)

| Campo | Valor |
|-------|-------|
| Decision | Close controller instance v1; treat outcome as operationalization failure |
| Reason | Not calibration — Γ̄ does not preserve offline burst; retune would destroy experimental value |
| Date | 2026-07-18 |
| Evidence | [15-gap5-passive-outcome.md](../15-gap5-passive-outcome.md) |

---

### D7 — Do not run PoC active (B0/B1/P0) until observable phase completes

| Campo | Valor |
|-------|-------|
| Decision | Block `run_gap5_adaptive_nhc_poc.py` by default |
| Reason | v1 inactive would measure B0 disguised as P0; separates property → observable → controller |
| Date | 2026-07-18 |
| Evidence | [15-gap5-passive-outcome.md](../15-gap5-passive-outcome.md), [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) |

---

### D8 — GAP-5 v2: discuss observables only; no control policy

| Campo | Valor |
|-------|-------|
| Decision | Prohibit control-policy discussion for entire GAP-5 v2 phase |
| Reason | Prevents mixing property identification with actuator design |
| Date | 2026-07-18 |
| Evidence | [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) §0.2 |

---

### D9 — No aggregate score during v2 characterization

| Campo | Valor |
|-------|-------|
| Decision | Characterize dimensions separately; synthesis only at report closure |
| Reason | Single index invites optimization over interpretation |
| Date | 2026-07-18 |
| Evidence | [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) §8–§9 |

---

### D10 — Do not write §17 lessons narrative until GAP-5 v2 fully closed

| Campo | Valor |
|-------|-------|
| Decision | `17-lessons-ekf-regime-identification.md` deferred until characterization + verdict frozen |
| Reason | Early narrative reinterprets experiments to fit story |
| Date | 2026-07-18 |
| Evidence | [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) §12 |

---

### D11 — New phase only if question not answerable from existing data

| Campo | Valor |
|-------|-------|
| Decision | Each new phase must address one question not resolved by prior frozen artifacts |
| Reason | Avoid methodology proliferation without new information |
| Date | 2026-07-18 |
| Evidence | Project discipline (reference meta-decision) |
