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

---

### D12 — G-ext interpretation: lockout core only; no North→longitudinal leap

| Campo | Valor |
|-------|-------|
| Decision | Claim “independent-trajectory lockout core” (K14/K15); do **not** claim full G1 confirmation; do **not** adopt “North → longitudinal” (or other axis remapping) before GAP-5 v2 falsifies candidates |
| Reason | Conservatism: missing fix#4 region ≠ model failure; axis dominance may be geometry — leave to H6-OBS |
| Date | 2026-07-18 |
| Evidence | [INTERPRETATION.md](../../benchmarks/real_run_19082026_baseline/INTERPRETATION.md) |

---

### D13 — Pause before opening GAP-5 v2 / H6 benchmark

| Campo | Valor |
|-------|-------|
| Decision | Do **not** open the H6 observable characterization benchmark immediately after G-ext; short pause to freeze chain status (consolidated vs open) |
| Reason | H6 is now justified by four prior results; starting the benchmark without an explicit knowledge boundary invites reopening closed questions |
| Date | 2026-07-18 |
| Evidence | [RESEARCH_STATUS.md](RESEARCH_STATUS.md) |

---

### D14 — Evidence Review before new experiment

| Campo | Valor |
|-------|-------|
| Decision | Complete systematic Evidence Review (K1–K15 × G1 × G-ext) before opening any new experimental phase |
| Reason | Do not run a new experiment while the current question might already be answerable from accumulated data |
| Date | 2026-07-18 |
| Evidence | [EVIDENCE_REVIEW.md](EVIDENCE_REVIEW.md) |

---

### D15 — OQ1 remains open → next step is exact H6 characterization only

| Campo | Valor |
|-------|-------|
| Decision | After Evidence Review: **OQ1 remains open**. When the pause ends, execute **only** the preregistered GAP-5 v2 characterization benchmark (`gap5-v2-observable-preregistration-v1.2`) — no controller, RMSE, adaptive NHC, P_pv/Q/R/Joseph retune |
| Reason | G-ext/K12–K15 justify and sharpen OQ1; they do not select or characterize the property (no C1–C7 / regime_model) |
| Date | 2026-07-18 |
| Evidence | [EVIDENCE_REVIEW.md](EVIDENCE_REVIEW.md) §3 |

---

### D16 — Critical Evidence Review done; methodological pause (review phase) ended

| Campo | Valor |
|-------|-------|
| Decision | Adversarial review of Evidence Review completed; overstatements on G-ext coverage corrected; **OQ1 still open**. Review-phase pause **ended**. H6 is justified because the question is unanswered by existing data — **not** because the protocol file exists. Next action when executing: H6 v1.2 with **zero** protocol edits |
| Reason | Reject inertia; accept necessity after failed attempts to close OQ1 from corpus alone |
| Date | 2026-07-18 |
| Evidence | [CRITICAL_EVIDENCE_REVIEW.md](CRITICAL_EVIDENCE_REVIEW.md) |

---

### D17 — Evidence Strength Audit; H6 criterion sentence frozen

| Campo | Valor |
|-------|-------|
| Decision | Strength/scope audit for K1–K15 completed; OQ3/OQ5 **bounded** (not closed); OQ1 fully open. Formal justification to run H6 is the frozen sentence: *“No existe evidencia suficiente para discriminar entre los observables candidatos mediante los experimentos ya realizados.”* Methodological pause **closed**. Execute H6 v1.2 only with zero protocol edits when chosen |
| Reason | Traceable answer to “why the observable benchmark?” = demonstrated insufficiency of existing evidence for OQ1 — not chronology |
| Date | 2026-07-18 |
| Evidence | [EVIDENCE_STRENGTH_AUDIT.md](EVIDENCE_STRENGTH_AUDIT.md) §3–§4 |

---

### D18 — H6 preflight OK; execution bindings frozen; characterization run authorized

| Campo | Valor |
|-------|-------|
| Decision | Preflight found no unregistered *scientific* decisions in v1.2; operational gaps (C7 ordinals, log paths, series formulas) closed via **execution bindings** without editing protocol v1.2. Ideas → `IDEAS_DURING_H6.md` only. Characterization = produce artifacts; **regime_model.md / winner language deferred** until after full numeric pass |
| Reason | Characterization ≠ optimization; four outcomes all valid |
| Date | 2026-07-18 |
| Evidence | [H6_PREFLIGHT.md](H6_PREFLIGHT.md) |

---

### D19 — Consistency review of H6 artifacts before `regime_model.md`

| Campo | Valor |
|-------|-------|
| Decision | Do **not** write `regime_model.md` yet. First freeze a consistency review of `observable_characterization.json`: distinct C1–C7 profiles, Paso 0 falsification table, and whether H7-MIN is *forced* by data. OQ1 remains open until a synthesis can name the property/minimal set and discard alternatives from the JSON |
| Reason | Keep interpretation at the same rigor as execution; avoid winner-first narrative |
| Date | 2026-07-18 |
| Evidence | [H6_ARTIFACT_CONSISTENCY_REVIEW.md](../../benchmarks/gap5_v2_observable_selection/H6_ARTIFACT_CONSISTENCY_REVIEW.md) |

---

### D20 — `regime_model.md` written; OQ1 remains partially open

| Campo | Valor |
|-------|-------|
| Decision | Write `regime_model.md` after D19 only. Freeze B1/B2 as benchmark results. Paso 0 uses Evaluada/Refutada/No refutada/Inconclusa (never «confirmada»). H7-MIN phrase: neither mono-Oi sufficiency nor minimal-set necessity concluded. OQ1 decision is **last** section of regime_model: **partially open** (partial map ≠ closed property/set). Map: R1←O1; R3←O3 provisional; R2←hueco; O2 fuera como eje distinto |
| Reason | Same rigor for interpretation as for execution; avoid verdict-first synthesis; prefer partial closure over forced complete closure |
| Date | 2026-07-18 |
| Evidence | [regime_model.md](../../benchmarks/gap5_v2_observable_selection/regime_model.md) §9 |
| Contribution | *La contribución principal de H6 no es identificar un controlador, sino transformar un conjunto de candidatos independientes en un modelo parcial del espacio de regímenes del filtro.* |

---

### D21 — Stage I closed: EKF regime-identification baseline frozen

| Campo | Valor |
|-------|-------|
| Decision | **Close Stage I** (regime identification) as a whole stage — not only individual docs. Freeze [STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md](STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md): scope, answered vs open, what future work may assume, what must not be rediscussed without new evidence. **Do not open GAP-5 v3** in this decision. When v3 opens, its preregistration first sentence must state that it assumes the H6 partial regime model and studies only compatible control policies |
| Reason | Protect consolidated knowledge; make Stage II a new stage, not a diffuse continuation of H6 |
| Date | 2026-07-18 |
| Evidence | [STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md](STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md) |
| Suggested tag | `stage-I-regime-identification-closed` |

---

### D22 — Pause after cand1 generalization close; consolidate methodology (no new experiment)

| Campo | Valor |
|-------|-------|
| Decision | After closing cand1 as a **multi-domain gate** (not after refuting H-ATT-d): **do not** open OQ8 reformulated, cand2, T₂-hunt, or GAP-5 v3 immediately. Freeze transversal pattern doc [OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md](OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md) (Γ̄ = Case A, cand1 = Case B). Stabilization: links, traceability, breathe. |
| Reason | Two scientific closures in sequence (Stage I instrument lessons + cand1 domain-of-validity). Next value is protecting methodology as program knowledge, not starting another line while structure is freshly written. |
| Date | 2026-07-19 |
| Evidence | [CAND1_GENERALIZATION_REVIEW.md](../../benchmarks/jacobian_imu_ab/CAND1_GENERALIZATION_REVIEW.md) · [OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md](OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md) · protocolo §13.22 |
