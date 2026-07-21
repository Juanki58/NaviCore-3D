# Open Questions

**Tipo:** documentación de referencia — solo preguntas **abiertas**.  
**Regla:** no reabrir preguntas cerradas en [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md).  
**Última actualización:** 2026-07-19

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
| Status | **Closed** — formulación por **gating** falsificada (§11.8); 5/5 brazos ABORT @ fix#2 |
| Prerequisite | Do not mix with GAP-5 v2 arms |
| Evidence | [13-gap4-gnss-velocity-protocol.md](../13-gap4-gnss-velocity-protocol.md) §11.7–§11.8; `G1_intervention/arm_1e_innov_h/h1e_section11_verdict.json` |
| Lesson | Un único evento `LEGITIMATE_HIGH_GAIN` @ fix#2 (innov_h≈29 m) no es alcanzable por gate cos/gap/magnitud. Omisión en degradada sí se cierra con H1e; eso no salva P1. |
| Future (not OQ7-open) | Clamp de ganancia cruzada tipo `ZUPT_MAX_GAIN` sobre el término pos→vel — preregistrar aparte si se retoma |

### OQ8 — ¿El Jacobiano NHC corregido es mejora neta a nivel de sistema? (post-bf2bfbd)

| Campo | Valor |
|-------|-------|
| Owner | E2E SLALOM / TUNNEL_STRESS — A/B Jacobiano × IMU |
| Status | **Open (mecanismo early-loop cerrado; H-ATT-d abierta; gate cand1 no generaliza a túnel)** |
| Early mechanism (closed) | Signo `H_att` → bucle `dx_att_z` → onset → pitch/K_y0 vía `P[ATT_Y,VEL_N]` por **déficit Joseph** tras zero-out post-hoc de Z → surge vel_NED → innov → (si Z cerrado) fuga `bias_gz`. Mapa 11 eslabones: `SESSION_CLOSE_2026-07-19.md`, protocolo §13.15. |
| Family closed 2026-07-19 | **Atacar canal Z vía δx** (b1 / H-ATT-c / λ=1): FAIL. Causa: Ky completo + λ post-hoc → Joseph recorta de menos → P_av crece. Detector cand1 sí separa A/C **en slalom**. §11–§12.9, §13.20. |
| Instrument closed 2026-07-19 | **cand1 como gate multi-dominio:** FAIL de generalización (E1/E2 §13.22). No es “subir T₂”. `docs/benchmarks/jacobian_imu_ab/CAND1_GENERALIZATION_REVIEW.md`. |
| Still open | **H-ATT-d** (unobs): hipótesis intacta. **Acotado:** el espacio de instrumentos capaces de contrastarla (no FP túnel ∧ no-regresión slalom ∧ significado comparable). Ese instrumento aún no existe. P2-tunnel/P4 bloqueados. **No** zero-out δx; **no** T₂-hunt. |
| Residual families (deferred) | **A** propiedad solo-slalom · **B** misma propiedad, otra operacionalización — ver patrón § Familias residuales. Representación, no tuning. |
| Deferred scientific form (not opened) | *¿Existe un observable de onset invariante entre dominios experimentales?* — pregunta de programa; **D22** prohíbe preregistro/experimento inmediato. |
| Project split | **(1)** Early-loop mecanismo cerrado; H-ATT-d ≠ cand1-gate; **separada** de OQ9. |
| Protocol | [18-jacobian-imu-ab-protocol.md](../18-jacobian-imu-ab-protocol.md) §4, §11–§13.22 |
| Pattern | [OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md](OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md) (Caso B) |
| Discipline | Seed fijo; entender ≠ arreglar; preregistro antes de código; `CORR_ABS_SCALE`; no mezclar (1) con OQ9; **pausa D22** |

### OQ9 — Régimen tardío SLALOM (t≳14 s) — problema **nuevo**

| Campo | Valor |
|-------|-------|
| Owner | E2E SLALOM A×C |
| Status | **Open — new problem** (not a tail of the early Jacobian loop) |
| Discarded label | `FEEDBACK_CONTINUES_OMEGA_DECOUPLED` — **descartado**. Check C: `SIGN_MIXED`. |
| Also discarded | Cliff puntual 22–25 s (NOT bursty). |
| **Discarded 2026-07-19 — do not reopen without new evidence** | «Early-slalom + OQ9 son el mismo mecanismo porque `bias_gz` diverge.» Probado: la divergencia fuerte de bias post-latch es **artefacto de cerrar Z** (ctrl A slope +0,002/s vs latch −0,021/s); en régimen natural early↔late **no** hay deriva de bias ininterrumpida. Evidencia: `hatt_c/hatt_c_bias_coherence_early_oq9_latch.md`, protocolo §12.9. |
| Positive open | Qué gobierna el crecimiento suave de `P_pp` late si no es lockstep de `dx_att_z` ni identidad con el bias post-latch. |
| When resumed (cheapest) | dP_pp/dt (14–25 s) vs `bias_gz` / actitud en telemetría **sin** latch H-ATT. No heredar lecturas de brazos λ=1. |
| Evidence | `slalom_oq9_late_dxattz_sign.md`, `slalom_oq9_late_p_and_burstiness.md`, `hatt_c_bias_coherence_early_oq9_latch.md` |
| Not the same as | **T6**; **not** project (1); **not** “bias early motor = OQ9” |
| Protocol | [18-jacobian-imu-ab-protocol.md](../18-jacobian-imu-ab-protocol.md) §4–§5c, §12.9 |
| Do not | Fusionar con (1) por firma superficial de bias; preregistrar late sobre label descartado; reabrir fusión early↔OQ9 vía bias sin evidencia nueva |

---

## Explicitly closed (do not reopen without new evidence)

- «Fix by tuning R_GNSS / Q / K alone» — refuted as primary path ([14-adaptive-nhc-protocol.md](../14-adaptive-nhc-protocol.md) §0)
- «P_pv is a bug» — refuted (K8)
- «Γ̄ v1 thresholds need retune 12→9» — refuted as primary reading ([15-gap5-passive-outcome.md](../15-gap5-passive-outcome.md))
- «Restore k_vel → restore accepts» — refuted (K3)
- «Gate P_pv (gap / cos / innov_h) fixes G1 abort @ fix#2» — falsified under §11 (§11.8: 5/5 same locus; gating formulation closed)
- «Atacar / modular solo el canal Z NHC (`dx_att_z`) arregla early SLALOM A» — falsified (H-ATT-b1 / H-ATT-c / λ=1; protocolo §11–§12.6); cerrar Z induce fuga a `bias_gz` (§12.9) y déficit Joseph → P_av (§13.20)
- «Early-slalom y OQ9 comparten causa porque `bias_gz` diverge» — falsified as identity (§12.9; divergencia fuerte = artefacto de latch)
- «Zero-out post-hoc de una componente de δx es equivalente a no observar ese canal» — falsified (§13.15–§13.20; Joseph sigue la K completa → underclip)
