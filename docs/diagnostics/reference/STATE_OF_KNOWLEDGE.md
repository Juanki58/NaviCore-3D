# State of Knowledge

**Tipo:** documentación de referencia — no narrativa, no cronología.  
**Regla:** solo afirmaciones con veredicto congelado (tags / informes congelados).  
**Última actualización:** 2026-07-18

---

## Confirmed mechanisms

### K1 — NHC frequency modulates covariance compression

NHC application rate controls `P_vv` level and `k_vel` (dose–response).

| Campo | Valor |
|-------|-------|
| Evidence | F1 (`docs/benchmarks/gap3_f1_nhc_dose_response/`) |
| Status | **Confirmed** |
| Source | [12-gap3-synthesis.md](12-gap3-synthesis.md) §2 |

---

### K2 — NHC ON vs OFF dominates GNSS accept count (not ZUPT alone)

With ZUPT removed from baseline: NHC ON → 7 accepts; NHC OFF → 56 accepts on same trajectory window.

| Campo | Valor |
|-------|-------|
| Evidence | Constraint matrix A–E (§8.10) |
| Status | **Confirmed** |
| Source | [12-gap3-synthesis.md](12-gap3-synthesis.md) §2 |

---

### K3 — Restoring Kalman gain (`k_vel`) does not restore GNSS accepts

Higher `P_vv` / `k_vel` from lower NHC frequency does not increase accepts (e.g. N=10: k_vel ×12, accepts still 7).

| Campo | Valor |
|-------|-------|
| Evidence | F1, F1.1 |
| Status | **Confirmed** |
| Source | [12-gap3-synthesis.md](12-gap3-synthesis.md) §2 |

---

### K4 — GNSS gate rejection is dominated by North-axis innovation

Rejects driven by `contrib_N`, `Λ_N`; innovation in nominal state grows while `S_N` falls.

| Campo | Valor |
|-------|-------|
| Evidence | F1.1 |
| Status | **Confirmed** |
| Source | [12-gap3-synthesis.md](12-gap3-synthesis.md) §2, §8.17 |

---

### K5 — NHC-induced `P_vv` cliff is bursty and state-conditioned

Burst concentrated in few NHC ticks (top-3 share high when meaningful); decimation shifts timing but burst persists; not purely frequency-driven.

| Campo | Valor |
|-------|-------|
| Evidence | F1.2, gap fix#2→#3 autopsy |
| Status | **Confirmed** |
| Source | [12-gap3-synthesis.md](12-gap3-synthesis.md) §2, §8.18 |

---

### K6 — In fix#2→#3 gap, NHC erosion dominates predict regeneration (offline Γ)

Gap-integrated ratio Σ|ΔP_vv|_NHC / ΣΔP_predict ≈ 19.7 (baseline F1 config, pos-only GNSS).

| Campo | Valor |
|-------|-------|
| Evidence | F1 baseline, cov_step audit |
| Status | **Confirmed** |
| Source | [12-gap3-synthesis.md](12-gap3-synthesis.md) §2 |

---

### K7 — Joseph GNSS update explains only part of `P_vv` drop at fix#2

Joseph accounts for ~31% of drop; remainder mechanistically tied to NHC in gap.

| Campo | Valor |
|-------|-------|
| Evidence | GAP-3.14 cliff anatomy |
| Status | **Confirmed** |
| Source | [12-gap3-synthesis.md](12-gap3-synthesis.md) §2 |

---

### K8 — `P_pv` is legitimate EKF cross-covariance, not an implementation bug

Position innovation transfers to velocity via `P_pv` under normal EKF operation.

| Campo | Valor |
|-------|-------|
| Evidence | GAP-4 autopsy, logged filter state |
| Status | **Confirmed** |
| Source | [13-gap4-gnss-velocity-protocol.md](13-gap4-gnss-velocity-protocol.md) §10.5 |
| Tag | `gap4-diagnostic-complete` |

---

### K9 — fix#4 `P_pv` policy bifurcates filter trajectories (two EKFs after fix#4)

Arms 1d vs 1d′ share pre-update at fix#4; post-update diverges; subsequent comparisons are not same-state policy tests.

| Campo | Valor |
|-------|-------|
| Evidence | GAP-4 divergence tree, truth table |
| Status | **Confirmed** |
| Source | [13-gap4-gnss-velocity-protocol.md](13-gap4-gnss-velocity-protocol.md) §10.5, §10.7 |

---

### K10 — Offline reconstruction of `cos(Δv, err)` does not substitute filter-logged gate values

Post-hoc K/cos recomputation can infer wrong gate trigger; `gnss_nis_audit.csv` fields are authoritative.

| Campo | Valor |
|-------|-------|
| Evidence | gps#32 autopsy (cos_tot ≈ −0.87 logged, +0.87 reconstructed) |
| Status | **Confirmed** (methodological rule) |
| Source | [13-gap4-gnss-velocity-protocol.md](13-gap4-gnss-velocity-protocol.md) §10.6 |

---

### K11 — Online Γ_inst tracks offline gap Γ in F1-equivalent config (~0.92 peak ratio)

Passive f1-bridge: offline gap Γ = 19.65; Γ_inst peak ≈ 18.14 in fix#2→#3 window.

| Campo | Valor |
|-------|-------|
| Evidence | GAP-5 v1 passive (`p0_passive_f1_bridge`) |
| Status | **Confirmed** |
| Source | [15-gap5-passive-outcome.md](15-gap5-passive-outcome.md) §3.1 |

---

### K12 — Γ̄ (EWMA τ=1 s) does not preserve the mechanistic burst for control

Γ̄ never reaches preregistered thresholds; burst ~0.39 s << τ and dwell; controller v1 inactive — **operationalization failure**, not EKF failure.

| Campo | Valor |
|-------|-------|
| Evidence | GAP-5 v1 passive |
| Status | **Confirmed** (methodological) |
| Source | [15-gap5-passive-outcome.md](15-gap5-passive-outcome.md) |

---

### K13 — Offline Γ loses mechanistic meaning under PoC filter config (pos_vel + p_pv none)

Same gap window: Γ_offline ≈ 0.13 vs ≈ 19.7 (F1 config) — magnitude and interpretation not invariant across configs.

| Campo | Valor |
|-------|-------|
| Evidence | GAP-5 v1 passive (`p0_passive_validation` vs `p0_passive_f1_bridge`) |
| Status | **Confirmed** |
| Source | [15-gap5-passive-outcome.md](15-gap5-passive-outcome.md) §3.1 |

---

## Classification rule

| Category | Document |
|----------|----------|
| Confirmed knowledge | **This file** |
| Refuted hypothesis | [RESEARCH_MAP.md](RESEARCH_MAP.md) (outcome per phase) |
| Open question | [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) |
| Design decision | [DECISION_LOG.md](DECISION_LOG.md) |

Do not add provisional or discussion items here.
