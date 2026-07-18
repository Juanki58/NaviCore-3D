# Redundancy Inventory — Solo listar (no borrar)

**Tipo:** editorial.  
**Regla:** ningún archivo se elimina en esta pasada.  
**Fecha:** 2026-07-18

---

## 1. Scripts con solapamiento funcional aparente

| Grupo | Scripts | Nota |
|-------|---------|------|
| GAP-3 cliff / fix2–3 | `audit_gap3_f1_cliff_anatomy.py`, `audit_gap3_nhc_cliff_mechanism.py`, `audit_gap3_fix2_fix3_tick_reconstruction.py`, `audit_gap3_fix2_fix3_autoconsume.py` | Misma ventana temática; roles distintos en síntesis — conservar; marcar canónico en 12-synthesis |
| GAP-3 GNSS NIS / K | `audit_gap3_gnss_nis_anatomy.py`, `audit_gap3_f1_nis_gate_anatomy.py`, `audit_gap3_gnss_k_block.py`, `audit_gap3_gnss_accepted_autopsy.py` | Anatomía vs accept autopsy |
| GAP-3 cov | `audit_gap3_cov_propagation.py`, `audit_gap3_cov_step_cycle.py`, `audit_gap3_observation_cycle.py` | Capas de instrumentación |
| GAP-4 P_pv post-hoc | `audit_gap4_ppv_gate_autopsy.py`, `audit_gap4_ppv_k_consistency.py`, `audit_gap4_ppv_truth_table.py`, `audit_gap4_ppv_divergence_tree.py` | Familia §10.x |
| GAP-4 cos / threshold | `audit_gap4_direct_cos_gate.py`, `audit_gap4_threshold_discrimination.py`, `audit_gap4_alignment_sweep.py` | Exploratorio vs discrimination |
| GAP-5 runners | `run_gap5_p0_passive_validation.py`, `run_gap5_adaptive_nhc_poc.py` | PoC activo **no ejecutar** con v1 (doc 14/15) — script sigue en repo |
| GAP-1 mount | `audit_gap1_delta_psi_constancy.py`, `audit_gap1_body_forward_axis.py` | Complementarios |

---

## 2. Artefactos benchmark con apariencia de depuración / twin

| Path | Observación |
|------|-------------|
| `docs/benchmarks/gap4_gnss_velocity/G1_intervention/test_crash*.csv` | Nombres de depuración |
| `…/test_nis5.csv`, `test_cov*.csv` | Idem |
| `G1/` vs `G1_control_full_ppv_none/` | Twin intencional (reproducción) — **no** redundancia basura |
| `G1_intervention/arm_*` | Brazos §11 / exploración; baseline canónico sigue siendo `G1/` |
| `docs/_archive_short_run_jul15/` (si presente) | Archivo histórico declarado en README |

---

## 3. Protocolos / docs sustituidos o multi-versión

| Ítem | Estado |
|------|--------|
| Tag `gap5-v2-observable-preregistration-frozen` vs `…-v1.2` | Línea v2 multi-tag — ver CONSISTENCY_AUDIT |
| Doc 14 (prereg v1) vs Doc 15 (outcome) vs Doc 16 (v2) | Cadena intencional, no duplicado |
| `FOUR_QUESTIONS.md` vs `INTERPRETATION.md` (G-ext) | INTERPRETATION = normativa; FOUR_QUESTIONS = evidencia cruda — roles distintos |
| `RESEARCH_STATUS` vs `SCIENTIFIC_CHRONOLOGY` vs `CURRENT_STATE…` | Capas: estado / cronología / artículo — no fusionar |

---

## 4. Datos real_run

| Path | Nota |
|------|------|
| `data/real_run/16072026/` | Run previo (~331 s) — G1 lineage |
| `data/real_run/19082026/` | G-ext |
| Posibles CSV en raíz `data/real_run/` (si existen) | Verificar no mezclar con subcarpetas fechadas |

---

## 5. JSON / informes posiblemente obsoletos

No se auditó caducidad semántica archivo-a-archivo. Criterio futuro: si un informe no está enlazado desde RESEARCH_MAP, síntesis 12/13/15, o K*, catalogarlo aquí antes de archivar.

Candidatos a revisar en sesión posterior:

- Informes H* antiguos en `docs/benchmarks/` no citados por K1–K15  
- Duplicados `gap4_*_report.json` en arms intervention vs G1 root  

---

## 6. Regla de limpieza futura

1. Listar aquí.  
2. Decidir archivo (`_archive/`) vs delete en decisión D-nueva.  
3. Nunca borrar artefactos citados por un `K*` o tag git.
