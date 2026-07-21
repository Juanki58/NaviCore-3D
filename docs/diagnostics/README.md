# Diagnóstico EKF — Real Run (H0–H9d)

Documentación técnica del pipeline experimental usado para auditar consistencia, geodesia, sincronización y propagación inercial del EKF sobre datos reales de vehículo (`data/real_run/`).

## Reference documentation (estado congelado)

**Entrada:** [reference/STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md](reference/STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md) (D21) → [reference/README.md](reference/README.md)

| Archivo | Contenido |
|---------|-----------|
| [reference/STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md](reference/STAGE_I_REGIME_IDENTIFICATION_CLOSURE.md) | **Cierre Stage I** — baseline científico de etapa |
| [reference/CURRENT_STATE_OF_THE_RESEARCH.md](reference/CURRENT_STATE_OF_THE_RESEARCH.md) | Artículo interno — qué sabemos / no sabemos |
| [reference/SCIENTIFIC_CHRONOLOGY.md](reference/SCIENTIFIC_CHRONOLOGY.md) | Cronología científica (no commits) |
| [reference/RESEARCH_STATUS.md](reference/RESEARCH_STATUS.md) | Fase del proyecto (Stage I cerrada) |
| [reference/RESEARCH_METRICS.md](reference/RESEARCH_METRICS.md) | Indicadores X/Y/Z + V/P/R |
| [reference/DEPENDENCY_MAP.md](reference/DEPENDENCY_MAP.md) | Dependencias K ↔ fases ↔ OQ |
| [reference/CONSISTENCY_AUDIT.md](reference/CONSISTENCY_AUDIT.md) | Auditoría de consistencia |
| [reference/REDUNDANCY_INVENTORY.md](reference/REDUNDANCY_INVENTORY.md) | Redundancias (solo listado) |
| [reference/STATE_OF_KNOWLEDGE.md](reference/STATE_OF_KNOWLEDGE.md) | Conocimiento consolidado (K1–K15) |
| [reference/DECISION_LOG.md](reference/DECISION_LOG.md) | Decisiones (D1–D22) |
| [reference/OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md](reference/OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md) | **Patrón transversal** — Γ̄ / cand1 (propiedad ≠ operacionalización) |
| [reference/RESEARCH_MAP.md](reference/RESEARCH_MAP.md) | Mapa fases → outcome / tag |
| [reference/OPEN_QUESTIONS.md](reference/OPEN_QUESTIONS.md) | OQ1–OQ9 (OQ7 cerrada; OQ8/OQ9 abiertas, **sin** experimento inmediato — D22) |

Índice (3 niveles: baselines / programa / en curso): [reference/README.md](reference/README.md)

---

Cerrar la **trazabilidad de señales** y verificar **conformidad con el [Body Frame Contract](08-body-frame-contract.md)**. Sin nuevas hipótesis numeradas hasta resolver ambigüedades de representación.

## Lectura recomendada

| Documento | Contenido |
|-----------|-----------|
| [01-overview.md](01-overview.md) | Metodología, cadena lógica H0→H9d, hechos sólidos vs abiertos |
| [02-data-and-frames.md](02-data-and-frames.md) | Fuentes de datos, cadena de marcos, convenciones |
| [03-experiments.md](03-experiments.md) | Catálogo completo de experimentos, scripts y artefactos |
| [04-findings.md](04-findings.md) | Resultados consolidados y decisiones |
| [05-attitude-investigation.md](05-attitude-investigation.md) | Bloque H9: actitud, triada gravitatoria, cadena de referencias |
| [06-reproduction.md](06-reproduction.md) | Compilar, ejecutar replay y reproducir auditorías |
| [07-signal-traceability.md](07-signal-traceability.md) | **Trazabilidad de señales** Android → mount → EKF; identidad física CSV |
| [08-body-frame-contract.md](08-body-frame-contract.md) | **Body Frame Contract** — especificación formal del modelo físico INS |
| [09-predict-conformance-audit.md](09-predict-conformance-audit.md) | **Auditoría de conformidad** predict() + replay vs contrato |
| [10-gap3-ins-model-audit.md](10-gap3-ins-model-audit.md) | GAP-3 auditoría detallada (§8.1–§8.18) |
| **[12-gap3-synthesis.md](12-gap3-synthesis.md)** | **GAP-3 cerrado** — síntesis A/B/C, diagrama causal |
| **[13-gap4-gnss-velocity-protocol.md](13-gap4-gnss-velocity-protocol.md)** | **GAP-4 diagnostic CERRADO** (`gap4-diagnostic-complete`) — autopsia P_pv |
| **[14-adaptive-nhc-protocol.md](14-adaptive-nhc-protocol.md)** | **GAP-5 v1 preregistrado** — PoC NHC adaptativo (instancia v1 **cerrada**) |
| **[15-gap5-passive-outcome.md](15-gap5-passive-outcome.md)** | **GAP-5 passive CONGELADO** — outcome v1; cierre instancia Γ̄ |
| **[16-gap5-v2-observable-selection.md](16-gap5-v2-observable-selection.md)** | **GAP-5 v2 CONGELADA** (v1.2) — propiedad → observable → caracterización → **modelo régimen** |
| **[11-replay-zupt-provenance.md](11-replay-zupt-provenance.md)** | **Proveniencia ZUPT legacy** — qué runs están condicionados |
| **[16-super-tunnel-ieee952-rerun-protocol.md](16-super-tunnel-ieee952-rerun-protocol.md)** | IEEE-952 / super_tunnel — cerrado; causa NHC ALWAYS |
| **[17-conditional-constraints-architecture.md](17-conditional-constraints-architecture.md)** | **Arquitectura única** ZUPT+NHC: disparo por estado, no reloj/ALWAYS |
| **[18-jacobian-imu-ab-protocol.md](18-jacobian-imu-ab-protocol.md)** | Jacobiano × IMU — early-loop cerrado; H-ATT-d abierta; cand1 no generaliza |
| **[19-ekf-explorer-protocol.md](19-ekf-explorer-protocol.md)** | **EKF Explorer** (Unity+Cesium) — instrumento científico; session pack v1 |
| **[20-h-seed-v-protocol.md](20-h-seed-v-protocol.md)** | **H-seed-v** — seed `v←GNSS`; H1 corrido (P1 PASS; course−yaw no) |
| **[21-h-seed-yaw-protocol.md](21-h-seed-yaw-protocol.md)** | **H-seed-yaw (H2)** — v+yaw:=course; preregistro + auditoría yaw; sin implementar |

## ⚠ Aviso de validez (Jul 2026)

Entre **H9** y **GAP-3.7**, el replay aplicaba ZUPT con `t≤30 s` OR `gps_speed≤0.1 m/s`. **Todos los experimentos full-filter de ese periodo deben considerarse condicionados** hasta repetirse con `--constraint-policy imu_stationary`. Los runs **predict-only** (H9) no están afectados. Detalle: [11-replay-zupt-provenance.md](11-replay-zupt-provenance.md).

## Pregunta operativa actual

**Pausa D22 (2026-07-19):** estabilizar documentación tras Stage I + cierre de generalización cand1. **No** abrir OQ8 experimental, cand2, ni GAP-5 v3 de inmediato.  
Patrón: [reference/OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md](reference/OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md). Estado: [reference/RESEARCH_STATUS.md](reference/RESEARCH_STATUS.md).

**Instrumento (en construcción):** [19-ekf-explorer-protocol.md](19-ekf-explorer-protocol.md) · `ekf_explorer/` — Unity+Cesium como plataforma de observación (session packs), no demo.

Cadena EKF cerrada a nivel Stage I: GAP-3 → GAP-4 → GAP-5 v1 → G-ext → H6 parcial.  
Hilo Jacobiano: mecanismo early cerrado; instrumento cand1 dominio-caracterizado; H-ATT-d abierta.

---

## Pregunta histórica (H9 / actitud)

> ¿Por qué `R_bn` desarrolla ~4° de error de inclinación al entrar en régimen dinámico, mientras el heading horizontal (`R_bn·e_x` vs GPS) puede seguir siendo coherente en el tramo crítico?

## Resumen ejecutivo (Jul 2026)

| Régimen | EKF ↔ Orientation | EKF ↔ gravedad | `a_lin,h` |
|---------|-------------------|----------------|-----------|
| Estático 0–2 s | **0.05°** | **0.09°** | **0.016 m/s²** |
| Dinámico 2–10 s | **4.07°** | **4.32°** | **0.74 m/s²** |

- **Descartado:** error global FRD/FLU, NED/ENU, `R_bn`/`R_nb`, orden Euler, `R_mount` variable, heading longitudinal en el arranque dinámico.
- **No demostrado:** que el EKF esté “equivocado” frente a Android (Orientation no es ground truth).
- **En curso:** auditoría de convenciones y divergencia entre estimadores bajo aceleración específica.

## Artefactos principales

Informes JSON y CSV en `docs/benchmarks/`. Gráficos PNG homónimos. Los scripts Python en la raíz del repo generan y actualizan estos artefactos.

## Commits relacionados

- `6510d5c` — pipeline Python + `real_run_replay.cpp`
- `bbbde6d` — target `NaviCore3D_Replay`, geodesy, hooks de audit en `ins_ekf`
