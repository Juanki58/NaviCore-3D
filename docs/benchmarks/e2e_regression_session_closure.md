# Regresión E2E — estado de sesión (NO cerrado)

**Fecha:** 2026-07-18 / 2026-07-19  
**Estado:** **NO cerrado.** La primera corrida E2E no cierra la sesión; **abre** una pregunta que precede a todo lo construido después de `bf2bfbd`.

## Hallazgo

El artefacto ancla de confianza de la cadena (Jacobiano NHC corregido, verificado por diferencias finitas) **nunca se confirmó** como mejora neta contra SLALOM / TUNNEL_STRESS. `bf2bfbd` mezcla ese fix con IMU dirty IEEE-952. El FAIL E2E (~54 m / ~432 m) es la misma clase que el primer FAIL de `history.json` (`16d6ccc`).

## Preflight (primera corrida)

| Check | Resultado |
|-------|-----------|
| `p_pv_policy` | `none` — Sim no setea gates §11 |
| Constraints | Hardcode de escenario; no `forced_time` Replay |

## Primera corrida (seed = reloj)

| Escenario | Medido | Límite | |
|-----------|--------|--------|--|
| SLALOM | 54.0 m | 0.15 m | FAIL |
| TUNNEL exit | 432.5 m | 15 m | FAIL |

**No** escribir «regresión E2E: PASS» ni «sesión cerrada».

## Siguiente paso obligatorio

A/B preregistrado: [18-jacobian-imu-ab-protocol.md](../diagnostics/18-jacobian-imu-ab-protocol.md)

- Seed fijo **71**
- Brazo A: Jacobiano correcto + IMU **ideal**
- Brazo B: Jacobiano correcto + IMU **dirty**

Orquestador: `python tools/run_jacobian_imu_ab.py`
