# Diagnóstico EKF — Real Run (H0–H9d)

Documentación técnica del pipeline experimental usado para auditar consistencia, geodesia, sincronización y propagación inercial del EKF sobre datos reales de vehículo (`data/real_run/`).

## Objetivo

Reducir por eliminación la causa de deriva horizontal y sobreconfianza del filtro durante aceleración longitudinal, sin tuning ciego ni nuevas hipótesis numeradas hasta cerrar la cadena de actitud.

## Lectura recomendada

| Documento | Contenido |
|-----------|-----------|
| [01-overview.md](01-overview.md) | Metodología, cadena lógica H0→H9d, hechos sólidos vs abiertos |
| [02-data-and-frames.md](02-data-and-frames.md) | Fuentes de datos, cadena de marcos, convenciones |
| [03-experiments.md](03-experiments.md) | Catálogo completo de experimentos, scripts y artefactos |
| [04-findings.md](04-findings.md) | Resultados consolidados y decisiones |
| [05-attitude-investigation.md](05-attitude-investigation.md) | Bloque H9: actitud, triada gravitatoria, cadena de referencias |
| [06-reproduction.md](06-reproduction.md) | Compilar, ejecutar replay y reproducir auditorías |

## Pregunta operativa actual

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
