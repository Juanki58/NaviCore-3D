# GAP-3.18 — F1.2 Anatomía del cliff NHC

## Pregunta

¿El burst depende del **estado** (P_pre al disparar NHC) o solo de la **frecuencia**?

**Veredicto:** `STATE_CONDITIONED_BURST`

Decimation cambia cuándo dispara NHC, pero |ΔP| en cada disparo depende de P_pre (corr N=1=0.70). Cliff persiste bursty (top3→96%); no es Riccati suave ni solo frecuencia.

## Cliff por política (gap fix#2→#3)

| Policy | N | nhc events | cliff tick | |ΔP| cliff | P_pre cliff | top3 share | 1st NHC tick | |ΔP| 1st NHC |
|--------|--:|-----------:|-----------:|---------:|------------:|-----------:|-------------:|-----------:|
| N=1 | 1 | 38 | 3 | 28.0 | 51.9 | 74% | 1 | 1.6 |
| N=10 | 10 | 4 | 26 | 36.9 | 66.5 | 96% | 6 | 2.5 |
| N=20 | 20 | 2 | 26 | 4.0 | 74.6 | 100% | 6 | 2.4 |

## K real en cliff (N=1, ticks 2–4)

| tick | k_vel_max | K_scalar_z | NIS | |ΔP_vv| | P_pre |
|------|----------:|-----------:|----:|-------:|------:|
| 2 | 2.344 | 0.446 | 3.23 | 9.9 | 61.4 |
| 3 | 3.547 | 0.551 | 0.03 | 28.0 | 51.9 |
| 4 | 2.327 | 0.357 | 0.00 | 8.4 | 24.1 |

## Implicación

- Frecuencia sola **no** explica el gate: decimar NHC mueve el cliff pero no lo elimina.
- Burst **condicionado por estado**: |ΔP|/P al disparar correlaciona con P_pre; primer NHC tras fix#2 a P≈62 cae poco (N=10), cliff grande cuando P reconstruido.
- Complementa F1.1: observabilidad (P,K) y nominal (r,S) son mecanismos separados.
