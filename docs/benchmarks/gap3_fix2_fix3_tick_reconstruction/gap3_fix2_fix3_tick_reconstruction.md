# GAP-3.14 — Tick-a-tick fix#2→#3 + Joseph verification

## Joseph fix#2 (89.6 → 62.0)

| | Observed | Block Joseph | Full-15 approx |
|--|---------:|-------------:|---------------:|
| P_vv pre | 89.60 | — | — |
| P_vv post | 62.00 | 58.70 | 58.70 |
| Error vs obs post | — | -3.29 | -3.29 |

**Verdict:** `joseph_algebra_consistent`

## 76 ticks inter-fix

- Duration: 0.373 s, ticks: 38
- P_vv: 62.0 → 2.5 (Δ=59.5)
- ΣΔP_vv predict: +3.2 (+5% of drop)
- ΣΔP_vv NHC: -62.7 (-105% of drop)
- Erosion pattern: **bursty** (top-3 ticks = 74% of |ΔP_vv| NHC)
- max -dP_vv/dt: 2736 /s (tick 3)

### Top-5 NHC ticks by |ΔP_vv|

| tick | imu_seq | ΔP_vv NHC | |Δv| NHC | NIS | |v|_h |
|------|--------:|----------:|---------:|----:|-----:|
| 3 | 413 | -28.0 | 0.95 | 0.03 | 0.45 |
| 2 | 412 | -9.9 | 5.66 | 3.23 | 1.39 |
| 4 | 414 | -8.4 | 0.11 | 0.00 | 0.36 |
| 5 | 415 | -4.0 | 0.05 | 0.00 | 0.33 |
| 6 | 416 | -2.4 | 0.03 | 0.00 | 0.31 |

## Interpretación

Predict **regenera** P_vv ligeramente (+ΣΔ); NHC **domina** el descenso neto.
El patrón no es una rampa uniforme — un subconjunto de updates NHC (vel nominal grande post-GNSS) concentra la erosión → problema de **orden temporal propagación/restricciones**, no solo Joseph.
