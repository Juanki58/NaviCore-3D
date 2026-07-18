# H4 — Consistency Test + P0 Sweep

## Consistency (P0=1x, fase en marcha)

- Muestras en marcha: 304
- Error horizontal medio: 2011.8 m
- Sigma declarada media (sqrt P_h): 3.1 m
- Ratio error/sigma medio: **610.7**
- Ratio error/sigma mediana: 662.0
- NEES medio (eje N): 52394.3
- NEES medio (eje E): 143765.7
- % NEES_N > 11.3: 99.7%

> Criterio: error/sigma ~ 1 indica consistencia. Valores >> 1 indican P/R optimistas.

## Barrido P0

| P0 scale | RMSE H (m) | Error final (m) | GNSS accept % | Rechazos | NIS medio |
|----------|------------|-----------------|---------------|----------|-----------|
| 1x | 2351.1 | 4586.7 | 3.0% | 321 | 197011 |
| 2x | 2279.3 | 4484.0 | 3.0% | 321 | 176414 |
| 5x | 2330.6 | 4549.8 | 3.3% | 320 | 182243 |
| 10x | 2317.0 | 4537.1 | 3.3% | 320 | 177524 |
| 20x | 2311.3 | 4534.0 | 3.3% | 320 | 175043 |
| 50x | 2311.8 | 4531.1 | 3.6% | 319 | 182228 |

Graficos: `h4_p0_sweep_rmse.png`, `h4_p0_sweep_accept.png`, `h4_consistency_analysis.png`
