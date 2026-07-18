# GAP-3.17 — F1.1 Anatomía del gate NIS

## Cadena tras F1

| Eslabón | Estado |
|---------|--------|
| NHC → P_vv | ✅ confirmado |
| P_vv → k_vel | ✅ confirmado |
| k_vel → accepts | ❌ **no confirmado** (N=10: k_vel×12, accepts=7) |

**Cuello de botella:** innovación / estado nominal (r), no K solo.

## Transición último accept → primer reject

| Policy | acc# | rej# | NIS_acc | NIS_rej | innov_h_acc | innov_h_rej | |Λ_N|_rej | S_NN_rej | k_vel_rej | dom axis |
|--------|-----:|-----:|--------:|--------:|------------:|------------:|---------:|---------:|----------:|---------|
| N=1 | 7 | 8 | 6.4 | 9.2 | 20.1 | 27.8 | 2.34 | 137 | 0.024 | n |
| N=10 | 7 | 8 | 7.3 | 9.4 | 25.3 | 30.4 | 2.28 | 173 | 0.149 | n |
| N=20 | 5 | 6 | 8.3 | 9.2 | 37.7 | 33.6 | 2.25 | 218 | 0.062 | n |
| OFF | 56 | 57 | 9.4 | 12.2 | 19.3 | 22.0 | 3.27 | 42 | 0.091 | n |

## Paradoja N=10 vs N=20

N=20 tiene **más** P_vv pre#3 (78 vs 22) y **más** k_vel (0.24 vs 0.09) pero **menos** accepts (5 vs 7).
Estado nominal peor (innovación mayor) compite con mejor S — el gate ve rᵀS⁻¹r.

## Rejects gps_index 8–14 (N=1) — descomposición NIS

| fix | NIS | contrib_N | contrib_E | contrib_D | dom | |Λ_N| | innov_h |
|-----|----:|----------:|----------:|----------:|-----|------:|--------:|
| 8 | 9 | 5 | 0.1 | 3.6 | n | 2.34 | 27.8 |
| 9 | 25 | 15 | 2.0 | 8.3 | n | 3.91 | 32.1 |
| 10 | 51 | 29 | 7.4 | 13.9 | n | 5.46 | 37.4 |
| 11 | 66 | 37 | 14.0 | 15.1 | n | 6.21 | 43.8 |
| 12 | 83 | 46 | 21.0 | 16.2 | n | 7.00 | 51.8 |
| 13 | 115 | 64 | 31.4 | 19.1 | n | 8.25 | 62.6 |
| 14 | 139 | 75 | 44.5 | 19.6 | n | 9.00 | 71.6 |

**Patrón:** contrib_N domina en rejects tempranos (fix 8–14); eje N crece monotónicamente.
No es crecimiento homogéneo — es **error nominal en N** mientras S_NN también cae.
