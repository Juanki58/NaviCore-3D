# GAP-3.16 — NHC cliff mechanism checks

## 1. ¿Cliff = K≈1?

| tick | K_scalar_z | k_vel_max | NIS | ΔP_vv | ||ΔP||/||Δx|| |
|------|----------:|----------:|----:|------:|---------------:|
| 2 | 0.446 | 2.344 | 3.23 | -9.9 | 3.6 |
| 3 | 0.551 | 3.547 | 0.03 | -28.0 | 47.2 |
| 4 | 0.357 | 2.327 | 0.00 | -8.4 | 122.2 |

**Verdict:** `multivariate_geometry_not_scalar_saturation` — K escalar HPH/(HPH+R) ∈ [0.35, 0.55], no ~0.99.

## 2. predict +3.2

- ΣΔP_vv predict: **+3.19**
- White Q frob est: **0.0003** (ratio **11951×**)
- ΣΔP_pv predict: +5.00
- Observed +3.19 >> white Q frob ~0.0003 (ratio 11951x) → growth from F*P cross-terms (att/bias/pos→vel), not Q alone.

## 3. ¿2.5 suelo?

- Últimos 10 ticks del gap: P_vv ≈ 2.57 ± 0.06, slope/tick=-0.0208
- **Gap tail verdict:** `soft_floor_in_gap` (2.5 es equilibrio predict↔NHC, no foto a mitad de caída libre)
- Post-fix#3 +4s: range 4.3 → `oscillating_equilibrium`

## 4. F1 cliff N=1 vs N=10

- Baseline cliff tick: **3** (max |ΔP|=28.0)
- F1c cliff tick: **26** (max |ΔP|=36.9)
- **Verdict:** `frequency_spreads_early_cliffs`
