# GAP-3.13 — Fix #2 autoconsume & K_vel algebra

## 1. Two-cell check: P_vv post#2 vs pre#3

| Metric | Value |
|--------|------:|
| fix#2 pre-GNSS P_vv | 89.60 |
| fix#2 post-GNSS P_vv (Joseph) | 62.00 |
| fix#3 pre-GNSS P_vv | 2.50 |
| post#2 / pre#3 ratio | 24.8× |
| Joseph drop (pre→post #2) | 27.6 (31%) |
| Inter-fix drop (post#2→pre#3) | 59.5 (96%) |
| Gap duration | 0.39 s |
| NHC ticks in gap | 76 |
| last NHC post = fix#3 pre? | True |

**Verdict:** `hybrid_joseph_then_nhc`

La identidad literal post#2 ≈ pre#3 **no se cumple** (62 vs 2.5). Joseph en fix#2 consume ~31% de P_vv;
el salto restante (~97% del post#2) ocurre en 0.39 s vía **76 updates NHC** — el último NHC post coincide
exactamente con pre#3.

## 2. K_vel algebra (7 accepts)

| gps | k_vel_csv | max|P·S⁻¹| | residual max | innov_h |
|-----|----------:|-------------:|-------------:|--------:|
| 2 | 0.1970 | 0.1970 | 6.22e-08 | 29.3 |
| 3 | 0.0078 | 0.0078 | 1.69e-09 | 32.0 |
| 4 | 0.0167 | 0.0167 | 2.30e-09 | 31.2 |
| 5 | 0.0013 | 0.0013 | 4.06e-10 | 23.8 |
| 6 | 0.0049 | 0.0049 | 4.96e-10 | 23.1 |
| 7 | 0.0233 | 0.0233 | 1.73e-09 | 20.1 |

K_vel_pos = P_vel_pos · S⁻¹ cierra algebraicamente (residual ~1e-6). k_vel_max CSV coincide con max abs del bloque predicho.

## 3. innov_h plano (hallazgo independiente)

- Media: 27.2 m, std: 4.4 m, rango: 11.9 m
- Pendiente lineal vs gps_index: -1.89 m/fix → **flat_no_convergence**
- innov_h stays ~20–32 m across 7 accepts in ~10 s — no convergence trend; position error re-accumulates between accepts (consistent with velocity never corrected).
