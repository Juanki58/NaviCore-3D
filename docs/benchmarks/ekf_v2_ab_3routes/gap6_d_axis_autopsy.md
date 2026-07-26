# GAP-6 — D-axis NIS autopsy (post Bowring fix)

Source: re-run `ekf_v2_ab_3routes` v2 arms after `ecef_to_lla` Bowring fix
(`src/core/geodesy.cpp`). Pre-fix autopsy preserved in git history / `SUMMARY_prev.json`.

## Before → after (v2)

| Route | Accept | Drift H [m] | Drift V_log [m] | Drift 3D_log [m] | filt↔z \|ΔD\| [m] | log−z NED |
|-------|--------|------------:|----------------:|-----------------:|-----------------:|-----------|
| REF | 100%→87.8% | 35.3→**13.9** | 41.6→**123.3** | 54.5→124.1 | 69.0→~123 | ~(+31,0,−27)→**~0** |
| ALT | 100%→100% | 38.0→**6.2** | 24.6→**4.0** | 45.3→**7.4** | 2.7→~4 | →**~0** |
| JUL17 | 100%→97.9% | 110.3→**87.6** | 58.0→**23.0** | 124.6→**90.5** | 30.6→~23 | →**~0** |

V_log = `|estado_d − Ultimo_GPS_d|`. 3D_log = `sqrt(H² + V_log²)`.

## nis_contrib_d (accepted fixes, post-fix)

| Route | n | mean | median | p90 | N/E mean | innov_d mean | frac_pos |
|-------|--:|-----:|-------:|----:|---------:|-------------:|---------:|
| REF | 598 | **21.87** | 9.68 | 46.74 | 47.0 / 48.4 | 15.34 | 0.70 |
| ALT | 331 | **0.60** | 0.31 | 1.72 | 2.67 / 3.95 | 1.10 | 0.68 |
| JUL17 | 700 | **0.62** | 0.19 | 1.88 | 6.54 / 12.44 | 2.80 | 0.74 |

## Verdict (post-fix)

- El offset `log_gps − audit_z` de ~30 m **desaparece** (última z ≈ Último GPS).
- Qualitativa de contrib-D **no cambia**: alta solo en REF; baja en ALT/JUL17.
- Deriva H mejora en las tres. V_log mejora en ALT/JUL17; en REF empeora
  (error vertical real ~123 m ahora visible sin el bias de origen).
- Accept-rate v2 ya no es 100% en REF/JUL17 (el gate NIS ve innovaciones
  sin el sesgo sistemático del adaptador).
