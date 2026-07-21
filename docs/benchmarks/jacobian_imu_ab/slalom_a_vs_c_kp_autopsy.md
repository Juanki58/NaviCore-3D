# SLALOM A vs C — NHC K/P autopsy (post-fix single run)

**Data guarantee:** every number below comes from one paired A→C re-run after deleting prior audit CSVs, with the memset/audit re-bind fix in the binary (`NaviCore3D_Sim.exe` mtime stamped in provenance). See `slalom_a_vs_c_kp_postfix_provenance.json`.

**Binary mtime UTC:** 2026-07-18T22:41:14.758674+00:00  
**Run wall UTC:** 2026-07-18T22:47:17.223243+00:00 → 2026-07-18T22:47:17.716781+00:00  
**Audit rows:** A=2501, C=2501  
**Audit sha16:** A=0043ca54c5b9166c, C=0ec5ee7237297061  

## Tick 0 (same run)

- innov A/C: **1e-06** / 1e-06 (same=True)
- k_att_max A/C: **0.05937** / 0.05937 (same=True)
- dx_att_z A/C: **-7.54189e-08** / 7.54189e-08 (ratio=-1, exact_negation=True)

## Cumulative dx_att_z (feedback vs constant offset)

Figure: `fig_slalom_kp_dxattz_cumsum_0_1s.png`

| t (s) | Σ dx_att_z A | Σ dx_att_z C | |ΣA−ΣC| | innov A |
|-------|--------------|--------------|---------|---------|
| 0.000 | -7.54189e-08 | 7.54189e-08 | 1.50838e-07 | 1e-06 |
| 0.370 | -3.15793e-06 | 1.66172e-06 | 4.81964e-06 | 6.7e-05 |
| 0.750 | -2.00341e-05 | 2.39708e-06 | 2.24311e-05 | 0.001021 |
| 1.000 | 2.93461e-05 | 2.61309e-06 | 2.6733e-05 | 0.004251 |
| 2.000 | 0.0118413 | 1.45411e-05 | 0.0118268 | 0.2817 |

### Growth metrics on |ΣA−ΣC| in [0, 0.75] s

- |sep| start → mid → end: **1.50838e-07** → 4.81964e-06 → **2.23e-05** (ratio end/start = 148)
- mean d|sep|/dt first half / second half: 1.26184e-05 / 4.72442e-05 (ratio 2nd/1st = **3.74**)
- innov_A end/start: **958**
- |v_body_y|_A end/start: **236**
- R² linear |sep| vs t: 0.9079; quad improvement: 0.967

**Growth verdict: `FEEDBACK_GROWTH`**

Thresholds: FEEDBACK if rate_ratio>2 or (innov_ratio>10 and sep_ratio>5); CONSTANT_OFFSET if rate_ratio∈[0.5,1.5] and innov_ratio<3 and R²>0.98.

First innov |A−C|>1e-3: t=0.75000006s (A=0.001021, C=6e-06)

0–1 s dx_att_z opposite-sign frac: **0.723**

## Causal reading (only if growth=FEEDBACK and tick0 negation)

1. Sign enters at `H_att`→`K_att`→`dx_att` from tick 0 (matched innov/|K|).
2. Cumulative separation of attitude corrections accelerates with innov/`|v_body_y|` — reinforcing NHC loop, not a frozen opposite offset.
3. ~2 s lag to position-divergence rate is the integrated consequence of that loop over many predict/update cycles.
4. Same `H_att` sign family as GAP-4 / `bf2bfbd`; SLALOM A×C (~143×) is the E2E face of that feedback.

**mechanism_closed (this run): `True`**

## OQ9 / T6 (unchanged)

- T6: punctual alias-shift miss — not structural.
- H-DEG-ATT-LAG (OQ9): turns 9–10 at drift_A ≳ 20 m — parked separately.
