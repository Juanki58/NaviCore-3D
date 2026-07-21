# OQ9 follow-up — late P A/C + burstiness 22–25 s

**Status: working-hypothesis checks — does NOT close** `FEEDBACK_CONTINUES_OMEGA_DECOUPLED`.

Post-fix audits sha A=0043ca54c5b9166c, C=0ec5ee7237297061.  
**Figure:** `fig_slalom_oq9_late_p_and_burstiness.png`  

## 1. What “P_C explodes” means (numbers)

Blocks logged: Frobenius norms `P_pre_aa`, `P_pre_vv`, `P_pre_pv`, `P_pre_pp` from NHC audit (pre-update each NHC tick).

### Late window 14–25 s

| Block | P_A start→end | P_C start→end | C end/start | C/A end |
|-------|---------------|---------------|-------------|---------|
| P_aa | 0.001659→0.0008189 | 0.6615→2.059 | **3.11** | **2.51e+03** |
| P_vv | 0.7946→0.7288 | 127→396.5 | **3.12** | **544** |
| P_pv | 2.912→3.735 | 890.4→4942 | **5.55** | **1.32e+03** |
| P_pp | 21.21→50.9 | 6287→6.173e+04 | **9.82** | **1.21e+03** |

**Largest C growth (end/start):** `P_pp` (×9.82).

### Early reference 0–4 s (same blocks)

| Block | P_A mean | P_C mean | C/A mean |
|-------|----------|----------|----------|
| P_aa | 0.01237 | 0.02996 | 2.42 |
| P_vv | 3.51 | 5.939 | 1.69 |
| P_pv | 4.378 | 8.754 | 2 |
| P_pp | 11.75 | 19.75 | 1.68 |

### “Mirror of non-failure” test (data, not slogan)

- explode block: **P_pp**
- mirror_like (criteria in JSON): **True**
- frac log10(P_C/P_A) increasing: 0.854
- log10(C/A): 2.472 → 3.084
- corr(innov_A, P_C on explode block): 0.433

'Mirror of non-failure' would require: same H_att sign family still driving A loud innov while C's covariance on the explode block grows (uncertainty opens) rather than A/C sharing a new late mechanism. mirror_like=True is necessary but not sufficient for that claim.

## 2. Burstiness of 22–25 s (|drift_A|)

Rule: **bursty** if top3_share > 0.5 OR B > 0.25 (B = max\|Δ\| / Σ\|Δ\|).

| Window | B | top3_share | bursty | sum\|Δ\| |
|--------|---|------------|--------|---------|
| early_0_4s | 0.0690 | 0.1648 | **False** | 16.028 |
| late_14_25s | 0.0037 | 0.0109 | **False** | 160.267 |
| burst_win_22_25s | 0.0100 | 0.0294 | **False** | 59.212 |

Dominating ticks in 22–25 s (`|drift_A|`):

- rank 1: t∈[22.540,22.550] s, dx=-0.5906, share=0.010
- rank 2: t∈[22.550,22.560] s, dx=-0.5842, share=0.010
- rank 3: t∈[22.530,22.540] s, dx=-0.5655, share=0.010

Shortest interval with ≥50% of 22–25s Σ\|Δ\|: **1.190 s** [22.000, 23.190] (share=0.50)

**Drift-shape label: `NOT_BURSTY_CONTINUOUS_LIKE`**

22–25s fails burstiness thresholds — more consistent with distributed accrual (continuous-like) than a GAP-3-style cliff.

## 3. What this does to the prior OQ9 verdict

Prior label `FEEDBACK_CONTINUES_OMEGA_DECOUPLED` stays a **working hypothesis** until both checks are accepted:

1. P mirror: mirror_like=True on P_pp (see criteria).
2. 22–25s shape: `NOT_BURSTY_CONTINUOUS_LIKE` (bursty=False).

**Not** a closed basis for a late-regime §11-style success criterion yet.
