# OQ9 — late dx_att_z sign (same check as early / tick 0)

**No verdict rename.** Discriminating datum only for `FEEDBACK_CONTINUES_OMEGA_DECOUPLED` working hypothesis.

Audits sha A=0043ca54c5b9166c, C=0ec5ee7237297061.  
**Figure:** `fig_slalom_oq9_late_dxattz_sign.png`  

## Sign metrics by window

| Window | opp frac | same frac | sign corr | ratio median | frac ratio≈−1 | rms A/C |
|--------|----------|-----------|-----------|--------------|----------------|---------|
| early_0_0p75s | 0.88 | 0.12 | 0.4292377895489358 | -2.092179112836698 | 0.29333333333333333 | 8.404312744054138 |
| early_0_4s | 0.4389027431421446 | 0.5610972568578554 | -0.005212287051586743 | 185.9593914273433 | 0.05486284289276808 | 18719.655203088336 |
| mid_4_14s | 0.554 | 0.446 | -0.0327837912104997 | -5072.615204058515 | 0.001 | 90974.19667255107 |
| late_14_25s | 0.5463636363636364 | 0.4536363636363636 | -0.006318184944304984 | -2909.3766759864175 | 0.0 | 38963.39232145118 |
| late_22_25s | 0.44333333333333336 | 0.5566666666666666 | 0.08672427903329931 | 3620.5261223378857 | 0.0 | 38624.89480256418 |

## Cumulative |ΣA−ΣC| growth

| Window | |sep| start→end | end/start | rate 2nd/1st half | R² linear |
|--------|----------------|-----------|------------------|-----------|
| early_0_0p75s | 1.508e-07→2.23e-05 | 147.8 | 3.74 | 0.908 |
| early_0_4s | 1.508e-07→0.05368 | 3.559e+05 | 3.54 | 0.771 |
| late_14_25s | 0.001756→0.2394 | 136.3 | 2.13 | 0.001 |
| late_22_25s | 0.003552→0.1395 | 39.28 | 0.419 | 0.056 |

## Does dx_att_z still dominate / couple to P_pp_C late?

- att energy fraction in z (A), 14–25 s: **0.744**
- corr(|dx_att_z_A|, dP_pp_C/dt): **0.261**
- corr(innov_A, dP_pp_C/dt): **0.434**

## Discriminating tag (not a rename)

**`SIGN_MIXED`**

Intermediate opposite frac — do not claim same-loop or new-mechanism yet.

- early looked persistent: False
- late att energy mostly z: True
- late |dx_att_z| coupled to dP_pp_C: False

Do not rename FEEDBACK_CONTINUES_OMEGA_DECOUPLED from this file alone; report late_sign_tag as the discriminating datum.

## Reading guide

- `SIGN_STILL_OPPOSITE_PERSISTENT` → strengthens same-loop reading (still not sufficient alone with P/burst checks).
- `SIGN_NO_LONGER_OPPOSITE` → weakens same-loop; late P_pp growth likely needs another driver name.
- `SIGN_MIXED` / `RATIO_NOT_LOCKED` → keep working hypothesis; do not preregister late success criteria yet.
