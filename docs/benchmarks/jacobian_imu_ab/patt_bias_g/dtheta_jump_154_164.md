# ‖δθ‖ jump autopsy [1.54→1.64] — K vs y vs state

**Verdict:** Y_JUMP_MULTI_TICK_NOT_K_SPIKE_DXZ_LATCHED_ZERO

Not a punctual K_att gain spike (k_att_max only ~1.12× post/pre; g_eff ~1.08×). NHC innov ||y|| rises smoothly across [1.54,1.64] (~2.24× post/pre) and ||dx_att|| tracks it (~2.41×) — Y_JUMP window. No single-tick cliff (max step ||dx|| only 1.20×). CRITICAL: under latch λ=1, dx_att_z ≡ 0 every tick — the applied Z correction is off; state ||δθ|| growth in this window is NOT from NHC ATT_Z updates. State ||δθ|| ramps multi-tick (0.70°→2.60° in table) dominated by roll growth; ctrl also ramps state error (even larger) without a y cliff. The 7.5× rise→surge mean jump (§13.14) is a multi-tick ramp of state attitude error coinciding with rising NHC innov (vel already dirty), not a ZUPT/GNSS-style K spike.

**Next:** Ask what grows state attitude (esp. roll) across [1.59,1.69] if not NHC Z: predict/gyro+bias, NHC dx_att_x/y, or latch side-effects. Compare roll error vs truth and bias_g / gyro integration in that window.

## Key facts

- Latch **dx_att_z ≡ 0** (λ=1) — not a Z update spike
- k_att_max / g_eff ≈ flat (~1.1×) — **not** K punctual gain
- ||y|| and ||dx_att|| ramp together (~2.2–2.4× post/pre) — **Y-driven**, multi-tick
- state ||δθ|| ramps 0.70°→2.60° in window; roll-dominated
If K_JUMP: preregister gain clamp on NHC→att (ZUPT/GNSS pattern). If Y_JUMP: step back — what feeds innov at that tick (state vel already dirty?). If PRODUCT: conditional on both. If state||δθ|| jumps without dx spike: accumulation / predict path, not one NHC update.

## Ratios post-break / pre-break (means)

| Arm | ‖dx_att‖ | ‖y‖ | k_att_max | g_eff | ‖δθ‖_state | P_aa_frob |
|-----|----------|-----|-----------|-------|------------|-----------|
| ctrl | 1.59× | 1.04× | 0.85× | 1.55× | 1.66× | 0.93× |
| latch | 2.41× | 2.24× | 1.12× | 1.08× | 2.10× | 0.99× |

## Max single-tick jump (latch)

- cause: **NO_CLEAN_DX_JUMP** @ t=1.610000134
- ‖dx_att‖: 3.8831e-03 → 4.6577e-03 (1.20×)
- ‖y‖: 0.5987 → 0.7100 (1.19×)
- k_att_max: 0.0159 → 0.0165 (1.04×)
- g_eff: 6.4857e-03 → 6.5600e-03 (1.01×)

## Tick table — latch

| t | ‖y‖ | k_att_max | g_eff | ‖dx_att‖ | dx_z | ‖δθ‖_state° | P_aa |
|---|-----|-----------|-------|----------|------|-------------|------|
| 1.540 | 0.2413 | 0.0149 | 5.7751e-03 | 1.3938e-03 | +0.000e+00 | 0.695 | 1.6120e-02 |
| 1.550 | 0.2773 | 0.0149 | 5.8998e-03 | 1.6362e-03 | +0.000e+00 | 0.791 | 1.6101e-02 |
| 1.560 | 0.3201 | 0.0149 | 6.0256e-03 | 1.9291e-03 | +0.000e+00 | 0.905 | 1.6080e-02 |
| 1.570 | 0.3714 | 0.0150 | 6.1508e-03 | 2.2843e-03 | +0.000e+00 | 1.039 | 1.6059e-02 |
| 1.580 | 0.4330 | 0.0152 | 6.2726e-03 | 2.7163e-03 | +0.000e+00 | 1.199 | 1.6035e-02 |
| 1.590 | 0.5077 | 0.0154 | 6.3865e-03 | 3.2427e-03 | +0.000e+00 | 1.390 | 1.6008e-02 |
| 1.600 | 0.5987 | 0.0159 | 6.4857e-03 | 3.8831e-03 | +0.000e+00 | 1.619 | 1.5979e-02 |
| 1.610 | 0.7100 | 0.0165 | 6.5600e-03 | 4.6577e-03 | +0.000e+00 | 1.892 | 1.5944e-02 |
| 1.620 | 0.8465 | 0.0173 | 6.5949e-03 | 5.5825e-03 | +0.000e+00 | 2.218 | 1.5904e-02 |
| 1.630 | 1.0135 | 0.0184 | 6.5721e-03 | 6.6607e-03 | +0.000e+00 | 2.604 | 1.5855e-02 |

## Tick table — ctrl

| t | ‖y‖ | k_att_max | g_eff | ‖dx_att‖ | ‖δθ‖_state° |
|---|-----|-----------|-------|----------|-------------|
| 1.540 | 0.5205 | 0.0170 | 9.7343e-03 | 5.0665e-03 | 1.991 |
| 1.550 | 0.5931 | 0.0170 | 1.0286e-02 | 6.1002e-03 | 2.319 |
| 1.560 | 0.6638 | 0.0167 | 1.0998e-02 | 7.3007e-03 | 2.687 |
| 1.570 | 0.7213 | 0.0158 | 1.2022e-02 | 8.6720e-03 | 3.080 |
| 1.580 | 0.7498 | 0.0142 | 1.3581e-02 | 1.0183e-02 | 3.475 |
| 1.590 | 0.7362 | 0.0136 | 1.5783e-02 | 1.1619e-02 | 3.849 |
| 1.600 | 0.6875 | 0.0140 | 1.8138e-02 | 1.2471e-02 | 4.186 |
| 1.610 | 0.6419 | 0.0139 | 1.9287e-02 | 1.2381e-02 | 4.501 |
| 1.620 | 0.6391 | 0.0136 | 1.8391e-02 | 1.1754e-02 | 4.824 |
| 1.630 | 0.6794 | 0.0132 | 1.6161e-02 | 1.0979e-02 | 5.175 |

Figure: `fig_dtheta_jump_154_164.png`

Full K_att rows not in audit; g_eff:=||dx_att||/||y|| is the realized gain scale. k_att_max is max|K| over att rows — relative jumps comparable.
