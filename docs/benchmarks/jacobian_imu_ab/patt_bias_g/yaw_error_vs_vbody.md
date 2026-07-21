# Yaw error ↔ filter v_body — quantitative check

**Verdict (latch):** `CASCADE_VIA_VEL_STATE_NOT_INSTANT_YAW_PROJECTION`

The simple equation filter_v_lat ≈ -V·sin(Δyaw) is FALSIFIED for latch: Δyaw only ~0.3–1.2° predicts |v_lat|≲0.29 m/s but observed max is 1.80 m/s. Instead filter_v_lat ≈ filter NED cross-track vs truth heading (pearson 0.98, frac var 0.96) — the lateral innov is the already-corrupted velocity state, not an instantaneous yaw misprojection of forward speed. filter_v_vert still tracks attitude projection of speed (roll/pitch; att_only pearson ~0.985) with partial amplitude. Cascade holds end-to-end, but the proximate equation at explosion time is: attitude loop → polluted vel_NED → body lat/vert via NHC, not v_lat=V·sin(Δψ) alone.

## Key numbers

- Δyaw: [0.28, 1.20]°
- max|-V sinΔψ|: **0.290** m/s vs max|filter_v_lat|: **1.805** m/s (~6× larger)
- pearson(v_lat, filter cross-track): **0.980**, frac var **0.960**
- pearson(v_lat, -V sinΔψ): **-0.709** (fails)
- roll range: [-10.0, -3.5]°; pitch: [-3.9, 3.8]°

## Tick table — latch

| t | Δyaw° | roll° | pitch° | v_cross | v_lat | −VsinΔψ | v_vert |
|---|-------|-------|--------|---------|-------|---------|--------|
| 1.690 | +0.28 | -3.5 | +3.8 | -1.640 | -1.635 | -0.068 | +1.672 |
| 1.700 | +0.40 | -4.5 | +2.6 | -1.650 | -1.796 | -0.097 | +1.304 |
| 1.710 | +0.58 | -5.3 | +0.9 | -1.498 | -1.805 | -0.140 | +0.622 |
| 1.720 | +0.76 | -5.5 | -0.7 | -1.208 | -1.623 | -0.185 | -0.224 |
| 1.730 | +0.92 | -5.6 | -2.0 | -0.835 | -1.298 | -0.224 | -0.998 |
| 1.740 | +1.05 | -6.0 | -3.0 | -0.402 | -0.897 | -0.256 | -1.581 |
| 1.750 | +1.16 | -6.8 | -3.7 | +0.076 | -0.436 | -0.280 | -1.977 |
| 1.760 | +1.20 | -8.0 | -3.9 | +0.525 | +0.083 | -0.290 | -2.182 |
| 1.770 | +1.12 | -9.3 | -3.3 | +0.802 | +0.574 | -0.271 | -2.117 |
| 1.780 | +0.90 | -10.0 | -2.0 | +0.841 | +0.863 | -0.219 | -1.707 |

## Implication for cascade close

- Qualitative cascade (Jacobian → attitude loop → innov explosion) **still holds**.
- Quantitative link at explosion time is **not** instant yaw projection of V.
- Proximate: **polluted filter vel_NED** (cross-track ≈ v_lat) + **roll/pitch** feeding v_vert.
- Intervention should target the cascade **before** velocity is already wrong (onset/early attitude path), not assume fixing Δyaw alone at t≈1.7 kills v_lat.

Figure: `fig_yaw_error_vs_vbody.png`
