# Innov composition break vs ‖ω‖

**Verdict:** BREAK_ON_RISING_LIMB_COLOCATED_WITH_PRIOR_DDRIFT_PEAK

Fine break at t=1.590s is NOT the arbitrary 1.5 bin and NOT the ||w|| peak at 2.0s. It sits on the rising limb of the first interior turn (|w|/peak≈0.80, 0.41s before peak), and coincides with the prior A×C max|dΔdrift/dt| at 1.61s (Δ=-0.020s). Early post-latch (desc+valley): latch innov smaller, cos≈1 — freeze-Z helps while no new turn. After break on rising limb: composition flips and latch innov/bias explode. K_bias decompose should be conditioned on interior-turn rising limb (post omega zero-x @1s / post-break), not crude |w|>threshold (polluted by t=0 descending limb) and not clock-only [1.5,2].

## Fine break (not the 1.5 bin)

- Primary break: **t = 1.590 s** (roll-median-5 cos<0.5, sustain 3)
- Leave parallel (cos<0.9): **1.540 s**; first cos<0: **1.610 s**
- Max single-step cos drop: **1.820 s** (closer to ω peak)
- ‖ω‖/peak at break: **0.80**; Δ to peak@2.0: **0.410 s**
- Δ to prior max|dΔdrift/dt|@1.61: **-0.020 s**
- ω zero-crossing @1.0; half-peak onset @1.34; break is **0.25 s after onset**

## Early latch smaller innov — confirmed

In desc-limb + valley (and rise pre-break), latch ‖y‖ ≤ ctrl and cos≈1. Freezing Z does **not** inflate innov until the rising limb breaks.

## Phases along the interior turn (not crude |ω| gate)

| Phase | n | median cos | ‖y‖_L/‖y‖_C | Σdx_bias_gz latch | mean|ω| |
|-------|---|------------|-------------|-------------------|--------|
| desc_limb_[0.39,1.0) from t=0 peak | 61 | 0.996 | 0.77 | -0.0000 | 0.097 |
| valley_[1.0,1.34) after omega zero-x | 34 | 0.992 | 0.73 | -0.0008 | 0.055 |
| rise_[1.34,2.0] half-peak→peak | 67 | 0.389 | 3.95 | -0.0259 | 0.179 |
| rise_pre_break_[1.34,1.590) | 25 | 0.972 | 0.55 | -0.0124 | 0.142 |
| rise_post_break_[1.590,2.0] | 42 | -0.815 | 5.98 | -0.0136 | 0.201 |

**Note:** crude |ω|≥0.5·peak is polluted (includes descending limb from t=0 → median cos stays ~1). Condition on **rising limb of interior turn**.

## Implication for K_bias decompose

Compare ticks in rise_post_break (or t∈[t_break,2.0]) vs pre-break post-latch; do NOT use undifferentiated |w|>=0.5*peak.

Figure: `fig_innov_break_vs_omega.png`
