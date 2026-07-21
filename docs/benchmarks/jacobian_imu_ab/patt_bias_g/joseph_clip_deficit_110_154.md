# Joseph underclip budget — ¿explica ΔP[ATT_Y,VN]?

**Verdict:** `JOSEPH_DEFICIT_LINEAR_SUFFICES`

Σ joseph underclip (+ predict excess) reconstructs the latch−ctrl ΔP to 110% (rmse/|ΔP_end|=0.00%). The ×260 is vs a tiny onset baseline, not a nonlinear compound beyond the sum of per-tick deficits. Boring complete explanation: less Joseph cut, linear accumulate.

n=44 ticks in [1.1,1.54].

## Linear budget (CORR_ABS_SCALE)

| qty | value |
|-----|-------|
| ΔP start (L−C) | +1.038006e-05 |
| ΔP end (L−C) | +2.363431e-03 |
| ΔP growth observed | +2.353051e-03 |
| Σ joseph deficit (L−C) | +2.016715e-03 |
| Σ predict excess (L−C) | +5.832750e-04 |
| Σ linear total | +2.599990e-03 |
| frac linear explains growth | **1.105** |
| frac joseph of growth | 0.857 |
| recon RMSE | 0.000e+00 (rel 0.00%) |
| recon end err | +0.000e+00 |

## The ×260

- max\|ΔP\| onset [0.40,1.10] = 8.110e-06
- max\|ΔP\| late [1.10,1.54] = 2.363e-03
- ratio = **291.4×** — vs tiny onset baseline, not a per-tick compound factor.

## Deficit shape (feedback?)

- mean deficit early/late = +6.651e-06 / +8.502e-05 (late/early = 12.78)
- CV(deficit) = 1.34
- corr(\|def\|,\|P_l\|) = +0.388 (mean\|def\|=4.583e-05, mean\|P_l\|=4.729e-03)
- corr(\|def\|,\|ΔP\|) = +0.998 (mean\|ΔP\|=4.717e-04)

## Counterfactual: latch predict + ctrl joseph

- P_cf end = 2.909478e-03
- P_latch end = 4.679775e-03
- P_ctrl end = 2.316344e-03
- cf − latch = -1.770e-03 (negative ⇒ ctrl joseph would have pulled latch down)

Figure: `fig_joseph_clip_deficit_110_154.png`
