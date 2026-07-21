# f_va multiplicative surge? |a| × attitude error [1.34→1.69]

**Verdict:** `ATTITUDE_JUMP_DRIVEN_SURGE`

Attitude error jumps in the surge window while |a| is already high/smooth — not primarily an accel spike; look at latch/composition-break attitude dynamics.

Truth |a_horiz|=3|cos(ωt)| rises smoothly toward peak at t=2.0; expect no cliff in |a| at 1.59 — if surge is sharp, attitude-side or update-side discontinuity is the sharper factor.

## Latch phase comparison

| Phase | mean |a|_h | mean ||δθ||° | mean prod | Δ|cross| | pearson d|c|/dt vs prod |
|-------|------------|---------------|-----------|----------|--------------------------|
| rise [1.34,1.59] | 2.006 | 0.45 | 0.0172 | +0.255 | 0.994 |
| surge [1.59,1.69] | 2.543 | 3.37 | 0.1511 | +1.326 | 0.837 |

## Ratios surge/rise

- |a|_h: **1.27×**
- ||δθ||: **7.47×**
- prod |a|·||δθ||: **8.79×**
- Δ|cross|: **5.20×**

## Design hint

If multiplicative: consider gating/damping f_va when |a| high AND attitude error energy already elevated — conditional, not blind Z-forget. If attitude-jump driven: stay on attitude-loop early intervention. If not multiplicative: re-examine NHC feedback into vel during break.

Figure: `fig_fva_multiplicative_surge.png`
