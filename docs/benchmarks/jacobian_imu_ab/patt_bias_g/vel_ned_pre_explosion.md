# vel_NED dirty before NHC explosion? [0.4 → 1.69]s

**Verdict:** `VEL_NED_DIRTY_BEFORE_NHC_EXPLOSION` (phase-aware)

Filter vel_NED cross-track is already **−1.64 m/s at t=1.69** (same magnitude NHC later sees as v_lat) — the turn *reveals* pre-existing contamination.

**Phase nuance (do not claim uniform drip from t=0.4):**
- Early [0.40→1.34): essentially clean (cross ~0.03→0)
- Rise [1.34→1.59): starts dirtying (→ −0.26)
- Break→explode [1.59→1.69]: **surge** (−0.31→−1.64 in ~100 ms)

Intervention: rise / attitude→vel (`f_va`) path — not NHC @ 1.7s.

## Full window summary

| Arm | cross start→end | Δcross | max|cross| | |e_h| start→end | slope |e_h| |
|-----|-----------------|--------|------------|-----------------|-------------|
| ctrl | +0.027→+0.247 | +0.220 | 0.465 | 0.027→0.902 | +0.288 |
| latch | +0.027→-1.640 | -1.667 | 1.640 | 0.027→1.671 | +0.446 |

## Latch phases

| Phase | cross start→end | slope |cross| | max|cross| | |e_h| end |
|-------|-----------------|---------------|------------|----------|
| P_early_loop | +0.027→-0.003 | -0.0275 | 0.027 | 0.018 |
| P_rise_pre_break | -0.004→-0.259 | +0.8552 | 0.259 | 0.468 |
| P_break_to_explode | -0.314→-1.640 | +15.5385 | 1.640 | 1.671 |

## Implication

If VEL_NED_DIRTY_BEFORE_NHC_EXPLOSION: full chain is NHC Jacobian sign → attitude loop → f_va pollutes vel_NED silently → heading turn reveals as v_lat → NHC innov explodes. Intervene early on attitude/f_va, not at NHC explosion.

Figure: `fig_vel_ned_pre_explosion.png`
