# OQ9 — H-DEG-ATT-LAG: degraded turns 9–10 vs clean 1/2/4

**No intervention.** Post-fix audits only (sha A=0043ca54c5b9166c, C=0ec5ee7237297061).  
**Figure:** `fig_slalom_oq9_degraded_regime.png`  

## State of the world (do not blur)

- Mechanism A×C (early feedback via `H_att` sign) is **understood**.
- Historical thresholds: still **FAIL** on the matrix cells — nothing is fixed.
- Intervention family **not decided** (gain-clamp / companion retune / other).
- **Do not design** a clamp from turn 1/2/4 until this degraded-regime read is in.

## Drift budget (|drift_A| accrual)

Final |drift_A| ≈ **53.37 m**; first ≥20 m @ t≈16.0 s.

| Epoch | Δ|drift_A| | frac of final | Δ(drift_A−C) |
|-------|------------|---------------|--------------|
| early_feedback_0_4s | 3.05 | 5.72% | 2.99 |
| clean_turns_4_14s | 7.34 | 13.75% | 7.19 |
| pre_degraded_14_18s | 9.40 | 17.61% | 9.34 |
| degraded_other_18_22s | 9.21 | 17.26% | 9.15 |
| tail_22_25s | 24.36 | 45.65% | 24.32 |

- early 0–4 s: **5.7%**
- mid (clean turns) 4–14 s: **13.8%**
- pre-degraded 14–18 s: **17.6%**
- turns 9–10 window 18–22 s: **17.3%**
- tail 22–25 s: **45.6%**
- **late total t≥14 s: 80.5%** (post-~20 m ≈ 62.9%)

Headline: the ω-locked early turns are **not** where most of the 54 m is earned.

## Per-turn NHC (stim ±1 s)

| Turn | role | drift_A | innov_A | innov A/C | |v_by|_A | k_att_A | P_aa_A | P_pv_A | P_aa_C | P_pv_C | opp dx_att_z |
|------|------|---------|---------|-----------|---------|--------|--------|--------|--------|--------|-------------|
| 1 | clean_delayed | 0.51 | 0.323 | 7.16e+03 | 0.217 | 0.0131 | 0.014 | 5.96 | 0.0277 | 7.46 | 0.21 |
| 2 | clean_delayed | 3.05 | 0.63 | 1.22e+04 | 0.543 | 0.0132 | 0.00867 | 3.05 | 0.0652 | 26.6 | 0.63 |
| 4 | clean_delayed | 0.14 | 1.75 | 6.75e+04 | 1.47 | 0.00685 | 0.00252 | 1.6 | 0.225 | 175 | 0.47 |
| 7 | last_clean_delayed | 10.39 | 1.02 | 1.02e+05 | 0.99 | 0.00181 | 0.00166 | 3.03 | 0.662 | 893 | 0.57 |
| 8 | alias_high_drift | 20.09 | 1.04 | 9.36e+04 | 0.775 | 0.00245 | 0.00179 | 4.24 | 0.86 | 1.33e+03 | 0.63 |
| 9 | degraded_other | 19.79 | 1.31 | 1.62e+05 | 1.07 | 0.00263 | 0.00194 | 6.17 | 1.08 | 1.88e+03 | 0.58 |
| 10 | degraded_other | 31.55 | 1.36 | 9.57e+04 | 1.09 | 0.00395 | 0.00164 | 6.05 | 1.33 | 2.56e+03 | 0.49 |

## Clean vs degraded contrast (means)

- innov_A rms: clean **0.9** vs degraded **1.33**
- |v_body_y|_A: clean **0.745** vs degraded **1.08**
- P_aa_A: clean 0.00839 vs degraded 0.00179
- P_pv_A: clean 3.54 vs degraded 6.11
- P_aa_C / P_pv_C: clean 0.106 / 69.7 vs degraded 1.21 / 2.22e+03
- k_att_A: clean 0.011 vs degraded 0.00329
- opp dx_att_z frac: clean 0.44 vs degraded 0.54

## Verdict (demoted to working hypothesis)

**FEEDBACK_CONTINUES_OMEGA_DECOUPLED** — *working hypothesis, not closed.*

Turns 9–10 still show loud A≫C innov/|v_body_y|, but per-turn xcorr vs |ω| broke. Most |drift_A| accrues at/after t≥14 s (~81%). Follow-up checks required before treating “same loop / not a new P mechanism” as closed:

→ **`slalom_oq9_late_p_and_burstiness.md`**: what “P_C explodes” means (`P_pp`/`P_pv` numbers); mirror test; burstiness of 22–25 s.

Do not preregister a late-regime success criterion on this label alone.

## Next (still no code)

- If `FEEDBACK_CONTINUES_OMEGA_DECOUPLED`: any future clamp must be stated in terms of innov/`v_body`/attitude-gain energy, not only ω-locked turn-1 windows; include a late-regime success criterion in the preregistration.
- If `SECOND_MECHANISM_P_REGIME`: preregister a P-facing arm separately (GAP-3/4 family), not as a footnote to attitude clamp.
- Either way: **preregister before implementing** (same discipline as §11).
