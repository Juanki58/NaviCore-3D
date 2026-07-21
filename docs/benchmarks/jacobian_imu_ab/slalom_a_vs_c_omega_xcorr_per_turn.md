# SLALOM A vs C — per-turn xcorr ‖ω‖ × |d(Δdrift)/dt|

Discriminates **(a)** delayed coupling (+ period alias) vs **(b)** early-window chance / regime-dependent. Alias labels require **shifted-stim confirmation**, not τ≈0 compatibility alone.

**Method:** native stim `[t_c±1.0]`; alias test stim `[(t_c−2)±1.0]`.  
**Figure:** `fig_slalom_omega_xcorr_per_turn.png`  

## Per-turn peaks (native stim)

| Turn | t_c | τ_peak | r_peak | Δr | mode | drift_A | std|ω| | std|dΔ/dt| | cov_peak |
|------|-----|--------|--------|-----|------|---------|--------|-----------|----------|
| 1 | 2.0 | 2.020 | 0.2790 | 0.4571 | delayed | 0.51 | 0.0670 | 15.9289 | 0.29789 |
| 2 | 4.0 | 1.980 | 0.6038 | 0.3415 | delayed | 3.05 | 0.0670 | 17.4802 | 0.70731 |
| 3 | 6.0 | 0.000 | 0.6027 | 0.0000 | alias0(confirmed) | 3.42 | 0.0670 | 17.4854 | 0.70630 |
| 4 | 8.0 | 2.080 | 0.8126 | 0.3037 | delayed | 0.14 | 0.0670 | 6.2434 | 0.34000 |
| 5 | 10.0 | 0.080 | 0.8126 | 0.0201 | alias0(confirmed) | 8.35 | 0.0670 | 6.2434 | 0.34000 |
| 6 | 12.0 | 0.000 | 0.6322 | 0.0000 | alias0(unconfirmed) | 8.11 | 0.0670 | 9.1621 | 0.38821 |
| 7 | 14.0 | 1.740 | 0.8260 | 0.3248 | delayed | 10.39 | 0.0670 | 6.9786 | 0.38631 |
| 8 | 16.0 | 0.000 | 0.5328 | 0.0000 | alias0(confirmed)/degraded | 20.09 | 0.0670 | 6.8604 | 0.24497 |
| 9 | 18.0 | 0.910 | 0.4624 | 0.6593 | other/degraded | 19.79 | 0.0670 | 11.0402 | 0.34211 |
| 10 | 20.0 | 2.530 | 0.5077 | 0.3940 | other/degraded | 31.55 | 0.0670 | 14.0253 | 0.47724 |

## Alias-shift confirmation (stim → previous turn)

Criterion: stim=[(t_c−2)±1]; alias confirmed if τ_peak near ~1.9–2.0 s and r_peak≥0.2

Confirmed: **3/4** (frac=0.75)

| Follower | stim@ | τ_peak | r_peak | r(0) | Δr | near~1.9 | confirmed |
|----------|-------|--------|--------|------|-----|----------|-----------|
| T3@6s | 4.0 | 1.980 | 0.6038 | 0.2622 | 0.3415 | True | **True** |
| T5@10s | 8.0 | 2.080 | 0.8126 | 0.5089 | 0.3037 | True | **True** |
| T6@12s | 10.0 | 0.080 | 0.8126 | 0.7925 | 0.0201 | False | **False** |
| T8@16s | 14.0 | 1.740 | 0.8260 | 0.5012 | 0.3248 | True | **True** |

## Clean vs degraded regime

Degraded threshold: drift_A ≥ **15.0 m** at t_c. Turns: **[8, 9, 10]** — scored **separately**; not folded into clean-regime explainable fraction.

- **Turns 9–10** (`other/degraded`, drift_A ≈ 20–32 m): pattern delayed/alias **breaks** here — do not count them inside any "% explainable" headline. Same family of "simple mechanism stops applying once state is badly wrong" seen in NHC/GNSS/ZUPT this session; **candidate for separate review**, not explained now.
- Turn 8 is over the drift threshold but still `alias0(confirmed)` by shift test — high-drift ≠ automatic pattern break; 9–10 are the clear break.

- clean n=7: delayed=0.57, alias0=0.43, alias_confirmed=0.29, explainable(a)=0.86
- degraded n=3

## r_peak vs signal magnitude

Pearson r is already variance-normalized within each window; amplitude growth alone cannot inflate r. Rising r across turns implies better shape alignment or higher SNR vs residual, not raw |dΔ/dt| scale. See per-turn std_* and cov_peak_unnormalized.

- r_peak turn1=0.2790, later median=0.6038, drop=-0.3247
- corr(r_peak, std|dΔ/dt| across turns) = **-0.612** (positive would suggest r tracks response energy; negative/near-zero ⇒ the turn1→later r rise is not an amplitude artifact)
- std|ω| is essentially constant across turns (ideal slalom); see cov_peak_unnormalized for raw scale.

## Alias0 unconfirmed (open)

Shifted-stim did **not** recover τ≈1.9–2.0 s for:

- T6 @ 12s: shift stim@10.0 → τ_peak=0.0799999999999983, r_peak=0.812586548086418 — leave open (not proven alias; not forced into (a))


## Verdict (a vs b)

**A_CONFIRMED_WITH_PERIOD_ALIAS**

delayed lag repeats on multiple clean-regime turns; shifted-stim test confirms alias0 turns are misaligned responses to the previous turn (τ≈1.9–2.0 s when stim→prev). Long-window Δr dilution is superposition. Degraded-regime turns (drift_A large) scored separately — not folded into the clean explainable fraction.

**Next:** anchor K/P to τ≈1.9–2.0 s on turn 1 (t_c≈2.0 → effect ≈4.0 s); control on turns 2 and 4 (`delayed`). Do not anchor K/P on alias0 followers (mixed stimuli).
