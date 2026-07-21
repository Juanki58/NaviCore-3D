# SLALOM A vs C — ω coincidence & drift-delta burstiness

**Scenario:** SLALOM seed 71 · A (`jcorrect`+`imuideal`) vs C (`jlegacy`+`imuideal`)  
**Artifacts:** `slalom_a_vs_c_omega_burstiness.json`, `fig_slalom_omega_vs_ddrift.png`  
**Scope:** coincidence / burstiness metrics only — no causal claim.

## CSV yaw_rate

- A all-zero: **True**
- C all-zero: **True**
- **Dead:** True → reconstructed `ω_truth` from `slalom_scenario.cpp`:
  - `kSlalomOmegaRadps = 2π / 4.0 = 1.57079633`
  - `kYawAmplitudeRad = 3.0 / (v·ω) = 0.13750987`
  - `yaw_rate = A·ω·cos(ω·t)`
- Secondary: finite-difference of filter `yaw` for A and C.

## Burstiness thresholds

Rule: **bursty** if `top3_share > 0.5` OR `B > 0.25`  
(same spirit as GAP-3: `B = max|Δ| / Σ|Δ|` on per-tick deltas).

### Window 1.3–2.0 s (primary)

| Series | Metrics |
|--------|---------|
| `x = drift_A - drift_C` (signed) | B=0.0862, top3_share=0.2263, bursty=False, max|dx|=0.133496, sum|dx|=1.54791 |
| `x = |drift_A|` | B=0.0858, top3_share=0.2250, bursty=False, max|dx|=0.133323, sum|dx|=1.55459 |

Dominating ticks (`drift_A - drift_C`):
  - rank 1: t∈[1.610,1.620] s, dx=-0.133496, share=0.086
  - rank 2: t∈[1.600,1.610] s, dx=-0.122388, share=0.079
  - rank 3: t∈[1.620,1.630] s, dx=-0.094478, share=0.061

**Verdict 1.3–2.0 s (signed Δdrift):** NOT bursty

### Context 1.0–5.0 s

| Series | Metrics |
|--------|---------|
| `drift_A - drift_C` | B=0.0413, top3_share=0.0986, bursty=False, max|dx|=1.10596, sum|dx|=26.7904 |

### Whole-run comparison (avoid overclaim from short window)

| Series | Metrics |
|--------|---------|
| `drift_A - drift_C` (whole run) | B=0.0036, top3_share=0.0087, bursty=False, max|dx|=1.10596, sum|dx|=310.967 |

Whole-run B for |d(Δdrift)| path (per-tick |dx| of signed delta): **0.0036** vs primary-window B **0.0862**.

## Coincidence ω vs divergence

| Question | Result |
|----------|--------|
| At max `|d(Δdrift)/dt|` in 1.3–2.0 s | t = **1.6100** s; `|ω_truth|` = **0.1767** rad/s; `|Δdrift|` = 0.215619 m |
| First argmax `|ω_truth|` in 0–5 s | t = **0.0000** s (tied also at ~2 s, ~4 s); `|ω|` = **0.2160** rad/s; `|Δdrift|` = 0 m |
| At first interior `|ω|` peak | t = **2.0000** s; `|ω|` = **0.2160** rad/s; `|Δdrift|` = 0.483977 m; `|d(Δdrift)/dt|` = 1.57085 m/s |
| Lag in 1–5 s (`t_div - t_ω`) | **+1.3800** s (argmax `|ω|` @ 2.0000 s vs argmax `|dΔdrift/dt|` @ 3.3800 s) |

Figure: `fig_slalom_omega_vs_ddrift.png` (0–5 s, band 1.3–2.0 s).

## Short factual summary

- CSV `yaw_rate`: **DEAD (all zeros)**.
- Burstiness in 1.3–2.0 s on signed `(drift_A - drift_C)`: **not bursty** (B=0.0862, top3_share=0.2263); whole-run B=0.0036.
- ω peak vs divergence-rate peak (1–5 s lag): **+1.3800 s** — coincidence metrics only.
