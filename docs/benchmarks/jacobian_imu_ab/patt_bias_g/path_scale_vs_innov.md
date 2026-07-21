# path_scale vs ‖y‖ — same explosion or separate coupling?

**Verdict:** `PATH_SCALE_TRACKS_INNOV`

Per-tick path_scale follows ‖y‖ closely in R2 under latch; path_per_innov stays ~stable vs ctrl. The R2 path_scale explosion is the same innov-magnitude explosion propagating through both H paths — not a separate K-coupling mechanism to attack. Intervention focus stays on why innov explodes at ~1.69–1.79s, not on cutting K_bias paths.

## R2 latch vs ctrl (key)

- pearson(path_scale, ‖y‖) latch: **0.951**
- mean‖y‖ L/C: **2.78×**
- mean path_scale L/C: **3.24×**
- mean (path_scale/‖y‖) L/C: **1.16×**
- frac var(path) explained by ‖y‖ (latch R2): **0.89**

## Correlations by window

| Arm | Window | n | pearson | spearman | mean ppi | frac var expl. |
|-----|--------|---|---------|----------|----------|----------------|
| ctrl | R1 | 25 | 0.995 | 1.000 | 0.306 | 0.98 |
| ctrl | R2 | 15 | 0.701 | 0.693 | 0.235 | 0.49 |
| ctrl | R3 | 27 | 0.994 | 0.996 | 0.167 | 0.88 |
| ctrl | full_rise | 67 | 0.883 | 0.927 | 0.234 | 0.77 |
| latch | R1 | 25 | 1.000 | 1.000 | 0.199 | 1.00 |
| latch | R2 | 15 | 0.951 | 0.939 | 0.272 | 0.89 |
| latch | R3 | 27 | 0.765 | 0.889 | 0.074 | 0.59 |
| latch | full_rise | 67 | 0.843 | 0.856 | 0.165 | 0.71 |

## Latch/ctrl ratios

| Window | ‖y‖ L/C | path_scale L/C | ppi L/C |
|--------|---------|----------------|---------|
| R1 | 0.51 | 0.31 | 0.65 |
| R2 | 2.78 | 3.24 | 1.16 |
| R3 | 4.83 | 3.06 | 0.44 |
| full_rise | 2.32 | 1.80 | 0.70 |

## R3 second split (homogeneous via_att sign? **False**)

| Arm | Sub | n | Σ total | Σ via_vel | Σ via_att | mean‖y‖ | mean path_scale |
|-----|-----|---|---------|-----------|-----------|---------|-----------------|
| ctrl | R3a1 | 7 | -0.00025 | -0.05590 | +0.05566 | 0.087 | 0.0159 |
| ctrl | R3a2 | 6 | +0.00001 | -0.03893 | +0.03894 | 0.071 | 0.0130 |
| ctrl | R3b1 | 7 | +0.00069 | -0.06117 | +0.06186 | 0.107 | 0.0176 |
| ctrl | R3b2 | 7 | +0.00204 | -0.10215 | +0.10419 | 0.215 | 0.0295 |
| latch | R3a1 | 7 | +0.04335 | -0.15845 | +0.20180 | 1.752 | 0.1834 |
| latch | R3a2 | 6 | -0.01442 | -0.12030 | +0.10588 | 0.356 | 0.0394 |
| latch | R3b1 | 7 | -0.00217 | +0.02700 | -0.02917 | 0.145 | 0.0080 |
| latch | R3b2 | 7 | -0.00120 | +0.00173 | -0.00293 | 0.068 | 0.0019 |

## Design

If PATH_SCALE_TRACKS_INNOV: do not attack K_bias/path_scale; return to innov explosion mechanism at 1.69–1.79. If DIVERGES: preregister gain/coupling lever separately.

Figure: `fig_path_scale_vs_innov.png`
