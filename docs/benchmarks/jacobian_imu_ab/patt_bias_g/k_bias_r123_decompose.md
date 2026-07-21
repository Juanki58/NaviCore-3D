# K_bias_gz path decompose — R1 / R2 / R3

**Verdict:** `NEAR_CANCEL_PATHS_LATCH_R2_SCALE_EXPLOSION`

H_vel vs H_att path split does NOT reveal a motor that switches att→vel across R1/R2/R3. In every tramo both paths are large, opposite, and cancel (cancel_ratio ≪ 1; |frac_att|≈0.50). The actionable latch effect is a path-SCALE explosion in R2 (latch path_scale≈6.3 vs ctrl≈0.20) while net Σdx_bias_gz stays small (−0.039). Net sign still changes by tramo under latch (R1/R2 negative, R3 positive). Dominant-path labels at frac≈0.5 are not meaningful — report cancel structure + path_scale + net residual. Sub-tramos: R3 halves flip path signs under latch (explosion front vs tail); R1 magnitude grows toward break; R2 coherent.

## Definition

- Split: `K_via_X = P H_X^T S^{-1}` (frozen S), `dx = K_via_X · y`, X ∈ {vel, att}
- Identity: `dx_bias_gz = via_vel + via_att` (rms resid ~1e-9)
- **cancel_ratio** = |Σnet| / (|Σvia_vel|+|Σvia_att|) — small ⇒ near-cancellation

## Primary tramos (signed Σ)

| Arm | Tramo | n | Σ total | Σ via_vel | Σ via_att | path_scale | cancel_ratio | mean‖y‖ |
|-----|-------|---|---------|-----------|-----------|------------|--------------|---------|
| ctrl | R1 [1.34,1.59) | 25 | -0.01411 | +1.14836 | -1.16247 | 2.311 | 0.006 | 0.263 |
| ctrl | R2 [1.59,1.74) | 15 | +0.02104 | -0.09019 | +0.11123 | 0.201 | 0.104 | 0.529 |
| ctrl | R3 [1.74,2.0] | 27 | +0.00249 | -0.25815 | +0.26065 | 0.519 | 0.005 | 0.122 |
| latch | R1 [1.34,1.59) | 25 | -0.01235 | +0.34687 | -0.35922 | 0.706 | 0.017 | 0.135 |
| latch | R2 [1.59,1.74) | 15 | -0.03912 | +3.13343 | -3.17255 | 6.306 | 0.006 | 1.471 |
| latch | R3 [1.74,2.0] | 27 | +0.02557 | -0.25001 | +0.27558 | 0.526 | 0.049 | 0.589 |

## Latch − ctrl

| Tramo | ΔΣ total | ΔΣ via_vel | ΔΣ via_att | Δ path_scale |
|-------|----------|------------|------------|--------------|
| R1 | +0.00176 | -0.80149 | +0.80325 | -1.605 |
| R2 | -0.06016 | +3.22362 | -3.28378 | +6.105 |
| R3 | +0.02307 | +0.00814 | +0.01493 | +0.007 |

## Sub-tramos (robustness — do not average over flips)

| Arm | Sub | n | Σ total | Σ via_vel | Σ via_att | path_scale | cancel_ratio | mean‖y‖ |
|-----|-----|---|---------|-----------|-----------|------------|--------------|---------|
| ctrl | R1a [1.34,1.465) | 13 | -0.00311 | +0.15663 | -0.15974 | 0.316 | 0.010 | 0.090 |
| ctrl | R1b [1.465,1.59) | 12 | -0.01100 | +0.99173 | -1.00273 | 1.994 | 0.006 | 0.449 |
| ctrl | R2a [1.59,1.665) | 8 | +0.02304 | +0.18440 | -0.16136 | 0.346 | 0.067 | 0.711 |
| ctrl | R2b [1.665,1.74) | 7 | -0.00200 | -0.27459 | +0.27260 | 0.547 | 0.004 | 0.321 |
| ctrl | R3a [1.74,1.87) | 13 | -0.00024 | -0.09484 | +0.09460 | 0.189 | 0.001 | 0.079 |
| ctrl | R3b [1.87,2.0] | 14 | +0.00273 | -0.16332 | +0.16605 | 0.329 | 0.008 | 0.161 |
| latch | R1a [1.34,1.465) | 13 | -0.00221 | +0.06452 | -0.06673 | 0.131 | 0.017 | 0.053 |
| latch | R1b [1.465,1.59) | 12 | -0.01015 | +0.28235 | -0.29250 | 0.575 | 0.018 | 0.224 |
| latch | R2a [1.59,1.665) | 8 | -0.03046 | +0.94416 | -0.97462 | 1.919 | 0.016 | 1.009 |
| latch | R2b [1.665,1.74) | 7 | -0.00866 | +2.18926 | -2.19793 | 4.387 | 0.002 | 1.998 |
| latch | R3a [1.74,1.87) | 13 | +0.02893 | -0.27874 | +0.30768 | 0.586 | 0.049 | 1.108 |
| latch | R3b [1.87,2.0] | 14 | -0.00337 | +0.02873 | -0.03210 | 0.061 | 0.055 | 0.107 |

## Implications

- Do not design intervention as cut via_att or cut via_vel alone — they nearly cancel; cutting one may unmask the other.
- Latch does not change which path dominates (both always ~equal); it amplifies both in R2.
- Net bias escape timing (silent R1, cost R2/R3) remains the design clock; path split explains coupling geometry, not a new switch.

Figure: `fig_k_bias_r123_paths.png`
