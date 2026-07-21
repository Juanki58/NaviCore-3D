# Innov ratio continuity along rising limb

**Verdict:** REGIME_CHANGE_AT_BREAK_NOT_CONTINUOUS_FROM_ONSET

Innov ratio does NOT climb continuously from onset: onset→break stays flat at ~0.55 (latch smaller than ctrl). The aggregate post-break mean ratio≈6 is post-break-dominated (~92% of Σ‖y‖_latch after 1.59) and mean-inflated by a later spike (~1.69–1.79, median ratio~24). At the break edge itself: cos flips +0.91→−0.93 (real composition event) while ratio only crosses ~0.48→1.11 — a regime change from “latch helps innov” to “latch hurts”, not a statistical threshold on a pre-existing ratio trend. Still: do not treat 1.59 as sole intervention trigger — signed bias escape is already ~48% complete in the silent onset→break window; and peak innov damage is ~1.7–1.8, not at break. K_bias window = full rise [1.34,2.0].

## Strict split: onset→break vs break→peak

| Window | n | median ratio | mean ratio | Σ‖y‖_L/Σ‖y‖_C | median cos | Σdx_bias_gz latch | Σ‖y‖_L share |
|--------|---|--------------|------------|---------------|------------|-------------------|--------------|
| onset→break [1.34,1.59) | 25 | 0.55 | 0.55 | 0.51 | +0.97 | -0.0124 | 8.2% |
| break→peak [1.59,2.0] | 42 | 2.44 | **5.98** | 3.38 | **-0.82** | -0.0136 | **91.8%** |
| full rise [1.34,2.0] | 67 | 0.69 | 3.96 | 2.32 | +0.39 | -0.0259 | 100% |

## Edge at break (±0.10 s)

- median ratio: **0.48 → 1.11** (jump +1.09; crosses unity, does not jump to ~6)
- median cos: **+0.91 → −0.93** (composition flip is the discrete event at 1.59)
- pre half-growth of median ratio: **−0.12** (flat / slightly latch-favoring — not climbing)
- max 50ms median-ratio step: **~24 @ t≈1.79** (well after break), not at 1.59

## Two clocks inside the rise

| Quantity | When it moves |
|----------|---------------|
| Signed Σdx_bias_gz latch | ~48% already in onset→break (silent) |
| Σ‖y‖_latch / ratio explosion | ~92% after break; peak bins 1.69–1.79 |
| cos composition flip | at break edge 1.59 |

## Design

- K_bias decompose window: **full rise [1.34 → 2.0]** (captures silent bias + later innov)
- Inside it, report onset→break vs break→peak separately
- Do **not** trigger only on 1.59

Figure: ig_innov_rise_continuity.png
