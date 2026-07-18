# GAP-3.15 — F1 NHC dose-response

| Policy | N | accepts | P_vv pre#3 | k_vel mean | k_vel#2 | k_vel#3 | innov_h | Σ|ΔP| NHC | Γ | top3 share |
|--------|--:|--------:|-----------:|-----------:|--------:|--------:|--------:|----------:|--:|-----------:|
| baseline | 1 | 7 | 2.5 | 0.0359 | 0.1970 | 0.0078 | 27.2 | 62.7 | 19.7 | 74% |
| F1a | 2 | 7 | 4.4 | 0.0457 | 0.1984 | 0.0172 | 27.7 | 63.5 | 11.7 | 76% |
| F1b | 5 | 7 | 11.0 | 0.0673 | 0.1993 | 0.0471 | 29.3 | 61.4 | 6.4 | 81% |
| F1c | 10 | 7 | 22.0 | 0.0965 | 0.1996 | 0.0923 | 31.4 | 57.7 | 3.4 | 96% |
| F1d | 20 | 5 | 77.6 | 0.1096 | 0.1998 | 0.2426 | 32.2 | 6.5 | 0.3 | 100% |
| OFF | ∞ | 56 | 123.6 | 0.2064 | 0.2008 | 0.2565 | 7.7 | 0.0 | — | — |

## Criterio de parada (N=1 vs N=10)

**Verdict:** `FREQUENCY_MECHANISM_CONFIRMED_GATE_UNCHANGED`

N=10 confirma mecanismo (P_vv pre#3, k_vel, Γ) pero accepts siguen en 7 — la compresión P_vv ya no es el único gate; estudiar política NHC antes de GNSS_MAX_GAIN.

Señales:
- P_vv_pre_fix3_up: True
- k_vel_up: True
- accepts_up: False
- innov_h_down: False
- gamma_down: True
