# §13 Caracterización P_att–bias_g — veredicto

**CHAR_WEAK** (`innov_amplified_K_bias`). No desbloquear zero-P.

## Hallazgo

Bajo latch λ=1, el escape a `bias_gz` es real (slope −0,021 vs +0,002 en ctrl; Σ dx_bias_gz ×3 y signo opuesto). Pero:

1. `P[ATT_Z,BIAS_GZ]` y ‖P_att,bias_g‖_F son **indistinguibles** ctrl vs latch.
2. `k_bias_gz` ≈ igual; **innov ×2**.
3. corr(P, dx_bias) solo **0,13**.

Interpretación: al congelar `dx_att_z`, la innovación NHC crece y el mismo `K_bias` inyecta más corrección en bias — fuga por **amplificación de innov**, no por reconfiguración del bloque P cruzado.

## Siguiente (no intervención aún)

Descomponer `K_bias_gz = f(P_bias,vel, P_bias,att, H, S⁻¹)` en contribuciones att vs vel en ctrl vs latch.
