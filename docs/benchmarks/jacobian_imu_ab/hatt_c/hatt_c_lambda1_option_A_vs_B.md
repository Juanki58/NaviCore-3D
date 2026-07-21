# H-ATT-c λ=1 — Option A vs B (pre-reset)

**Datos:** corridas §12.6 (`c-E-l1`, `c-L-l1`) + control.  
**Figuras:** `fig_hatt_c_lambda1_option_A_vs_B.png`, `…_zoom.png`  
**JSON:** `hatt_c_lambda1_option_A_vs_B.json`

## Comprobación

Tras latch con λ=1, `dx_att_z` **aplicado** queda en cero → Σdx applied **plano** (ΔΣ = 0 hasta t=2 s y hasta el final). Construcción del freeze verificada.

| Brazo | Latch | \|drift\| @ latch | \|drift\| final | Share post-latch | Slope latch→2 s |
|-------|-------|-------------------|-----------------|------------------|-----------------|
| c-E-l1 | 0,39 s | **0,0049 m** | 95,8 m | **99,995 %** | 0,64 m/s |
| c-L-l1 | 0,58 s | **0,0073 m** | 65,4 m | **99,989 %** | 0,72 m/s |

## Veredicto

**No es la Opción A ingenua** (“los metros finales ya están hechos a 0,39/0,58 s”). Casi todo el drift E2E nace **después** del freeze de flujo NHC-Z.

Eso **sí** confirma la intuición de Opción B a nivel de *flujo NHC*: cortar `dx_att_z` no para el motor del daño. Antes de preregistrar reset de Σdx_NHC hace falta separar dos subcasos:

| Subcaso | Mecanismo | Implica para reset Σdx / actitud |
|---------|-----------|----------------------------------|
| **B1 — actitud ya torcida** | Pre-latch dejó attitude error en el estado; sin más correcciones NHC-Z, esa actitud mala sigue integrando v→pos | Reset/corrección de **estado de actitud** (o undo del Σ aplicado al nominal) al latch **puede** ayudar |
| **B2 — vía independiente** | bias_gz / otro canal arrastra tras el latch aunque actitud estuviera bien | Reset de Σdx_att NHC **no** basta; hay que atacar esa vía |

El FEEDBACK_GROWTH de P3-A con λ=1 en 0–0,75 s es en parte artefacto de la métrica A×C sobre Σ (C sigue moviendo dx; A no) + crecimiento pre-latch en la 1ª mitad — no prueba por sí sola que el *flujo* NHC-Z siga activo en A.

## Siguiente paso barato (antes de preregistro reset)

Comparar post-latch: `yaw−des_heading` y `bias_gz` en c-E-l1 vs control — si el error de yaw a latch ya es grande y sigue, favorece B1; si yaw está bien y bias_gz/otro diverge, favorece B2.
