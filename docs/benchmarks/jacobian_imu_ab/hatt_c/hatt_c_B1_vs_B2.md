# B1 vs B2 — A(λ=1 latch) vs C post-latch

**Comparación:** trayectorias A−C (no absolutos). Ventana post-latch → 2 s.  
**Figuras:** `fig_hatt_c_B1_vs_B2_A_vs_C.png`, `fig_hatt_c_B1_vs_B2_bias_long.png`  
**JSON:** `hatt_c_B1_vs_B2.json`

## Resultados [latch, 2 s]

| Brazo | Δyaw A−C start→end | slope Δyaw | Δbias_gz start→end | slope Δbias_gz | Veredicto |
|-------|--------------------|------------|--------------------|----------------|-----------|
| c-E-l1 (0,39) | ≈0 → +0,012 rad | +0,0066 /s | ≈0 → **−0,027** | **−0,021 /s** | **B2** |
| c-L-l1 (0,58) | ≈0 → +0,013 rad | +0,0089 /s | ≈0 → **−0,029** | **−0,028 /s** | **B2** |
| ctrl A−C (ref 0,39–2) | ≈0 → +0,013 rad | +0,012 /s | ≈0 → +0,008 | +0,002 /s | mild (no freeze) |

Pre-latch (misma duración): Δyaw y Δbias_gz A−C ≈ 0 en todos.

## Lectura

- **No B1:** no hay “cicatriz” de yaw constante grande vs C post-latch (Δyaw ~1° y similar al control). Reset/re-alineación puntual de yaw no está justificada por estos datos.
- **Sí B2:** con `dx_att_z` congelado, **bias_gz de A se separa de C** con pendiente ~10× la del control en la misma ventana. Motor activo independiente del canal Z NHC.
- Implicación de diseño: no preregistrar reset de Σdx_att / yaw-scar. Converge con la línea **OQ9 / bias de giro** (temprano slalom + tardío como la misma causa en escalas distintas).
