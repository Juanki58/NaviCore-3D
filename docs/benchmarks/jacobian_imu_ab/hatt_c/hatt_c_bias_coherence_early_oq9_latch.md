# Coherencia bias: temprano↔OQ9 + ¿depende del latch?

**Figura:** `fig_hatt_c_bias_latch_vs_oq9.png`  
**JSON:** `hatt_c_bias_coherence_early_oq9_latch.json`

## Check 2 — ¿motor preexistente o fuga inducida por el latch?

Pendiente de `bias_gz` **absoluta en A** (no solo A−C):

| Serie | slope [0,39→2] s | Δ bias |
|-------|------------------|--------|
| **ctrl A** (sin latch) | **+0,0023 /s** | +0,0084 |
| **E_l1** (latch 0,39) | **−0,0208 /s** | −0,0268 |
| **L_l1** (latch 0,58) | **−0,0280 /s** | −0,0292 |

E/ctrl ≈ **−9×** y **signo opuesto**. La pendiente fuerte negativa **solo aparece con el latch activo**.

**Veredicto check 2: fuga de canal inducida por la intervención** (cerrar `dx_att_z` empuja el error a `bias_gz` vía acoplamiento P), **no** un motor de bias preexistente paralelo al NHC-Z.

El hallazgo B2 de §12.8 (Δbias A−C ~−0,027) se reinterpreta: era A−C bajo latch, no evidencia de bias autónomo en el régimen natural.

## Check 1 — signo temprano vs OQ9 (t≥14 s)

| Serie | early Δ [0,39→2] | mid Δ [2→14] | late Δ [14→25] | mismo signo early↔late | mid continuo |
|-------|------------------|--------------|----------------|------------------------|--------------|
| **ctrl A** (régimen OQ9 natural) | **+0,008** | −0,021 | **+0,008** | sí (+) | **no** (flip en mid) |
| E_l1 | −0,027 | +0,043 | −0,048 | sí (−) | **no** |
| L_l1 | −0,029 | +0,139 | −0,005 | sí (−) | **no** |

En control (donde se estudió OQ9) no hay una deriva negativa sostenida e ininterrumpida early→late. Hay flips de signo / mid inconsistente. La firma negativa fuerte es la del brazo latcheado, no la del baseline OQ9.

**Veredicto check 1:** **no** soporta “una sola línea causal ininterrumpida early+OQ9 vía el mismo bias”. Comparten el *síntoma posible* (bias en el estado) sin la misma trayectoria de signo/continuidad.

## Implicación de diseño

1. **No fusionar** aún temprano-slalom y OQ9 como un solo mecanismo de bias.
2. El B2 post-latch es en gran parte **artefacto del latch** → no preregistrar ataque a `bias_gz` como motor preexistente del early loop.
3. Siguiente diseño (si se sigue con detector H-ATT-c): o bien **no cerrar Z sin vía de escape** (p.ej. no latch λ=1 puro), o atacar **acoplamiento P_att–bias_g** / reabrir canal de forma controlada — no “matar bias” como si siempre hubiera estado arrastrando.
4. OQ9 sigue su propio hilo (dP_pp/dt vs bias en régimen **sin** latch), sin heredar el resultado de hoy como prueba de identidad.
