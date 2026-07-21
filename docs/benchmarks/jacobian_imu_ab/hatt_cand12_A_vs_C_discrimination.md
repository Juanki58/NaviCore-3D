# H-ATT next family — cand1 vs cand2 discrimination (pre-preregistro)

**Datos:** audits post-fix SLALOM A×C seed 71 (sin corrida nueva).  
**Figura:** `fig_hatt_cand12_A_vs_C_0_2s.png`  
**JSON:** `hatt_cand12_A_vs_C_discrimination.json`

## Candidato 1 — forma de Σ|dx_att_z| en [0, 0.75] s

| Serie | rate_ratio 2ª/1ª mitad | sep_ratio | innov_ratio | R² lin→quad | Verdict |
|-------|------------------------|-----------|-------------|-------------|---------|
| A Σ\|dx\| | **5,43** | 264 | 958 | 0,87→0,996 | **FEEDBACK_GROWTH** |
| C Σ\|dx\| | **0,51** | 33 | 5 | 0,95→0,97 | **OTHER** (decelera) |

C no es “racha larga = mismo mecanismo”: el acumulado **decelera** (rate_ratio &lt; 1) y se queda ~2e−6 rad a 0,75 s; A acelera y llega ~2e−5.  
Separación |ΣA|/|ΣC|: ×2 @ **0,39 s**, ×5 @ **0,58 s**, ×10 @ 0,86 s.

## Candidato 2 — |dx_att_z|/innov_norm por tick

En la ventana de detección propuesta **0–0,4 s**: median A/C ≈ **1,05**; p10(A) &lt; p90(C) → **solapamiento total**.  
No discrimina el régimen temprano (coherente con k_att A≈C: misma ganancia×innov ≈ mismo |dx| tick a tick; diverge el *acumulado* vía feedback).

## Implicación para preregistro

- **Primaria:** detector sobre forma/crecimiento del acumulado (cand1), no sobre |dx|/innov.
- **Cand2:** descartado como discriminante temprano (o solo brazo negativo si se quiere documentar).
- Ventana &lt; 0,65 s: a **0,4 s** la razón acumulada es ~2× (margen); umbral robusto tipo ×5 aparece ~**0,58 s** — aún bajo el bound de racha opuesta.
- P3 debe incluir: **no disparo en C** durante su racha larga de signo.
