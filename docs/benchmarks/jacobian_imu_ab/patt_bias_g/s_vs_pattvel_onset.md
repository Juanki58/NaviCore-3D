# S vs P_att–vel — onset [0.40→1.10] (wide to 1.54)

**Verdict:** `S_STABLE__PATT_VEL_LATE_CARRIER`

**S (candidato 2):** fuera — diagonales/cond idénticas; max|Δs_yz|=9e-6 ≪ mean|s_yy| (relΔ~28% cerca de cero = ruido; misma trampa que P_yy).

**P_att–vel (candidato 1):** estable en onset silencioso [0.40→1.10] (donde ΔK_y0 aún es ~0), pero **porta en [1.10→1.54]** cuando ΔK_y0 acelera:
- `P[ATT_Y,VN]` max|Δ| **2.7e-3**, max relΔ **0.59** (~260× onset); corr(|ΔK_y0|,|Δ|)≈0.98
- S sigue estable en ventana ancha
- Co-tiempo con la fase late de FEEDBACK_GROWTH de K_y0 — no es el onset temprano, es el mismo acelerón post-1.10

**Next:** Congelar componente dominante (`P[ATT_Y,VN]` vs filas ATT_Z–vel) y ligar a H_NHC; diseño = cortar Z deja derivar el bloque P_av que alimenta K_pitch.

## S — onset summary

| qty | mean relΔ | max relΔ | end ctrl | end latch |
|-----|-----------|----------|----------|-----------|
| s_cond | 0.0000 | 0.0000 | +3.9156e+00 | +3.9156e+00 |
| s_yy | 0.0000 | 0.0000 | +2.6532e-01 | +2.6532e-01 |
| s_yz | 0.0094 | 0.2848 | +9.9497e-04 | +9.8579e-04 |
| s_zz | 0.0000 | 0.0000 | +1.0389e+00 | +1.0389e+00 |
| s_inv_yy | 0.0000 | 0.0000 | +3.7691e+00 | +3.7691e+00 |
| s_inv_yz | 0.0094 | 0.2848 | -3.6098e-03 | -3.5765e-03 |
| s_inv_zz | 0.0000 | 0.0000 | +9.6258e-01 | +9.6260e-01 |

### corrs |ΔK_y0| vs |ΔS| (onset)

- corr_|dky0|_vs_|d_s_cond| = **+0.785**
- corr_|dky0|_vs_|d_s_yz| = **+0.962**
- corr_|dky0|_vs_|d_s_inv_yz| = **+0.962**
- corr_|dky0|_vs_|d_s_yy| = **-0.173**
- corr_|dky0|_vs_|d_s_zz| = **+0.988**

## P_att–vel — onset summary

| qty | mean relΔ | max relΔ | ptp(Δ) | corr|dky0| |
|-----|-----------|----------|--------|-----------|
| P_pre_att_y_vn | 0.0007 | 0.0025 | +1.506e-05 | +0.557 |
| P_pre_att_y_ve | 0.0001 | 0.0002 | +8.244e-06 | +0.920 |
| P_pre_att_y_vd | 0.0000 | 0.0000 | +2.623e-06 | +0.267 |
| P_pre_att_z_vn | 0.0000 | 0.0000 | +4.746e-06 | +0.984 |
| P_pre_att_z_ve | 0.0005 | 0.0105 | +4.761e-06 | +0.945 |
| P_pre_att_z_vd | 0.0002 | 0.0007 | +4.790e-06 | +0.879 |
| P_pre_vel_att_frob | 0.0000 | 0.0001 | +1.314e-05 | +0.978 |

Figure S: `fig_s_onset_ky0.png`

Figure P: `fig_pattvel_onset_ky0.png`