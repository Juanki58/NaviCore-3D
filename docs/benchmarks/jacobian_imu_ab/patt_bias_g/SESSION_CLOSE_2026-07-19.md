# Cierre sesión 2026-07-19 — cadena early-loop completa

**Estado:** cerrado punta a punta. **Sin intervención implementada hoy.**

Protocolo: `docs/diagnostics/18-jacobian-imu-ab-protocol.md` §13.15–§13.21.

---

## Estado real (registro)

| | |
|--|--|
| **Entendido** | Por qué el Jacobiano corregido regresa en slalom — mapa de 11 eslabones hasta el mecanismo estructural (déficit Joseph tras corrección parcial post-hoc). Evidencia sólida punta a punta. |
| **No resuelto** | Ninguna intervención validada. Matriz A/B/C/D sigue FAIL en los cuatro umbrales. |
| **Candidata** | H-ATT-d — preregistro **congelado** §13.21 (P1=§11, no “≤66,2 m”). Implementación solo tras “adelante”. |

---

## Lección de proceso (congelada)

Cada cierre aparente de esta noche (P_yy, S, analogía Joseph in-tick, f_va unificador elegante) fue trampa de escala o hipótesis atractiva falsa. En cada caso, magnitud absoluta o contraste con control la desmontó **antes** de basar una intervención.

Si la sesión hubiera parado en cualquiera de esos puntos, H-ATT-d se habría diseñado sobre diagnóstico equivocado — y habría fallado como b1/H-ATT-c, sin saber por qué.

**Regla:** no preregistrar intervención sobre un eslabón intermedio “bonito”; solo sobre el mapa cerrado. `CORR_ABS_SCALE` (§13.18) es parte de esta disciplina, no un detalle de reporting.

---

## Mapa causal verificado (11 eslabones)

Cada eslabón tiene autopsia con datos (noche 1 + hoy). Falsos positivos descartados en el camino.

| # | Eslabón | Evidencia / veredicto |
|---|---------|----------------------|
| 1 | Signo Jacobiano NHC corregido | Math + diferencias finitas; regresión A×C |
| 2 | Bucle refuerzo `dx_att_z` desde tick 0 | `FEEDBACK_GROWTH`, R²≈0,996 |
| 3 | Onset silencioso [1,34→1,59) | Actitud acumula; cross-track aún modesto |
| 4 | Salto ‖δθ‖ @ break — **dominado por pitch** | No cliff \|a\|; `ATTITUDE_JUMP_DRIVEN_SURGE` |
| 5 | Latch: `dx_att_y` pierde la inversión de signo de ctrl | \|dx_y\|≈igual; signo neto opuesto |
| 6 | Inversión rastrea `K_y0` divergiendo desde ~1,10 s | No Joseph in-tick; trayectoria acumulada |
| 7 | `K_y0` arrastrado por `P[ATT_Y,VEL_N]` (×~290 vs onset) | No S, no P_yy, no P_att cruzado puro |
| 8 | ΔP_av = déficit Joseph lineal (Σ/growth≈1,10) | `JOSEPH_DEFICIT_LINEAR_SUFFICES`; 86% joseph |
| 9 | Actitud/vel contaminadas → surge vel_NED | cross-track ≈ v_lat, r≈0,98 |
| 10 | Giro proyecta a v_body → NHC ve violación | innov explota |
| 11 | Z cerrado (brazos exp.) → fuga `bias_gz` | K normal, no reorg P |

---

## Principio estructural (hallazgo general)

> Congelar o atenuar una componente de una corrección Kalman **después** de resolver el sistema completo (`δx ← Ky`, luego `δx_i ← 0`), **sin** recalcular K/S/Joseph sobre el subconjunto observado, deja un **déficit de recorte de covarianza** que se acumula y contamina canales correlacionados.

Es la misma familia que “algo que normalmente se cancela deja de cancelarse” (vías K_bias, inversión `dx_att_y` en ctrl). Explica por qué **b1** y **H-ATT-c** fallaron: actuaban sobre `δx` sin ajustar cómo Joseph procesa la corrección parcial.

Lección metodológica congelada: **`CORR_ABS_SCALE`** (§13.18) — pearson siempre con magnitudes absolutas.

---

## Falsos positivos descartados hoy

| Candidato | Por qué fuera |
|-----------|----------------|
| Reorg P_att–bias | CHAR_WEAK |
| Joseph in-tick (analogía GNSS fix#2) | λ post-hoc; K intacto en el tick |
| P_yy porta K_y0 | corr alta, \|Δ\| minúsculo |
| S / s_yz porta K_y0 | idem trampa de escala |
| P_att cruzado Z→pitch | ρ_yz débil/plano |
| f_va diferencial L↔C | r(f_va)=0,98; predict≈igual |
| Cliff de aceleración | \|a\| solo ~1,27×; δθ ~7,5× |
| ×260 = interés compuesto misterioso | vs baseline onset ~8e-6; presupuesto lineal cierra |

---

## Intervención candidata (próxima sesión — solo preregistro)

**No implementar hoy.**

| | |
|--|--|
| **Nombre tentativo** | H-ATT-d / “Z no observado” |
| **Idea** | Tratar el canal Z como no observado *antes* de resolver: reducir H (o inflar R_Z), recalcular S/K/Joseph en dimensión reducida — **no** zero-out post-hoc de `dx_att_z` |
| **Por qué distinto** | Ataca el déficit Joseph por construcción; Joseph recorta exactamente lo que el sistema reducido dice |
| **Contraste** | b1 / H-ATT-c: solución completa → λ en δx → Joseph “hace menos trabajo” → fuga P_av → K_y0 |
| **Preregistro** | Protocolo §13.21 — umbrales PASS/FAIL y brazos **antes** de código |

OQ9 permanece **separado** (no fusionar vía bias).

---

## Artefactos clave de hoy

| Tema | Path bajo `patt_bias_g/` |
|------|-------------------------|
| ‖δθ‖ / K vs y | `dtheta_jump_154_164.*` |
| P_att cross | `patt_cross_154_164.*` |
| Estado/predict | `state_att_predict_154_164.*` |
| K_y0 fila | `k_pitch_row_158.*` |
| Trayectoria K_y0 | `ky0_trajectory_onset.*` |
| S vs P_att–vel | `s_vs_pattvel_onset.*` |
| f_va vs ΔP | `fva_pattvel_110_154.*` |
| Presupuesto Joseph | `joseph_clip_deficit_110_154.*` |
