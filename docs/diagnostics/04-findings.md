# Resultados consolidados

Tabla de veredictos por hipótesis. Valores numéricos de referencia: predict-only + H9a, primeros 60 s, `data/real_run/`.

## Matriz de eliminación

| Hipótesis | Estado | Evidencia |
|-----------|--------|-----------|
| Montaje sensor→body incorrecto | **Refutada** | L1 residual ~0; RMSE roll/pitch OK con Rodrigues |
| Error global FRD/NED/FLU/ENU | **Refutada** | Tríada estática ~0.05°; L7 ancla ~0.07° |
| Origen NED / datum ~13 m | **Acotada** | H7b + `geodesy.cpp` WGS84; no explica leak predict |
| Desincronización GPS–IMU sola | **Insuficiente** | H5/H6; no corrige km de deriva |
| P0 mal escalado | **Insuficiente** | H4 barrido 1×–100× |
| Q/R NHC grid | **Insuficiente** | H5 grid |
| Updates GNSS causan deriva | **Refutada** | H9 predict-only: leak persiste |
| Init actitud (roll/pitch) incorrecta | **Refutada** | H9a: estático 0.09°; salto temporal en dinámica |
| Heading longitudinal erróneo (onset) | **Refutada** | `R_bn·e_x` vs GPS ~1° en 2–10 s |
| Bias accel en marco incorrecto | **Refutada** | Cadena propagación: bias no explica salto |
| Error solo al restar gravedad | **Refutada** | H9d: componente horizontal en `R_bn·a_body` antes de −g |
| EKF “equivocado” vs Android | **No demostrado** | Divergencia mutua; Orientation no es absoluto |
| Diferencia modelo dinámico estimadores | **Abierta** | L2/L5/L6 crecen con aceleración |

## Métricas clave por régimen

| Métrica | Estático 0–2 s | Dinámico 2–10 s | Crucero 11–25 s |
|---------|----------------|-----------------|-----------------|
| EKF ↔ Orientation (tilt) | 0.05° | 4.07° | 4.49° |
| EKF ↔ accel (tilt) | 0.09° | 4.32° | 3.03° |
| `a_lin,h` [m/s²] | 0.016 | 0.74 | 0.52 |
| Heading `R_bn·e_x` vs GPS | N/A (yaw=0) | **−1.2°** (media) | ~135° (yaw init) |
| Android Gravity ↔ TotalAccel | 0.09° | 1.94° | 2.08° |
| `R_mount` residual [m/s²] | ~0 | ~0 | ~0 |

## Perfil temporal del salto (pred ↔ ref)

| Tiempo | GPS speed | `a_lin,h` | pred ↔ ref |
|--------|-----------|-----------|------------|
| 2–6 s | ~0 m/s | <0.1 m/s² | ~0.1° |
| 6–7 s | ↑ | 1.13 m/s² | **3.8°** |
| 7–8 s | — | 2.05 m/s² | pico ~13° (transitorio) |
| 11+ s | >5 m/s | ~0.5 m/s² | 2–4° persistente |

El salto coincide con **aceleración longitudinal**, no con el paso del reloj ni con giro.

## Cadena de propagación (H9d + chain audit)

Orden confirmado en `ins_ekf::predict`:

1. Restar bias en body  
2. Integrar giroscopio  
3. `a_nav = R_bn · a_corr`  
4. Restar gravedad en NED  

**Hallazgos:**

- `corr(a_lin,h, a_nav_body_h) = 1.0` → la componente horizontal aparece **antes** de restar gravedad.
- `bias_frame_consistent = true`; `bias_explains_a_lin_jump = false`.
- `mechanism_1_attitude`: error en representación de inclinación de `R_bn`.

## Consistencia estadística (H4)

| Métrica | Valor |
|---------|-------|
| NIS medio (móvil) | ~197 000 |
| error / σ_h | ~611 |
| % NEES_n > 11 | >99% |

El filtro es **inconsistente** bajo aceleración; inflar covarianza no arregla el sesgo de propagación.

## Geodesia (H7)

| Métrica | Valor |
|---------|-------|
| parse vs EKF seed (horizontal) | ~58 km (pre-H7b) |
| nav vs geodesy independiente (max) | ~13.6 m |
| Tras WGS84 estricto + origen unificado | Ya no candidato principal |

## Interpretación actual

1. **Arquitectura del filtro** sobrevive a auditorías H0–H9d; no hay bug grosero evidente.
2. **Ancla estática** fija convenciones y montaje.
3. En dinámica, **tres estimadores de inclinación** divergen:
   - EKF (giro integrado),
   - Android Orientation / Gravity,
   - Acelerómetro total (contaminado por aceleración específica).
4. Android **separa** `Gravity.csv` de `TotalAcceleration.csv` en dinámica (+1.8°); el EKF no tiene el mismo modelo de down-weighting.
5. Pregunta acotada: ¿por qué la **inclinación** de `R_bn` evoluciona ~4° con aceleración mientras el **eje longitudinal horizontal** puede seguir al GPS?

## Qué no hacer todavía

- Abrir H10+ o retuning masivo de Q/R/P.
- Modificar `ins_ekf.cpp` antes de cerrar auditoría de convenciones.
- Usar acelerómetro como ground truth en dinámica.

## Referencias cruzadas

- Detalle actitud: [05-attitude-investigation.md](05-attitude-investigation.md)
- Reproducir: [06-reproduction.md](06-reproduction.md)
