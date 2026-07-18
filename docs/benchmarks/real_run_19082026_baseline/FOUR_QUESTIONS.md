# G-ext — Cuatro preguntas (evidencia cruda)

**Lectura normativa:** [INTERPRETATION.md](INTERPRETATION.md) — usar ese texto para claims.  
**Config:** shell G1 (`pos_vel`, `p_pv=none`, ZUPT off, NHC N=1).

## Smoke 60 s vs G-ext

| Run | Config | Resultado |
|-----|--------|-----------|
| Smoke temprano | `imu_stationary` | 7/50 en 60 s (trayecto “difícil”) |
| **G-ext** | shell G1 | **1/680** en 677 s |

---

## 1. ¿Otro “fix#4” / bifurcación?

**No alcanza esa región del espacio de estados.**

Sin segundo accept → **ni valida ni invalida** la bifurcación GAP-4.  
Ver [INTERPRETATION.md](INTERPRETATION.md) § Bifurcación.

## 2. ¿Crece |P_pv|?

**Sí** (0 → ~17 en t≈2.7–6.3 s, antes/durante primeros rejects).

## 3. ¿Norte domina?

**No se reproduce la dominancia Norte** como en G1; la componente dominante parece ligarse a la geometría del trayecto.  
`Λ_N` sigue elevado (gate estresado). No concluir aún “Norte no universal” ni “Norte → longitudinal”.

## 4. ¿Varios regímenes?

Vehículo: varios. Filtro: lockout continuo, incluso en ~**506 s** de GNSS limpio.  
Ese desacople externo/interno es el hallazgo más útil para H6-OBS.
