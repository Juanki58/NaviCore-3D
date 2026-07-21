# Autopsia arranque — `N_always` 0–10 s (cierre hilo IEEE-952 / 481→1416)

**Fecha:** 2026-07-18  
**Artefactos:** `cold_start_verdict.json`, anatomía/trazas en este directorio  
**Script:** `tools/audit_super_tunnel_cold_start.py`

---

## Preguntas (antes del resultado)

1. ¿El estado a t=10 s (P_pv 0.016→18, drift≈255 m) es la misma familia que ganancia alta con P inflado / desync Joseph?
2. ¿Eso explica el 481→1416 original sin IEEE-952?

---

## Aclaración crítica de escenario

`N_always` en 0–10 s **no** es “NHC puro sin GNSS”. Hay **GNSS ON** (`fix_valid`) **y** NHC **cada tick** desde t=0. El apagón empieza a 10 s.

El experimento de control es `B` / `A`: mismo P0, mismo GNSS, **NHC OFF** hasta el túnel.

---

## Contraste causal (misma semilla, IMU ideal)

| t_ms | A drift / P_pv | B drift / P_pv | N_always drift / P_pv |
|------|----------------|----------------|------------------------|
| 0 | 0.23 / 0.017 | 0.23 / 0.017 | 0.23 / 0.016 |
| 1000 | 2.33 / 0.55 | 2.33 / 0.55 | **4.36 / 0.40** |
| 5000 | 2.14 / 0.32 | 2.14 / 0.32 | **93.0 / 16.5** |
| 10000 | **2.15 / 0.11** | **2.15 / 0.10** | **255 / 18.0** |

Hasta t=10 s, **A ≡ B** (NHC aún no actúa en B). Toda la divergencia de `N_always` es el NHC aplicado en frío **mientras GNSS también actualiza**.

No es “P0 alto solo”: A/B arrancan con el mismo P0 y GNSS mantiene drift≈2 m y **comprime** P_pv. El disparador es la **política ALWAYS desde t=0**.

---

## ¿Cliff de un tick o acumulación?

| Métrica | Valor |
|---------|-------|
| Ratio P_pv(10 s)/P_pv(0) | **1111×** |
| Saltos \|ΔP_pv\|/P_pv > 10 en un tick | **0** |
| Ticks k_max>0.9 ∧ innov<0.05 | **0** |
| k_max máx en 0–500 ms | **0.15** |
| Primera vez drift≥10 / 50 / 100 / 200 m | 1.7 s / 3.5 s / 5.2 s / 8.4 s |
| Primera vez P_pv≥1 / 10 / 15 | 2.35 s / 4.35 s / 4.86 s |

El “salto” a t=10 s es el **estado acumulado**, no un evento puntual en ese instante. Familia: **crecimiento gradual**, no cliff tipo ZUPT de un tick.

Ventana histórica ~350 ms: misma dirección que la cascada E antigua (crece `innov_z`, baja `vel_e`) pero **más suave** en el binario actual (yaw sigue ~90°, sin flip a 74°). Misma familia, distinta violencia.

---

## Algebra de ganancia: ¿K≈P/(P+R) o desync Joseph?

**Proxy 1D `innov²/NIS` → k_bayes** es engañoso aquí (NHC es 2×15): da k_bayes≈0.99 con k_obs≈0.02 temprano. Eso **no** prueba desync; prueba que el escalar 1D no es la ganancia de estado.

Comprobación útil (multivariable):

```
coherencia  dx_vel / (k_max · innov_norm)   para innov>0.05, 0–10 s
  n=966, mediana≈0.38, 99.6% ∈ [0.1, 10]
```

Los Δx guardan proporción con la K logueada. **No** hay el patrón Joseph/fix#2 (K grande con innov minúscula y ΔP desproporcionado).

P0 inicial (vel diag=1, att 5–10°) es el combustible; NHC ALWAYS es el mechero; la ganancia actúa de forma **bayesiana/coherente** sobre una covarianza cruzada que el propio ciclo NHC+predict infla (P_pv sube mientras en A/B baja).

**Familia:** `COLD_START_NHC_ALWAYS_DURING_GNSS` — misma familia amplia que “arranque con alta incertidumbre + update agresivo”, **no** bug de signo Jacobiano ni desync Joseph, **no** IEEE-952.

---

## ¿Explica el 481→1416 original?

Sí, de forma suficiente y más simple:

1. El aislamiento histórico usaba NHC **ALWAYS** (igual que `N_always`).
2. Ideal≈dirty ya entonces (~1408 vs ~1422) — IEEE-952 sobraba.
3. Hoy, con IMU ideal: a t=10 s ya hay **255 m** de deriva **antes del túnel**; exit=1422 m.
4. Control A/B: sin NHC pre-túnel → ~2 m a t=10 s.

Occam: el misterio era **NHC desde el primer tick en arranque en frío (con GNSS)**, no el modelo de sensor sucio.

---

## Implicación de sistema

Cualquier arranque en frío con política tipo ALWAYS (o NHC ON con fix GNSS aún fresco y P0 ancho) puede sembrar el mismo daño **antes** de cualquier GPS-denial. No hace falta túnel ni IMU dirty.

Mitigaciones candidatas (no ejecutadas aquí; solo implicación):

- No armar NHC hasta que P haya colapsado bajo GNSS / gracia post-seed.
- Políticas ya exploradas (`CONSTANT_VEL_ONLY`, `NO_GNSS_FIX`) evitan este modo en 0–10 s (véase B).

---

## Veredicto de cierre del hilo

| Hipótesis | Estado |
|-----------|--------|
| Sesgo IEEE-952 causa 481→1416 | **REJECTED** (sesión previa + este contraste) |
| Desync Joseph / K vs P en este tramo | **REJECTED** (coherencia dx↔k·innov; 0 cliffs) |
| Causa real: NHC agresivo en frío (ALWAYS) con P0 ancho | **ACCEPTED** (A/B vs N_always) |
| Mismo playbook que “alta ganancia en alta incertidumbre” | **Sí** (familia), disparador = política de arranque NHC, no gap GNSS |
