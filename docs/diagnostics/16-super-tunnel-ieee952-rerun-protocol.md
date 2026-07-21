# Protocolo — repetición limpia super_tunnel B vs B_dirty (IEEE-952)

**Estado:** preregistrado (antes de ejecutar)  
**Fecha:** 2026-07-18  
**CLI:** `NaviCore3D_Sim.exe --nhc-bd-rerun`  
**Análisis:** `python tools/audit_super_tunnel_bd_rerun.py`  
**Artefactos:** `docs/benchmarks/super_tunnel_bd_rerun/`

---

## 0. Proveniencia del 481 m / 1416 m (antes de interpretar)

| Hecho | Evidencia |
|-------|-----------|
| Cifras originales | Aislamiento `DIRTY_FULL` + NHC **ALWAYS**, seed `424242`: ~480 m sin NHC / ~1422 m con NHC (usuario citó 481/1416). Matriz del mismo día: ideal+NHC ~1408 m. |
| Commit Jacobiano | `bf2bfbd` — `fill_nhc_attitude_coupling_rows` + test FD |
| Banner actual | “Jacobiano corregido” = **texto**, no pin de binario |
| ¿Mismo binario hoy? | **No.** Desde `bf2bfbd`: `ins_ekf.cpp` +~1776/−100 (auditorías, GNSS Joseph, P_pv policy, `nhc_every_n`); `InsEkf15State` wrapper; harness `super_tunnel` pasó de `InsEkfFilter` directo a `create_default_navi_filter()`. |
| ¿Cambió el Jacobiano NHC? | **No.** Coeficientes idénticos a `bf2bfbd`. `f_va` en predict: solo copia a audit. Default `ppv_policy=NONE`, `nhc_every_n=1`. ZUPT: `apply_constraints(false,…)`. |
| Tercera vía posible | Wrapper `InsEkf15State` / seeding GNSS / instrumentación — no el signo NHC ni ZUPT de reloj. Por eso se repite sobre el binario **actual**, no se cita el 481/1416 como resultado de hoy. |

**B histórico ≠ aislamiento 481/1416:** `B` / `B_dirty` usaban `NHC_CONSTANT_VEL_ONLY` (apagón + vel.const → ~1041 / ~1108 m). El 481/1416 era `NHC_ALWAYS` + dirty. Esta corrida ejecuta **ambos** paneles con etiquetas distintas.

---

## 1. Brazos (fijos)

Seed `424242`. ZUPT nunca armado. R NHC nominal (`0.1` / `0.05` m/s salvo override).

| ID | NHC policy | IMU |
|----|------------|-----|
| `A` | OFF | IDEAL |
| `A_dirty` | OFF | DIRTY_FULL |
| `B` | CONSTANT_VEL_ONLY | IDEAL |
| `B_dirty` | CONSTANT_VEL_ONLY | DIRTY_FULL |
| `N_always` | ALWAYS | IDEAL |
| `N_always_dirty` | ALWAYS | DIRTY_FULL |

Panel **B** = pregunta histórica B vs B_dirty.  
Panel **N_always** = réplica de la comparación que motivó 481/1416.

---

## 2. Criterios preregistrados (sin ambigüedad y/o)

Definiciones:

```
Δ_B      = drift_exit(B)            − drift_exit(A)
Δ_Bdirty = drift_exit(B_dirty)     − drift_exit(A_dirty)
Δ_N      = drift_exit(N_always)    − drift_exit(A)
Δ_Ndirty = drift_exit(N_always_dirty) − drift_exit(A_dirty)
```

Umbrales (metros, drift a salida de túnel):

| Código | Condición | Significado |
|--------|-----------|-------------|
| **C1** | `Δ_B ≤ +50` | Con IMU limpia, NHC (política B) no empeora materialmente vs baseline |
| **C2** | `Δ_Bdirty ≥ +400` | Con IMU dirty, NHC empeora de forma sustancial |
| **C3** | `Δ_Bdirty − Δ_B ≥ +300` | El exceso dirty−clean explica la mayor parte del daño |
| **D1** | `Δ_B ≥ +400` | NHC empeora también con IMU limpia (magnitud comparable al daño “IEEE”) |
| **D2** | `\|Δ_Bdirty − Δ_B\| < 150` | Penalidad casi igual dirty vs clean |

**Veredicto panel B (preregistrado):**

- `IEEE952_BIAS_CONFIRMED` ⟺ **C1 ∧ C2 ∧ C3**
- `IEEE952_BIAS_REJECTED` ⟺ **D1 ∨ D2**
- `INCONCLUSIVE` ⟺ ni confirmado ni rechazado

Mismos umbrales C1–C3 / D1–D2 aplicados a `Δ_N` / `Δ_Ndirty` → veredicto panel `N_always` (independiente).

**Regla anti-y/o:** no se permite reclamar “sesgo IEEE confirmado” si solo se cumple un subconjunto de C1–C3. No se permite “descartado” por un solo Δ sin D1 o D2.

---

## 3. Anatomía obligatoria (no solo drift final)

Por cada brazo, CSV tick-a-tick `*_anatomy.csv`:

- `t_ms`, `gps_outage`, `nhc_applied`
- `P_vv_frob`, `P_pv_frob`, `P_vv_trace`
- `vel_norm_mps`, `drift_h_m`
- si NHC: `dx_pos_norm`, `dx_vel_norm`, `k_max`, `innov_norm`

**Salto de un tick (flag automático):**

```
|Δ drift_h| > 5 m   OR   |Δ P_vv_frob| / max(P_vv_frob_pre, 1e-9) > 0.5
```

en un solo intervalo de 10 ms → `single_tick_jump=true` para ese brazo.

Si el veredicto agregado es “NHC empeora con IMU limpia” **o** hay `single_tick_jump`, el protocolo **exige** autopsia tick-a-tick (mismo espíritu que GAP-3 fix#2/#3) antes de atribuir a IEEE-952. La cifra agregada sola **no** cierra.

---

## 4. Orden de ejecución

1. Congelar este documento (umbrales arriba).  
2. Build `NaviCore3D_Sim`.  
3. `.\build\NaviCore3D_Sim.exe --nhc-bd-rerun`  
4. `python tools/audit_super_tunnel_bd_rerun.py` → escribe `verdict.json` **solo** evaluando §2–§3.  
5. Si aplica autopsia → no declarar cierre IEEE-952 en este paso.

---

## 5. Resultado de la corrida (2026-07-18) — post-preregistro

Ejecutado tras congelar §1–§4. Artefactos: `docs/benchmarks/super_tunnel_bd_rerun/`.

| Arm | drift_exit (m) |
|-----|----------------|
| A | 299.07 |
| A_dirty | 303.64 |
| B | 995.25 |
| B_dirty | 1220.81 |
| N_always | 1421.56 |
| N_always_dirty | 1327.38 |

| Panel | Δ_clean | Δ_dirty | Flags | Veredicto |
|-------|---------|---------|-------|-----------|
| B | +696.18 | +917.18 | D1 | `IEEE952_BIAS_REJECTED` |
| N_always | +1122.48 | +1023.74 | D1∧D2 | `IEEE952_BIAS_REJECTED` |

**Overall:** `IEEE952_BIAS_REJECTED`.

Autopsia (`autopsy.md`): **0** saltos de un tick en ventana de apagón (el salto a t≈55010 es reaparición GNSS). Daño con IMU ideal es gradual / pre-apagón en `N_always` (drift≈255 m ya a t=10 s; P_pv_frob 0.016→18). No atribuir a IEEE-952.

Baseline A≠481 m histórico → binario distinto (§0); usar deltas.

### 5.1 Autopsia 0–10 s `N_always` (cierre causal)

Ver `docs/benchmarks/super_tunnel_bd_rerun/cold_start_autopsy.md`.

- A/B (NHC off pre-túnel): drift≈2.15 m, P_pv≈0.11 a t=10 s.
- `N_always`: drift≈255 m, P_pv≈18 a t=10 s — **mismo P0**, diferencia = NHC ALWAYS con GNSS.
- Crecimiento gradual (0 cliffs); dx coherente con k·innov (no Joseph).
- **Causa del 481→1416:** NHC en frío desde t=0, no IEEE-952.

### 5.2 Proveniencia producción (`pico2_hardware`) — misma rigurosidad que ZUPT

| Pregunta | Evidencia |
|----------|-----------|
| ¿Llama `apply_constraints` / `ins_ekf_set_nhc_enabled(true)`? | **No.** `bsp_sensors.cpp` solo: `ins_ekf_init` → `ins_ekf_predict` → `ins_ekf_update_gnss` (si fix). |
| ¿NHC armado tras init? | `ins_ekf_init` fija `nhc_enabled = false` (`ins_ekf.cpp`). Nadie lo enciende en el target Pico. |
| ¿`predict` puede aplicar NHC igual? | Solo si `nhc_enabled`; con false el bloque NHC en `ins_ekf_predict` no corre. |
| Única mención NHC en `main.cpp` | Lee `ins_ekf_nhc_enabled` para flag UDP — siempre false en campo. |
| `NAVICORE_INS_EKF_NHC_EVERY_N_TICKS=2U` en CMake Pico | Default de stride **si** alguien armara NHC; hoy es código muerto. |

**Veredicto campo:** igual que el ZUPT de reloj — el daño ALWAYS+GNSS es del **harness / benchmarks**, no del firmware desplegado hoy. Urgencia de campo ≈ 0 hasta que producción arme NHC sin condicionar a ausencia de GNSS.

**Nota de diseño:** si en el futuro se activa NHC en Pico, **no** usar ALWAYS incondicional con fix GNSS; exigir política tipo `NO_GNSS_FIX` / gracia post-seed.  
**Documento canónico (ZUPT+NHC juntos):** [17-conditional-constraints-architecture.md](17-conditional-constraints-architecture.md).

---

## 6. Historial

| Versión | Fecha | Notas |
|---------|-------|-------|
| 1.0 | 2026-07-18 | Preregistro; proveniencia binario ≠ 481/1416; paneles B y N_always |
| 1.1 | 2026-07-18 | Resultados + autopsia; IEEE952_BIAS_REJECTED |
| 1.2 | 2026-07-18 | Cierre causal: NHC ALWAYS en frío; no Joseph; no IEEE-952 |
| 1.3 | 2026-07-18 | Proveniencia Pico: NHC nunca armado (como ZUPT) |
