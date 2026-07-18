# H6 series binding — fórmulas y ventanas (pre-ejecución)

**Estado:** CONGELADO pre-ejecución H6 (D18)  
**Configs / paths:** ver [H6_PREFLIGHT.md](../../diagnostics/reference/H6_PREFLIGHT.md)

## Datos de entrada

| Config | Directorio |
|--------|------------|
| C-F1 | `docs/benchmarks/gap5_adaptive_nhc/p0_passive_f1_bridge/` |
| C-PoC | `docs/benchmarks/gap5_adaptive_nhc/p0_passive_validation/` |

Archivos: `cov_step_audit.csv`, `gnss_nis_audit.csv`.  
Opcional O1/O2: `controller_audit.csv` (`gamma_raw`, `gamma_filtered`) si existe — **preferido** para O1/O2 cuando presente; si no, reconstruir desde cov_step.

## Regímenes (timestamps)

De `gnss_nis_audit.csv`, accepts ordenados por `gps_index`:

| Régimen | Ventana |
|---------|---------|
| R0 | Desde primer sample hasta `t_fix2` (2º accept); si &lt;2 accepts → N/A |
| R1 | (`t_fix2`, `t_fix3`) — burst; si no hay accept#3 → N/A |
| R2 | [`t_fix3`, `t_fix4`) si existe accept#4; si no, [`t_fix3`, `t_fix3+15s`] acotado al fin de log |
| R3 | [`t_fix4`, `t_fix4+10s`] si existe accept#4; si no → **N/A** |
| R4 | Desde `max(t_fix2, 30s)` hasta fin de log (crucero largo); si duración &lt; 5s → N/A |

## Observables

### O1 — Γ_inst

Si `controller_audit.csv` tiene `gamma_raw`: usar esa serie vs `timestamp_s`.  
Si no: por cada `imu_seq` con predict+nhc en cov_step,  
`Γ_tick = |ΔP_vv_nhc| / max(|ΔP_vv_predict|, 1e-12)`  
y Γ_inst = media móvil causal 1.0 s de Γ_tick (misma idea doc 14 ventana corta).

### O2 — Γ̄

Si `gamma_filtered` existe: usarlo.  
Si no: EWMA de O1 con α = dt/(τ+dt), τ=1.0 s.

### O3 — ‖P_pv‖ / P_vv

Serie en cada fila cov_step con `P_vv_frob > 0`:  
`ratio = P_pv_frob / P_vv_frob`.  
Usar todas las fases; para stats por régimen, filtrar por `timestamp_s` en ventana.

### O4 — Λ_N

Por cada fila gnss: `Λ_N = abs(innov_n_m) / sqrt(max(s_nn, 1e-12))`.

### O5 — dΛ_N/dt

Entre GNSS sucesivos i−1 → i:  
`(Λ_N[i] - Λ_N[i-1]) / max(dt, 1e-3)`.

## C1–C6 (boolean / corto)

Evaluar **después** de series, con reglas fijas:

| C | Regla |
|---|--------|
| C1 | En R1, `max(O)` ≥ 1.5 × `median(O in R0)` → sí |
| C2 | En R2, comportamiento: si Paso0 espera alto/pico y ordinal∈{alto,pico} → coherente; si espera bajo y ordinal bajo → coherente; else no |
| C3 | Comparar ordinal C7 R1–R4 entre C-F1 y C-PoC: si ≥3 regímenes con mismo ordinal (ignorando N/A) → significado conservado (sí); si Γ-like colapso (todos bajo en PoC tras pico en F1) → no |
| C4 | Si O es O2 (Γ̄): memoria incompatible con R1 si `max(O2 in R1) < 0.5 * max(O1 in R1)` → sí (incompatible); else no. Otros Oi: C4=N/A o no |
| C5 | Causal sin futuro: O1–O5 definidos causalmente → sí para todos en este binding |
| C6 | Local tick/ventana ≤1 s: O1–O5 sí excepto O2 (τ=1s) sigue local → sí |

No usar estas reglas para “elegir ganador”; solo rellenar caracterización.
