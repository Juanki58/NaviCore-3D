# GAP-6 — Blast-radius audit (geodesia Bowring / `ecef_to_lla`)

**Alcance:** solo auditoría. Sin re-lanzar benchmarks ni reinterpretar veredictos científicos.  
**Pregunta:** ¿qué código sintético y qué informes de `docs/benchmarks/` tocaban el camino `ned_to_lla` → `ecef_to_lla` (bug ~30 m), y sus conclusiones son **absolutas** o **relativas**?

**Bug (recordatorio):** `ecef_to_lla` incorrecto. Afecta rutas que hacen **NED→LLA**.  
**No** afecta `lla_to_ned` / `lla_to_ecef` (camino forward), usado al puntuar drift desde LLA de verdad.

---

## (a) Escenarios / benchmarks C++ — ¿camino crítico de medida?

| Unidad | Uso de geodesia | ¿Camino crítico de medida GNSS? | Detalle |
|--------|-----------------|--------------------------------:|---------|
| `src/scenarios/slalom_scenario.cpp` | `truth_to_gps_sample` → `ned_to_lla` → `GpsSample` → `update_gnss_from_sample` | **Sí** | Misma familia que el adaptador PC: truth NED → LLA (bug) → EKF `lla_to_ned`. |
| `src/targets/generic_pc/slalom_benchmark.cpp` | Igual (`truth_to_gps_sample`) | **Sí** | Duplicado de lógica slalom (TC04). |
| `src/targets/generic_pc/constant_slope_benchmark.cpp` | `truth_to_gps_sample` → `ned_to_lla` | **Sí** | Misma construcción de medida sintética. |
| `src/scenarios/tunnel_stress.cpp` | (1) `gps_truth_to_ned_m` = **`lla_to_ned`** (score); (2) `apply_gps_glitch_from_filter` = **`ned_to_lla`** | **Parcial** | Medida normal: LLA del `gps_simulator` (sin `ned_to_lla`). Solo el **glitch** de reentrada reescribe GPS vía `ned_to_lla`. Score de drift: forward OK. |
| `src/targets/generic_pc/super_tunnel_benchmark.cpp` | Solo `gps_truth_to_ned_m` = **`lla_to_ned`** | **No** | GNSS al EKF = muestra LLA del simulador; geodesia solo para truth/RMS/logging. Offset Bowring **no** entra en la medida. |

**Nota Pico / firmware:** fuera de este alcance (ya documentado en GAP-6): no usa el roundtrip NED→LLA→NED del adaptador.

---

## (b) Informes en `docs/benchmarks/` vs esas fuentes

| Árbol de informes | ¿Usa slalom / tunnel_stress / super_tunnel / constant_slope? | Fuente real de datos |
|-------------------|-------------------------------------------------------------|----------------------|
| `super_tunnel_bd_rerun/` | **Sí — super_tunnel** | `super_tunnel_benchmark` (A/B/N_always × clean/dirty) |
| `jacobian_imu_ab/slalom_a_vs_c_*` | **Sí — slalom** | `slalom_scenario` / Sim A vs C (Jacobiano NHC) |
| `jacobian_imu_ab/` (resto: `ab_report`, `hatt_*`, `cand1_*`, `patt_bias_g`, …) | **Sí — slalom y/o tunnel_stress** | Telemetría `*_ab_*_s71_telemetry.csv`, audits NHC de Sim |
| `gap3_*` | **No** (no esos TUs) | **real_run replay** (`NaviCore3D_Replay` + `InsEkf15State`) — ver nota abajo |
| `gap4_*` | **No** | **real_run replay** (misma familia) |
| `gap5_*` | **No** | **real_run replay** (p.ej. duración ~331 s = ALT) |
| *(constant_slope)* | — | **Sin** árbol dedicado bajo los globs pedidos |

**Nota gap3/4/5:** no usan los escenarios listados en (a), pero **sí** usaban el adaptador `InsEkf15State::update_gnss` (NED CSV → `ned_to_lla` → EKF), que **era** camino crítico. Eso ya se rebaselinó en parte con `ekf_v2_ab_3routes` post-Bowring; no es blast-radius del slalom/túnel sintético.

---

## (c) Tipo de conclusión — absoluta vs relativa

| Informe / familia | Conclusión típica | Tipo | ¿Offset constante ~30 m invalida la conclusión? |
|-------------------|-------------------|------|-----------------------------------------------|
| `super_tunnel_bd_rerun` | Deltas A/B/N_always, clean vs dirty (cientos–miles de m); veredictos C1–D2 / IEEE952 | **Relativa** (umbrales sobre Δ) + cifras absolutas de exit drift | Relativa: **no**. Absolutas de exit drift: sesgo cosmético frente a magnitudes ≫30 m; camino medida **sin** bug. |
| `jacobian_imu_ab/slalom_a_vs_c_*` | A vs C (Jacobiano); first divergence; ratios de drift | **Relativa** (+ abs. max\|drift\| por brazo) | Relativa: **no** (mismo bias en A y C). Abs. “max drift = X m”: posible sesgo ~O(30 m); timing A vs C y orden de magnitud siguen válidos. |
| `jacobian_imu_ab/ab_report` + tunnel arms | Pass/fail slalom lateral; tunnel ideal vs dirty | **Mixta** | Slalom abs. thresholds: posible contaminación leve. Tunnel: medida sin `ned_to_lla` → abs. OK. Comparaciones ideal/dirty: **relativas OK**. |
| `jacobian_imu_ab/hatt_*`, `cand1_*`, `patt_bias_g` | A/B NHC, gates, innov/K anatomía | **Relativa** / estructural | **No** requieren re-examen por Bowring (mismas condiciones en brazos). |
| `gap3_*` | Anatomía NHC/NIS/K en real_run; cliff NHC | **Mixta** | No son blast de slalom/túnel. Cifras `innov_h≈30 m` tempranas en autopsias **eran** el bias Bowring del adaptador; conclusiones NHC-off vs always (**relativas**, cientos de m) siguen en pie. |
| `gap4_*` | Régimen pos_vel / P_pv / gates | **Relativa** / estructural | Idem: no slalom/túnel; re-examen Bowring solo si se citan abs. de innov/posición pre-fix como “verdad”. |
| `gap5_*` | Passiveividad controlador / Gamma temporal | **Relativa** / operativa | Idem; no depende de abs. de posición a escala 30 m. |

---

## (d) Tabla resumen — ¿requiere re-examinar?

| Informe | Geodesia en camino critico de **esa** fuente | Tipo de conclusión | ¿Requiere re-examinar por GAP-6? |
|---------|---------------------------------------------:|--------------------|--------------------------------:|
| `super_tunnel_bd_rerun/*` | **No** (`lla_to_ned` score only) | Relativa (+ abs. grandes) | **No** |
| `jacobian_imu_ab/slalom_a_vs_c_*` | **Sí** (`ned_to_lla` → GPS) | Relativa (+ abs. drift) | **No** para veredictos A/B; **opcional** solo si se republican cifras abs. de drift al metro |
| `jacobian_imu_ab/` (hatt/cand1/patt/ab_report tunnel) | Slalom: **sí**; tunnel: **no** (salvo glitch) | Casi todo relativa | **No** |
| `gap3_*` | N/A a escenarios listados; real_run adaptador **sí** (histórico) | Mixta | **No** como blast sintético; ya cubierto por re-run phone / nota histórica ~30 m |
| `gap4_*` | Idem real_run | Relativa | **No** (salvo citas abs. pre-fix) |
| `gap5_*` | Idem real_run | Relativa | **No** |
| `constant_slope` reports | **Sí** en código; **sin** pack bajo globs | — | N/A (no hay informe listado) |

---

## Tamaño real del problema (sin re-lanzar nada)

1. **Super-túnel / tunnel_stress (medida nominal):** fuera del blast de `ecef_to_lla`.  
2. **Slalom / constant_slope:** sí estaban en camino critico; las conclusiones **A/B y de mecanismo** no caen; solo cifras absolutas finas de posición/deriva merecerían un asterisco o re-run si se van a citar al metro.  
3. **gap3/4/5:** no son hijos de esos escenarios; el riesgo era el adaptador real_run (ya tratado en GAP-6 / `ekf_v2_ab_3routes`).  
4. **Prioridad de re-run:** baja para el corpus sintético listado; no hay obligación de relanzar `super_tunnel_bd_rerun` ni la matriz jacobian por este bug.

---

## Referencias de código

| Función | Archivo | Rol |
|---------|---------|-----|
| `truth_to_gps_sample` | `slalom_scenario.cpp` ~136–162 | Critico |
| `truth_to_gps_sample` | `slalom_benchmark.cpp` ~157–179 | Critico |
| `truth_to_gps_sample` | `constant_slope_benchmark.cpp` ~144–166 | Critico |
| `apply_gps_glitch_from_filter` | `tunnel_stress.cpp` ~138–161 | Critico solo glitch |
| `gps_truth_to_ned_m` | `tunnel_stress.cpp` ~49–72; `super_tunnel_benchmark.cpp` ~282–305 | Score / logging (`lla_to_ned`) |
