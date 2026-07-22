# Roadmap — tres vías (código · hardware · visibilidad)

**Estado:** vigente · **Posicionamiento:** resiliencia GNSS degradado/denegado (civil) + **ultra-bajo consumo medible**.  
**Principio:** por partes; spoof solo por inyección SW (no RF sin autorización CNMC).

**Rigor científico ya bancado** (no está “pendiente de README”): ver [README § Evidence — scorecard](../README.md#scientific-rigor-scorecard-what-is-already-done) — Monte Carlo N=100, matriz NHC (GAP-3), Allan IEEE 952 tooling, EKF v2 A/B en 3 trazas reales.

---

## S · Campañas científicas (rigor)

Estas campañas **ya están hechas** (artefactos en repo). No confundir “Allan fit pendiente de log estático” con “no hay método Allan”.

| # | Campaña | Estado | Resultado / artefacto |
|---|---------|--------|------------------------|
| S1 | **Monte Carlo** `TUNNEL_STRESS` | **Hecho** | N=100 · mean **13.0 m** @ t=30 s · p95 16.1 m · 0% diverge · `docs/monte_carlo/` |
| S2 | **NHC experiment matrix** | **Hecho** (GAP-3 CLOSED) | NHC-off 493 m exit vs `B_always` 1408 m — NHC agresivo empeora · `docs/nhc_experiments/manifest.json` |
| S3 | **Allan variance** IEEE Std 952 | **Herramienta hecha** · tabla ARW/BI pendiente de log | [`analyze_allan.py`](../analyze_allan.py) · falta `docs/imu_static_log.csv` (horas) |
| S4 | **EKF v2 vs v1** (3 phone drives) | **Hecho** | Accept → 100% · drift ~35 / 38 / 110 m · `docs/benchmarks/ekf_v2_ab_3routes/` |
| S5 | GAP-1…4 / G-ext diagnostics | **CLOSED** | Mapa en README § EKF diagnostics |

---

## A · Código (orden de prioridad)

| # | Ítem | Estado | Notas |
|---|------|--------|-------|
| A1 | Documentar ESKF (estado, Q/R, innovaciones) | **Hecho** | [README § Fusion algorithm](../README.md#fusion-algorithm--what-it-is--what-it-is-not) |
| A2 | Detección de **inconsistencia** (reglas / gate) | **Hecho (v1)** | `reject_reason=3`; gap corto; test SW spoof + **RapidCheck integrity** |
| A2b | Spoof / inconsistencia **on-device ligero** (reglas→modelo) | Más tarde | Ideal en **Apollo4** (edge AI); no sustituye A2 v1 |
| A3 | Perfiles de dominio configurables (Q/R tierra/aire/mar) | Pendiente | Núcleo unificado + tuning por vertical |
| A4 | Suite de tests formal + spoof + properties | **Mejorado** | Catch2 + RapidCheck + `--safety-inject` + edge + **integrity props** + **NHC ops CI** |
| A5 | cppcheck / clang-tidy + sanitizers + cobertura | **Triage hecho** | [TRIAGE_A5.md](benchmarks/static_analysis/TRIAGE_A5.md); fixes style; tidy CI soft; ASan hard |
| A6 | Fuzzing parsers NMEA/UBX/WT61C (libFuzzer) | **Hecho (v1)** | CI smoke 60 s; corpus `tests/fuzz/corpus/` |
| A7 | Fault injection policy (host) + lab protocol | **Parcial** | Host OK; banco físico pendiente de campaña registrada |
| A8 | Matriz NavMode documentada + tests | **Hecho (v1)** | [NAV_MODE_DEGRADATION.md](NAV_MODE_DEGRADATION.md) |
| A9 | WDT externo independiente del die | **API lista** | `bsp_ext_wdt` + GP15 |
| A11 | Vocabulario genérico estimate vs Nav* | **Hecho (v1)** | [ESTIMATE_ENGINE_VS_NAV_VOCAB.md](ESTIMATE_ENGINE_VS_NAV_VOCAB.md) · aliases · sin rewrite EKF |
| A12 | Política NHC operativa (GAP-3 freeze) | **Hecho (v1)** | [NHC_OPS_POLICY.md](NHC_OPS_POLICY.md) · `nhc_ops_policy.hpp` · `ALWAYS` no production-safe |

### Spoofing — solo software

**No** transmitir GPS falso ni jammer por RF en España/UE sin autorización CNMC.  
Validar con **inyección NMEA / salto en pipeline** (teleport / velocidad mentira).

---

## B · Hardware

### B0 · Mensaje de producto (por qué Ambiq)

El claim “ultra-low power” del repo hoy es **arquitectural** (zero-heap, edge MCU). El actor de referencia en µA/MHz es **Ambiq** (SPOT: ~3–4 µA/MHz en Apollo4 vs MCU convencionales).  
Historia vendible: **tracker / boya meses con pila + navegación resiliente a pérdida de GPS** — mismo núcleo, distinto silicio.

### B1 · Pruebas de valor (Pico primero)

| # | Prueba | Estado | Notas |
|---|--------|--------|-------|
| B1 | Campo + verdad de terreno (túnel/urbano) — GPX móvil OK | Checklist listo · run pendiente | [field_outage/CHECKLIST.md](benchmarks/field_outage/CHECKLIST.md) · **cierra con README** ([EVIDENCE_CLOSEOUT](EVIDENCE_CLOSEOUT.md)) |
| B2 | Vibración real (vehículo/dron) | Pendiente | El sintético no sustituye |
| B3 | Consumo **PPK2 en Pico 2 W** | **Bloqueante** | Baseline → tabla mA/mW en README Power |
| B4 | Marino cualitativo (lago/piscina + metal) | Opcional | Solo si se apunta AUV |
| B5 | Fault injection **en banco** (IMU unplug, UART, power, WDT) | Host smoke **hecho** · físico pendiente | [20260722_host](benchmarks/fault_injection/20260722_host/) · **cierra con README** · cuidado flash ([lab](FAULT_INJECTION_LAB.md)) |
| B6 | Log estático multi-hora → **Allan fit** publicado | Runbook + smoke · falta DUT horas | [allan/RUNBOOK.md](allan/RUNBOOK.md) · **cierra con README** Evidence Allan |

### B2 · Escalera Ambiq (menor → mayor esfuerzo)

**No cambiar de chip hasta tener el número PPK2 del Pico.** Luego, mismo escenario GPS-denied documentado en cada placa:

| Paso | Plataforma | Esfuerzo | Objetivo |
|------|------------|----------|----------|
| 0 | **Pico 2 W** + PPK2 | Bajo | Baseline mA/mW (commit + perfiles idle/EKF/IMU+GNSS) |
| 1 | **SparkFun Artemis / Apollo3** (~15–20 €) | Medio | Port C++17 zero-heap; **mismo** escenario outage; comparar **consumo + latencia EKF** vs Pico |
| 2 | **Apollo4 Plus / Blue Plus** | Alto | ~3 µA/MHz + aceleración edge AI; unir resiliencia + ULP; candidato natural para A2b |
| 3 | **Apollo510** | Opcional | Techo inferencia; overkill ahora |

Frase objetivo (README/vídeo): *mismo núcleo de navegación resiliente, **X× menos consumo** según hardware*.

Pi Zero: solo logger de verdad de terreno, **no** segundo NaviCore.

### B3 · Host PC (Artemis / Apollo) — dos niveles

| Nivel | Herramienta | Para qué |
|-------|-------------|----------|
| Debug crudo | Arduino Serial Monitor / Serial Plotter | Ver líneas y grafos en vivo; cero instalación extra |
| Captura de campo | `tools/serial_navstate_capture.py` | Un solo USB CDC → CSV estructurado (comparable a Sim/Replay) |

**Decisión de firmware (bloqueada):** el EKF corre **en el MCU**; el host solo recibe **NavState fusionado** (texto CSV, mismo esquema que `TelemetryFileLogger` / `NavigationState`).  
No validar el core enviando IMU/GPS crudo al PC y fusionando ahí — eso no prueba el silicio. Crudos opcionales solo como canal de debug, no como verdad del experimento.

Esquema CSV (una línea por muestra):

`timestamp_us,lat_rad,lon_rad,alt_m,vn_mps,ve_mps,vd_mps,roll_rad,pitch_rad,yaw_rad,health_flags,pos_uncertainty_m,att_uncertainty_rad`

---

## C · Visibilidad (orden importa)

| # | Acción | Estado |
|---|--------|--------|
| C1 | Vídeo 1–2 min GAP-3 NHC | **Publicado en GitHub** (ES+EN) | [ES](https://github.com/Juanki58/NaviCore-3D/blob/main/docs/video_gap3/NaviCore_GAP3_NHC.mp4) · [EN](https://github.com/Juanki58/NaviCore-3D/blob/main/docs/video_gap3/NaviCore_GAP3_NHC_en.mp4) · YouTube opcional |
| C2 | Repo + README PNT + **Evidence scorecard** (MC/NHC/Allan/v2) | **Hecho (v1)** en GitHub — ampliar con PPK2/campo |
| C3 | Comunidades + Show HN **con** campo + PPK2 (± Ambiq cuando haya) | Pendiente |
| C4 | LinkedIn: 2–3 posts técnicos espaciados | Pendiente |
| C5 | Telefónica internos (Wayra / IoT-edge) si aplica | Opcional |
| C6 | Clientes pequeños ES (drones ag, AUV, robótica) antes que gigantes | Opcional |
| C7 | Licencia (MIT vs dual comercial) **antes** de viralizar | Pendiente de decisión |

---

## Orden operativo recomendado

**Secuencia mínima (no romper):**  
`vídeo GAP-3` → `Allan fit` → `outage Pico` → `PPK2` → **entonces** hablar fuera / pensar silicio (Ambiq).

No adelantar Ambiq, ZUPT “porque apetece”, ni visibilidad fuerte porque llegó un kit — el orden es esfuerzo/impacto, no moda.

**Regla de cierre:** Allan, outage y fault-injection **físico** terminan cada uno con tabla en **README Evidence**, no solo CSV — [`EVIDENCE_CLOSEOUT.md`](EVIDENCE_CLOSEOUT.md).

0. **Ya bancado (no rehacer):** S1–S5 + Evidence + A12 NHC ops + integrity RapidCheck + GAP-3 video **pack** + host fault smoke  
1. **GAP-3 MP4 publicados en GitHub** (C1) + link en README Visibility  
2. Log estático → **Allan fit** (B6) → **pegar ARW/BI en README**  
3. Campo outage Pico (B1) → **pegar curva en README** + fault físico (B5) → **pegar pass/fail en README** (cuidado flash en brownout: ver FAULT_INJECTION_LAB)  
4. **PPK2 Pico** (B3) → tabla mA/mW en README Power  
5. Port Artemis/Apollo3 + A/B consumo/latencia vs Pico  
6. Apollo4 + A2b si el mercado lo pide  
7. Visibilidad externa fuerte (C3+) **solo** con números medidos de hardware en Evidence · YouTube mirror del GAP-3 opcional  

**Tu turno (hardware):** IMU quieto horas · una salida Pico con outage · PPK2.

---

## No-competidores

Honeywell, BAE, Thales, Northrop, Collins = fuera de alcance.  
Hueco: civil barato, MIT, auditable, zero-heap, resiliencia + ULP medible (Ambiq como destino de silicio, no como claim vacío).
