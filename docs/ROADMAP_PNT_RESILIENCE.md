# Roadmap — tres vías (código · hardware · visibilidad)

**Estado:** vigente · **Posicionamiento:** resiliencia GNSS degradado/denegado (civil) + **ultra-bajo consumo medible**.  
**Principio:** por partes; spoof solo por inyección SW (no RF sin autorización CNMC).

---

## A · Código (orden de prioridad)

| # | Ítem | Estado | Notas |
|---|------|--------|-------|
| A1 | Documentar ESKF (estado, Q/R, innovaciones) | **Hecho** | [README § Fusion algorithm](../README.md#fusion-algorithm--what-it-is--what-it-is-not) |
| A2 | Detección de **inconsistencia** (reglas / gate) | **Hecho (v1)** | `reject_reason=3`; gap corto; test SW spoof. **No** bloquea B-Ambiq |
| A2b | Spoof / inconsistencia **on-device ligero** (reglas→modelo) | Más tarde | Ideal en **Apollo4** (edge AI); no sustituye A2 v1 |
| A3 | Perfiles de dominio configurables (Q/R tierra/aire/mar) | Pendiente | Núcleo unificado + tuning por vertical |
| A4 | Suite de tests formal + spoof + properties | **Parcial** | Catch2 + **RapidCheck** (`test_properties_rapidcheck.cpp`) + `--safety-inject`; MC/NHC = campañas |
| A5 | cppcheck / clang-tidy + sanitizers + cobertura | **Parcial** | Baseline + CI [code-audit.yml](../.github/workflows/code-audit.yml) |

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
| B1 | Campo + verdad de terreno (túnel/urbano) — GPX móvil OK | Pendiente | DUT = **Pico 2 W** |
| B2 | Vibración real (vehículo/dron) | Pendiente | El sintético no sustituye |
| B3 | Consumo **PPK2 en Pico 2 W** | **Bloqueante** | Baseline obligatorio antes de comparar Ambiq |
| B4 | Marino cualitativo (lago/piscina + metal) | Opcional | Solo si se apunta AUV |

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
| C1 | Vídeo 1–2 min Unity/Cesium: pérdida/inconsistencia GNSS | Pendiente |
| C2 | Repo público + README PNT resilience | Pendiente (hoy showcase) |
| C3 | Comunidades + Show HN **con** campo + PPK2 (± Ambiq cuando haya) | Pendiente |
| C4 | LinkedIn: 2–3 posts técnicos espaciados | Pendiente |
| C5 | Telefónica internos (Wayra / IoT-edge) si aplica | Opcional |
| C6 | Clientes pequeños ES (drones ag, AUV, robótica) antes que gigantes | Opcional |
| C7 | Licencia (MIT vs dual comercial) **antes** de viralizar | Pendiente de decisión |

---

## Orden operativo recomendado

1. **PPK2 Pico** (B3) → publicar tabla en README  
2. Campo outage Pico (B1) + vídeo Unity (C1)  
3. Port Artemis/Apollo3 + A/B consumo/latencia vs Pico  
4. Apollo4 + A2b (spoof más sofisticado / on-device) si el mercado lo pide  
5. Visibilidad externa fuerte solo con números medidos  

---

## No-competidores

Honeywell, BAE, Thales, Northrop, Collins = fuera de alcance.  
Hueco: civil barato, MIT, auditable, zero-heap, resiliencia + ULP medible (Ambiq como destino de silicio, no como claim vacío).
