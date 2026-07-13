# 🧭 NaviCore-3D: Guía Viva de Desarrollo y Estado del Proyecto (Roadmap & Context)

> **NOTA PARA LA IA / CURSOR AGENT:** Al iniciar una nueva sesión de desarrollo, lee este archivo obligatoriamente. Aquí se detalla la arquitectura exacta, el último commit verificado y los siguientes pasos pendientes para evitar regresiones de código o pérdida de contexto.

---

## 🏗️ 1. Arquitectura del Sistema (Estado Actual)

El proyecto está estructurado bajo la filosofía **Zero-Heap** (cero asignación dinámica en runtime) y determinismo estricto para entornos críticos.

- **`src/core/`**: Motor matemático universal (X, Y, Z). Aislado de la plataforma física.
- **`src/core/api_ingest.*`**: API universal de entrada de datos de sensores.
- **`src/core/vehicle_bus_adapter.*`**: Adaptador que traduce tramas de bus CAN (`ImuCanFrame`, `OdoCanFrame`) al formato nativo del filtro (`ImuSample`).
- **`src/targets/generic_pc/`**: Simulador de estrés en PC (`NaviCore3D_Sim`) y Demo de bus de coche (`NaviCore3D_VehicleDemo`).
- **`src/targets/pico2_hardware/`**: ★ Target validado en banco — Raspberry Pi Pico 2 W, laboratorio Comarruga @ 100 Hz (`NaviCore3D_Pico2`). Ver `docs/comarruga_lab_hardware.md`.
- **`src/targets/pico_w/`**: Prototipo Pico W (Wi-Fi / UDP).
- **`src/targets/ambiq_apollo/`**: Capa estructural de drivers bare-metal (DMA, SPI, GPIO, UART, Power Monitor) con stubs para compilación cruzada en host.
- **`tools/visualizer.py`**: Visualizador dinámico e interactivo 3D en Python (replay CSV).
- **`tools/remote_visualizer.py`**: Visualizador UDP en tiempo real (10 Hz HIL) para telemetría remota.
- **`tools/telemetry_protocol.py`** / **`tools/telemetry_receiver.py`**: Codec y capa de transporte UDP compartida.

---

## 🛡️ 1b. Prioridades RT — Pico 2 Comarruga (software crítico)

> **Filosofía:** detectar → clasificar → responder. Sin medidas de banco, no se ajustan umbrales.

| Prioridad | Ámbito | Estado (`7f37724+`) | Pendiente |
|-----------|--------|---------------------|-----------|
| **P1** | Health Monitor + políticas de degradación | Parcial: `SystemHealth`, `RuntimeHealth`, `fault_tolerance` (tabla evento→acción) | Unificar en módulo `health_monitor`; políticas explícitas y recuperables documentadas |
| **P2** | Starvation / progreso de tareas | Parcial: `TaskMonitor`, deadline housekeeping 500 ms | Extender a todas las tareas críticas; umbrales derivados de P3 |
| **P3** | Campaña WCET en banco | Documentado: `docs/comarruga_lab_hardware.md` § Prioridad 2, escenarios S0–S7 | **Ejecutar en placa**; foco en `max_wifi_us` / GP21 bajo S2/S3/S7 |
| **P4** | Umbrales de degradación | Valores iniciales en `hw_config.hpp` (suposición) | Reemplazar por datos de P3 antes de certificar |

### P1 — Políticas actuales (`fault_tolerance`)

| Evento | Acción |
|--------|--------|
| `loop_budget_exceeded` > 5 en 1 s | DEGRADED + Wi-Fi off |
| `uart0_overflows` > 3/s | Confianza IMU ↓ |
| `uart1_overflows` > 3/s | Confianza GNSS ↓ |
| `missed_ticks` > 2 (backlog) | Omitir tick de navegación |
| `i2c_recoveries` > 5 | UPS OFFLINE permanente |
| `loop > 20 ms` × 3 | CRITICAL + `watchdog_reboot()` |
| `housekeeping` idle > 500 ms | CRITICAL + `watchdog_reboot()` |

**Limitación conocida:** `cyw43_arch_poll()` no tiene WCET acotado en SDK — P3 debe caracterizarlo antes de endurecer políticas Wi-Fi.

### Regla para agentes / desarrolladores

1. No añadir umbrales nuevos sin etiqueta `// SUPUESTO — pendiente P3`.
2. No afirmar "tiempo real duro" en rutas Wi-Fi hasta datos S7.
3. Toda nueva política: detección + clasificación (`SystemHealth`) + acción + métrica en `RuntimeHealth`.

---

## 🏁 2. Hitos Consolidados (Pico 2 Comarruga: `7f37724+`)

- [x] Rings UART SPSC, FSM I2C, ticks atómicos, WDT al final de ciclo
- [x] `RuntimeHealth` + `SystemHealth` + `fault_tolerance` (políticas P1)
- [x] `TaskMonitor` + deadline housekeeping 500 ms (P2 parcial)
- [x] Protocolo campaña WCET S0–S7 documentado (P3 — pendiente ejecución)
- [ ] Umbrales calibrados con datos de banco (P4)

### Hitos históricos (PC / Ambiq)

- [x] **Core matemático unificado**: Gestión de dominios (Tierra, Aire, Mar).
- [x] **Simulación de escenarios de estrés**: Pérdida de GPS por 10 s e inmersión submarina (presión).
- [x] **Drivers estructurales Ambiq**: Arquitectura de drivers HAL lista (simulada con stubs).
- [x] **Integración de automoción**: Ingestión de odometría de ruedas y lectura emulada de bus CAN con 5 ticks funcionales en limpio (`NaviCore3D_VehicleDemo` genera `HMI nav: lat=... lon=...`).
- [x] **Gemelo Digital 3D**: Visualizador gráfico operativo con `matplotlib` y `pandas`.

---

## 🛠️ 3. Próximos Pasos Inmediatos (Pendiente de Ejecución)

### FASE A: Robustez y Física Real (Corto Plazo)

- [x] **Lógica de marcha atrás (`reverse = true`)**: Modificar `fusion.cpp` para invertir la proyección geométrica del avance del coche cuando se activa la marcha atrás en la trama CAN.
- [x] **Ajuste dinámico de covarianza**: Incrementar los valores de la matriz de ruido del filtro inercial durante frenazos bruscos detectados por variaciones extremas en `accel_mps2[0]`.
- [x] **Inyección de fallos en simulador PC**: Implementada en `src/targets/generic_pc/sensors_sim.*` la estructura `SensorFaultInjection` con enumeración `SensorScenario` y bucle unificado `sensors_simulation_tick`. Soporta tres modos configurables desde `main.cpp`: escenario limpio (`SCENARIO_CLEAN`, sin anomalías), pérdida de GPS a partir del tick 3 (`SCENARIO_GPS_LOSS`, `fix_valid = false` y `satellites = 0`) y deriva acumulativa en la IMU (`SCENARIO_IMU_DRIFT`, bias creciente por tick en acelerómetro y giroscopio).

### FASE B: Hardware Real (Medio Plazo)

> **Guía oficial (LOCKED):** [`docs/navicore_v1_hardware_blueprint_locked.md`](docs/navicore_v1_hardware_blueprint_locked.md) — tag `navicore_v1_hardware_blueprint_locked`

**Fase 1 — Banco de pruebas (mesa de laboratorio):**

- [ ] Apollo4 Plus / Blue Lite EVB + J-Link SWD
- [ ] IMU MPU-9250 o BMX160 (SPI) + GNSS u-blox NEO-M8N/M9N (UART 9600)
- [ ] Breadboard, Dupont, analizador lógico 8ch @ 24 MHz

**Fase 2 — Drivers bare-metal (sustituir stubs):**

- [ ] **IOM SPI @ 1 MHz + DMA**: lectura atómica 6 ejes IMU (`ambiq_iom_master`, `ambiq_spi_imu_stub`)
- [ ] **UART0 GNSS RX @ 9600**: ISR → buffer circular 128 slots, drop-oldest (`command_ingestor` como patrón)
- [ ] **Parser NMEA estático**: `$GNGGA`, checksum, lat/lon → metros, zero-heap → `fusion`
- [ ] **Remoción de stubs Ambiq**: llamadas reales `am_hal_*` del SDK Apollo4 sobre silicio
- [ ] **Filtro de deriva inercial (bias térmico)**: calibración estática inicial en laboratorio

### FASE C: Certificación e Industrialización (Largo Plazo)

- [ ] **MRAM-conscious relocation**: Mapear tablas de constantes trigonométricas a memoria no volátil mediante `constexpr`.
- [ ] **Análisis WCET**: Auditoría de tiempos de ejecución límite para cumplimiento de normas de seguridad crítica.

---

## 📋 4. Cómo Validar el Estado Actual

Para asegurar que nada se ha roto antes de continuar, ejecuta en la terminal:

```powershell
# Verificar simulación y exportación CSV
cmake --build build --target NaviCore3D_Sim
./build/NaviCore3D_Sim.exe

# Verificar la integración CAN del vehículo
cmake --build build --target NaviCore3D_VehicleDemo
./build/NaviCore3D_VehicleDemo.exe

# Lanzar el Gemelo Digital (requiere: pip install matplotlib pandas)
python tools/visualizer.py

# Telemetría UDP en vivo (requiere: pip install matplotlib numpy)
python tools/remote_visualizer.py
./build/NaviCore3D_Sim.exe
./build/NaviCore3D_Sim.exe --no-udp   # sin red (CI / headless)

# Pruebas del protocolo UDP (32 bytes v3: escenario, nav_mode, cross/along, temperatura)
python tools/test_udp_telemetry.py
python tools/test_udp_faults.py
python tools/test_udp_integration.py
```

**Salida esperada — VehicleDemo (5 ticks):**

```
HMI nav: lat=41.387402 lon=2.168600
HMI nav: lat=41.387417 lon=2.168600
...
```

**Salida esperada — Sim:** resúmenes de estrés en consola + `docs/telemetria_navicore.csv` (~302 filas).

---

## 📦 5. Targets de Build (CMake)

| Target | Descripción |
|--------|-------------|
| `NaviCore3D_Sim` | Simulador PC + volcado CSV |
| `NaviCore3D_VehicleDemo` | Demo bus CAN + HMI |
| `NaviCore3D_Ambiq` | Loop bare-metal 100 ms (stubs host) |
| `NaviCore3D_Pico2` | Pico 2 W Comarruga @ 100 Hz — build standalone en `src/targets/pico2_hardware/` |

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

---

## 🚧 6. Notas de Infraestructura y Bloqueos

El Core matemático en `src/core/` está restaurado y verificado en C++17. Sin embargo, las pruebas locales de los ejecutables (`NaviCore3D_VehicleDemo.exe`) quedan pausadas debido a un bloqueo estricto de directivas de Control de Aplicaciones de Windows (WDAC), requiriendo autorización de IT para la ruta `C:\NaviCore-3D\build\` o para el toolchain de MinGW antes de poder realizar pruebas dinámicas en local.
