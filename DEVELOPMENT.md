# 🧭 NaviCore-3D: Guía Viva de Desarrollo y Estado del Proyecto (Roadmap & Context)

> **NOTA PARA LA IA / CURSOR AGENT:** Al iniciar una nueva sesión de desarrollo, lee este archivo obligatoriamente. Aquí se detalla la arquitectura exacta, el último commit verificado y los siguientes pasos pendientes para evitar regresiones de código o pérdida de contexto.

> **Investigación EKF:** [`EVIDENCE_STRENGTH_AUDIT.md`](docs/diagnostics/reference/EVIDENCE_STRENGTH_AUDIT.md) — OQ1 abierta; pausa **cerrada** (D17); H6 justificado por insuficiencia de evidencia para discriminar candidatos. Ejecutar solo v1.2 sin editar. No controlador.

---

## 🏗️ 1. Arquitectura del Sistema (Estado Actual)

El proyecto está estructurado bajo la filosofía **Zero-Heap** (cero asignación dinámica en runtime) y determinismo estricto para entornos críticos.

```
NaviCore-3D/
├── src/
│   ├── core/                   # Motor matemático universal (agnóstico de plataforma)
│   │   ├── NavState.*          # Estado de navegación unificado
│   │   ├── vector3d.*          # Modelo de coordenadas 3D permanente
│   │   ├── waypoint.*          # Rutas con buffer fijo
│   │   ├── fusion.*            # Dead reckoning + fusión de sensores
│   │   ├── navigation_cortex.* # Orquestación + guardas de seguridad
│   │   ├── math_utils.hpp      # Umbrales FPU (sqrtf/sinf/cosf)
│   │   └── sensor_types.hpp    # Muestras IMU/GPS/presión portables
│   └── targets/
│       ├── generic_pc/         # Simulador host + VehicleDemo (PC)
│       │   ├── main.cpp
│       │   ├── sensors_sim.*
│       │   ├── power_state_machine.*
│       │   └── telemetry_udp_sender.*
│       └── pico2_hardware/     # ★ Pico 2 W — laboratorio Comarruga (validado)
│           ├── main.cpp        # Bucle 100 Hz, health_monitor, WDT
│           ├── health_monitor.*  # SystemHealth + políticas de degradación
│           ├── task_monitor.*    # Starvation / progreso de tareas
│           ├── loop_metrics.*    # RuntimeHealth (WCET por bloque)
│           ├── bsp_sensors.*   # Orquestador HAL
│           ├── bsp_wt61c.*     # IMU UART0 @ 115200 (WT61C-232)
│           ├── bsp_gnss.*      # GNSS UART1 / NMEA $GNGGA (NEO-M9N)
│           ├── bsp_power.*     # UPS I2C FSM cooperativa (Waveshare)
│           ├── safe_log.*      # USB con presupuesto por ciclo
│           └── hw_config.hpp   # Mapa de pines + invariantes RT
├── docs/
│   ├── comarruga_lab_hardware.md
│   ├── sil_architecture.md
│   └── telemetria_navicore.csv
├── tools/
│   ├── visualizer.py
│   └── remote_visualizer.py
├── CMakeLists.txt
└── build/
```

**Targets activos (solo dos):**

- **`src/core/`**: Motor matemático universal (X, Y, Z). Aislado de la plataforma física.
- **`src/core/api_ingest.*`**: API universal de entrada de datos de sensores.
- **`src/core/vehicle_bus_adapter.*`**: Adaptador que traduce tramas de bus CAN (`ImuCanFrame`, `OdoCanFrame`) al formato nativo del filtro (`ImuSample`).
- **`src/targets/generic_pc/`**: Simulador de estrés en PC (`NaviCore3D_Sim`) y Demo de bus de coche (`NaviCore3D_VehicleDemo`).
- **`src/targets/pico2_hardware/`**: ★ Target validado en banco — Raspberry Pi Pico 2 W, laboratorio Comarruga @ 100 Hz (`NaviCore3D_Pico2`). Ver `docs/comarruga_lab_hardware.md`.
- **`tools/visualizer.py`**: Visualizador dinámico e interactivo 3D en Python (replay CSV).
- **`tools/remote_visualizer.py`**: Visualizador UDP en tiempo real (10 Hz HIL) para telemetría remota.
- **`tools/telemetry_protocol.py`** / **`tools/telemetry_receiver.py`**: Codec y capa de transporte UDP compartida.

---

## 🛡️ 1b. Prioridades RT — Pico 2 Comarruga (software crítico)

> **Filosofía:** detectar → clasificar → responder. Sin medidas de banco, no se ajustan umbrales.

| Prioridad | Ámbito | Estado (`7f37724+`) | Pendiente |
|-----------|--------|---------------------|-----------|
| **P1** | Health Monitor + políticas de degradación | `health_monitor`: `SystemHealth`, tabla `HealthPolicyDescriptor` (recuperable/permanente), acciones | Calibrar umbrales tras P3 |
| **P2** | Starvation / progreso de tareas | `TaskMonitor` + `health_monitor_check_task_deadline()` en RxPump, NavTick, Housekeeping, Wifi, Loop | Calibrar umbrales tras P3 |
| **P3** | Campaña WCET en banco | Documentado: `docs/comarruga_lab_hardware.md` § Prioridad 2, escenarios S0–S7 | **Ejecutar en placa**; foco en `max_wifi_us` / GP21 bajo S2/S3/S7 |
| **P4** | Umbrales de degradación | Valores iniciales en `hw_config.hpp` (suposición) | Reemplazar por datos de P3 antes de certificar |

### P1 — Políticas actuales (`health_monitor`)

| Evento | Clasificación | Recuperación | Acción |
|--------|---------------|--------------|--------|
| `loop_budget_exceeded` > 5 en 1 s | DEGRADED | Permanente | Wi-Fi off |
| `uart0_overflows` > 3/s | DEGRADED | Permanente | Confianza IMU ↓ |
| `uart1_overflows` > 3/s | DEGRADED | Permanente | Confianza GNSS ↓ |
| `missed_ticks` > 2 (backlog) | DEGRADED | Recuperable | Omitir tick de navegación |
| `i2c_recoveries` > 5 | CRITICAL | Permanente | UPS OFFLINE |
| `loop > 20 ms` × 3 | CRITICAL | Permanente | `watchdog_reboot()` |
| `housekeeping` idle > 500 ms | CRITICAL | Permanente | `watchdog_reboot()` |
| `rx_pump` idle > 30 ms | CRITICAL | Permanente | `watchdog_reboot()` |
| `nav_tick` idle > 30 ms (si permitido) | CRITICAL | Permanente | `watchdog_reboot()` |
| `wifi` idle > 200 ms (si poll permitido) | CRITICAL | Permanente | `watchdog_reboot()` |
| `loop` idle > 25 ms (si tick pendiente) | CRITICAL | Permanente | `watchdog_reboot()` |

**Limitación conocida:** `cyw43_arch_poll()` no tiene WCET acotado en SDK — P3 debe caracterizarlo antes de endurecer políticas Wi-Fi.

### Regla para agentes / desarrolladores

1. No añadir umbrales nuevos sin etiqueta `// SUPUESTO — pendiente P3`.
2. No afirmar "tiempo real duro" en rutas Wi-Fi hasta datos S7.
3. Toda nueva política: detección + clasificación (`SystemHealth`) + acción + métrica en `RuntimeHealth`.

---

## 🏁 2. Hitos Consolidados (Pico 2 Comarruga: `7f37724+`)

- [x] Rings UART SPSC, FSM I2C, ticks atómicos, WDT al final de ciclo
- [x] `health_monitor` + `RuntimeHealth` + políticas P1 (recuperable/permanente)
- [x] `TaskMonitor` + deadlines de starvation P2 (umbrales SUPUESTO)
- [x] Protocolo campaña WCET S0–S7 documentado (P3 — pendiente ejecución)
- [ ] Umbrales calibrados con datos de banco (P4)

### Hitos históricos (PC / Ambiq)

- [x] **Core matemático unificado**: Gestión de dominios (Tierra, Aire, Mar).
- [x] **Simulación de escenarios de estrés**: Pérdida de GPS por 10 s e inmersión submarina (presión).
- [x] **Drivers estructurales Ambiq**: Arquitectura de drivers HAL lista (simulada con stubs).
- [x] **Integración de automoción**: Ingestión de odometría de ruedas y lectura emulada de bus CAN con 5 ticks funcionales en limpio (`NaviCore3D_VehicleDemo` genera `HMI nav: lat=... lon=...`).
- [x] **Gemelo Digital 3D**: Visualizador gráfico operativo con `matplotlib` y `pandas`.

---

## 🛠️ 3. Próximos Pasos Inmediatos

> **Fuente de verdad del producto:** [`docs/ROADMAP_PNT_RESILIENCE.md`](docs/ROADMAP_PNT_RESILIENCE.md)  
> (PPK2 Pico → campo → Artemis · A3–A5 código · visibilidad con datos).  
> La campaña WCET S0–S7 (§1b P3) sigue siendo **hardware RT**, no el único “siguiente paso” del proyecto.

### FASE A: Robustez y Física Real (Corto Plazo)

- [x] **Lógica de marcha atrás (`reverse = true`)**: Modificar `fusion.cpp` para invertir la proyección geométrica del avance del coche cuando se activa la marcha atrás en la trama CAN.
- [x] **Ajuste dinámico de covarianza**: Incrementar los valores de la matriz de ruido del filtro inercial durante frenazos bruscos detectados por variaciones extremas en `accel_mps2[0]`.
- [x] **Inyección de fallos en simulador PC**: Implementada en `src/targets/generic_pc/sensors_sim.*` la estructura `SensorFaultInjection` con enumeración `SensorScenario` y bucle unificado `sensors_simulation_tick`. Soporta tres modos configurables desde `main.cpp`: escenario limpio (`SCENARIO_CLEAN`, sin anomalías), pérdida de GPS a partir del tick 3 (`SCENARIO_GPS_LOSS`, `fix_valid = false` y `satellites = 0`) y deriva acumulativa en la IMU (`SCENARIO_IMU_DRIFT`, bias creciente por tick en acelerómetro y giroscopio).

### FASE B: Hardware Real (Medio Plazo)

> **Target activo:** `src/targets/pico2_hardware/` — implementado y compilando; **validación en banco físico pendiente**. Diseño: [`docs/comarruga_lab_hardware.md`](docs/comarruga_lab_hardware.md). Fusión publicada hasta la fecha: trazas SensorLogger (móvil), no Pico encendido.

**Fase 1 — Banco de pruebas (mesa de laboratorio):**

- [x] Diseño + firmware: Raspberry Pi Pico 2 W + WT61C-232 (UART0) + NEO-M9N (UART1) + UPS Waveshare (I2C) — ver mapa de pines
- [ ] **Encendido físico** Pico 2 W + sensores cableados y telemetría CDC estable
- [x] Plan analizador lógico 8ch @ 24 MHz — GP22 (tick), GP21 (Wi-Fi poll) documentado

**Fase 2 — Endurecimiento RT (en curso):**

- [x] Rings UART SPSC, FSM I2C, `health_monitor`, `TaskMonitor`, campaña WCET documentada (S0–S7)
- [ ] **Campaña WCET en placa** (P3): caracterizar `cyw43_arch_poll()` y recalibrar umbrales (P4)
- [ ] **Telemetría UDP en vivo** desde `NaviCore3D_Pico2`

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
| `NaviCore3D_Pico2` | Pico 2 W Comarruga @ 100 Hz — build standalone en `src/targets/pico2_hardware/` |

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

---

## 🚧 6. Notas de Infraestructura y Bloqueos

El Core matemático en `src/core/` está restaurado y verificado en C++17. Sin embargo, las pruebas locales de los ejecutables (`NaviCore3D_VehicleDemo.exe`) quedan pausadas debido a un bloqueo estricto de directivas de Control de Aplicaciones de Windows (WDAC), requiriendo autorización de IT para la ruta `C:\NaviCore-3D\build\` o para el toolchain de MinGW antes de poder realizar pruebas dinámicas en local.
