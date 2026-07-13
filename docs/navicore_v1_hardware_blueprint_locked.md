# NaviCore-3D · Hardware Blueprint v1 (LOCKED)

> **Tag:** `navicore_v1_hardware_blueprint_locked`  
> **Estado:** **ARCHIVADO** — blueprint histórico Ambiq Apollo4. No modificar (LOCKED).  
> **Target activo:** `src/targets/pico2_hardware/` → [`docs/comarruga_lab_hardware.md`](comarruga_lab_hardware.md)  
> **Targets eliminados:** `src/targets/ambiq_apollo/`, `src/targets/pico_w/` (prototipo Pico 1 W)  
> **Prerrequisito de software (histórico):** Core zero-heap, alineación SRAM 32 bits (Fase Y), buffer circular estático (Fase X) y capa `NaviCore3D_Ambiq` con stubs host verificados.

---

## Fase 1 — Banco de pruebas físico (hardware inicial)

Componentes mínimos para que el firmware bare-metal interactúe con el mundo real.

### 1. Placa de desarrollo base

| Campo | Detalle |
|-------|---------|
| **Modelo recomendado** | Ambiq Apollo4 Plus EVB o Apollo4 Blue Lite EVB |
| **Función** | Aloja el Cortex-M4F de ultra-bajo consumo. Target de build `NaviCore3D_Ambiq`. La SRAM ejecuta la alineación estricta de 32 bits optimizada en la Fase Y. |
| **Firmware** | `src/targets/ambiq_apollo/main_ambiq.cpp` — superloop determinista @ 100 ms |

### 2. Sonda de depuración (hardware debugger)

| Campo | Detalle |
|-------|---------|
| **Modelo recomendado** | Segger J-Link Edu Mini o J-Link Base |
| **Función** | Conexión SWD (Serial Wire Debug) → USB. Flasheo del binario y depuración paso a paso sobre registros físicos reales. |
| **Flujo** | IDE / OpenOCD / Segger Ozone → `NaviCore3D_Ambiq.elf` → silicio Apollo4 |

### 3. Suite de sensores inerciales y satelitales

#### IMU (Unidad de Medición Inercial)

| Campo | Detalle |
|-------|---------|
| **Modelo recomendado** | Placa compacta con InvenSense **MPU-9250** o Bosch **BMX160** |
| **Bus** | SPI full-duplex @ **1 MHz** en prototipo (seguro para breadboard) |
| **Driver preparado** | `src/targets/ambiq_apollo/ambiq_iom_master.*` + `drivers/ambiq_spi_imu_stub.cpp` |
| **Salida** | 6 ejes: aceleración (m/s²) + velocidad angular (rad/s) en ráfaga DMA atómica |

#### GNSS (GPS)

| Campo | Detalle |
|-------|---------|
| **Modelo recomendado** | u-blox **NEO-M8N** o **NEO-M9N** con antena cerámica integrada |
| **Alimentación** | 3.3 V |
| **Interfaz** | UART TX del módulo → **UART0 RX** del Ambiq |
| **Velocidad inicial** | **9600 baud** (arranque estándar u-blox; reconfigurable a 115200 en campo) |
| **Formato** | Tramas NMEA (`$GNGGA`, …) en ráfaga continua tras fix de satélites |

### 4. Conectividad y diagnóstico de señal

| Componente | Función |
|------------|---------|
| **Breadboard + cables Dupont** (M-M / M-H) | Entrelazar sensores con buses del Ambiq sin soldador |
| **Analizador lógico USB 8ch @ 24 MHz** | Captura en paralelo de líneas SPI y UART; verificación de tramas binarias y ruido eléctrico en PC |

---

## Fase 2 — Firmware bare-metal (siguiente paso en Cursor)

Sustituir stubs de simulación por código que manipula el silicio directamente.

### Flujo de datos objetivo (GNSS → Kalman)

```
[Módulo GPS u-blox]  ──(NMEA por UART)──► [ISR UART0 + DMA] ──► [Buffer circular estático]
                                                                        │
                                                                 (Throttling O(1))
                                                                        ▼
[Filtro de Kalman]  ◄── (metros locales) ◄── [Parser NMEA]  ◄── [command_ingestor / ingest]
```

### Tareas de implementación

| # | Módulo | Acción | Archivos objetivo |
|---|--------|--------|-------------------|
| 1 | **IOM SPI Master** | Configurar registros del periférico Ambiq Apollo: reloj SPI **1 MHz**, DMA habilitado, lectura IMU 6 ejes en una sola ráfaga atómica sin bloquear el superloop | `ambiq_iom_master.cpp`, `drivers/ambiq_spi_imu_stub.cpp`, `ambiq_driver_config.hpp` |
| 2 | **UART0 GNSS RX** | Inicializar UART @ **9600 baud**, acoplar ISR al buffer circular estático de **128 paquetes** (política **drop-oldest** ya blindada) | Nuevo: `ambiq_uart_gnss_rx.*`; patrón en `command_ingestor.*` |
| 3 | **Parser NMEA estático** | Extraer `$GNGGA`, validar checksum XOR, convertir lat/lon → metros locales float; **cero heap** | Nuevo: `nmea_parser.*` en `src/core/` o `ambiq_apollo/` |
| 4 | **Integración fusion** | Alimentar `GNSSMeasurement` vía `api_ingest` → `fusion.cpp` / Filtro de Kalman | `api_ingest.*`, `fusion.*`, `bsp_sensors.cpp` |

### Parámetros de referencia (prototipo v1)

| Parámetro | Valor prototipo | Notas |
|-----------|-----------------|-------|
| Tick de navegación | 100 ms (10 Hz) | `AMBIQ_TICK_INTERVAL_MS` |
| SPI IMU (lab) | 1 MHz | Subir a 24 MHz en PCB final (`AMBIQ_SPI_IMU_HZ`) |
| UART GNSS (boot) | 9600 8N1 | Estándar u-blox factory |
| UART telemetría TX | 115200 8N1 | `AMBIQ_UART_TELEM_BAUD` — salida CSV/debug |
| Buffer RX | 128 slots estáticos | `COMMAND_INGESTOR_HW_RX_CAPACITY` |
| Política de overflow | Drop-oldest | O(1), sin malloc |

### Criterios de aceptación en laboratorio

- [ ] `cmake --build build --target NaviCore3D_Ambiq` enlaza sin stubs en rutas críticas IMU/GNSS.
- [ ] Analizador lógico: burst SPI IMU limpio (12 B) @ 1 MHz sin glitches.
- [ ] UART GNSS: tramas `$GNGGA` visibles; parser rechaza checksum inválido.
- [ ] Superloop 100 ms mantiene WCET estable con IMU DMA + ingest GNSS concurrente.
- [ ] Telemetría UART TX o volcado SWD confirma transición `INITIALIZING` → `GPS` / `HYBRID`.

---

## Mapa código ↔ silicio

```
src/targets/ambiq_apollo/
├── main_ambiq.cpp              # Superloop 100 ms + deep sleep
├── ambiq_iom_master.*          # SPI master (IMU)
├── ambiq_imu_driver.*          # Burst read → ImuSample
├── ambiq_uart_telemetry.*      # UART0 TX telemetría
├── ambiq_hardware_timer.*      # STimer @ 32.768 kHz
├── bsp_sensors.*               # Orquestación HAL sensores
├── power_state_machine.*       # Periféricos + deep sleep
└── drivers/
    ├── ambiq_spi_imu_stub.cpp  # → reemplazar por am_hal_iom_*
    ├── ambiq_gpio_gnss_stub.cpp
    ├── ambiq_dma_stub.cpp
    └── ambiq_driver_config.hpp # Pines, baudios, timeouts
```

---

## Orden de trabajo recomendado

1. Montar banco Fase 1 y verificar alimentación 3.3 V + SWD.
2. Flashear `NaviCore3D_Ambiq` actual (stubs) — confirmar tick 100 ms por GPIO/LED.
3. Activar SPI IMU real (tarea 1) + analizador lógico.
4. Activar UART GNSS + parser NMEA (tareas 2–3).
5. Cerrar lazo con `fusion` y comparar contra `NaviCore3D_Sim` / CSV black-box.

---

*Documento sellado como hoja de ruta oficial de hardware para NaviCore-3D v1. Cuando los componentes estén sobre la mesa de laboratorio, continuar con drivers bare-metal en Cursor.*

**Autor:** Juan Carlos Pulido Mellado · **Proyecto:** NaviCore-3D
