# Laboratorio físico Comarruga — Hardware validado

> **Target CMake:** `src/targets/pico2_hardware/` → `NaviCore3D_Pico2`  
> **Placa:** `PICO_BOARD=pico2_w`  
> **Firmware de referencia:** commit `b8ca728` y posteriores

Este documento describe el hardware del banco Comarruga, la arquitectura de tiempo real del firmware y el **procedimiento de validación en banco** diseñado para medir WCET (Worst-Case Execution Time) con osciloscopio y telemetría USB.

---

## Bill of Materials (100 % validado en banco)

| Subsistema | Componente | Interfaz | Notas |
|------------|------------|----------|-------|
| **MCU** | Raspberry Pi Pico 2 W (RP2350) | USB stdio + CYW43439 Wi-Fi | Dual Cortex-M33 @ 150 MHz |
| **Energía** | Waveshare UPS Module | I2C1 @ 100 kHz, addr `0x43` | Celdas AVESO 14500; monitoreo de batería |
| **IMU / AHRS** | WitMotion WT61C-232 | UART0 @ 115200 8N1 | Filtro de Kalman integrado en el módulo |
| **GNSS** | u-blox NEO-M9N | UART1 @ 115200 8N1 | GPS+GLONASS+Galileo+BeiDou concurrentes |

### Instrumentación de banco

| Equipo | Uso |
|--------|-----|
| **Osciloscopio 2 canales** | GP22 (tick sensores) y GP21 (poll Wi-Fi) |
| **Analizador lógico** (opcional) | Verificación UART/I2C en arranque |
| **Terminal serie USB** | Métricas WCET y avisos `safe_log` |
| **Fuente / carga UART** (opcional) | Inyección de tráfico para provocar overflow |

---

## Mapa de pines (Pico 2 W)

| Función | Bus | GPIO TX / SDA | GPIO RX / SCL | Baud / reloj |
|---------|-----|---------------|---------------|--------------|
| WT61C-232 | UART0 | GP0 | GP1 | 115200 |
| NEO-M9N | UART1 | GP4 → RX GNSS | GP5 ← TX GNSS | 115200 |
| Waveshare UPS | I2C1 | GP6 (SDA) | GP7 (SCL) | 100 kHz @ `0x43` |
| **Benchmark tick** | GPIO salida | **GP22** | — | Pulso HIGH = `pico2_bsp_sensors_tick()` |
| **Benchmark Wi-Fi** | GPIO salida | **GP21** | — | Pulso HIGH = `cyw43_arch_poll()` |

Constantes en `src/targets/pico2_hardware/hw_config.hpp`:

```c
#define PICO2_GPIO_BENCHMARK         22U   /* GP22: sensors_tick */
#define PICO2_GPIO_BENCHMARK_WIFI    21U   /* GP21: cyw43_poll */
#define PICO2_LOOP_METRICS_REPORT_MS 1000U
```

---

## Arquitectura de tiempo real

### Flujo de datos

```
ISR UART (PL011) ──► ring SPSC 512 B ──► rx_pump() ──► parseo WT61C / NMEA
                                              │
tick @ 100 Hz ──► pico2_bsp_sensors_tick() ──► fusion (dead reckoning)
housekeeping() ──► pico2_bsp_power_poll() FSM ──► batería UPS (I2C cooperativo)
cyw43_arch_poll() ──► stack Wi-Fi (periférico blando)
```

### Bucle principal (`main.cpp`)

Cada iteración del `while(true)` ejecuta, en este orden:

1. `pico2_bsp_sensors_rx_pump()` — drena rings UART (hasta 8 rondas × 64 B/UART)
2. `watchdog_update()` — alimenta WDT de 50 ms
3. Si hay tick pendiente: **GP22 ↑** → `pico2_bsp_sensors_tick()` → **GP22 ↓**
4. `pico2_bsp_sensors_housekeeping()` — FSM I2C UPS (un paso por llamada)
5. **GP21 ↑** → `cyw43_arch_poll()` → **GP21 ↓**
6. `loop_metrics_on_loop_complete()` — mide duración total de la iteración
7. `loop_metrics_report_due()` — emite WCET por USB cada 1 s
8. `safe_log_flush_pending()` — vacía cola de log no bloqueante
9. `__wfi()` si no hay tick pendiente y los sensores permiten dormir

### Invariantes de diseño

| Invariante | Valor | Propósito |
|------------|-------|-----------|
| Tick de navegación | 10 ms (100 Hz) | Separación fusión / I/O |
| WDT | 50 ms | Reset si el bucle se bloquea |
| Timeout I2C por paso FSM | 2 ms | WCET acotado en `power_poll()` |
| Ring UART | 512 B (potencia de 2) | Margen entre ISR y `rx_pump()` |
| Presupuesto `rx_pump` | 64 B × 8 rondas/UART/loop | Hasta 512 B drenados por UART e iteración |

---

## Build, flash y conexión USB

```powershell
$env:PICO_SDK_PATH = 'C:\ruta\a\pico-sdk'
cmake -S src/targets/pico2_hardware -B build_pico2 -G Ninja
cmake --build build_pico2
```

Copia `wifi_config.h.example` → `wifi_config.h` con la red del laboratorio.

El firmware compila con **`-O3`** (peor caso de reordenación para los rings SPSC atómicos). USB stdio es **no bloqueante** (`PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0`).

### Terminal serie

- Puerto: dispositivo CDC USB de la Pico 2 W (115200 en muchos hosts; el firmware no depende de ese baud para las métricas internas).
- Abrir la terminal **antes o después** del flash; el arranque no espera conexión USB.
- Las métricas WCET y los avisos críticos de runtime usan `safe_logf()` (cola de 24 líneas, escritura CDC no bloqueante).

---

## Procedimiento de pruebas en banco

Validación en tres fases: **arranque**, **WCET con osciloscopio + USB**, y **degradación por overflow UART**.

### Fase 0 — Checklist de cableado (pre-encendido)

- [ ] WT61C TX → GP1 (UART0 RX), WT61C RX ← GP0
- [ ] NEO-M9N TX → GP5 (UART1 RX), GNSS RX ← GP4
- [ ] UPS SDA → GP6, SCL → GP7, GND común
- [ ] GP22 y GP21 accesibles para sonda de osciloscopio (massa común con la Pico)
- [ ] USB de datos conectado al host de telemetría

---

### Fase 1 — Arranque y subsistemas

Tras flash y reset, verificar en la terminal USB:

| Mensaje esperado | Significado |
|------------------|-------------|
| `Conectado. IP local: ...` | Wi-Fi asociado correctamente |
| `BSP Comarruga: WT61C @ UART0 115200 baud \| NEO-M9N @ UART1 115200 baud \| UPS I2C1` | BSP sensores inicializado |
| `NavigationCortex @ 100 Hz — WDT 50 ms — WCET GP22 tick GP21 wifi — hardware Comarruga validado` | Bucle de navegación activo |
| `WCET loop max_loop_time_us=...` (cada ~1 s) | Métricas de bucle en funcionamiento |

Con analizador lógico (opcional en esta fase):

- [ ] Tramas `0x55` del WT61C en GP1 @ 115200
- [ ] Líneas `$GNGGA` / `$GPGGA` en GP5 @ 115200
- [ ] ACK del UPS Waveshare en dirección `0x43` (GP6/GP7)

Criterio de paso: sin mensajes `Error:` de init; WDT sin reset espurio (la placa no reinicia en bucle).

---

### Fase 2 — WCET: osciloscopio en GP22 y GP21

Esta fase es el **gate principal de tiempo real**. Los pines de benchmark delimitan en hardware las secciones críticas; la telemetría USB reporta el WCET del bucle completo.

#### Conexión del osciloscopio

| Canal | Pin | Señal |
|-------|-----|-------|
| **CH1** | GP22 | Duración de `pico2_bsp_sensors_tick()` |
| **CH2** | GP21 | Duración de `cyw43_arch_poll()` |
| GND | GND Pico | Referencia común |

Configuración sugerida: 2 ms/div, trigger CH1 flanco ascendente, modo normal o persistencia limitada.

#### Qué mide cada pulso

**GP22 (tick sensores @ 100 Hz)**

- Se activa **solo cuando hay tick pendiente** (cada 10 ms en régimen estable).
- El pulso cubre: actualización de ventanas de overflow, fusión dead-reckoning, aplicación de confianza degradada.
- **No incluye** I2C UPS ni `rx_pump()` (eso ocurre fuera del pulso GP22).

Interpretación:

| Observación en CH1 | Diagnóstico |
|--------------------|-------------|
| Pulso periódico estable cada 10 ms | Tick @ 100 Hz correcto |
| Ancho de pulso ≪ 10 ms (típ. sub-ms a pocos ms) | `sensors_tick()` dentro de presupuesto |
| Ancho de pulso > 5 ms | Revisar carga de fusión o frecuencia efectiva de tick |
| Pulso ausente o irregular | Problema de timer, bloqueo del bucle o WDT reset |

**GP21 (poll Wi-Fi)**

- Se activa **en cada iteración** del `while(true)`, con o sin tick.
- El pulso cubre el trabajo del stack CYW43 en modo poll (sin bloqueo SDK).
- Su ancho es **variable**: depende del tráfico Wi-Fi, retransmisiones y estado de la interfaz.

Interpretación:

| Observación en CH2 | Diagnóstico |
|--------------------|-------------|
| Pulsos frecuentes (mucho más densos que CH1) | Normal: una iteración de poll por vuelta de bucle |
| Ancho ocasionalmente mayor | Tráfico Wi-Fi o eventos de red |
| Ancho sostenido alto en todas las iteraciones | Posible contención con el resto del bucle; correlacionar con `max_loop_time_us` |

#### Correlación con métricas USB

Cada **1000 ms** el firmware emite por `safe_log`:

```
WCET loop max_loop_time_us=<N> (ventana 1000 ms)
```

| Campo | Significado |
|-------|-------------|
| `max_loop_time_us` | **Mayor duración observada del bucle completo** en la ventana de 1 s |
| Ventana 1000 ms | Se reinicia el máximo tras cada reporte; no es un promedio |

El bucle medido incluye, en orden: `rx_pump` + `watchdog_update` + (opcional) `sensors_tick` + `housekeeping` + `cyw43_arch_poll` + overhead de métricas/log.

**Relación osciloscopio ↔ USB:**

```
max_loop_time_us  ≥  ancho_pulso_GP22  (en la misma iteración con tick)
max_loop_time_us  ≥  ancho_pulso_GP21  (siempre)
max_loop_time_us  ≈  suma de tramos + rx_pump + housekeeping + overhead
```

La API `loop_metrics_max_loop_time_us()` conserva el **máximo desde arranque** (útil para depuración; el reporte periódico usa el máximo por ventana).

#### Umbrales de aceptación WCET

Referencia: WDT = **50 ms** → el bucle debe permanecer muy por debajo de ese límite.

| Métrica | Condición nominal | Límite de alerta | Acción si se supera |
|---------|-------------------|------------------|---------------------|
| Pulso GP22 | < 2 ms | > 5 ms | Perfil fusión / carga CPU en tick |
| Pulso GP21 (pico) | < 3 ms | > 10 ms sostenido | Revisar tráfico Wi-Fi o prioridad del bucle |
| `max_loop_time_us` (USB) | < 5 ms | > 25 ms | Investigar bloqueo I2C, UART o poll Wi-Fi |
| WDT reset | ninguno | cualquier reset | `max_loop_time_us` se acercó a 50 ms o hay bloqueo |

> **Nota:** 25 ms es el 50 % del WDT — margen de seguridad antes de un reset. En operación nominal con sensores conectados y Wi-Fi asociado, se espera `max_loop_time_us` en el rango de **centenas de µs a pocos ms**.

#### Procedimiento paso a paso (WCET)

1. Flash del firmware de banco y apertura de terminal USB.
2. Conectar sondas en GP22 (CH1) y GP21 (CH2); masa común.
3. Dejar correr **≥ 60 s** en condiciones nominales (IMU + GNSS + Wi-Fi activos).
4. Anotar el valor **máximo** de `max_loop_time_us` observado en la ventana.
5. Medir en osciloscopio: ancho máximo de CH1 y CH2 en el mismo intervalo.
6. Verificar: `max_loop_time_us` por encima de los anchos medidos de GP22/GP21, y por debajo de 25 ms.
7. (Opcional) Repetir con tráfico Wi-Fi adicional (ping continuo al host) y comparar incremento de GP21 y `max_loop_time_us`.

**Criterio de paso Fase 2:** sin resets WDT; `max_loop_time_us` estable por debajo de 25 ms; GP22 periódico @ 10 ms con ancho < 5 ms.

---

### Fase 3 — Overflow UART y degradación de confianza

Los rings de recepción UART son buffers SPSC de **512 bytes** (`bsp_uart_rx_ring.hpp`). La ISR vacía la FIFO hardware (32 B) hacia el ring; si el ring está lleno, el byte se descarta e incrementa `overflow_count`.

#### Mecanismo de overflow

```
ISR UART ──push()──► ring 512 B
                         │
              si lleno: overflow_count++
                         │
rx_pump() ◄──pop()───────┘
```

Contadores expuestos:

- `pico2_bsp_uart_get_overflow_count(0)` — UART0 / IMU
- `pico2_bsp_uart_get_overflow_count(1)` — UART1 / GNSS

#### Ventana y umbrales de degradación

Definidos en `hw_config.hpp`:

```c
#define PICO2_RING_OVERFLOW_WINDOW_MS         1000U
#define PICO2_RING_OVERFLOW_DEGRADE_THRESHOLD 3U
#define PICO2_RING_DEGRADED_QUALITY_FACTOR    0.50f
```

| Parámetro | Valor | Efecto |
|-----------|-------|--------|
| Ventana de evaluación | 1000 ms | Se cuentan eventos de overflow por UART en cada segundo de navegación |
| Umbral de degradación | **≥ 3 overflows** en la ventana | Activa bandera de confianza degradada para ese UART |
| Factor de calidad | × 0.5 por subsistema degradado | Reduce `estimate_quality` en la fusión |
| Fin de ventana | Cada 1000 ms | Se reinicia el contador de eventos y **se limpia** la degradación si no hubo nuevos overflows |

#### Efecto en la fusión (`bsp_sensors.cpp`)

| Bandera | Condición | Acción en `DeadReckoningFilter` |
|---------|-----------|--------------------------------|
| `imu_degraded` | ≥ 3 overflows IMU en 1 s | `estimate_quality × 0.5` |
| `gnss_degraded` | ≥ 3 overflows GNSS en 1 s | `gps_trusted = false` y `estimate_quality × 0.5` |

Si **ambos** subsistemas están degradados en la misma ventana, el factor se aplica **secuencialmente**: `quality × 0.5 × 0.5 = 0.25` (con clamp a `[0, 1]`).

Las banderas son consultables en runtime:

```c
SensorConfidenceFlags flags;
pico2_bsp_sensors_get_confidence_flags(&flags);
// flags.imu_degraded, flags.gnss_degraded
```

#### Cómo provocar overflow en banco

El objetivo es saturar el productor (ISR) sin que `rx_pump()` drene a tiempo:

1. **Tráfico UART alto sostenido** — IMU WT61C ya emite a 115200; combinar con GNSS a máxima cadencia NMEA.
2. **Bloqueo artificial del consumidor** — temporalmente impedir que el bucle principal progrese (solo en banco, p. ej. breakpoint o carga host en depuración).
3. **Inyección externa** — generador UART o segundo MCU enviando bytes continuos a GP1 o GP5 a 115200.

Estimación de orden de magnitud: a 115200 baud (~11,5 kiB/s), llenar 512 B tarda ~45 ms si no hay drenaje. Con `rx_pump` activo (hasta 512 B/loop), hacen falta **ráfagas sostenidas** o bloqueo del bucle para acumular ≥ 3 overflows en 1 s.

#### Procedimiento paso a paso (overflow)

1. Operación nominal 60 s → verificar **ausencia** de degradación (contadores de overflow estables en host test; en placa, sin síntomas de `gps_trusted` forzado a falso sin causa RF).
2. Aplicar estímulo de saturación UART (método 1 o 3) durante **≥ 5 s**.
3. Observar en telemetría / depuración:
   - Incremento de `overflow_count` en IMU y/o GNSS
   - Tras ≥ 3 eventos en 1 s: `imu_degraded` y/o `gnss_degraded` activos
4. Confirmar efecto funcional: menor `estimate_quality`; si GNSS degradado, `gps_trusted == false`.
5. Retirar estímulo y esperar **> 1 s** (nueva ventana) → las banderas deben **desactivarse** si no hay nuevos overflows.
6. Verificar recuperación de confianza en fusión.

**Criterio de paso Fase 3:**

- [ ] Con tráfico nominal: 0 degradaciones en 60 s
- [ ] Con saturación: degradación activada tras ≥ 3 overflows/s en el UART afectado
- [ ] Tras cesar saturación: degradación se limpia al cerrar la ventana de 1 s sin nuevos overflows
- [ ] Sin reset WDT durante la prueba

#### Test de regresión en host (previo a banco)

Antes del ensayo en placa, compilar y ejecutar la simulación cooperativa:

```powershell
# Desde el directorio del target; requiere toolchain C++17 en el host
g++ -std=c++17 -O3 -o ring_stress_host_test ring_stress_host_test.cpp
.\ring_stress_host_test.exe
```

Salida esperada: `OK: overflow_count == 0 en ambos rings tras 60 s` con tráfico dual @ 115200 y bloqueos I2C simulados de 2 ms cada 100 ms.

> El test en host valida la corrección del ring bajo `-O3`; **no sustituye** la Fase 3 en hardware con UART física.

---

## Resumen de criterios de aceptación (gate de banco)

| # | Prueba | Criterio |
|---|--------|----------|
| 1 | Arranque BSP | Mensajes de init correctos, sin error fatal |
| 2 | UART / I2C | Tráfico visible en pines de datos (lógico o terminal) |
| 3 | GP22 @ 100 Hz | Pulso cada 10 ms, ancho < 5 ms |
| 4 | GP21 poll Wi-Fi | Pulsos por iteración; picos < 10 ms en régimen nominal |
| 5 | USB WCET | `max_loop_time_us` < 25 ms sostenido, sin WDT reset |
| 6 | Overflow UART | Degradación tras ≥ 3 overflows/s; recuperación tras ventana limpia |
| 7 | Host stress test | `overflow_count == 0` @ 60 s (`ring_stress_host_test`) |

---

## Referencia rápida de archivos

| Archivo | Rol |
|---------|-----|
| `hw_config.hpp` | Pines, umbrales, invariantes WDT/UART/I2C |
| `main.cpp` | Bucle principal, GPIO benchmark, WDT |
| `loop_metrics.cpp` | `max_loop_time_us` y reporte periódico |
| `safe_log.cpp` | Log no bloqueante por USB CDC |
| `bsp_uart_rx_ring.hpp` | Ring SPSC atómico + `overflow_count` |
| `bsp_sensors.cpp` | Ventana de overflow y degradación de confianza |
| `ring_stress_host_test.cpp` | Simulación de estrés 60 s en host |

---

## Fusión y telemetría (próxima fase)

- Transición `INITIALIZING` → `HYBRID` con fix GNSS activo (validación funcional adicional).
- Telemetría UDP hacia el host del laboratorio (`HOST_IP`, `UDP_PORT` en `wifi_config.h`).
