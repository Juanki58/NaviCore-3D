# Laboratorio físico Comarruga — Hardware validado

> **Target CMake:** `src/targets/pico2_hardware/` → `NaviCore3D_Pico2`  
> **Placa:** `PICO_BOARD=pico2_w`  
> **Firmware de referencia:** commit `fc28d70` y posteriores (`RuntimeHealth`, `health_monitor`, `SystemHealth`)

Este documento describe el hardware del banco Comarruga, la arquitectura de tiempo real del firmware, el **procedimiento de validación en banco** diseñado para medir WCET (Worst-Case Execution Time) con osciloscopio y telemetría USB, y el **plan estratégico de desarrollo** del NaviCore Runtime (horizonte 12 meses).

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
2. Si hay tick pendiente: **GP22 ↑** → `pico2_bsp_sensors_tick()` → **GP22 ↓** (omitido si `max_tick_backlog > 2`)
3. `pico2_bsp_sensors_housekeeping()` — FSM I2C UPS (un paso por llamada)
4. **GP21 ↑** → `cyw43_arch_poll()` → **GP21 ↓** (omitido si sin presupuesto o `health_monitor` deshabilitó Wi-Fi)
5. `loop_metrics_sync_uart_overflows()` + `safe_log_flush_pending()` (máx. 256 B/ciclo)
6. `loop_metrics_on_loop_complete()` + `health_monitor_on_loop_complete()`
7. `watchdog_update()` — alimenta WDT de 50 ms **solo si el ciclo terminó**
8. `__wfi()` si no hay tick pendiente y los sensores permiten dormir

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

### Fase 2 — WCET: osciloscopio (smoke test rápido)

Validación rápida previa a la campaña formal. Ver **Prioridad 2** para la demostración exhaustiva de WCET.

1. Flash, terminal USB, sondas GP22 (CH1) y GP21 (CH2).
2. Correr **≥ 60 s** en nominal.
3. Verificar: GP22 @ 10 ms, sin WDT reset, `max_loop_us` < 25 ms (lectura vía depurador, ver abajo).

---

## Prioridad 2 — Campaña WCET (demostración de presupuesto temporal)

> **Objetivo:** no comprobar que “funciona en nominal”, sino **intentar romper el presupuesto temporal** bajo peor caso reproducible.  
> **No basta** con mirar `max_wifi_us` aislado: el WCET del lazo es multi-variable.

### Métricas obligatorias (`RuntimeHealth`)

Leer al final de **cada escenario** (máximos desde arranque o tras `loop_metrics_init()` al inicio del escenario):

| Campo | Qué acota | Correlación osciloscopio |
|-------|-----------|--------------------------|
| `max_tick_us` | WCET de `pico2_bsp_sensors_tick()` | Ancho pulso **GP22** |
| `max_wifi_us` | WCET de `cyw43_arch_poll()` | Ancho pulso **GP21** |
| `max_logging_us` | WCET de `safe_log_flush_pending()` | — |
| `max_loop_us` | WCET del ciclo completo | Suma inferior a periodo 10 ms |
| `max_tick_backlog` | Peor retraso de ticks (`pending - 1`) | Huecos GP22 > 10 ms |

Métricas de contexto (no sustituyen las anteriores): `loop_budget_exceeded`, `wifi_skipped_budget`, `uart0/1_overflows`, `i2c_recoveries`, `SystemHealth`.

### Presupuestos de referencia (`hw_config.hpp`)

| Umbral | Valor | Significado |
|--------|-------|-------------|
| `PICO2_LOOP_BUDGET_US` | 10 000 µs | Periodo nominal @ 100 Hz |
| `PICO2_LOOP_DEGRADED_US` | 8 000 µs | Degradación de ritmo |
| `PICO2_LOOP_RESTART_US` | 20 000 µs | 3 ciclos → reinicio controlado |
| `PICO2_LOOP_CRITICAL_US` | 25 000 µs | 50 % del WDT (50 ms) |
| `PICO2_WDT_TIMEOUT_MS` | 50 ms | Techo absoluto (bloqueo) |

**Criterio de ruptura de presupuesto:** cualquier escenario donde `max_loop_us > PICO2_LOOP_BUDGET_US` de forma sostenida, o `max_tick_backlog > 2`, o activación de `health_monitor` / `SystemHealth::CRITICAL`.

### Lectura de métricas en banco

El firmware **no imprime** `RuntimeHealth` en caliente (evita bloquear USB). Opciones:

**A — Depurador (recomendado en campaña)**

```text
# OpenOCD + GDB, halt al final del escenario:
(gdb) p *loop_metrics_health()
(gdb) p health_monitor_system_health()
(gdb) p *health_monitor_runtime()
(gdb) p *task_monitor_get((TaskId)1)   /* NavTick: last_execution_tick, executions */
```

**B — Reset de métricas por escenario**

Añadir breakpoint al inicio de cada escenario y llamar `loop_metrics_init()` solo en banco (o reiniciar placa entre escenarios S0…S7).

**C — Correlación obligatoria con osciloscopio**

Para cada escenario, capturar **captura de pantalla** del osciloscopio (GP22/GP21) **y** volcado GDB de `RuntimeHealth`. Sin ambos, el escenario no es válido para certificación.

### Matriz de escenarios

Ejecutar en orden. Duración mínima por escenario: **120 s** (WCET es evento raro). Anotar temperatura ambiente y tensión UPS.

| ID | Escenario | Estímulo | Objetivo de estrés |
|----|-----------|----------|-------------------|
| **S0** | Baseline | Todo conectado, Wi-Fi asociado, sin tráfico host | Referencia |
| **S1** | Wi-Fi inactivo | Asociado a AP; host sin ping/UDP/iperf; verificar `wifi_skipped` bajo | Aislar carga radio idle |
| **S2** | UDP continuo | `iperf3 -u -b 5M -t 120` o script Python → `HOST_IP:UDP_PORT` @ ≥ 100 Hz | Saturar `cyw43_arch_poll` |
| **S3** | Pérdida de paquetes | `tc netem loss 30%` en AP o router entre Pico y host | WCET Wi-Fi con retransmisiones |
| **S4** | GNSS multi-constelación | u-blox: GPS+GLO+GAL+BDS, NMEA a máxima cadencia (UBX-CFG-RATE) | Saturar UART1 / parser |
| **S5** | IMU máxima frecuencia | WT61C: 115200, salida acel+giro a máx. Hz (software WitMotion) | Saturar UART0 / parser |
| **S6** | I²C recovery | Glitch SDA (pull-down 1 ms cada 2 s) o bloqueo SDA breve | FSM `power_poll` + recovery |
| **S7** | **Todo simultáneo** | S2+S3+S4+S5+S6 activos a la vez | **Peor caso compuesto** |

### Procedimiento por escenario

1. **Preparación**
   - [ ] Flash `NaviCore3D_Pico2` (`fc28d70+`)
   - [ ] `wifi_config.h` con IP host del banco
   - [ ] Osciloscopio: CH1=GP22, CH2=GP21, 2 ms/div, trigger CH1
   - [ ] GDB/OpenOCD conectado (lectura sin parar el lazo, o halt a los 120 s)

2. **Inicio de escenario**
   - Reiniciar placa **o** `loop_metrics_init()` + `health_monitor_init()` (solo banco)
   - Aplicar estímulo del escenario
   - Cronometrar **120 s**

3. **Durante el escenario**
   - Observar: resets WDT, mensajes `FT:` por USB, `SystemHealth`
   - No interrumpir salvo reset espurio (anotar)

4. **Fin de escenario**
   - Halt GDB → volcar `*loop_metrics_health()`
   - Captura osciloscopio (máximos anchos GP22/GP21)
   - Completar fila en plantilla de registro

5. **Criterio de fallo del escenario**
   - `max_loop_us` > 10 000 µs **y** `loop_budget_exceeded` > 5 en la ventana
   - `max_tick_backlog` > 2
   - `SystemHealth::CRITICAL` o reinicio controlado
   - Cualquier reset WDT no provocado

### Configuración de estímulos (notas de banco)

**S1 — Wi-Fi inactivo**

- Pico asociada al AP; host en la misma subred pero **sin** tráfico (cerrar `iperf`, ping, visualizador UDP).
- Esperado: `max_wifi_us` bajo; útil como contraste para S2/S3.

**S2 — UDP continuo**

```powershell
# Host Windows/Linux — adaptar IP y puerto de wifi_config.h
python tools/telemetry_receiver.py   # en una terminal
# Emisor: NaviCore cuando UDP esté activo, o iperf:
iperf3 -u -c <IP_PICO> -p 5005 -b 2M -t 120
```

**S3 — Pérdida de paquetes**

```bash
# Linux en el router/AP o PC puente:
sudo tc qdisc add dev wlan0 root netem loss 30% delay 5ms
# Retirar tras S3:
sudo tc qdisc del dev wlan0 root
```

**S4 — GNSS todas las constelaciones**

- u-center / u-blox NEO-M9N: habilitar GPS+GLONASS+Galileo+BeiDou; rate 10 Hz NMEA si el módulo lo permite.
- Verificar en analizador lógico GP5: ráfagas `$GNGGA`/`$GNRMC` continuas @ 115200.

**S5 — IMU máxima frecuencia**

- Configurar WT61C-232 vía software WitMotion: salida acelerómetro + giroscopio a máxima cadencia @ 115200.
- Verificar GP1: tramas `0x55` continuas sin huecos > 5 ms.

**S6 — I²C recovery**

- **Método seguro:** desconectar momentáneamente pull-up SDA (≤ 10 ms) cada 2 s con relay bajo control del operador.
- **Método alternativo:** fuente de glitch en GP6 con resistencia serie 1 kΩ (solo con UPS sin carga crítica).
- Esperado: `i2c_recoveries` incrementa; observar `max_housekeeping_us` y FSM en `safe_log`.

**S7 — Todo simultáneo**

- Activar S2+S3+S4+S5+S6 juntos durante 120 s.
- Este escenario define el **WCET compuesto** del programa. Si solo un escenario puede romper el presupuesto, debe ser este.

### Plantilla de registro

Copiar una fila por escenario; archivar en `docs/banco_wcet_YYYYMMDD.csv` o cuaderno de laboratorio.

```csv
escenario,duracion_s,max_tick_us,max_wifi_us,max_logging_us,max_loop_us,max_tick_backlog,loop_budget_exceeded,wifi_skipped,uart0_ovf,uart1_ovf,i2c_recov,system_health,wdt_reset,notas
S0,120,,,,,,,,,,,,,
S1,120,,,,,,,,,,,,,
...
S7,120,,,,,,,,,,,,,
```

`system_health`: 0=NOMINAL, 1=DEGRADED, 2=CRITICAL.

### Análisis post-campaña

1. **Gráfico de barras** por escenario: `max_loop_us`, `max_tick_us`, `max_wifi_us`, `max_logging_us`.
2. Identificar **dominante**: si S7 ≈ suma de parciales, los subsistemas son aditivos; si S7 >> suma, hay contención no lineal (priorizar).
3. Comparar `max_wifi_us` (GDB) con ancho máximo GP21 (osciloscopio); discrepancia > 10 % → revisar medición.
4. Documentar **margen al WDT:** `50000 - max_loop_us` en S7.
5. Conclusión binaria: ¿el presupuesto de 10 ms se mantiene en S7 bajo peor caso? Si no, enumerar subsistema dominante y acción de `health_monitor` observada.

### Gate de salida Prioridad 2

| # | Criterio |
|---|----------|
| 1 | S0–S7 ejecutados 120 s cada uno con registro completo |
| 2 | Cada escenario tiene captura osciloscopio + volcado `RuntimeHealth` |
| 3 | S7 documentado con margen WDT y subsistema dominante |
| 4 | Si `max_loop_us` > 10 ms en S7: informe de ruptura con causa raíz (no bloquear sin evidencia) |

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
| 5 | Campaña WCET S0–S7 | Registro completo + S7 con margen WDT documentado |
| 6 | Overflow UART | Degradación tras ≥ 3 overflows/s; recuperación tras ventana limpia |
| 7 | Host stress test | `overflow_count == 0` @ 60 s (`ring_stress_host_test`) |

---

## Plan estratégico de desarrollo — NaviCore Runtime

> **Horizonte:** 12 meses · **Estado del chasis físico:** estable (BSP Comarruga, bucle @ 100 Hz, `RuntimeHealth`, campaña WCET S0–S7).  
> **Objetivo:** construir y demostrar la Propiedad Intelectual del **NaviCore Runtime** — motor de navegación OEM portátil — independiente de la plataforma de referencia.

### 1. Filosofía de producto: OEM Navigation Runtime

La **Raspberry Pi Pico 2 W (RP2350)** es la plataforma de referencia y desarrollo de bajo coste del laboratorio Comarruga: valida el chasis defensivo en hardware real, acota el WCET y expone los puntos de fallo del bus (UART, I2C, Wi-Fi). **No es el producto.**

El producto es **NaviCore Runtime**: un motor de navegación **determinista**, **portátil** y **agnóstico de plataforma**, compilable hacia:

| Target | Rol |
|--------|-----|
| **RP2350** (`pico2_hardware`) | Plataforma de referencia y banco WCET |
| **STM32H7 / STM32U5** | Integración OEM en vehículos y drones |
| **NXP** | Variantes industriales y automoción |
| **Linux** | Gateway de misión y telemetría |
| **Simulador host** (`generic_pc`) | SIL, replay y acumulación de horas virtuales |

La separación `src/core/` ↔ `src/targets/` materializa este contrato: el núcleo matemático y las políticas de navegación viven en **core**; los drivers, el scheduler y el I/O viven en **targets**.

El valor diferencial no reside en un pinout concreto, sino en:

- **Chasis defensivo** — guardas de geometría, divergencia, recuperación y `SystemHealth` con degradación activa.
- **Runtime síncrono** — bucle fijo @ 100 Hz, presupuesto temporal explícito (`PICO2_LOOP_BUDGET_US`), WDT y `RuntimeHealth` por bloque.
- **Gestión de sensores con tolerancia a fallos** — rings SPSC, contadores de overflow UART, degradación de confianza por ventana temporal.
- **Políticas de degradación activa** — de `NOMINAL` a `DEGRADED` / `CRITICAL` con acciones medibles (Wi-Fi omitido, `gps_trusted` retirado, transición a `SAFE_MODE`).

La Pico 2 W es el **chasis estable** sobre el que se endurece y certifica el runtime; el entregable comercial es el **motor**, no la placa.

---

### 2. Plan estratégico a 12 meses (por hitos)

#### Trimestre 1 — El cerebro estimador (INS/EKF profesional)

Objetivo: sustituir la fusión heurística por un estimador de estados con garantías estadísticas y comportamiento predecible bajo pérdida de GNSS.

| Entregable | Descripción |
|------------|-------------|
| **ESKF 15 estados** | Posición NED, velocidad, actitud (roll/pitch/yaw), bias de acelerómetro (3) y bias de giroscopio (3). Matrices fijas, aritmética `float`/FPU, zero-heap. |
| **Propagación @ 100 Hz** | Mecánica strapdown pura gobernada por la IMU; `predict()` síncrono en cada tick de navegación. |
| **Integridad GNSS** | Innovación normalizada (NIS) y distancia de Mahalanobis para rechazo estricto de outliers; contadores `gnss_accept` / `gnss_reject`. |
| **Pérdida y recuperación de fix** | Dead Reckoning consistente cuando el GNSS degrada o se rechaza; transición controlada `HYBRID` ↔ `DEAD_RECKONING` con `estimate_quality` acotada. |

**Criterio de cierre T1:** EKF integrado en banco Comarruga y en simulador; NIS y bias visibles en telemetría; sin divergencia en escenario nominal de 60 s.

---

#### Trimestre 2 — El multiplicador de fuerza (simulador SIL avanzado y replay)

Objetivo: desacoplar el ritmo de desarrollo del hardware físico; acumular evidencia estadística a velocidad acelerada.

| Entregable | Descripción |
|------------|-------------|
| **SIL host** | Evolución de `generic_pc` hacia entorno Software-in-the-Loop con escenarios reproducibles (`MISION_CLEAN`, stress, fault injection) y puente JSBSim documentado en `docs/sil_architecture.md`. |
| **Replay de logs** | `NaviCore3D_Replay`: alimentación offline del EKF con archivos de campo (UART/flash/USB) para regresión determinista sin placa. |
| **Caja negra expandida** | `FlightRecorder` (`src/core/flight_recorder.*`): volcado síncrono en CSV de los 15 estados del EKF, √diag(P), sesgos estimados, innovación/NIS residual y métricas WCET (`loop_us`, `missed_ticks`). |

**Criterio de cierre T2:** 100 h de vuelo virtual acumulables en < 1 h de CPU host; replay bit-a-bit del EKF sobre log de banco; CSV con esquema v1 congelado.

---

#### Trimestre 3 — El piloto táctico (guiado 3D y planificador de misión)

Objetivo: cerrar el lazo de control de misión con leyes de guiado acotadas en WCET y máquina de estados jerárquica.

| Entregable | Descripción |
|------------|-------------|
| **Guiado 3D** | Pure Pursuit + Line-of-Sight: cross-track por proyección lineal (sin producto vectorial redundante), rumbo objetivo, velocidad variable por \|e_xt\| y tasa de ascenso deseada. Cache de tramo: ≤ 2 `sqrtf`/tick. |
| **Mission HFSM** | `INIT` → `WAIT_GPS` → `READY` → `NAVIGATE` → `RETURN_HOME` → `SAFE_MODE`; rutas 3D con buffer fijo de waypoints. |
| **Failsafe estratégico** | Acoplamiento `RuntimeHealth` ↔ transiciones automáticas: `missed_ticks > 5`, overflows UART ≥ 3 → `SAFE_MODE` con home grabado en `WAIT_GPS`. |

**Criterio de cierre T3:** misión 4 WP 3D completada en sim (NAVIGATE → RETURN_HOME → READY); guiado cableado en Pico2; SAFE_MODE activado bajo estrés de backlog verificable.

---

#### Trimestre 4 — El búnker de verificación (evidencia y pruebas de estrés)

Objetivo: empaquetar evidencia empírica de consistencia estadística y robustez del runtime bajo peor caso compuesto.

| Entregable | Descripción |
|------------|-------------|
| **Horas de vuelo virtuales** | Campaña SIL acelerada: acumulación y registro de trayectorias, innovaciones y covarianzas a máxima velocidad de simulación. |
| **Inyección de fallos masivos** | Caídas de tensión simuladas en I2C, saturación de colas UART, interferencias EMI en tramas WT61C/NMEA; correlación con degradación de confianza y `SystemHealth`. |
| **Evidencia estadística** | Paquetes de telemetría + matrices de covarianza exportadas para demostrar consistencia del filtro (NIS acotado en nominal, P estable, bias convergente). |

**Criterio de cierre T4:** informe de campaña con S0–S7 (banco) + suite SIL automatizada (host); gate de salida binario documentado para OEM.

---

### 3. Herramientas en paralelo (infraestructura de depuración)

Desarrollo concurrente al plan por trimestres; ningún algoritmo se mergea sin visibilidad en datos.

| Herramienta | Ruta | Evolución planificada |
|-------------|------|----------------------|
| **Visualizador de telemetría** | `tools/visualizer.py` | Mapa interactivo trayectoria real vs. ideal; perfil de velocidad y climb; panel de NIS/innovación; elipse de incertidumbre geométrica a partir de √diag(P) del EKF. |
| **Receptor UDP live** | `tools/remote_visualizer.py` | Telemetría en vuelo desde banco Comarruga vía Wi-Fi. |
| **FlightRecorder CSV** | `docs/telemetria_navicore.csv` | Esquema v1 (22 columnas legacy + 56 de estado interno); compatible con replay offline. |
| **SIL multi-UAV** | `tools/sil_*.py`, `docs/sil_architecture.md` | Flota JSBSim × NaviCore para validación autónoma pre-vuelo. |

**Regla de oro:** los bugs de navegación no se encuentran mirando el código; se encuentran mirando los datos. Cada hito del plan exige un campo nuevo en `FlightRecorder` y, como mínimo, un panel en el visualizador.

---

## Referencia rápida de archivos

| Archivo | Rol |
|---------|-----|
| `hw_config.hpp` | Pines, umbrales, invariantes WDT/UART/I2C |
| `main.cpp` | Bucle principal, GPIO benchmark, WDT |
| `loop_metrics.cpp` | `RuntimeHealth`, `SystemHealth`, máximos por bloque |
| `health_monitor.cpp` | Clasificación `SystemHealth` + políticas de degradación |
| `safe_log.cpp` | Log no bloqueante por USB CDC |
| `bsp_uart_rx_ring.hpp` | Ring SPSC atómico + `overflow_count` |
| `bsp_sensors.cpp` | Ventana de overflow y degradación de confianza |
| `ring_stress_host_test.cpp` | Simulación de estrés 60 s en host |
| `src/core/ins_ekf.*` | ESKF 15 estados (T1) |
| `src/core/flight_recorder.*` | Caja negra expandida — esquema CSV v1 (T2) |
| `src/core/guidance.*` | Guiado 3D Pure Pursuit / LOS (T3) |
| `src/core/mission.*` | Mission HFSM y failsafe estratégico (T3) |
| `tools/visualizer.py` | Visualizador de telemetría (infraestructura de depuración) |
