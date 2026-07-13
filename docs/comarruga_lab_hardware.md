# Laboratorio físico Comarruga — Hardware validado

> **Target CMake:** `src/targets/pico2_hardware/` → `NaviCore3D_Pico2`  
> **Placa:** `PICO_BOARD=pico2_w`  
> **Firmware de referencia:** commit `fc28d70` y posteriores (`RuntimeHealth`, `fault_tolerance`, `SystemHealth`)

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
2. Si hay tick pendiente: **GP22 ↑** → `pico2_bsp_sensors_tick()` → **GP22 ↓** (omitido si `max_tick_backlog > 2`)
3. `pico2_bsp_sensors_housekeeping()` — FSM I2C UPS (un paso por llamada)
4. **GP21 ↑** → `cyw43_arch_poll()` → **GP21 ↓** (omitido si sin presupuesto o `fault_tolerance` deshabilitó Wi-Fi)
5. `loop_metrics_sync_uart_overflows()` + `safe_log_flush_pending()` (máx. 256 B/ciclo)
6. `loop_metrics_on_loop_complete()` + `fault_tolerance_on_loop_complete()` + `loop_metrics_update_system_health()`
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

**Criterio de ruptura de presupuesto:** cualquier escenario donde `max_loop_us > PICO2_LOOP_BUDGET_US` de forma sostenida, o `max_tick_backlog > 2`, o activación de `fault_tolerance` / `SystemHealth::CRITICAL`.

### Lectura de métricas en banco

El firmware **no imprime** `RuntimeHealth` en caliente (evita bloquear USB). Opciones:

**A — Depurador (recomendado en campaña)**

```text
# OpenOCD + GDB, halt al final del escenario:
(gdb) p *loop_metrics_health()
(gdb) p loop_metrics_system_health()
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
   - Reiniciar placa **o** `loop_metrics_init()` + `fault_tolerance_init()` (solo banco)
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
5. Conclusión binaria: ¿el presupuesto de 10 ms se mantiene en S7 bajo peor caso? Si no, enumerar subsistema dominante y acción de `fault_tolerance` observada.

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

## Referencia rápida de archivos

| Archivo | Rol |
|---------|-----|
| `hw_config.hpp` | Pines, umbrales, invariantes WDT/UART/I2C |
| `main.cpp` | Bucle principal, GPIO benchmark, WDT |
| `loop_metrics.cpp` | `RuntimeHealth`, `SystemHealth`, máximos por bloque |
| `fault_tolerance.cpp` | Acciones automáticas ante degradación |
| `safe_log.cpp` | Log no bloqueante por USB CDC |
| `bsp_uart_rx_ring.hpp` | Ring SPSC atómico + `overflow_count` |
| `bsp_sensors.cpp` | Ventana de overflow y degradación de confianza |
| `ring_stress_host_test.cpp` | Simulación de estrés 60 s en host |

---

## Fusión y telemetría (próxima fase)

- Transición `INITIALIZING` → `HYBRID` con fix GNSS activo (validación funcional adicional).
- Telemetría UDP hacia el host del laboratorio (`HOST_IP`, `UDP_PORT` en `wifi_config.h`).
