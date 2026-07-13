# Laboratorio físico Comarruga — Hardware validado

> **Target CMake:** `src/targets/pico2_hardware/` → `NaviCore3D_Pico2`  
> **Placa:** `PICO_BOARD=pico2_w`

## Bill of Materials (100 % validado en banco)

| Subsistema | Componente | Interfaz | Notas |
|------------|------------|----------|-------|
| **MCU** | Raspberry Pi Pico 2 W (RP2350) | USB stdio + CYW43439 Wi-Fi | Dual Cortex-M33 @ 150 MHz |
| **Energía** | Waveshare UPS Module | I2C1 @ 100 kHz, addr `0x43` | Celdas AVESO 14500; monitoreo de batería |
| **IMU / AHRS** | WitMotion WT61C-232 | UART0 @ 115200 8N1 | Filtro de Kalman integrado en el módulo |
| **GNSS** | u-blox NEO-M9N | UART1 @ 115200 8N1 | GPS+GLONASS+Galileo+BeiDou concurrentes |

## Mapa de pines (Pico 2 W)

| Función | Bus | GPIO TX / SDA | GPIO RX / SCL | Baud / reloj |
|---------|-----|---------------|---------------|--------------|
| WT61C-232 | UART0 | GP0 | GP1 | 115200 |
| NEO-M9N | UART1 | GP4 → RX GNSS | GP5 ← TX GNSS | 115200 |
| Waveshare UPS | I2C1 | GP6 (SDA) | GP7 (SCL) | 100 kHz @ `0x43` |

Definido en `src/targets/pico2_hardware/hw_config.hpp`.

## Flujo de datos

```
WT61C (UART0) ──► bsp_wt61c.cpp ──► ImuSample ──► fusion (dead reckoning)
NEO-M9N (UART1) ──► bsp_gnss.cpp ──► GpsSample ($GNGGA) ──► fusion
UPS (I2C1) ──► bsp_power.cpp ──► umbral batería baja
Wi-Fi (CYW43) ──► telemetría UDP (próxima fase)
```

## Build y flash

```powershell
$env:PICO_SDK_PATH = 'C:\ruta\a\pico-sdk'
cmake -S src/targets/pico2_hardware -B build_pico2 -G Ninja
cmake --build build_pico2
```

Copia `wifi_config.h.example` → `wifi_config.h` con la red del laboratorio.

## Verificación en banco

- [ ] USB: mensaje `BSP Comarruga: WT61C @ UART0 115200 baud ...`
- [ ] Analizador lógico: tramas 0x55 del WT61C en GP1 @ 115200
- [ ] UART GNSS: `$GNGGA` / `$GPGGA` visibles en GP5 @ 115200
- [ ] I2C: ACK del UPS Waveshare en dirección `0x43` (GP6 SDA, GP7 SCL)
- [ ] Fusión: transición `INITIALIZING` → `HYBRID` con fix GNSS activo
- [ ] WDT: mensaje de arranque `WDT 50 ms`; sin reset espurio en bucle @ 100 Hz
