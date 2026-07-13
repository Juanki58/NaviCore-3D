# NaviCore-3D — Arquitectura SIL Multi-UAV

Entorno **Software-in-the-Loop** para validar algoritmos autónomos, visión artificial y protocolos de seguridad con hasta **7 UAVs** antes del vuelo real.

## Estructura del repositorio (targets activos)

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
│       ├── generic_pc/         # Simulador host (NaviCore3D_Sim, NaviCore3D_VehicleDemo)
│       │   ├── main.cpp
│       │   ├── sensors_sim.*
│       │   ├── power_state_machine.*
│       │   └── telemetry_udp_sender.*
│       └── pico2_hardware/     # ★ Target embebido — Pico 2 W, banco Comarruga
│           ├── main.cpp
│           ├── health_monitor.*
│           ├── task_monitor.*
│           ├── loop_metrics.*
│           ├── bsp_sensors.* / bsp_wt61c.* / bsp_gnss.* / bsp_power.*
│           ├── safe_log.*
│           └── hw_config.hpp
├── docs/
│   ├── sil_architecture.md     # Este documento
│   ├── comarruga_lab_hardware.md
│   └── telemetria_navicore.csv
├── tools/
│   ├── visualizer.py           # CSV replay 3D
│   ├── remote_visualizer.py    # UDP live telemetry
│   └── sil_*.py / jsbsim_sil_bridge.py  # SIL multi-UAV
├── CMakeLists.txt
└── build/
```

**Targets de build:**

| CMake target | Directorio | Plataforma |
|--------------|------------|------------|
| `NaviCore3D_Sim` | `src/targets/generic_pc/` | PC host |
| `NaviCore3D_VehicleDemo` | `src/targets/generic_pc/` | PC host |
| `NaviCore3D_Pico2` | `src/targets/pico2_hardware/` | RP2350 (build standalone) |

## Principio de separación

NaviCore-3D es un **estimador de navegación** (fusión + salud). JSBSim es el **plant de dinámica**. El motor gráfico es el **cliente visual y generador de sensores sintéticos**.

No se reutiliza el paquete de telemetría v3 (`0x4E43`, 32 B) para pose 6DOF. Cada canal tiene magic y propósito propios.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Por UAV (×1…7)                                  │
├─────────────────────────────────────────────────────────────────────────┤
│  JSBSim ──SilSensorPacket (0x4E53, 70B)──► NaviCore3D_SIL (futuro)   │
│  JSBSim ◄─SilActuatorPacket (0x4E41, 16B)──  mandos de superficie     │
│  JSBSim ──SilTruthPacket   (0x4E54, 48B)──► Unity / Unreal (mesh)      │
│  NaviCore ─Telemetry v3    (0x4E43, 32B)──► HUD / salud / ruta        │
│  NaviCore ─Eventos         (0x4E45,  8B)──► alertas SAFE_STOP, etc.    │
└─────────────────────────────────────────────────────────────────────────┘
```

## Mapa de puertos (convención por defecto)

| UAV | Truth (gráfico) | Sensor (NaviCore) | Actuator (JSBSim) | Telemetría NaviCore |
|-----|-----------------|-------------------|-------------------|---------------------|
| 1   | 5301            | 5401              | 5501              | 5201                |
| 2   | 5302            | 5402              | 5502              | 5202                |
| …   | 5300+N          | 5400+N            | 5500+N            | 5200+N              |
| 7   | 5307            | 5407              | 5507              | 5207                |

Configurable vía `docs/sil_fleet_manifest.example.json` (esquema: `docs/manifest.schema.json`).

## Paquetes binarios

Definición canónica: `src/core/sil_protocol.hpp` · espejo Python: `tools/sil_protocol.py`.

### SilTruthPacket — 48 B, magic `0x4E54`

Pose NED local para el motor gráfico: posición, velocidad, actitud (roll/pitch/yaw °).

### SilSensorPacket — 70 B, magic `0x4E53`

IMU + magnetómetro + GNSS en convención NaviCore (`lat_deg`, `lon_deg`, `alt_m` → `api_ingest`).

### SilActuatorPacket — 16 B, magic `0x4E41`

Mando normalizado `[-1, 1]` por superficie (throttle, aileron, elevator, rudder).

## Traducción de coordenadas

| Marco | Unity (Y-Up, LH) | Unreal (Z-Up, LH) |
|-------|------------------|-------------------|
| NED → motor | X=Este, Y=−Abajo, Z=Norte | X=Norte, Y=Este, Z=−Abajo |

NaviCore mantiene **X=lat°, Y=lon°, Z=alt m** en `GpsSample`; el bridge JSBSim convierte geodético → esa convención en `SilSensorPacket`.

## Flota y spawning

1. `SimulationManager` lee el manifiesto JSON al arrancar.
2. Por cada entrada: instancia el prefab 3D (Addressables / Soft Object Reference).
3. Vincula un receptor UDP al `truth_port` del UAV.
4. Sensores sintéticos (cámara, LWIR, rangefinder) se montan en sockets del mesh; salida por IPC o UDP local.

## Determinismo y escalado

- Bucle de red en **FixedUpdate** (Unity) o **PrePhysics** (Unreal), alineado con JSBSim (100–250 Hz).
- **Ring-buffer lock-free** por UAV con política drop-oldest.
- **Lerp / Slerp** entre ticks de dinámica y frames de render.

## Banco de validación (Paso 3)

1. Lanzar 7 bridges: `python tools/sil_fleet_launcher.py`
2. Arrancar motor gráfico con manifiesto → 7 meshes en posiciones iniciales.
3. Escalón de mando solo en UAV 3 (`control_step` en manifiesto) → solo UAV 3 asciende.
4. Inyectar fallo GPS en UAV 5 → evento `GPS_LOST` en telemetría `:5205`, mesh sigue con truth.

## Herramientas incluidas (SIL-0 / SIL-1)

| Herramienta | Rol |
|-------------|-----|
| `tools/jsbsim_sil_bridge.py` | Publica truth + sensor UDP por UAV (modo sintético o JSBSim) |
| `tools/sil_fleet_launcher.py` | Arranca N instancias del bridge según manifiesto |
| `tools/sil_truth_monitor.py` | Monitor de consola para validar flota |
| `tools/test_sil_protocol.py` | Pruebas de codec y aislamiento multi-UAV |

## Fases siguientes

| Fase | Entregable |
|------|------------|
| SIL-2 | `sil_dynamics_adapter` C++ → `api_ingest` |
| SIL-3 | Target CMake `NaviCore3D_SIL` |
| SIL-4 | Plugin Unity/Unreal (7 receptores + spawning) |
| SIL-5 | Sensores GPU (RGB, LWIR, rangefinder) |

## Referencias en el repo

- Simulador host: `src/targets/generic_pc/`
- Target embebido: `src/targets/pico2_hardware/` — ver `docs/comarruga_lab_hardware.md`
- Telemetría UDP: `src/targets/generic_pc/telemetry_udp.hpp`
- Ingesta universal: `src/core/api_ingest.hpp`
- Patrón adaptador: `src/core/vehicle_bus_adapter.hpp`
