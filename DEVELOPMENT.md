# 🧭 NaviCore-3D: Guía Viva de Desarrollo y Estado del Proyecto (Roadmap & Context)

> **NOTA PARA LA IA / CURSOR AGENT:** Al iniciar una nueva sesión de desarrollo, lee este archivo obligatoriamente. Aquí se detalla la arquitectura exacta, el último commit verificado y los siguientes pasos pendientes para evitar regresiones de código o pérdida de contexto.

---

## 🏗️ 1. Arquitectura del Sistema (Estado Actual)

El proyecto está estructurado bajo la filosofía **Zero-Heap** (cero asignación dinámica en runtime) y determinismo estricto para entornos críticos.

- **`src/core/`**: Motor matemático universal (X, Y, Z). Aislado de la plataforma física.
- **`src/core/api_ingest.*`**: API universal de entrada de datos de sensores.
- **`src/core/vehicle_bus_adapter.*`**: Adaptador que traduce tramas de bus CAN (`ImuCanFrame`, `OdoCanFrame`) al formato nativo del filtro (`ImuSample`).
- **`src/targets/generic_pc/`**: Simulador de estrés en PC (`NaviCore3D_Sim`) y Demo de bus de coche (`NaviCore3D_VehicleDemo`).
- **`src/targets/ambiq_apollo/`**: Capa estructural de drivers bare-metal (DMA, SPI, GPIO, UART, Power Monitor) con stubs para compilación cruzada en host.
- **`tools/visualizer.py`**: Visualizador dinámico e interactivo 3D en Python.

---

## 🏁 2. Hitos Consolidados y Verificados (Último Commit: `af1978b`)

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

### FASE B: Hardware Real (Medio Plazo)

- [ ] **Remoción de stubs Ambiq**: Sustituir el entorno en `src/targets/ambiq_apollo/drivers/` por las llamadas reales a las librerías `am_hal_*` del SDK de Ambiq Apollo4 sobre silicio.
- [ ] **Filtro de deriva inercial (bias térmico)**: Diseñar la calibración estática inicial para compensar el drift térmico de los giroscopios en laboratorio.

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

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

---

## 🚧 6. Notas de Infraestructura y Bloqueos

El Core matemático en `src/core/` está restaurado y verificado en C++17. Sin embargo, las pruebas locales de los ejecutables (`NaviCore3D_VehicleDemo.exe`) quedan pausadas debido a un bloqueo estricto de directivas de Control de Aplicaciones de Windows (WDAC), requiriendo autorización de IT para la ruta `C:\NaviCore-3D\build\` o para el toolchain de MinGW antes de poder realizar pruebas dinámicas en local.
