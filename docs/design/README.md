# Diseños de aiding — índice

Hoja de ruta de **aiding opcional** sobre el ESKF IMU+GNSS actual. Estos documentos
son diseño (abierto o cerrado), no implementación. No sustituyen Evidence ni el
orden operativo de hardware en [`docs/ROADMAP_PNT_RESILIENCE.md`](../ROADMAP_PNT_RESILIENCE.md).

## Orden de la hoja de ruta

| # | Pieza | Estado del diseño | Doc |
|---|--------|-------------------|-----|
| **1** | **ZUPT** (detector de estancia + pseudo-medida v≈0) | **Abierto** — 2/5 checklist; falta caracterización por dominio (peatón / vehículo / naval) con datos de banco | [ZUPT_DESIGN.md](ZUPT_DESIGN.md) |
| **2** | **Mag / baro** | Tipos scaffolded en código; fusión aún no — diseño de update pendiente tras ZUPT | (aún sin doc aquí; ver README raíz / `sensor_types.hpp`) |
| **3** | **Ultrasonido** (rango → altura / aiding vertical) | **Cerrado** en papel; en **reserva** — no adelantar compra ni código frente a 1–2 | [ULTRASONIC_RANGE_DESIGN.md](ULTRASONIC_RANGE_DESIGN.md) *(depositar archivo — aún no en repo)* |

```
ZUPT  →  mag/baro  →  ultrasonido (reserva)
```

## Lectura rápida

- **ZUPT primero:** cero hardware nuevo; el bloqueo es empírico (detector por dominio), no álgebra del EKF. No pasar a `ins_ekf.cpp` hasta cerrar el checklist del doc.
- **Mag/baro después:** aiding con sensores ya previstos en kit / scaffold; no abrir fusión “porque el BNO055 trae mag”.
- **Ultrasonido en reserva:** formulación lista para implementar cuando toque; no es el siguiente paso de código ni de BOM.

## Relación con constraints existentes

Política de disparo ZUPT/NHC en harness/firmware (estado del sistema, no reloj/ALWAYS):
[`docs/diagnostics/17-conditional-constraints-architecture.md`](../diagnostics/17-conditional-constraints-architecture.md).
