# Datos y cadena de marcos

## Dataset principal

**Ubicación:** `data/real_run/` (Sensor Logger / Android)

| Archivo | Contenido | Uso |
|---------|-----------|-----|
| `AccelerometerUncalibrated.csv` | Accel crudo (parse → replay) | Entrada IMU replay |
| `Gyroscope.csv` | Giroscopio | Entrada IMU replay |
| `Location.csv` | GPS lat/lon/alt, speed, bearing | GNSS + referencia velocidad/rumbo |
| `Orientation.csv` | roll, pitch, yaw (Android) | Referencia externa de actitud |
| `Gravity.csv` | Vector gravedad estimado por Android (sensor) | Fusión AHRS |
| `TotalAcceleration.csv` | Aceleración total incl. gravedad (m/s²) | Comparación con Gravity.csv |
| `Metadata.csv` | Metadatos de grabación | Descubrimiento `t0` |

**Nota:** `Accelerometer.csv` (sin “Uncalibrated”) contiene aceleración lineal en **g**, no comparable directamente con `TotalAcceleration.csv`.

## Pipeline de ingestión

```
Sensor Logger CSV
       │
       ▼
parse_mobile_log.py  →  docs/benchmarks/real_run_replay.csv
       │
       ▼
NaviCore3D_Replay (real_run_replay.cpp)
       │
       ├── R_mount (calibration/imu_mount.json)
       ├── ins_ekf::predict()
       └── CSV de audit / output
```

## Cadena de marcos

```
Frame SENSOR (S) — ejes del móvil / IMU
       │
       │  v_body = R_mount · v_sensor
       ▼
Frame BODY (B) — body FRD, actitud q_att : body → NED
       │
       │  predict(): a_corr = a_body − bias
       │            w_corr = w_body − bias
       │            integra actitud (giro)
       │            a_ned = R_bn · a_corr
       │            a_ned[2] -= g   (gravedad en NED, DESPUÉS de rotar)
       ▼
Frame NED (N)
```

### Convenciones EKF (`ins_ekf.cpp`)

| Elemento | Convención |
|----------|------------|
| Cuaternión | Hamilton; `q_dot = 0.5 · q ⊗ ω` (perturbación **derecha**) |
| DCM `R_bn` | `v_ned = R_bn · v_body` |
| Gravedad NED | `[0, 0, +g]` (Down positivo) |
| Euler | Secuencia 3-2-1 (roll, pitch, yaw) |
| NHC Jacobiano | Alineado con perturbación derecha (`q' = q * dq`) |

Tests sintéticos de coherencia interna: `audit_attitude_conventions.py` (todos PASS).

## Montaje IMU

**Archivo:** `calibration/imu_mount.json`

- Generado por `audit_imu_chain.py` (Rodrigues / alineación gravedad).
- `rotation_matrix`: transformación **sensor → body**.
- Residual en replay: **~0 m/s²** en todo el registro (L1 cadena de referencias).

## Referencias externas y límites

| Referencia | Rol | Limitación |
|------------|-----|------------|
| GPS bearing / speed | Rumbo y velocidad horizontal | No observa inclinación directamente |
| Orientation.csv | Actitud Android | Fusión propietaria; FLU/ENU; posible movimiento relativo teléfono–vehículo |
| Acelerómetro normalizado | Tilt estático | En dinámica mide gravedad + aceleración específica |
| Gravity.csv | Gravedad estimada Android | Separa de `TotalAcceleration` en dinámica (~1.8° salto) |

## Ventanas temporales estándar

| Ventana | Uso |
|---------|-----|
| 0–2 s | Ancla estática; offsets de montaje Orientation |
| 2–10 s | Arranque dinámico / primer pico aceleración |
| 11–25 s | Crucero aproximado (speed > 5 m/s) |
| 0–60 s | Predict-only (H9+) |
