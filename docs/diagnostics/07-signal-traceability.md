# Trazabilidad de señales — cadena IMU → EKF

Documento de **ingeniería de sistemas**: cada magnitud tiene un significado físico único y ese significado debe conservarse en toda la cadena. Complementa la campaña experimental H0–H9d; no abre nuevas hipótesis numeradas.

**Auditoría Android (identidad física):** `tools/audit_android_signal_identity.py` → `docs/benchmarks/android_signal_identity_audit.json`

---

## Diagrama de marcos (obligatorio leer antes del código)

```
Android device frame (Sensor Logger: ejes x,y,z del CSV)
        │  v_sensor  [m/s²]  specific force medida por el IMU
        │  R_mount   (calibration/imu_mount.json, sensor→body)
        ▼
Body frame (convención documentada: FRD — Forward, Right, Down)
        │  a_body = R_mount · v_sensor
        │  a_corr = a_body − bias_a
        │  integración actitud: ω_body → q_att (body→NED)
        │  R_bn = DCM(q_att)
        ▼
NED navigation frame (North, East, Down; Down positivo)
        │  a_nav_pre_g = R_bn · a_corr     ← incluye gravedad como specific force
        │  a_lin       = a_nav_pre_g − g_NED   con g_NED = [0, 0, +9.80665] m/s²
        ▼
Integración velocidad / posición (usa a_lin)
```

**Regla de oro:** si una variable “incluye gravedad” en un marco, no puede restarse gravedad en otro marco sin transformar antes.

---

## Tabla de trazabilidad

| Etapa | Variable | Qué representa físicamente | Marco | Unidades | ¿Incluye gravedad? | Evidencia / verificación |
|-------|----------|---------------------------|-------|----------|-------------------|--------------------------|
| **Sensor Logger** | `AccelerometerUncalibrated.csv` | Specific force del IMU (sin calibración de fábrica Android) | Device (x,y,z CSV) | m/s² | **Sí** (≈ g+a en reposo) | Idéntico a `TotalAcceleration.csv` en Patrón Oro (median \|Δ\| = 0). \|g\| vía `Gravity.csv` ≈ 9.80665 m/s² |
| **Sensor Logger** | `TotalAcceleration.csv` | Duplicado de Uncalibrated en Sensor Logger 1.61.0 | Device | m/s² | **Sí** | `uncal − total`: median 0 m/s² |
| **Sensor Logger** | `Gravity.csv` | Estimación Android de gravedad en device frame (fusión AHRS) | Device | m/s² | Solo g (filtrada) | \|vector\| mean = 9.8066 m/s² |
| **Sensor Logger** | `Accelerometer.csv` | Aceleración lineal (Android TYPE_LINEAR_ACCELERATION vía app) | Device | **m/s²** (no g en este dataset) | **No** | Identidad: `Uncal ≈ Gravity + Accelerometer` con residual median 0.001 m/s², p95 ≈ 0.35–0.48 m/s² por régimen |
| **Sensor Logger** | `Orientation.csv` | Actitud Android (fusión propietaria) | Device / ENU-like | deg | N/A | Referencia externa; no alimenta el EKF |
| **parse_mobile_log** | `row.accel` | Copia directa de Uncalibrated (x,y,z) | Device | m/s² | **Sí** | `parse_mobile_log.py`: `ACCEL_FILE = AccelerometerUncalibrated.csv` |
| **Replay** | `row.accel` | Igual que CSV de entrada | Device | m/s² | **Sí** | `real_run_replay.cpp` lectura replay |
| **Mount** | `aligned_accel` | `R_mount · v_sensor` — specific force en body | Body (target: Z+ down) | m/s² | **Sí** | `mat3_vec3_mul(mount_matrix, row.accel, …)` una sola vez. Residual estático L1 ≈ 0 (`imu_mount.json`) |
| **Mount** | `aligned_gyro` | `R_mount · ω_sensor` | Body | rad/s | N/A | Misma matriz que accel |
| **EKF predict** | `imu_sample.accel_mps2` | Entrada = `aligned_accel` (ya en body) | Body | m/s² | **Sí** | `real_run_replay.cpp` → `filter->predict(..., aligned_accel, …)` |
| **EKF predict** | `a_corr` | Specific force body menos bias acelerómetro | Body | m/s² | **Sí** | `vec3_sub(accel, bias_a_, a_corr)` en `ins_ekf.cpp` |
| **EKF predict** | `a_nav_pre_g` | `R_bn · a_corr` — specific force en NED antes de restar g | NED | m/s² | **Sí** | Columnas H9d `a_nav_pre_g_*`. En reposo: ≈ [0,0,+g] |
| **EKF predict** | `a_lin` | Aceleración lineal (kinemática) en NED | NED | m/s² | **No** | `a_lin = a_nav_pre_g − [0,0,g]`. H9d: leak horizontal ya en `a_nav_pre_g` |
| **EKF predict** | `q_att` / `R_bn` | Orientación body→NED | — | — | N/A | `quat_to_dcm_bn` + `body_to_ned`: `ned = R_bn · body`. Tests sintéticos PASS |

---

## Identidad física Android (Patrón Oro, `data/real_run/`)

Comprobación empírica (no alimenta el EKF; solo verifica coherencia del logger):

```
AccelerometerUncalibrated  ≈  Gravity  +  Accelerometer
         (m/s²)                 (m/s²)        (m/s²)
```

| Modelo | Residual median | p95 | Veredicto |
|--------|----------------:|----:|-----------|
| `Uncal − (Gravity + Linear)` con Linear en **m/s²** | **0.001 m/s²** | **0.35 m/s²** | **Coherente** |
| `Uncal − (Gravity + Linear × 9.80665)` (Linear en g) | 6.05 m/s² | 14.3 m/s² | **Rechazado** |
| `Uncal − TotalAcceleration` | 0 m/s² | 0.35 m/s² | **Idénticos** en esta grabación |

Por régimen (residual de identidad, m/s²):

| Régimen | Median | p95 |
|---------|-------:|----:|
| Estático 0–30 s | 0.13 | 0.44 |
| Marcha 2–10 s | 0.05 | 0.45 |
| Crucero 11–25 s | 0.20 | 0.48 |

**Interpretación:** Sensor Logger produce señales **globalmente coherentes** con el modelo Android estándar (specific force = gravedad estimada + aceleración lineal). Los residuales p95 ~0.4 m/s² reflejan filtros/fusión propietaria y desalineación temporal, no un fallo grosero de unidades. **No asumir `Accelerometer.csv` en g** en este dataset — la identidad falla por orden de magnitud si se interpreta así.

---

## Filas ambiguas / candidatos (prioridad de cierre)

| # | Etapa | Ambigüedad | Acción |
|---|-------|------------|--------|
| 1 | **Body frame** | `imu_mount.json` declara target `"body Z+ (EKF down)"`; docs EKF dicen **FRD**. ¿Z+ body = eje D FRD? | Documentar ejes body con diagrama; verificar con vector gravedad post-mount |
| 2 | **Device frame** | Ejes x,y,z Android no documentados en repo como FLU/FRD explícito | Cruzar con `Orientation.csv` y tríada estática; no reutilizar nombre “NED” para device |
| 3 | **Entrada EKF vs Android** | EKF usa **Uncalibrated**; comparaciones con Orientation usan otro estimador | Separar “ground truth” de “referencia cruzada” en informes |
| 4 | **`a_lin_h`** | Magnitud horizontal **NED** — mezcla longitudinal/lateral/error de marco | Usar descomposición vehículo (`tools/vehicle_frame_nav_audit.py`) o `a_nav_pre_g` por ejes |
| 5 | **Signo gravedad** | Android device ≠ NED Down+; mount debe absorber cambio de convención | Confirmar que solo `a_lin` resta g en NED, nunca en body |

---

## Qué afirmar / qué no afirmar (post H0–H9d)

**Afirmar (respaldado por datos):**

- La transformación completa **medida → NED** no reproduce el comportamiento esperado **en dinámica** (H8, H9b, H9c, H9d convergen).
- El leak horizontal aparece en **`a_nav_pre_g`**, no en la resta de gravedad (H9d, corr = 1.0).
- En reposo, cadena Android + mount + EKF es coherente (~0.05° tilt, ~0.02 m/s² horizontal).
- `a_lin_h` es magnitud horizontal **NED**, no “aceleración longitudinal del vehículo”.

**No afirmar todavía:**

- “`R_bn` está mal” (Caso A) — puede ser consistente internamente y aun así no representar la física si `a_body` o el marco body no son los asumidos (Casos B, C, E).
- “El EKF está equivocado y Android tiene razón” — son estimadores distintos.

**Descartado como causa dominante:** parser, GPS raw, geodesia WGS84, sync, P0, Jacobianos NHC solos, móvil en la mano, init estática H9a, montaje aplicado dos veces (D).

---

## Preguntas que este documento debe responder sin leer código

| Pregunta | Respuesta corta |
|----------|-----------------|
| ¿Qué entra al EKF como acelerómetro? | Specific force en body: `R_mount · AccelerometerUncalibrated`, menos bias |
| ¿Incluye gravedad? | **Sí**, hasta restar `g_NED` en `predict()` |
| ¿Qué es `a_nav_pre_g`? | Specific force en NED **antes** de restar gravedad |
| ¿Qué es `a_lin`? | Aceleración lineal cinemática en NED (sin gravedad) |
| ¿Qué CSV usa el replay? | `AccelerometerUncalibrated.csv` — **no** Linear ni Gravity |
| ¿Android es coherente consigo mismo? | Sí, ≈ `Gravity + Accelerometer` en m/s² (p95 ~0.4 m/s²) |

---

## Herramientas de verificación

| Script | Propósito |
|--------|-----------|
| `tools/audit_android_signal_identity.py` | Identidad Gravity + Linear vs Uncal |
| `tools/vehicle_frame_nav_audit.py` | `a_nav_pre_g` en ejes vehículo (long/lat/vert) |
| `audit_attitude_conventions.py` | Coherencia interna quat/DCM/body↔NED |
| `audit_imu_chain.py` | Cadena sensor→mount→body |

---

## Contrato formal

La definición normativa del marco body, invariantes e hipótesis del modelo están en **[08-body-frame-contract.md](08-body-frame-contract.md)**.

## Próximo paso (ingeniería, no experimento)

Verificar conformidad de `predict()` y replay contra **08-body-frame-contract.md** (§4 variables, §6 invariantes). El objetivo es encontrar **inconsistencia entre dos definiciones**, no otra métrica nueva.
