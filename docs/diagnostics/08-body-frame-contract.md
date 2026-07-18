# Body Frame Contract — especificación formal del modelo INS

**Estado:** normativo (modelo físico)  
**Versión:** 1.0  
**Alcance:** propagación inercial NaviCore (replay, SIL, firmware)  
**Complementa:** [07-signal-traceability.md](07-signal-traceability.md)

Este documento **no describe qué hace el código**. Define **qué significa físicamente cada magnitud** y qué leyes debe respetar cualquier implementación (C++, Raspberry Pi, MCU, Unity, ROS 2, …).

---

## 1. Objetivo

Responder, para **cada vector** de la cadena INS, una única pregunta:

> ¿Qué significa exactamente esta variable físicamente?

Una implementación es **conforme** si:

1. Usa los marcos y convenciones de este contrato.
2. Respeta los **invariantes físicos** (§6).
3. Declara explícitamente qué **hipótesis del modelo** (§7) asume.

---

## 2. Marcos de referencia

Cuatro marcos. No más. No mezclar nombres (`device` ≠ `body` ≠ `NED`).

```
┌─────────────────────────────────────────────────────────────────┐
│  D — Android Device Frame (Sensor Logger)                       │
│      Ejes: columnas x, y, z del CSV                             │
│      Origen: IMU del dispositivo                                │
│      Sin relación definida con el vehículo                      │
└────────────────────────────┬────────────────────────────────────┘
                             │  f_D  (specific force, m/s²)
                             │  R_mount  (constante, 3×3)
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  B — EKF Body Frame (vehículo, FRD)                             │
│      +X = Forward (sentido de avance del vehículo)              │
│      +Y = Right   (estribor)                                    │
│      +Z = Down    (gravedad, NED-down)                          │
│      Mano derecha, ortonormal                                   │
└────────────────────────────┬────────────────────────────────────┘
                             │  f_B  (specific force, m/s²)
                             │  R_bn  (actitud, body → NED)
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  N — Navigation Frame (NED)                                     │
│      +N = North,  +E = East,  +D = Down                         │
│      Down positivo (no Up)                                      │
└────────────────────────────┬────────────────────────────────────┘
                             │  f_N = R_bn · f_B
                             │  a_N = f_N − g_N
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  Aceleración lineal (NED)                                       │
│      a_N  — sin gravedad, m/s²                                  │
└─────────────────────────────────────────────────────────────────┘
```

**Convención de rotación:** `v_N = R_bn · v_B` (columna). Cuaternión `q_att`: **body → NED** (Hamilton, perturbación derecha).

**Gravedad en NED:** `g_N = [0, 0, +g]ᵀ` con `g = 9.80665 m/s²`.

---

## 3. Contrato de cada marco

### 3.1 Device frame (D)

| Atributo | Contrato |
|----------|----------|
| **Procedencia** | Sensor Logger / Android `TYPE_ACCELEROMETER` (uncalibrated en replay) |
| **Ejes** | `x`, `y`, `z` tal como se exportan en CSV; sistema dextero del dispositivo |
| **Relación con vehículo** | **Ninguna.** No se asume forward/right/down del coche |
| **Unidades** | m/s² |
| **Magnitud medida** | **Specific force** (fuerza específica aparente, incluye reacción de gravedad) |
| **Verificación dataset** | `Uncalibrated ≈ Gravity + Linear` (Sensor Logger 1.61+, Patrón Oro); ver `tools/audit_android_signal_identity.py` |

### 3.2 Body frame (B) — **definición normativa**

| Atributo | Contrato |
|----------|----------|
| **Qué representa** | **El vehículo**, no el sensor ya “nivelado”. |
| **Convención** | **FRD** (Forward–Right–Down) |
| **+X** | Eje longitudinal hacia **delante** del vehículo (sentido de marcha a sideslip nulo) |
| **+Y** | Eje **lateral derecho** (estribor) |
| **+Z** | Eje **vertical hacia abajo** (coincide con Down NED cuando el vehículo está nivelado) |
| **Specific force en reposo nivelado** | `f_B ≈ [0, 0, +g]ᵀ` |
| **Uso en observaciones** | NHC: `v_y ≈ 0`, `v_z ≈ 0` (velocidad lateral y vertical en body); `v_x` libre (avance) |

**Eliminación de ambigüedad:** “body Z+ down” **no es** una definición alternativa de body. Es solo la **condición estática** que `R_mount` debe cumplir: alinear la gravedad medida en **+Z body**. La definición completa del marco exige **FRD vehicular** para X e Y.

**Obligación de calibración (`R_mount`):**

- `R_mount` es la rotación **constante** sensor → **vehículo FRD**.
- En fase estática: `R_mount · mean(f_D) ≈ [0, 0, +g]ᵀ`.
- Método mínimo aceptable (Rodrigues): alinea gravedad en +Z; **no observa yaw de montaje** alrededor de la vertical. Instalación rígida + repetibilidad del Patrón Oro satisfacen H3–H4; para yaw de montaje explícito, fijar eje forward (GPS, marcador mecánico, o segunda restricción).

### 3.3 Navigation frame (N)

| Atributo | Contrato |
|----------|----------|
| **Sistema** | **NED** local (tangente al elipsoide WGS84 en origen de trayecto) |
| **+N** | Norte geodésico |
| **+E** | Este |
| **+D** | **Down** (positivo hacia el centro de la Tierra) |
| **Excepciones** | **Ninguna.** No ENU. No Up positivo. |
| **Gravedad** | `g_N = [0, 0, +g]ᵀ` restada **solo** al formar aceleración lineal |

### 3.4 Actitud `R_bn`

| Atributo | Contrato |
|----------|----------|
| **Significado** | Orientación del **body vehículo (FRD)** respecto a **NED** |
| **Transformación** | `v_N = R_bn · v_B` |
| **Integración** | `ω_B` (giroscopio en body) → `q_att` (body→NED), convención documentada en `ins_ekf.cpp` |
| **Yaw** | No observable solo con gravedad; init separado (`yaw_init`) |

---

## 4. Contrato de variables

Cadena replay + `InsEkfFilter::predict()` (referencia NaviCore 3D).

| Variable | Marco | Significado físico | Unidades | ¿Incluye gravedad? | Observable |
|----------|-------|-------------------|----------|-------------------|------------|
| `row.accel` | D | Specific force del IMU (Android) | m/s² | **Sí** | **Directo** (CSV / replay) |
| `aligned_accel` | B | `R_mount · row.accel` — specific force en body vehículo | m/s² | **Sí** | **Indirecto** (tras mount) |
| `aligned_gyro` | B | `R_mount · ω_sensor` | rad/s | N/A | **Directo** (CSV) + mount |
| `imu_sample.accel_mps2` | B | Entrada EKF = `aligned_accel` | m/s² | **Sí** | Indirecto |
| `imu_sample.gyro_radps` | B | Entrada EKF = `aligned_gyro` | rad/s | N/A | Directo + mount |
| `bias_a`, `bias_g` | B | Sesgos estimados en body | m/s², rad/s | N/A | Estado filtro |
| `a_corr` | B | Specific force corregida: `f_B − b_a` | m/s² | **Sí** | **Indirecto** (audit H9d) |
| `ω_corr` | B | `ω_B − b_g` | rad/s | N/A | Indirecto |
| `q_att`, `R_bn` | B→N | Actitud body vehículo → NED | — | N/A | Indirecto (output / audit) |
| `a_nav_pre_g` | N | `R_bn · a_corr` — specific force en NED **antes** de restar g | m/s² | **Sí** | **Indirecto** (H9d CSV) |
| `a_lin` | N | `a_nav_pre_g − g_N` — aceleración lineal (cinemática) | m/s² | **No** | **Estado interno** (integración) |
| `vel`, `pos` | N | Velocidad / posición integradas desde `a_lin` | m/s, m | N/A | Estado interno + GNSS |

**Regla de propagación (orden obligatorio):**

1. Restar bias en **B**.
2. Integrar actitud con **ω_corr** en **B**.
3. `a_nav_pre_g = R_bn · a_corr` en **N**.
4. `a_lin = a_nav_pre_g − g_N`.
5. Integrar `a_lin` → velocidad → posición en **N**.

**Prohibido:** restar gravedad en body y luego rotar (orden invertido).

---

## 5. Señales Android (referencia, no entrada EKF)

| CSV Sensor Logger | Rol | Entra al EKF |
|-------------------|-----|:------------:|
| `AccelerometerUncalibrated.csv` | Specific force → replay | **Sí** |
| `TotalAcceleration.csv` | Duplicado de Uncalibrated (v1.61 Patrón Oro) | No |
| `Gravity.csv` | Estimación Android de g en device | No (verificación) |
| `Accelerometer.csv` | Linear acceleration (m/s² en Patrón Oro) | No (verificación) |
| `Orientation.csv` | Actitud Android (otro estimador) | No (referencia cruzada) |

---

## 6. Invariantes físicos

Una implementación conforme debe respetar estos invariantes **en las condiciones del modelo** (§7). Violación en Patrón Oro indica no conformidad o hipótesis rota.

### I1 — Reposo, specific force en body

**Condición:** vehículo parado, montaje rígido, bias ≈ 0.  
**Invariante:** `|f_B| ≈ g` (≈ 9.80665 m/s²).

### I2 — Reposo, specific force en NED (antes de −g)

**Condición:** vehículo parado y nivelado, `R_bn` coherente con roll/pitch.  
**Invariante:** `a_nav_pre_g ≈ [0, 0, +g]ᵀ` (componentes N,E ≈ 0).

### I3 — Reposo, aceleración lineal

**Condición:** tras restar `g_N` correctamente.  
**Invariante:** `a_lin ≈ [0, 0, 0]ᵀ`; `|a_lin|_h ≈ 0`.

### I4 — Gravedad no genera horizontal con actitud nivelada

**Condición:** `roll = pitch = 0`, solo gravedad en specific force.  
**Invariante:** `a_nav_pre_g = [0, 0, g]ᵀ` ⇒ `|a_lin|_h = 0`.  
**Corolario:** cualquier `|a_nav_pre_g|_h` significativo con vehículo parado indica **actitud incorrecta o marco incorrecto**, no “ruido GNSS”.

### I5 — Longitudinal puro no es lateral

**Condición:** aceleración del vehículo **solo longitudinal** en body (`a_x ≠ 0`, `a_y ≈ 0`, `a_z ≈ 0` en body, sin roll/pitch).  
**Invariante:** la proyección en NED no debe aparecer como componente **lateral** dominante salvo **rotación física** del vehículo (curva, cambio de rumbo).  
**Uso diagnóstico:** descomponer `a_nav_pre_g` en ejes vehículo (`tools/vehicle_frame_nav_audit.py`); comparar con `d(GPS speed)/dt`.

### I6 — Identidad Android (dataset)

**Condición:** Sensor Logger coherente.  
**Invariante:** `f_uncal ≈ g_Android + a_linear` en device frame; residual p95 ≲ 0.5 m/s² (Patrón Oro).

### I7 — Conservación de marco en NHC

**Condición:** NHC activo.  
**Invariante:** observación `y = [-v_y, v_z]` usa el **mismo** body FRD que `a_corr` y `R_bn`.

---

## 7. Hipótesis del modelo

Si una hipótesis deja de cumplirse, las secciones afectadas quedan **invalidadas** (no hace falta redescubrirlo con H10, H11, …).

| ID | Hipótesis | Si falla… |
|----|-----------|-----------|
| **M1** | El acelerómetro mide **specific force** (no aceleración cinemática pura) | Toda la cadena `f → R → −g → a_lin` pierde sentido |
| **M2** | El **body frame** es **FRD del vehículo** (§3.2), no “sensor nivelado” | NHC, mounting, comparación con dinámica del coche inválidos |
| **M3** | Montaje **rígido** (sin flexión significativa sensor–vehículo) | Patrón Oro no generaliza; mano / soporte blando rompe M3 |
| **M4** | `R_mount` **constante** durante el trayecto | Recalibrar en movimiento; no mezclar montajes |
| **M5** | Gravedad local **constante** `g = 9.80665 m/s²`, dirección Down NED | Modelo válido para trayectos cortos; altitud extrema → revisar |
| **M6** | `R_bn` representa actitud **body vehículo → NED** (no sensor→NED) | Mezclar mount y actitud; error en dinámica |
| **M7** | Giroscopio en **mismo body FRD** que acelerómetro | NHC / acoplamiento actitud incoherente |

**Hipótesis explícitamente no asumidas:**

- `Orientation.csv` como verdad absoluta (es otro estimador, referencia cruzada).
- Device frame alineado con vehículo.
- Aceleración lineal Android idéntica a `a_lin` del EKF (marcos y filtros distintos).

---

## 8. Criterios de conformidad (Patrón Oro)

Dataset: `data/real_run/`. Herramientas en `tools/`.

| Invariante | Umbral indicativo (Patrón Oro) | Herramienta |
|------------|--------------------------------|-------------|
| I1 | `\|f_B\|` mean ≈ 9.81 m/s² | H8 / mount audit |
| I2–I3 estático 0–2 s | `\|a_lin\|_h` ≲ 0.05 m/s²; tilt ≲ 0.1° | H9c |
| I6 | identity median ≲ 0.01 m/s² | `audit_android_signal_identity.py` |
| Dinámica (regresión) | Si cambio en `predict()` no mejora **simultáneamente** tilt, `a_lin_h`, GPS %, RMSE → no conforme o hipótesis M* rota | Replay + benchmarks |

**Regresión mínima tras cualquier cambio en propagación:**

| Métrica | Baseline Patrón Oro (Jul 2026) |
|---------|-------------------------------|
| GPS aceptado | ~2 % |
| `a_lin_h` crucero | ~1.0 m/s² |
| Tilt vs Orientation (2–10 s) | ~4° |
| RMSE horizontal final | ~4500 m |

Mejora sustancial simultánea ⇒ cambio alineado con contrato; sin mejora ⇒ revisar qué fila de §4 o invariante de §6 se violó.

---

## 9. Referencias de implementación (informativas)

| Artefacto | Rol |
|-----------|-----|
| `parse_mobile_log.py` | Device → replay CSV |
| `real_run_replay.cpp` | `R_mount`, llamada a `predict()` |
| `ins_ekf.cpp` | `predict()`, NHC, `body_to_ned` |
| `calibration/imu_mount.json` | `R_mount` persistido |
| `attitude_kinematics.py` | Réplica matemática para auditoría |
| [07-signal-traceability.md](07-signal-traceability.md) | Trazabilidad señal a señal |

**Conformidad:** la implementación actual satisface **I1, I2, I3, I6, I7 en estático**. En dinámica, **I2/I3 fallan** (leak horizontal en `a_nav_pre_g` antes de −g), lo que indica **no conformidad en régimen dinámico** bajo M2–M7, no necesariamente violación de la sintaxis de `body_to_ned`.

---

## 10. Historial

| Versión | Fecha | Cambio |
|---------|-------|--------|
| 1.0 | 2026-07-18 | Contrato inicial post H0–H9d; body = vehículo FRD normativo; invariantes e hipótesis M1–M7 |
