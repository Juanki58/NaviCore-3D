# Diseño: ZUPT (Zero-Velocity Update) para NaviCore-3D

**Estado: DISEÑO ABIERTO — PENDIENTE DE RESOLVER PERFILES POR DOMINIO ANTES DE CODIFICAR**
**Prioridad: 1 (primera pieza de la hoja de ruta de aiding, antes de mag/baro y muy antes de ultrasonido)**
**Fecha de apertura del diseño:** 2026-07-25
**Autoría:** Juan Carlos + revisión de diseño con Claude (peer review, no ejecución de código)

---

## 0. Por qué es prioridad 1

Cero componentes nuevos, cero coste de hardware — usa la IMU que ya está en la
lista de compra de Fase 1. Cierra deriva de velocidad en paradas/apoyos sin
tocar el presupuesto de energía del sistema (a falta de confirmar el punto 5).
A diferencia del diseño de ultrasonido (ver
`docs/design/ULTRASONIC_RANGE_DESIGN.md`), este no está bloqueado por compra
de hardware — puede empezar a validarse con los primeros datos de banco del
ICM-20948 en cuanto llegue.

**A diferencia del diseño de ultrasonido, este NO está matemáticamente
cerrado todavía.** El pendiente no es de álgebra del EKF (la pseudo-medida
v=0 es estándar y trivial), es de **caracterización empírica por dominio**
— ver sección 2. No pasar a `ins_ekf.cpp` hasta resolver eso.

---

## 1. Concepto

Cuando el detector de estancia confirma reposo, se inyecta una pseudo-medida
de velocidad cero en el marco de navegación:

```
z_zupt = 0₃ₓ₁          (velocidad nav esperada durante estancia)
ν = 0 − v̂_n            (innovación: toda la velocidad estimada es "error")
H_zupt = [0 | I₃ | 0]   (bloque identidad sobre el sub-vector de velocidad)
```

Además de acotar `δv`, el reposo confirmado permite que el filtro reduzca
significativamente la incertidumbre de los biases de acelerómetro/giro en esa
ventana — es el mecanismo real por el que ZUPT frena la deriva a largo plazo,
no solo la corrección puntual de velocidad.

Esta parte **sí está cerrada** — es formulación estándar de INS/GNSS, sin
las sutilezas geométricas que tuvo el ultrasonido (no hay lever-arm que
rotar, no hay dominio de medida que discutir).

---

## 2. El pendiente real: detector de estancia por dominio

**Este es el punto que hay que cerrar antes de implementar.**

Un único umbral de varianza de accel/giro sobre ventana deslizante no sirve
para "multi-domain" sin matizar:

- **Peatonal (PDR, pie apoyado):** señal de estancia limpia, ventana corta,
umbral estrecho — caso fácil, bajo riesgo de falso positivo/negativo.
- **Vehicular en parada real (semáforo, motor encendido):** vibración de
motor puede dar varianza engañosamente alta → falso negativo (el detector
no dispara aunque el vehículo esté parado).
- **Vehicular a baja velocidad constante y suave:** puede parecer "parada"
si el umbral está mal calibrado → falso positivo (el detector dispara con
el vehículo moviéndose, inyectando v=0 incorrectamente — este es el caso
peligroso, corrompe el estado en vez de mejorarlo).
- **Naval/embarcación al pairo:** oleaje residual con motor apagado puede
dar varianza no nula estando efectivamente "parado" en el sentido que
importa para navegación.

**Decisión de diseño pendiente:** ¿un único detector con umbral configurable
por perfil de vehículo (seleccionado en config, como ya haces con otros
parámetros por target), o un detector con lógica distinta por dominio
(no solo distinto umbral, sino distinta ventana temporal y distinta métrica
de decisión)? Esto se resuelve con datos de banco reales por dominio, no en
papel — de ahí que el diseño quede abierto en vez de cerrado.

---

## 3. Estructura propuesta (a validar con banco, no a implementar directo)

```cpp
typedef enum {
    ZUPT_PROFILE_PEDESTRIAN = 0,
    ZUPT_PROFILE_VEHICLE    = 1,
    ZUPT_PROFILE_MARINE     = 2   // provisional, valorar si se necesita
} ZuptProfile;

typedef struct {
    float accel_var_threshold;
    float gyro_var_threshold;
    uint32_t window_ms;
    uint32_t min_stance_duration_ms;  // evitar disparo por parada instantánea espuria
} ZuptDetectorConfig;
```

Cada perfil necesita su propia caracterización empírica (sección 4) antes de
fijar valores por defecto — no adivinar umbrales, medirlos, mismo criterio
que ya aplicas al resto del proyecto.

---

## 4. Trabajo de caracterización necesario antes de cerrar el diseño

- Captura de datos de banco en reposo real por dominio (peatón quieto,
vehículo parado con motor encendido, vehículo parado con motor apagado).
- Captura de datos en movimiento lento/suave por dominio, para caracterizar
el límite inferior de velocidad detectable sin falso positivo.
- Mini-fase de investigación tipo GAP: hipótesis preregistrada, criterios de
aceptación de falsos positivos/negativos por dominio, igual que se hizo
para NHC en GAP-3.
- Medición real del delta de consumo del detector de estancia (ver punto 5)
— no asumir "coste cero", verificarlo.

---

## 5. Nota de energía (matiz sobre el "0 µA")

El detector en sí (test estadístico de varianza sobre ventana deslizante) sí
consume ciclos de CPU extra, aunque no añada ningún sensor ni periférico
nuevo. En Apollo3 es previsiblemente insignificante frente al muestreo de la
IMU, pero **medirlo, no asumirlo** — mismo estándar de rigor que el resto del
proyecto aplica a todo lo demás. Incluir en la misma campaña de medición de
consumo que se haga para el núcleo IMU+GNSS.

---

## 6. Artefactos necesarios

- `zupt_detector.hpp/.cpp` en `src/core/`, con los perfiles de la sección 3.
- Datos de banco por dominio (sección 4) — no requiere hardware nuevo, solo
tiempo de captura con el hardware de Fase 1.
- Fase GAP dedicada para caracterización de falsos positivos/negativos por
dominio.
- Actualizar README (sección "Fusion algorithm") cuando pase de diseño a
fusión activa — mismo patrón que se hará con mag/baro y, eventualmente,
ultrasonido.

---

## 7. Checklist de cierre del diseño (NO todo cerrado todavía)

- [x] Formulación de la pseudo-medida v=0 y del bloque H (estándar, sin sutilezas geométricas).
- [x] Mecanismo de reducción de incertidumbre de biases durante estancia identificado.
- [ ] Umbrales y ventanas por dominio — **pendiente de datos de banco**.
- [ ] Decisión de arquitectura: single-detector-configurable vs. multi-detector-por-dominio — **pendiente de sección 2**.
- [ ] Medición real de consumo del propio detector — **pendiente de banco**.

**No pasar a implementación en** `ins_ekf.cpp` **hasta marcar los tres puntos
pendientes.** A diferencia del ultrasonido, aquí el bloqueo no es de compra
de hardware sino de datos — se puede empezar en cuanto llegue el primer lote
de capturas del ICM-20948.
