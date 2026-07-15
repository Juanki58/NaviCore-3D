# REPORTE DE VERIFICACIÓN FORMAL: TEST DE ESTRÉS Y FALLO EN TIEMPO REAL

**Proyecto:** NaviCore-3D  
**Módulo Auditado:** Planificador de Misión síncrono (HFSM), Control PID & Estimador INS/EKF  
**Fecha de Ejecución:** Julio de 2026  
**Ingeniero de Sistemas:** Juan Carlos Pulido Mellado  
**Estado del Test:** APROBADO (PASSED)

---

## 1. Objetivo del Test

Validar la capacidad de recuperación, tolerancia a fallos de tiempo real e integridad del software de vuelo agnóstico de NaviCore-3D bajo condiciones de degradación crítica de CPU. Se evalúa el correcto funcionamiento de los mecanismos de mitigación síncronos (Time Guard), la degradación inteligente de velocidad y la ejecución segura de la rampa de parada de emergencia (Safe Stop).

## 2. Configuración del Escenario de Prueba (SCENARIO_HIGH_STRESS)

La prueba se ha ejecutado en el simulador de PC (`NaviCore3D_Sim.exe --stress --no-udp`) a una frecuencia de ciclo nominal de 100 Hz (intervalos de 10 ms). El circuito de prueba simula la ruta cuadrada estándar de Comarruga (Tarragona).

### Parámetros Críticos y Umbrales

| Parámetro / Métrica | Valor Nominal | Límite de Tolerancia | Acción al Exceder |
|---|---|---|---|
| Frecuencia de Ciclo | 100 Hz (10 ms) | — | — |
| Tiempo de Cómputo (WCET) | < 3 ticks (CPU) | 6 ticks (Límite Guard) | Penalización de salud (-40 ptos) |
| Umbral de Salud Misión | 100 (Excelente) | < 10 (Crítico) | Transición forzada a SAFE_MODE |
| Velocidad de Crucero | 15.0 m/s | — | Reducción a 8.0 m/s (Degradado) |
| Radio de Waypoint | 3.0 m (Precisión) | — | Ampliación a 15.0 m (Degradado) |

## 3. Análisis Cronológico del Evento (Forense de Telemetría)

El análisis detallado de los datos de la caja negra (`docs/telemetria_navicore.csv`) revela la siguiente secuencia matemática de eventos durante la simulación:

```
[Fase Nominal]  --> [t=10.0s: Inyección de Estrés]  --> [t=10.2s: Colapso de Salud]  --> [t=12.7s: Parada Total (0 m/s)]
```

### ⏱️ t = 0.0s a t = 9.6s: Operación Nominal

- El vehículo navega de forma impecable en modo PERFORMANCE con una salud excelente (health=85).
- El guiado dinámico va alcanzando y conmutando los tramos rápidamente (WP4 → WP5 → WP6 → WP7) a la velocidad consignada de 15.0 m/s.

### 🚨 t = 10.0s: Detección de Violación Temporal (WCET)

En el tick 100 (t=10.0 s), se inyecta de forma deliberada una latencia artificial en el lazo rápido. El tiempo de ejecución del ciclo real sube a 9 ticks, superando el límite de seguridad de 6 ticks establecido por el Time Guard.

**Respuesta de Seguridad:** El planificador detecta instantáneamente la anomalía y aplica una penalización de -40 puntos a la salud general, bajando de 85 a 45 (health=45).

**Activación de Failsafe de Nivel 1:** El sistema transiciona al estado `MISSION_MODE_DEGRADED` y activa el modo de conservación de energía (`power=CONSERVATION`). El guiado táctico adopta de forma inmediata las siguientes consignas de emergencia:

- Reduce la velocidad de forma adaptativa a 8.0 m/s para mitigar la inercia del vehículo.
- Ensancha el radio de aceptación de los waypoints a 15.0 metros para compensar la pérdida de resolución temporal.

### 🛑 t = 10.1s a t = 10.2s: Colapso Controlado e Inicio de Rampa de Frenado

Dado que el estrés de la CPU persiste, el sistema encadena violaciones del tiempo de ciclo consecutivas. La salud decae a 5 en t=10.1 s y colapsa a 0 en t=10.2 s.

**Activación de Failsafe de Nivel 2:** Al llegar al umbral crítico de salud, la HFSM decreta el estado `SAFE_STOP`. El piloto automático toma el control directo de los PIDs para forzar una deceleración controlada (evitando la pérdida de estabilidad o el derrape centrípeto del vehículo):

- t = 11.0 s → Velocidad = 9.90 m/s
- t = 12.0 s → Velocidad = 3.90 m/s
- t = 12.7 s → Velocidad = 0.00 m/s (Parada absoluta).

### ⚡ t = 12.7s a t = 20.0s: Enclavamiento y Desenergización

Una vez lograda la parada total del vehículo de forma segura (0.0 m/s), el planificador activa el enclavamiento (latch) de `POWER_SAFE_SHUTDOWN` con la máscara de periféricos `0x0000000F`.

Se corta la energía de los motores primarios y actuadores, manteniendo únicamente encendidas las líneas de datos de telemetría y el cómputo de la matriz de covarianza del EKF hasta el fin de la simulación.

## 4. Conclusiones de la Auditoría de Seguridad

### ⚖️ VERDICTO FINAL: APROBADO CON EXCELENCIA (PASSED)

Los sistemas defensivos de NaviCore-3D han respondido de acuerdo con los criterios de diseño militar/aeroespacial más exigentes:

- **Sensibilidad:** Detección de fallos en tiempo real en menos de 10 milisegundos.
- **Progresividad:** Intento preventivo de salvar la navegación degradando la velocidad de crucero.
- **Seguridad Absoluta:** Detención controlada por rampa inercial en lugar de apagado abrupto, garantizando la integridad física del chasis del vehículo y de la carga útil.
