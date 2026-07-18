# GAP-5 — Informe passive congelado (cierre v1)

**Estado:** **CONGELADO** — resultado experimental; cierra la instancia controlador v1.0  
**Fecha congelación:** 2026-07-18  
**Prerequisito:** [14-adaptive-nhc-protocol.md](14-adaptive-nhc-protocol.md) v1.0 (`gap5-preregistration-frozen`); hook replay passive implementado  
**PoC activo (B0/B1/P0):** **no ejecutado** — bloqueado por este informe  
**Fase siguiente:** **GAP-5 v2** — [16-gap5-v2-observable-selection.md](16-gap5-v2-observable-selection.md) (selección del observable; sin controlador)

---

## 0. Outcome (frase congelada)

> **H5 controller instance v1 is inactive under the preregistered operationalization.**  
> The inactivity is **not** caused by implementation errors.  
> It is caused by the mismatch between the temporal support of the mechanistic burst (~0.4 s) and the operational signal (EWMA τ = 1 s), and — in the preregistered PoC filter config — by a change in the meaning of Γ itself.

Traducción operativa:

> **v1 no se activa porque la operacionalización de Γ̄ no preserva el fenómeno observado offline.**

Esto es un **resultado experimental positivo**, no un fallo de implementación ni una mera mala calibración de umbrales.

---

## 1. Separación de hipótesis (metodología)

| ID | Enunciado | Estado tras passive |
|----|-----------|---------------------|
| **H-impl** | La implementación del estimador Γ + controlador es correcta y causal | **Parcialmente confirmada** (puente F1); ver §4 |
| **H5-PoC** | Existe política adaptativa basada en **Γ̄** que mejora O1–O3 vs B0 | **No testeada** (PoC activo no ejecutado — correcto) |
| **H-ops** | La operacionalización preregistrada (Γ̄ = EWMA τ=1 s + umbrales 12/22 + dwell 1 s) **detecta el régimen** que F1 caracterizó offline | **Refutada** |

**Lo falsado no es H5-PoC en abstracto.** Lo falsado es la cadena operativa concreta:

```
fenómeno mecanicista (burst fix#2→#3, ~0.39 s)
        ↓
Γ_inst  (estimador online rolling)
        ↓
Γ̄      (EWMA τ = 1 s — señal de control preregistrada)
        ↓
controlador v1 (umbrales 12/22, dwell 1 s)
```

El problema **no** está en el EKF, ni en F1, ni en GAP-4. Está en la **traducción** diagnóstico offline → señal de control online.

---

## 2. Diseño experimental (passive only)

**Objetivo:** validar hipótesis **H-impl** y **H-ops** sin contaminar el replay con acción del controlador.

**Prohibido en esta fase:** RMSE, accepts, NIS como criterios; PoC activo B0/B1/P0.

**Duración:** replay completo `real_run` (~332 s).

**Modo replay:** `--adaptive-nhc passive` (controlador observa; N fijo = 1).

| Perfil | CLI / config | Propósito |
|--------|--------------|-----------|
| **f1-bridge** | pos-only GNSS (default); sin `--gnss-obs-mode pos_vel` | Puente escala con Γ offline F1 (misma definición gap fix#2→#3) |
| **gap5-poc** | pos_vel + `--p-pv-policy none` (§4.2 preregistrado) | Lo que vería el PoC tal como estaba preregistrado |

**Scripts:**

```bash
python tools/run_gap5_p0_passive_validation.py --profile f1-bridge
python tools/run_gap5_p0_passive_validation.py --profile gap5-poc
python tools/audit_gap5_passive_controller_validation.py --run-dir docs/benchmarks/gap5_adaptive_nhc/p0_passive_f1_bridge --plot
python tools/audit_gap5_passive_controller_validation.py --run-dir docs/benchmarks/gap5_adaptive_nhc/p0_passive_validation --plot
```

---

## 3. Resultados cuantitativos congelados

### 3.1 Puente escala (perfil f1-bridge) — dato central

Ventana fix#2→#3 (t ≈ 5.66–6.05 s; gap ≈ 0.39 s; definición F1 estricta):

| Magnitud | Valor | Notas |
|----------|-------|-------|
| **Γ offline** (gap integrado) | **19.65** | Coincide exactamente con F1 histórico |
| **Γ_inst pico** (online) | **18.14** @ t≈5.74 s | Ratio 0.92 vs offline — estimador instantáneo ve el burst |
| **Γ̄ pico en burst** | **9.97** | EWMA destruye la señal antes del umbral 12 |
| **Γ̄ max global** | **10.92** @ t≈1.53 s | Artefacto init/bootstrap; no es el burst mecanicista |

**Lectura:** el fenómeno físico sigue ahí; Γ_inst lo ve; **el suavizado es el que hace desaparecer la señal de control**.

### 3.2 Actividad del controlador v1 (ambos perfiles)

| Métrica | f1-bridge | gap5-poc |
|---------|-----------|----------|
| Transiciones | **0** | **0** |
| Tiempo en N=1 | **100 %** | **100 %** |
| Γ̄ ≥ 12 (algún tick) | **nunca** | **nunca** |
| Γ_inst ≥ 12 | sí (73 ticks, 0.22 %; burst) | **nunca** (max 7.97) |
| Γ offline gap | 19.65 | **0.13** |

**Secuencia hipotética completa (passive audit):**

| Tiempo | Γ̄ | N propuesto | Motivo |
|--------|-----|-------------|--------|
| 1.53 s | 10.9 | 1 | init |
| 1.53–332 s | ≈0 (mesetas estables) | 1 | hold |

Durante el burst (f1-bridge), Γ_inst cruza 12 brevemente, pero el controlador evalúa **Γ̄**, no Γ_inst, y el burst (~0.39 s) **< T_dwell** (1.0 s).

### 3.3 Γ̄ como variable de régimen

**No.** En la operacionalización v1:

- Casi todo el trayecto: Γ̄ ≈ 0 con mesetas estables (σ ≪ 1) de decenas de segundos.
- Un único burst mecanicista en fix#2→#3.
- Γ̄ no permanece elevada varios segundos; no hay transiciones lentas entre regímenes; no oscila en umbrales — simplemente **no alcanza** el umbral de actuación.

### 3.4 Dependencia de configuración (alarma conceptual)

En **pos_vel + p_pv none** (config PoC preregistrada):

- Γ offline gap ≈ **0.13** (≈150× menor que F1).
- Γ_inst max ≈ **8** — **ni siquiera aparece el mismo régimen** que motivó el diseño.

El controlador v1 intentaba gobernar una magnitud cuya **significación mecanicista cambia** entre configuraciones de filtro. Eso no es solo calibración de umbrales; es **incompatibilidad conceptual** entre el observable elegido y el experimento PoC preregistrado.

---

## 4. Veredicto por hipótesis

| Hipótesis | Veredicto |
|-----------|-----------|
| H-impl (implementación) | **Aceptada con reservas:** Γ_inst ≈ 0.92 × Γ offline en config F1; split predict/NHC aproximado es suficiente para ver el burst |
| H-ops (Γ̄ detecta régimen) | **Refutada** |
| H5-PoC (política mejora filtro) | **Indeterminada** — no ejecutar PoC activo con v1 sería medir B0 disfrazado |
| «Solo hay que mover 12→10 o τ→0.5» | **Rechazado como lectura principal** — ver §6 |

---

## 5. Qué **no** implica este resultado

| No implica | Por qué |
|------------|---------|
| H5-PoC es falsa | No se probó ninguna política adaptativa activa |
| El hook replay está mal | Puente F1 confirma estimador online |
| F1 estaba equivocado | Γ offline reproducido bit-a-bit |
| Hay que retunear v1 ya | Retune sin redefinir observable sería post-hoc |

---

## 6. Por qué no abrir v2 como «retune de parámetros»

Antes de passive, la tentación era: τ más corto, umbrales más bajos, usar Γ_inst.

Tras passive, las opciones abiertas son **conceptualmente distintas**:

| Opción | Tipo | Notas |
|--------|------|-------|
| A | Reducir τ | Sigue asumiendo que Γ es el observable correcto |
| B | Controlar sobre Γ_inst | Viola la operacionalización preregistrada v1 |
| C | **Abandonar Γ** como señal de control | Candidatos reservados en §2.1 de [14-adaptive-nhc-protocol.md](14-adaptive-nhc-protocol.md): ‖P_pv‖/P_vv, Λ_N, dΛ_N/dt |

**GAP-5 v2 no pregunta «¿qué umbrales?». Pregunta «¿cuál es la variable de estado que define el régimen?».**

Elegir el observable es más importante que mover 12→10 o τ=1→0.5.

---

## 7. Cierre oficial GAP-5 v1

| Campo | Valor |
|-------|-------|
| Instancia | Controlador v1.0 (§4.1 protocolo 14) |
| Estado | **CERRADA** — inactiva bajo operacionalización preregistrada |
| PoC activo | **NO EJECUTAR** con v1 |
| Protocolo 14 v1.0 | Permanece congelado como registro preregistrado |
| Cambios permitidos | Documentación de outcome (este archivo); GAP-5 v2 en documento nuevo |

**Regla:** no retunear umbrales v1 post-hoc. Cualquier continuación = **GAP-5 v2** con nueva preregistración del observable.

---

## 8. Apertura GAP-5 v2 (solo alcance, sin diseño)

**Pregunta rectora:** ¿Cuál es la variable de estado que realmente define el régimen del filtro en tiempo real?

**Criterios de selección (borrador para v2):**

1. Preserva el fenómeno bajo la **config de filtro del experimento** (no solo bajo config F1).
2. Soporte temporal compatible con la dinámica del régimen (burst ~0.4 s vs mesetas ~30 s).
3. Interpretación mecanicista estable entre offline audit y señal online.
4. Separación explícita: observable de régimen vs palanca de actuación (frecuencia NHC).

**Candidatos reservados (no priorizados aún):** ‖P_pv‖/P_vv, Λ_N, dΛ_N/dt, NIS trend — ver §2.1 protocolo 14 y preregistración [16-gap5-v2-observable-selection.md](16-gap5-v2-observable-selection.md).

**Fuera de alcance v2 inicial:** PoC activo, retune 12/22, comparación RMSE, diseño de controlador.

---

## 9. Artefactos congelados

```
docs/benchmarks/gap5_adaptive_nhc/
  p0_passive_f1_bridge/
    controller_audit.csv
    cov_step_audit.csv
    gnss_nis_audit.csv
    passive_validation_report.json
    passive_gamma_regime.png
  p0_passive_validation/          # perfil gap5-poc (pos_vel)
    (misma estructura)
  gap5_passive_outcome_frozen.json # resumen numérico consolidado
```

Implementación hook (reversible): `src/targets/generic_pc/adaptive_nhc_controller.{hpp,cpp}`, wiring en `real_run_replay.cpp`.

---

## 10. Trazabilidad

```
gap5-preregistration-frozen (14-adaptive-nhc-protocol.md v1.0)
        │
        ▼
  hook replay passive + run_gap5_p0_passive_validation.py
        │
        ▼
  15-gap5-passive-outcome.md (CONGELADO) ← este documento
        │
        ▼
  16-gap5-v2-observable-selection.md (preregistración ABIERTA)
        │
        ▼
  passive operacionalización observable (futuro)
        │
        ▼
  controlador v3 + validación PoC (futuro)
```

**Regla metodológica congelada:** la transición diagnóstico offline → señal de control online **no es automática**. Debe demostrarse en passive antes de cualquier PoC activo.

---

## Changelog

| Versión | Fecha | Notas |
|---------|-------|-------|
| **1.0** | **2026-07-18** | Informe passive congelado; cierre instancia v1; apertura GAP-5 v2 |
