# Scientific Chronology — Programa EKF (no cronología de commits)

**Tipo:** consolidación editorial — narrativa científica continua a partir de docs ya congelados.  
**Regla:** no interpreta ni añade hipótesis; solo ordena lo existente.  
**Fuentes:** RESEARCH_MAP, docs 09/12/13/15/16, INTERPRETATION G-ext, RESEARCH_STATUS.  
**Fecha:** 2026-07-18

---

## Tabla maestra

| Fase | Pregunta científica | Resultado (congelado) | Anclas |
|------|---------------------|----------------------|--------|
| **GAP-1** | ¿`R_mount` alinea body vehículo FRD? | **Cerrado** — Rodrigues + yaw_init; veredicto `GAP-1_CLOSED_YAW_INIT_REQUIRED` | [09-predict-conformance-audit.md](../09-predict-conformance-audit.md) §5 |
| **GAP-2** | ¿La transformación medida→NED / dinámica strapdown explica la ruptura? | **Cerrado** — identidad gravedad OK; ruptura en uso de f completa + corrección insuficiente → abre GAP-3 | [09…](../09-predict-conformance-audit.md) §5, audits gap2 |
| **GAP-3** | ¿Qué mecanismo degrada el EKF (covarianza / accepts)? | **Respondida** — NHC domina P; gate nominal-driven; modelo mecanicista cerrado | [12-gap3-synthesis.md](../12-gap3-synthesis.md), K1–K7 |
| **F1** | ¿Menos NHC restaura P_vv y accepts? | **Refutada** — P_vv/k_vel suben; accepts no | `gap3_f1_nhc_dose_response/` |
| **F1.1** | ¿Rechazo = solo K bajo? | **Refutada** — innovación / Λ_N domina | F1.1 |
| **F1.2** | ¿Cliff = solo frecuencia NHC? | **Refutada** — burst state-conditioned; decimación no lo elimina | F1.2, K5 |
| **GAP-4** | ¿`P_pv` es bug o mecanismo? | **Respondida** — acoplamiento legítimo; fix#4 bifurca; cos desde logs | `gap4-diagnostic-complete`, K8–K10 |
| **GAP-5 v1** | ¿Γ̄ (EWMA) sirve como observable operacional de régimen? | **Respondida (negativa)** — operacionalización H-ops refutada; H5-PoC no testeada | [15-gap5-passive-outcome.md](../15-gap5-passive-outcome.md), K11–K13 |
| **G-ext** | ¿El mecanismo de bloqueo reaparece fuera de G1? | **Respondida (parcialmente positiva)** — núcleo sí (K14/K15); secuencia G1 completa no; región fix#4 no alcanzada | [INTERPRETATION.md](../../benchmarks/real_run_19082026_baseline/INTERPRETATION.md) |
| **GAP-5 v2** | ¿Qué observable interno permanece coherente cuando GNSS externo no explica el régimen? | **Preregistrada — benchmark no abierto** (D13) | [16…](../16-gap5-v2-observable-selection.md), OQ1–OQ4 |

---

## Narrativa continua (solo hechos ya documentados)

1. **GAP-1/2** cerraron la cadena de marcos y la ruptura dinámica medida→NED lo bastante como para localizar el problema en el modelo INS/constraints (no en un mount globalmente invertido).  
2. **GAP-3 + F1\*** construyeron y falsaron explicaciones dominantes hasta dejar un núcleo: NHC comprime covarianza; restaurar K no basta; el cliff es bursty/state-conditioned; el gate stress se ve en innovaciones / Λ.  
3. **GAP-4** estableció que `P_pv` es mecanismo EKF legítimo y que políticas sobre él bifurcan trayectorias en la región de accepts múltiples (fix#4). Intervención §11 queda preregistrada, no ejecutada (OQ7).  
4. **GAP-5 v1** preregistró un controlador vía Γ̄; el passive falsó la operacionalización (H-ops), no la hipótesis abstracta de política (H5-PoC).  
5. **G-ext** reejecutó el shell G1 en un recorrido independiente: el **bloqueo** reaparece; no la secuencia causal completa de G1; aparece el desacople GNSS limpio externo vs reject interno.  
6. **GAP-5 v2** queda preregistrada (H6-OBS / H7-MIN) con OQ1 afilada por G-ext; **D13** impone pausa antes del benchmark.

Secuencia normativa ya escrita en RESEARCH_STATUS:

```
mecanismo → generalidad → observable → controlador
```

---

## Fuera de esta cronología (otro eje)

- Campaña hardware Pico 2 / WCET — ver `DEVELOPMENT.md` (no es la cadena GAP EKF).  
- H0–H9d actitud — pipeline diagnóstico previo; no sustituye la tabla GAP anterior.
