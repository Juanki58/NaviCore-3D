# GAP-5 v2 — Paso 0: catálogo de estados internos del EKF

**Estado:** **CONGELADO** con [16-gap5-v2-observable-selection.md](../../diagnostics/16-gap5-v2-observable-selection.md) v1.0  
**Tag:** `gap5-v2-observable-preregistration-frozen`  
**Fecha:** 2026-07-18

Este documento es el **Paso 0 obligatorio** antes de cualquier script de benchmark. No evalúa «qué funciona mejor»; cataloga **qué observa cada candidato, qué no observa, y bajo qué condiciones sería refutado**.

---

## Catálogo principal

| ID | Observable | Qué observa | Qué **no** observa | Escala temporal | Dependencias | Hipótesis (Paso 0) | ¿Qué resultado lo refutaría? |
|----|------------|-------------|-------------------|-----------------|--------------|-------------------|------------------------------|
| **O1** | Γ_inst | Consumo **instantáneo** de covarianza (tasa \|ΔP_vv\| NHC vs predict) | Consistencia GNSS; calidad del nominal; acoplamiento P_pv | Muy corta (~1 s ventana rolling) | predict(), NHC update, bloque P_vv | En R1 refleja sobreobservación NHC con significado estable entre configs | Cambio de **significado** C-F1 ↔ C-PoC sin reinterpretación mecanicista |
| **O2** | Γ̄ | Media temporal de O1 (EWMA τ=1 s) | Eventos más breves que τ; consistencia GNSS; estructura P | Media (τ=1 s) | O1, reloj IMU | Régimen **sostenido** de desequilibrio predict/NHC | **Parcialmente refutado v1:** burst ~0.4 s no eleva Γ̄; memoria destruye señal |
| **O3** | ‖P_pv‖ / P_vv | **Estructura de correlación** posición→velocidad | Calidad nominal; innovación GNSS directa; tasa de consumo de P | Media (por tick / update) | Matriz P, bloques pos/vel/cruzado | Distingue R0/R3/R4; sube cuando acoplamiento activo importa (GAP-4) | No distingue R0–R4 mejor que ruido baseline; colapsa interpretabilidad en C-PoC |
| **O4** | Λ_N | **Consistencia estadística** filtro vs medida GNSS | Estructura de P; consumo de covarianza por NHC | Corta (por update GNSS) | innovación, S, gate | Crece en R2 cuando nominal deja de explicar medidas (F1.2) | Permanece **plano** en R2/R3 pese a degradación documentada |
| **O5** | dΛ_N/dt | **Transición de régimen** (velocidad de deterioro de consistencia) | Estado estacionario; estructura de P; consumo P por NHC | Muy corta (derivada/discreta) | Serie temporal de O4 | Pico al **entrar** en R2; bajo en mesetas R4 | Domina ruido instantáneo; o suavizado necesario pierde localidad (refuta **operacionalización**) |
| **O6** | Combinación Oi+Oj | *(preregistrar — no en v1.0)* | *(depende de fórmula)* | *(preregistrar)* | *(preregistrar)* | *(v1.1 + nuevo tag)* | *(v1.1 + nuevo tag)* |

---

## Lecturas cruzadas («qué no observa»)

Evita pedir a un observable lo que por definición no puede medir:

| Observable | Ciego a… | Implicación |
|------------|----------|-------------|
| Γ_inst / Γ̄ | Nominal, innovación GNSS, P_pv | No sustituye Λ_N ni ‖P_pv‖/P_vv |
| ‖P_pv‖ / P_vv | Innovación, NIS, consumo NHC directo | No sustituye Γ ni Λ_N |
| Λ_N | Estructura de P, compresión NHC | No sustituye Γ ni ratio P_pv |
| dΛ_N/dt | Nivel absoluto de P o de acoplamiento estático | Complementa O4; no reemplaza O3 |

---

## Regímenes de referencia (mapeo esperado, no garantizado)

| Régimen | Ventana | Propiedades más relevantes (hipótesis) |
|---------|---------|----------------------------------------|
| R0 | Pre-fix#2 | Baseline bajo en O1, O4 |
| R1 | fix#2→#3 (~0.39 s) | Pico O1; posible O3 |
| R2 | Post-fix#3 → fix#4 | Elevación O4; posible pico O5 |
| R3 | fix#4 bifurcación | O3 (acoplamiento) |
| R4 | Crucero largo | Mesetas en O1/O4; O5 ≈ 0 |

El benchmark **verifica** estos mapeos; no los asume.

---

## Hipótesis exploratoria H7-MIN (conjunto mínimo)

Si ningún Oi aislado basta, el outcome v2 puede ser un **vector de estado de régimen**, p.ej.:

- **Estructural:** O3  
- **Consistencia:** O4  
- **Temporal:** O5  

Eso sería pasar de «elegir una variable» a «definir el estado interno del régimen». Evaluar solo **después** de caracterizar cada fila de este catálogo.

---

## Reglas

1. No modificar filas post-tag salvo typo → versión 1.1 + nuevo tag.  
2. No añadir O6 en v1.0.  
3. No discutir controlador, umbrales ni NHC en informes derivados de este catálogo (§0.2 protocolo 16).
