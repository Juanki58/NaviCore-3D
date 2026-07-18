# G-ext — Interpretación congelada (conservadora)

**Fecha:** 2026-07-18  
**Estado:** lectura normativa del experimento G-ext  
**Artefactos:** este directorio · protocolo en [PROTOCOL.md](PROTOCOL.md)

---

## Afirmación sólida (usar esta, no otra)

> **G-ext reproduce el mecanismo de bloqueo del filtro bajo un recorrido independiente, aunque no reproduce toda la secuencia causal observada en G1.**

No afirmar: «G-ext confirma G1».

---

## Qué sí parece robusto

| Observación | G-ext |
|-------------|-------|
| Compresión de `P_vv` (floor NHC) | sí |
| Crecimiento de `P_pv` en arranque | sí |
| Gate bajo innovaciones enormes (`Λ_N` elevado en rejects) | sí |
| Desaparición del reenganche GNSS (1 accept / 680 rejects) | sí |

Núcleo mecanicista **compartido** con G1: bloqueo interno del EKF, no un “bug de un solo CSV”.

---

## Qué no valida / no invalida

### Bifurcación tipo fix#4 (GAP-4)

G-ext **no tiene segundo accept**.  
Por tanto **no puede validar ni invalidar** la bifurcación de política `P_pv` (GAP-4 §11 / K9).

No porque el modelo falle: el recorrido **nunca entra** en la región del espacio de estados donde las políticas 1d y 1d′ podrían divergir.

Queda explícito:

> Este experimento no interroga la región de bifurcación post-fix#4.

### Dominancia del eje Norte

**No escribir:** «el Norte no es universal».

**Sí escribir:**

> La dominancia del eje Norte no se reproduce en G-ext; la componente dominante parece depender de la geometría del recorrido.

Quedan abiertas (sin decidir aún):

1. era realmente una propiedad del Norte;
2. era una propiedad del **eje de innovación dominante**.

Eso es problema de **H6-OBS / GAP-5 v2**, no de GAP-4.  
**Prohibido** adelantar la explicación «Norte → longitudinal» (u otra) antes del benchmark.

---

## Hallazgo más fuerte de G-ext (no es el eje)

> **~506 s de GNSS limpio (hAcc / speed) con el filtro atrapado en reject continuo.**

Desacopla dos magnitudes que en G1 covariaban:

- calidad **externa** del GNSS;
- régimen **interno** del EKF.

Eso convierte G-ext en banco privilegiado para observables internos.

---

## Reformulación motivadora de H6 (sin sustituir la hipótesis formal)

Antes (énfasis informal): *¿Qué observable detecta el régimen?*

Ahora (motivación G-ext):

> **¿Qué observable interno permanece coherente cuando la calidad externa del GNSS deja de explicar el comportamiento del filtro?**

La hipótesis formal H6-OBS sigue en [16-gap5-v2-observable-selection.md](../../diagnostics/16-gap5-v2-observable-selection.md) §2.  
Esta pregunta es la **lectura experimental** que G-ext hace pertinente; no introduce candidatos nuevos ni reescribe O1–O5.

---

## Implicación para GAP-5 v2

Base más sólida que “un solo caso”:

- dos recorridos;
- comportamientos distintos (G1: accepts parciales + bifurcación; G-ext: lockout total sin reenganche);
- **núcleo mecanicista compartido** (bloqueo / P_vv / P_pv / gate).

GAP-5 v2 tiene sentido sobre esa base.  
No requiere decidir aún el eje de innovación.
