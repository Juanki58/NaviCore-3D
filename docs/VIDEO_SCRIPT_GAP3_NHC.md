# Vídeo / guion — GAP-3: NHC agresivo empeora el coast

**Duración objetivo:** 90–120 s  
**Escena:** Unity EKF Explorer / Cesium (mapa + HUD de modo/calidad)  
**Mensaje único:** *“NHC always-on no es gratis — en nuestro banco, empeora el dead reckoning frente a NHC off.”*

Artefactos: [`docs/nhc_experiments/manifest.json`](nhc_experiments/manifest.json) · Evidence scorecard en README.

---

## Hook (0–10 s)

**VO / overlay:**  
“En INS embargado, mucha gente pone Non-Holonomic Constraints a tope. Nosotros medimos qué pasa.”

**Visual:** vehículo en mapa, GNSS OK → entra en túnel (pérdida de fix). HUD: `HYBRID` → `DEAD_RECKONING`.

---

## Setup (10–25 s)

**VO:**  
“Mismo escenario sintético *super-tunnel*, mismo IMU, misma salida. Solo cambia la política NHC.”

**Visual:** split screen o A/B toggle:
- Brazo **A — NHC off**
- Brazo **B_always — NHC cada tick**

Overlay pequeño: `NaviCore3D_Sim --nhc-experiments` · `docs/nhc_experiments/`.

---

## Resultado (25–70 s)

**Tabla en pantalla (números del manifest):**

| Brazo | Drift @ salida túnel | Drift final |
|-------|---------------------:|------------:|
| A NHC off | **493 m** | ~2 m (reacquire) |
| B_always | **1408 m** | 1554 m |
| Mejor G-arm | 758 m | 887 m |

**VO:**  
“Con NHC off, al reaparecer el GNSS recuperamos. Con NHC always-on, el filtro se come la covarianza de velocidad — y el coast se dispara. El mejor brazo de tuning G sigue peor que apagar NHC.”

**Visual:** dos trayectorias en Cesium — la “always” se desvía; la “off” se reancla al salir. Callout: `P_vv` comprimida / over-observe (sin jerga excesiva: “el filtro se cree demasiado la velocidad del vehículo”).

---

## Política resultante (70–95 s)

**VO:**  
“Por eso en producción no vendemos NHC always-on. Política: **NHC off**, o **gap-triggered** en el core v2 — no ‘siempre’.”

**Visual:** HUD con badge `NHC: off | gap-triggered` · enlace mental a EKF v2.

---

## Cierre (95–120 s)

**VO:**  
“NaviCore-3D: resiliencia GNSS con evidencia publicada — Monte Carlo, matriz NHC, tooling Allan. MIT, zero-heap, auditable.”

**Pantalla final:**
- github.com/Juanki58/NaviCore-3D  
- README → Evidence → Scientific rigor scorecard  
- “GAP-3 CLOSED”

---

## Notas de producción

- **No** RF spoof; no claims mil-grade.
- Si el gemelo aún no reproduce ambos brazos en vivo: grabar Sim CSV → `tools/export_ekf_explorer_session.py` / replay de trazas `*_trace.csv` del experimento.
- Música: baja; prioridad a VO + números.
- CTA: “Reproduce: `--nhc-experiments`.”

## Checklist de veracidad

- [ ] Números coinciden con `manifest.json` actual  
- [ ] Se dice explícitamente que es escenario **sintético** super-tunnel  
- [ ] Se muestra reacquire post-GNSS en brazo A  
- [ ] No se afirma “NHC nunca sirve” — solo “always-on / dose alta puede empeorar”
