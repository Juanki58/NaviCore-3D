# CAND1 — revisión de generalización entre dominios

**Fecha:** 2026-07-19  
**Estado:** **cerrada** (instrumento / operacionalización).  
**No es:** refutación de H-ATT-d · búsqueda de T₂ · diseño de cand2.

Protocolo: `docs/diagnostics/18-jacobian-imu-ab-protocol.md` §13.21–§13.22.  
Datos: `hatt_d/`, `cand1_gate_e12/`, `hatt_cand12_A_vs_C_discrimination.*`.

---

## Separación de niveles (obligatoria)

```
H-ATT-d          ← hipótesis científica (Z no observado en H; Joseph coherente)
      │
Observable       ← “crecimiento superlineal / feedback de Σ|dx_att_z| en racha A”
      │
cand1            ← operacionalización: acumular Σ|dx_att_z|, latch si ≥ T₂, t≤tmax
      │
Gate             ← cuándo armar la acción H-ATT-d
      │
Implementación   ← unobs H[*][ATT_Z] post-latch
```

**Fallo localizado:** en **cand1 → Gate** al cruzar de slalom → túnel.  
**No localizado (aún):** en H-ATT-d. El instrumento que decide *cuándo* mirar la hipótesis no es estable entre dominios; eso **bloquea** evaluar P2-tunnel / P4, no cierra la hipótesis.

Analogía metodológica (GAP-5 / Γ̄): no se concluyó que la propiedad subyacente fuera falsa; se concluyó que **esa operacionalización** no generalizaba. Misma estructura aquí.

**Patrón de programa (Caso B):** [OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md](../../diagnostics/reference/OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md) · decisión **D22** (pausa; no abrir OQ8 experimental ahora).

---

## Frase de cierre (congelada)

> El comportamiento de cand1 depende del dominio experimental. No existe evidencia de que un único ajuste del gate preserve simultáneamente el comportamiento validado en slalom y elimine los falsos positivos en túnel.

Eso es un **resultado científico**, no un fracaso de H-ATT-d.

---

## Cuatro preguntas (auditoría tipo H6)

### 1. ¿Qué propiedad física pretende medir cand1?

En el dominio de calibración (SLALOM A×C, seed 71):

- Acumulación temprana de correcciones NHC en yaw-error (`Σ|dx_att_z|`).
- Forma **superlineal / feedback** en A (jcorrect) vs acumulado que **decelera** en C (legacy) — ver discriminación cand1 vs cand2.
- Operacionalmente: “la racha A ha entrado en el bucle de refuerzo early-loop”, no “hay un tick ruidoso”.

No pretende medir deriva lateral, ni P_av, ni el déficit Joseph. Solo el **onset del bucle dx_att_z** que el mapa de 11 eslabones ligó a ese régimen en slalom.

### 2. ¿Esa propiedad sigue teniendo el mismo significado en túnel?

**No hay evidencia de que sí.** En túnel:

| Hecho | Por qué rompe la equivalencia |
|-------|-------------------------------|
| NHC solo arma tras GPS outage (~10 s escenario) | Epoch `t_s` ≠ tiempo de escenario; “arranque” = onset NHC en dead-reckoning, no el mismo régimen cinemático que slalom t≈0 |
| Latch @ `t_s=0` en A/B/C/D | Dispara antes de poder observar *forma* superlineal — basta un tick |
| C/D (legacy) disparan en túnel; en slalom no | El control negativo de calibración **no** se traslada |
| A/B: `\|dx_z\| ≳ T₂` de forma sostenida tras onset | No es spike aislado; régimen de magnitud distinto al feedback lento de slalom |
| Dirty: sumabs/T₂ ~2870× en tick 1 | Escala incompatible con el umbral absoluto de slalom |

La cifra `Σ|dx_att_z|` sigue siendo “cuánto se corrigió yaw vía NHC”, pero **deja de ser un proxy fiable de “mismo bucle early-loop de slalom A”**.

### 3. ¿Qué evidencia apoya la equivalencia slalom ↔ túnel?

Casi ninguna, más allá de la **homonimia del símbolo**:

- Misma fórmula (`sumabs`, T₂, tmax) y misma acción downstream (H-ATT-d).
- Ambos escenarios usan NHC y el mismo estado ATT_Z.

Eso no es equivalencia de propiedad; es reutilizar un detector fuera de su dominio de validación (P3-C solo en slalom).

### 4. ¿Qué evidencia la contradice?

| Brazo / dato | Resultado | Lectura |
|--------------|-----------|---------|
| H-ATT-d P2-tunnel | C/D ×19–21 vs ctrl | Intervención armada por fuego inválido de cand1 |
| E1 gracia | No preserva G1∧G2∧G3; N=1 salva C pero no A/B; N=32 rompe A@0,39 y solo retrasa fuego | No es “solo transitorio de arranque” |
| E2 Pzz-freeze | C nofire; B/D siguen @0; A aún temprano | No es “solo umbral / escala absoluta” |
| E1+E2 | Rompe slalom C (G3) | Correcciones no componen hacia un gate universal |
| Escala ctrl | Pzz túnel onset ~29× slalom; dirty ×2870 en sumabs/T₂; Pzz colapsa tick 2 | Mundos distintos, no un factor O(1) |

Patrón: el observable **deja de representar la misma propiedad** al cambiar de escenario — no meramente “T₂ mal calibrado”.

---

## Qué queda abierto / cerrado

| Pieza | Estado |
|-------|--------|
| Mapa early-loop 11 eslabones (slalom) | **Cerrado** (sesión 2026-07-19) |
| Familia δx post-hoc (b1 / H-ATT-c / λ=1) | **Cerrada** como vía |
| cand1 como discriminante **dentro de slalom** A×C | **Válido en su dominio** (P3-C) |
| cand1 como gate **entre dominios** (slalom↔túnel) | **FAIL de generalización** — esta nota |
| H-ATT-d (hipótesis unobs / Joseph) | **Abierta** — acotado el *espacio de instrumentos* para contrastarla, no la hipótesis |
| P2-tunnel / P4 bajo cand1 actual | **No continuar** hasta instrumento nuevo o alcance explícitamente slalom-only |
| Búsqueda de T₂ / cand2 inmediato | **Fuera de orden** |

---

## Decisiones diferidas (no abrir hoy)

Tras este cierre, elegir **una** línea (preregistro aparte):

1. **Detector de dominio** — p.ej. no armar cand1 (ni H-ATT-d) en túnel hasta tener validación propia; scorecard H-ATT-d slalom-only con P2-tunnel = N/A explícito.  
2. **Observable más fundamental** — invariante entre escenarios (misma propiedad física, otra operacionalización), *después* de responder qué se quiere detectar en túnel.  
3. **No** retocar T₂ ni inventar cand2 por inercia.

---

## Prohibido al retomar

- Tratar el FAIL de P2-tunnel H-ATT-d como evidencia contra unobs sin gate saneado o alcance restringido.  
- “Subir el umbral un poco” como siguiente experimento.  
- Fusionar OQ9 con este cierre de instrumento.
