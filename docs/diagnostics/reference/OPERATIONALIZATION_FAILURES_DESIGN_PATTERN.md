# Operationalization Failures — Design Pattern

**Tipo:** conocimiento transversal del **programa** (no protocolo de un GAP).  
**Fecha:** 2026-07-19  
**Estado:** congelado como patrón; no abre experimento nuevo.

No está ligado al EKF ni al Jacobiano en exclusiva. Registra una estructura que ya se ha repetido y que debe asumirse al diseñar el siguiente instrumento.

---

## Cadena normativa

```
Fenómeno / hipótesis
        ↓
   Propiedad (qué se quiere detectar o controlar)
        ↓
   Observable candidato
        ↓
   Operacionalización  ←  aquí suele romperse la generalidad
        ↓
   Implementación / gate / controlador
        ↓
   Experimento
```

**Regla:** un FAIL en el experimento no implica FAIL de la hipótesis.  
Primero localizar el nivel. Si la operacionalización no es invariante entre dominios, documentar el dominio de validez del instrumento y **parar** — no retocar el parámetro que define el umbral.

---

## Caso A — Γ̄ (GAP-5 v1)

| Nivel | Contenido |
|-------|-----------|
| Hipótesis | H5-PoC — política NHC adaptativa puede mejorar sin romper accepts |
| Fenómeno | Estrés / compresión del régimen NHC–covarianza |
| Observable | Γ (instantáneo) → Γ̄ (media / EWMA) |
| Operacionalización | Bandas + histéresis + dwell sobre Γ̄ → N |
| Resultado | Instancia v1 **cerrada**: Γ̄ no preserva el burst offline; no generaliza como proxy operacional |
| Hipótesis | **Permanece abierta** (refutada la operacionalización, no H5 en abstracto) |
| Anclas | [15-gap5-passive-outcome.md](../15-gap5-passive-outcome.md) · K11–K13 · D6 |

**Frase tipo:** el fenómeno es válido en el corpus; **esa** operacionalización es insuficiente.

---

## Caso B — cand1 (Jacobiano / H-ATT)

| Nivel | Contenido |
|-------|-----------|
| Hipótesis | H-ATT-d — Z no observado en H antes de S/K/Joseph |
| Propiedad pretendida | Onset del bucle early-loop vía crecimiento de Σ\|dx_att_z\| (slalom A vs C) |
| Observable / detector | cand1 |
| Operacionalización | `sumabs ≥ T₂`, `t ≤ tmax`, latch → acción |
| Resultado | **Dominio de validez caracterizado:** ✅ slalom A×C · ❌ túnel (gracia E1 y norm E2 no recuperan G1∧G2∧G3) |
| Hipótesis | **Permanece abierta** — lo acotado es el **espacio de instrumentos** capaces de contrastarla, no H-ATT-d en sí |
| Anclas | [CAND1_GENERALIZATION_REVIEW.md](../../benchmarks/jacobian_imu_ab/CAND1_GENERALIZATION_REVIEW.md) · §13.21–§13.22 · OQ8 |

**Frase tipo:** el comportamiento de cand1 depende del dominio experimental; no hay evidencia de un único ajuste de gate que preserve slalom y limpie túnel.

**No decir:** “hemos acotado H-ATT-d.”  
**Sí decir:** hemos acotado el espacio de instrumentos capaces de contrastar H-ATT-d (sin FP túnel, con no-regresión slalom, con significado comparable entre escenarios). Ese instrumento **aún no existe**.

### Familias residuales (cuando se retome — no ahora)

| Familia | Contenido | Implicación |
|---------|-----------|-------------|
| **A** | La propiedad (Σ\|dx_att_z\| como firma de onset) solo describe bien slalom | Dominio de validez limitado del instrumento — aceptable si se declara alcance |
| **B** | La propiedad es la correcta; falla el gate absoluto / operacionalización | Otra forma de medir la **misma** propiedad — no otro T₂ |

Ambas son cuestiones de **representación**, no de tuning. Prohibido: barridos de umbral/ganancia como siguiente paso.

---

## Isomorfismo (lectura de programa)

| Investigación EKF | Investigación Jacobiano |
|-------------------|-------------------------|
| H5 | H-ATT-d |
| Γ̄ | cand1 |
| Operacionalización falla | Operacionalización falla |
| Hipótesis abierta | Hipótesis abierta |
| Se documenta el fallo del instrumento | Idem |

Misma disciplina: **propiedad → observable → operacionalización → controlador**; no saltar al controlador ni “tocar el número” tras un FAIL informativo.

---

## Disciplina derivada (checklist)

1. Preregistrar PASS/FAIL **antes** de código.  
2. Distinguir hipótesis ≠ instrumento ≠ umbral.  
3. Congelar resultados negativos útiles (dominio de validez).  
4. Tras FAIL de generalización: documento de instrumento (tipo H6 / CAND1 review) **antes** de rediseñar variables.  
5. Magnitudes absolutas / controles (`CORR_ABS_SCALE` y análogos) antes de creer correlaciones.  
6. No abrir la siguiente OQ experimental en el mismo aliento que el cierre del instrumento.

---

## Qué no es este documento

- No es preregistro de OQ8 reformulada (“¿existe onset invariante entre dominios?”).  
- No autoriza cand2, T₂-hunt, ni GAP-5 v3.  
- No cierra H5 ni H-ATT-d.

## Fase del programa (frase única)

> Ya no estáis intentando descubrir qué ocurre; estáis construyendo herramientas fiables para observar algo que ya sabéis que existe.

Stage I (EKF / regímenes) = modelo parcial cerrado.  
Hilo Jacobiano = acotado, abierto en el nivel instrumento.  
Etapa II (cuando se abra) = visualizar / instrumentar / explotar — no reabrir “¿hay bug?”. Un visor futuro (p.ej. Unity+Cesium) solo encaja si nace como **instrumento científico** (trayectorias, eventos, observables internos, contraste de hipótesis), no como demo.

**Pausa (D22):** estabilizar trazabilidad; no abrir línea experimental inmediata.
