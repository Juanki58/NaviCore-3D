# C7 labeling binding — ordinales antes de figuras H6

**Estado:** CONGELADO pre-ejecución H6 (D18)  
**No altera** O1–O5, C1–C7, ni v1.2 científico.  
**Solo** operacionaliza el vocabulario `{bajo|pico|alto|meseta|N/A}` de §5.2.

## Orden obligatorio

1. Escribir en `observable_characterization.json` las **estadísticas numéricas** por (Oi, config, R).  
2. **Sin abrir figures/** para etiquetar, aplicar esta tabla.  
3. Escribir ordinales en el mismo JSON campo `c7_ordinal`.

## Estadísticas numéricas requeridas (por celda)

Para señal `x(t)` del observable en la ventana del régimen R:

- `n_samples`
- `median`
- `max`
- `p95`
- `t_at_max` (relativo al inicio del régimen, s)
- `iqr` (p75−p25)
- `baseline_median_R0` (median de R0 en la misma config; si R=R0, igual a median)

Si `n_samples == 0` → ordinal **N/A** (stop).

## Reglas relativas (únicas permitidas)

Sea `m = median`, `M = max`, `b0 = baseline_median_R0` (si `b0 == 0`, usar `b0 = max(1e-12, p95_R0)`).

| Ordinal | Condición (primera que cumpla, en este orden) |
|---------|-----------------------------------------------|
| **N/A** | Régimen indefinido en esta config (p.ej. R3 sin fix#4) o `n_samples==0` |
| **pico** | `M >= 2.0 * b0` **y** `M >= 1.5 * m` **y** el máximo está en el tercio temporal central del régimen **o** `t_at_max` dentro del 20–80% de la duración |
| **alto** | `m >= 1.5 * b0` y no es **pico** |
| **meseta** | `iqr <= 0.35 * max(m, 1e-12)` **y** `m` entre `0.7*b0` y `1.3*b0` (cerca de baseline) **o** (si R≠R0) `iqr <= 0.35*m` y `0.8*m <= M <= 1.25*m` |
| **bajo** | `m <= 1.25 * b0` y no meseta/alto/pico |
| **default** | Si nada aplica → **bajo** si `m <= b0` else **alto** |

**Prohibido:** ajustar estos factores (2.0, 1.5, 0.35, …) tras ver resultados. Si el binding resulta inadecuado → outcome metodológico / v1.3, no retune mid-H6.

## Match con Paso 0

Campo aparte `paso0_contrast`: `{match|higher|lower|peak_vs_flat|flat_vs_peak|unknown}` comparando ordinal observado vs celda Paso 0 (tratando `?` como unknown esperado).
