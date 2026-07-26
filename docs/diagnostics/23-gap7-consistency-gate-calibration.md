# GAP-7 — ¿Maniobra mal gateada o multipath genuino?

**Alcance:** solo diagnóstico. Sin cambios en `ins_ekf.hpp` / `ins_ekf.cpp`.  
**Ruta:** `REF_19082026` v2 (post Bowring).  
**Datos:** IMU cruda de `docs/benchmarks/real_run_19082026_baseline/real_run_replay.csv`;  
rechazos de `…/REF_19082026/v2/gnss_nis_audit.csv`.  
**JSON:** `docs/benchmarks/ekf_v2_ab_3routes/gap7_consistency_imu_diag.json`.

Contexto previo: [`22-gap7-ref-vertical-divergence.md`](22-gap7-ref-vertical-divergence.md)  
(80.7% de reason=3 en ventanas de pico H; gaps ≤~3 s).

---

## Umbrales actuales (`ins_ekf.hpp`)

| Macro | Valor | Rol |
|-------|------:|-----|
| `NAVICORE_INS_EKF_CONSISTENCY_MAX_GAP_S` | **2.0** | Si gap &gt; esto → no dispara reason=3 (reaquisición vía NIS) |
| `NAVICORE_INS_EKF_CONSISTENCY_MAX_POS_JUMP_M` | **120** | Tope duro de \|innov_h\| en track continuo |
| `NAVICORE_INS_EKF_CONSISTENCY_POS_MARGIN_M` | **30** | Margen fijo en el gate de posición |
| `NAVICORE_INS_EKF_CONSISTENCY_SIGMA_K` | **6** | Multiplicador de σ_h = √(Pnn+Pee) |
| `NAVICORE_INS_EKF_CONSISTENCY_MAX_VEL_JUMP_MPS` | **35** | Tope de \|v_gps − v_ins\|_h |

Gate de posición (código ~1304–1310):

```text
plausible = v_h * gap_s + POS_MARGIN + SIGMA_K * sigma_h
gate      = min(MAX_POS_JUMP, max(plausible, 40))
reject si innov_h > gate
```

### ¿El umbral de velocidad es más estricto que una frenada urbana?

**No — es muy holgado.**

- `MAX_VEL_JUMP = 35 m/s` entre GPS e INS en un gap corto (~1 s a 1 Hz).  
- Una frenada dura urbana (~7–8 m/s²) en 1 s cambia la velocidad ~**7–8 m/s**, no 35.  
- Incluso atribuyendo 35 m/s a un gap de 3 s → ~12 m/s² equivalentes, aún por encima de una frenada fuerte típica.

El disparador observado en REF no es el gate de velocidad “apretado”, sino **\|innov_h\| grande** frente al **tope duro de 120 m** (y/o `plausible` cuando σ_h no abre el gate).

Entre los 83 rejects: mediana `innov_h` ≫ 120 m; casi todos superan el hard cap de posición.

---

## (a) Dinámica IMU cruda: picos vs quiet

Proxy de dinámica: `a_excess = ||a_body|| − g` y `||ω||` (rad/s) sobre muestras IMU del CSV (≈100 Hz).  
Ejes del teléfono no están alineados vehículo; por eso se usa norma, no un eje “longitudinal” inventado.

| Ventana | a_excess p50 / p95 / max | frac a_excess&gt;2 | ‖ω‖ p50 / p95 / max (deg/s max) | GPS speed mean/max | Δspeed max (fix a fix) | rejects | innov_h rej min–max |
|---------|--------------------------:|------------------:|--------------------------------:|-------------------:|-----------------------:|--------:|--------------------:|
| **peak 220–240** | 0.30 / 0.92 / 2.03 | 0.1% | 0.034 / 0.062 / **7.4°/s** | 16.9 / 18.6 | 0.75 m/s | 11 | **104–235 m** |
| **peak 390–420** | 0.29 / 0.87 / 2.84 | 0.1% | 0.024 / 0.059 / **10.7°/s** | 18.8 / 21.7 | 0.70 m/s | 16 | **75–243 m** |
| **peak 580–640** | 0.12 / 0.72 / 5.72 | 0.5% | 0.044 / 0.336 / **31.7°/s** | 4.7 / 11.3 | 1.56 m/s | 36 | **117–492 m** |
| quiet 300–350 | 0.31 / 1.00 / 2.61 | 0.2% | 0.033 / 0.085 / 18.0°/s | 19.2 / 21.7 | 0.76 m/s | **0** | — |
| quiet 450–530 | 0.27 / 0.90 / 3.77 | 0.2% | 0.027 / 0.062 / 25.5°/s | 18.1 / 22.4 | 1.25 m/s | **0** | — |

### Lectura

- En **220–240** y **390–420** la IMU es **indistinguible del quiet**: misma `a_excess`, giros &lt;11°/s, GPS speed estable (~17–22 m/s) con Δspeed &lt;1 m/s/fix.  
  Mientras tanto `innov_h` en rejects es **100–240 m**. Eso **no** es una frenada/giro brusco mal calibrado: el vehículo (según IMU+speed) circula normal y el GNSS (vs predictor) salta cientos de metros.

- En **580–640** hay un poco más de yaw (max ~32°/s; p95 0.34 rad/s) y speed más baja (~5 m/s media) — posible tramo lento / curvas suaves — pero **sigue sin explicar** innov_h de **120–500 m**. `a_excess` p95 (0.72) es **menor** que en quiet; solo un max puntual 5.7 m/s² (0.5% samples &gt;2).

---

## (b) Umbral vs dinámica observada

| Chequeo | ¿El umbral es el problema? |
|---------|----------------------------|
| Vel jump 35 m/s | **No.** Holgado frente a dinámica urbana medida (Δspeed GNSS ≤~1.6 m/s/fix; IMU sin frenadas extremas sostenidas). |
| Pos hard cap 120 m | Es el techo que están cruzando los rejects. Con IMU normal, un salto GNSS–INS de &gt;120 m en gap ≤2–3 s es **físicamente inverosímil como maniobra**; coherente con multipath / fix espurio / o predictor ya divergido. |
| POS_MARGIN 30 + 6σ | No se retocan aquí; el síntoma no es “frenada de 8 m/s² rechazada por margen corto”. |

---

## Conclusión

**Multipath / GNSS espurio (o INS ya divergido) correctamente marcado por el gate de consistencia de posición — no maniobra urbana mal gateada por un umbral de velocidad demasiado estricto.**

Evidencia fuerte:

1. IMU en las ventanas de rechazo ≈ quiet (sobre todo en los dos primeros picos).  
2. Speed GNSS estable; sin Δv que merezca acercarse a 35 m/s.  
3. `innov_h` en rejects sistemáticamente del orden de **10² m**, compatible con el hard cap de 120 m, no con una calibración “anti-frenada”.

**No se propone cambio de umbral en esta fase.** Relajar `MAX_POS_JUMP` / márgenes sin más análisis podría **aceptar basura GNSS** justo cuando el gate está haciendo su trabajo. Si más adelante se toca calibración, el candidato no es `MAX_VEL_JUMP` (ya holgado), sino entender la cadena *GNSS malo → divergencia → innov_h enorme → reason=3 en ráfaga* (reaquisición / gating pos vs NIS tras gaps cortos encadenados).

---

## Cómo reproducir

```text
python tools/audits/audit_gap7_consistency_imu_windows.py
```

Escribe `docs/benchmarks/ekf_v2_ab_3routes/gap7_consistency_imu_diag.json`.
