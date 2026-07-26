# GAP-6 — Investigación: desfase `log_gps` vs `audit_z`

**Estado:** causa raíz confirmada y **corregida** (Bowring en `ecef_to_lla`).  
Hipótesis inicial (desfase de orígenes CSV↔filtro) **descartada** como causa del offset ~30 m.

**Hallazgo previo (pre-fix):** en las tres rutas v2 de `ekf_v2_ab_3routes`,  
`log_gps − audit_z ≈ (+31 N, 0 E, −27.3 D)` m (ver `gap6_d_axis_autopsy.md`).

---

## PASO 1 — Dos orígenes de referencia

### (a) Origen del CSV (`pos_n/e/d`)

| Ítem | Valor |
|---|---|
| Script | `tools/analysis/parse_mobile_log.py` |
| Líneas | **441–447** (`main`) |
| Conversión | `build_gnss_rows` → `geodesy.lla_to_ned_scalars` (líneas 302–308) |
| Origen | **primera fila de `Location.csv`** del log del móvil |

```441:447:tools/analysis/parse_mobile_log.py
    ref = locations[0]
    ref_lat_deg = ref.latitude
    ref_lon_deg = ref.longitude
    ref_alt_m = ref.altitude

    imu_rows = build_imu_rows(accel, gyro)
    gnss_rows = build_gnss_rows(locations, ref_lat_deg, ref_lon_deg, ref_alt_m)
```

Orígenes LLA reales (primera fila de cada grabación):

| Ruta | lat0 | lon0 | alt0 (m) | Fuente `Location.csv` |
|---|---:|---:|---:|---|
| ALT_16072026 | 41.1921220 | 1.5191053 | 84.400 | `data/real_run/16072026/` |
| REF_19082026 | 41.1557750 | 1.4282767 | 74.900 | `data/real_run/19082026/` |
| JUL17_20260717 | 41.1178752 | 1.2438357 | 74.000 | `data/real_run/20260717_142942/` |

No hay punto fijo hardcodeado en el parser: cada CSV ancla NED a su propio primer fix.

### (b) Origen del filtro en replay (`ref_lat/lon/alt`)

Cuando la campaña **no** pasa `--gnss-ref-*` (`gnss_ref_overridden == false`), el seed usa:

| Ítem | Valor |
|---|---|
| Fichero | `src/core/ins_ekf_15_state.cpp` |
| Líneas | **103–106** |
| Valor | **hardcoded Barcelona** `{41.3874, 2.1686, 12.0}` |

```103:106:src/core/ins_ekf_15_state.cpp
bool InsEkf15State::seed_from_ned_pos(const double pos_ned_m[3], NavDomain domain)
{
    const double barcelona_ref_deg[3] = {41.3874, 2.1686, 12.0};
    return seed_from_ned_pos(pos_ned_m, barcelona_ref_deg, domain);
```

Ese LLA se copia a `filter->ref_*` vía `ins_ekf_init` (`src/core/ins_ekf.cpp` ~2263–2265).

Llamada desde replay (rama sin override):

```5326:5328:src/targets/generic_pc/real_run_replay.cpp
                } else {
                    seeded = filter_impl->seed_from_ned_pos(row.pos_ned, NAVICORE_DOMAIN_AIR);
                }
```

El log de campaña confirma esa rama:  
`inicializado @ t=… | ref NED=(0.000, 0.000, -0.000) m`  
(sin imprimir `ref LLA=…`, que solo aparece si `gnss_ref_overridden`).

`pos_` del filtro se rellena con el **NED del CSV tal cual** (mismo vector numérico), no se re-proyecta al origen Barcelona.

---

## Cómo se generan `log_gps` y `audit_z`

| Señal | Fuente | Transformación |
|---|---|---|
| `log_gps` / `last_gps_pos_ned` | `row.pos_ned` del CSV | Ninguna (copia directa al final del run) |
| `audit_z` | `simulate_adapter_measurement_z` + mismo camino en `InsEkf15State::update_gnss` | `NED(csv) → LLA` con `ref=Barcelona` → `LLA → NED` otra vez con el mismo ref |

Adapter de auditoría:

```1103:1120:src/targets/generic_pc/real_run_replay.cpp
void simulate_adapter_measurement_z(
    float ref_lat_deg,
    ...
    const geodesy::NED measurement = geodesy::lla_to_ned(point_lla, ref);
    z_ned[0] = measurement.north_m;
    ...
```

Update real del filtro (`InsEkf15State::update_gnss`, ~240–249): mismo `ned_to_lla(ref_Barcelona, csv_ned)` y luego, dentro de `InsEkfFilter::update_gnss`, `lla_to_ned` otra vez.

Un roundtrip NED→LLA→NED con el **mismo** ref debería ser identidad. En los datos **no lo es**.

---

## PASO 2 — Comparación numérica

### 2.1 ¿El desfase de orígenes explica el offset?

Origen ruta expresado en NED Barcelona (`lla_to_ned(route0, BCN)`):

| Ruta | N (m) | E (m) | D (m) |
|---|---:|---:|---:|
| ALT | −21484 | −54486 | 196 |
| REF | −25459 | −62139 | 290 |
| JUL17 | −29519 | −77664 | 479 |

Offset observado `log_gps − audit_z`: **≈ (+31, 0, −27.3)** en las tres rutas.

**No coincide** ni en magnitud ni en dirección (decenas de km vs decenas de metros; E no es ~0 en el desfase de orígenes).

### 2.2 ¿Qué sí reproduce el offset?

Bug en `ecef_to_lla` (fórmula de Bowring incorrecta) en:

- `src/core/geodesy.cpp` ~61–64  
- `tools/lib/geodesy.py` ~61–65  

Implementación actual (incorrecta) del término en `z`:

```text
z + (e² * (1−e²) * a * sin³θ) / (1−e²)  =  z + e² * a * sin³θ
```

Bowring correcto:

```text
z + e'² * b * sin³θ ,  con  b = a(1−f),  e'² = e²/(1−e²)
```

Efecto medido (ref = Barcelona hardcodeado):

| Test | Resultado |
|---|---|
| `lla_to_ecef(BCN)` → `ecef_to_lla` (código actual) | `(41.387120, 2.1686, −15.367)` ≠ BCN |
| Misma ECEF → Bowring corregido | `(41.3874, 2.1686, 12.0)` = BCN |
| Roundtrip `NED(0,0,0)` vía código actual | `(−31.055, 0, +27.367)` |
| `0 − roundtrip(0)` | **(+31.055, 0, −27.367)** |
| `csv_last − roundtrip(csv_last)` ALT | **(+31.022, ~0, −27.338)** |
| Offset medido ALT `log_gps − audit_z` | **(+30.837, ~0, −27.337)** |

Coincide en magnitud y dirección (residuo &lt; 0.2 m en N, &lt; 0.01 m en D; la diferencia residual es float/`audit` vs CSV double).

Con Bowring corregido, el roundtrip del último fix ALT vuelve a ser identidad a nivel de ruido numérico (~1e-9 m).

### 2.3 Por qué es idéntico en tres rutas distintas

1. Las tres campañas seedan con el **mismo** ref Barcelona.  
2. El bias del Bowring roto, proyectado a NED en ese ref, es **casi constante** para desplazamientos de unos pocos km.  
3. `log_gps` nunca pasa por `ecef_to_lla`; `audit_z` / la medida interna del EKF sí.  
4. Por eso el offset no depende de la geografía de la ruta (Tarragona/Costa), solo del ref hardcodeado + el bug de geodesia.

El desajuste CSV-origen vs Barcelona (decenas de km) es un **segundo problema real** (el filtro etiqueta el frame con un LLA que no es el del CSV), pero **no es la causa** del offset `log_gps` vs `audit_z` de ~30 m.

---

## PASO 3 — Conclusión (veredicto final)

### Causa raíz (confirmada + fix aplicado)

**Bug matemático en `ecef_to_lla` (método de Bowring)** — **no** un desfase de orígenes CSV↔filtro.

Dos errores en la misma función (`src/core/geodesy.cpp`, espejo en `tools/lib/geodesy.py`):

1. Término de latitud: `e²·a·sin³θ` en vez de `e'²·b·sin³θ`.
2. `theta`: usaba `a·(1−e²)` en vez del semieje menor `b = a·√(1−e²)`.

Expuesto por el adaptador PC  
`CSV_NED → ned_to_lla(ref) → lla_to_ned(ref)`  
(`simulate_adapter_measurement_z` + `InsEkf15State::update_gnss`).

Tras el fix: roundtrip unitario en mm (`tests/unit/test_geodesy_bowring.cpp`);  
`log_gps − audit_z ≈ 0` en las tres rutas re-ejecutadas. Impacto: `gap6_bowring_fix_impact.json`.

### Firmware Pico — no afectado

`src/targets/pico2_hardware/` **no** se ve afectado por este bug en operación GNSS:

- El DUT llama `ins_ekf_update_gnss` con `GpsSample.position` ya en **LLA** (NMEA/UBX).
- El camino de medida en firmware es solo **LLA→NED** (`lla_to_ned` / ECEF forward), que estaba correcto.
- El roundtrip roto **NED→LLA→NED** es propio del adaptador PC `InsEkf15State` + auditoría de `real_run_replay.cpp`.
- El problema queda contenido a la **toolchain de PC / benchmarks / replay**, no al filtro en vuelo sobre Pico.

### Problema secundario (sigue abierto, distinta escala)

Seed sin `--gnss-ref` usa Barcelona hardcodeado mientras el CSV está anclado al primer fix de cada ruta (~20–30 km). Eso sigue siendo un desajuste de etiqueta LLA; **no** causaba el offset de ~30 m.

### Hipótesis descartadas

1. ~~Desfase de orígenes CSV vs filtro como causa del (+31,0,−27.3)~~ — magnitudes no cuadran; el offset era el bias Bowring.  
2. ~~Última fila GPS distinta entre log y audit~~ — mismo `pos_ned`; el delta salía del roundtrip.  
3. ~~Redondeo/suavizado del CSV~~ — descartado.

### Nota para informes históricos

Cifras de deriva absoluta generadas por `real_run_replay.cpp` **antes** de este fix pueden incluir un offset sistemático de origen de hasta **~30 m** (sobre todo en vertical / 3D al mezclar `log_gps` con estado del filtro). Ver nota en `SUMMARY.md` y README Evidence.

---

## Referencias de código

| Pieza | Ubicación |
|---|---|
| Origen CSV = `locations[0]` | `tools/analysis/parse_mobile_log.py:441-447` |
| Seed Barcelona hardcodeado | `src/core/ins_ekf_15_state.cpp:103-106` |
| `ref_*` en init | `src/core/ins_ekf.cpp:2263-2265` |
| Rama seed sin override | `real_run_replay.cpp:5326-5328` |
| Adapter audit z | `real_run_replay.cpp:1103-1120` |
| NED→LLA en update | `ins_ekf_15_state.cpp:240-249` |
| `ecef_to_lla` (fix Bowring) | `src/core/geodesy.cpp`, `tools/lib/geodesy.py` |
| Test regresión | `tests/unit/test_geodesy_bowring.cpp` |
| Autopsia D-axis | `docs/benchmarks/ekf_v2_ab_3routes/gap6_d_axis_autopsy.md` |
| Impacto post-fix | `docs/benchmarks/ekf_v2_ab_3routes/gap6_bowring_fix_impact.json` |
