# 19 — EKF Explorer (Unity + Cesium) — instrumento científico

**Estado:** preregistro de plataforma **CONGELADO** (v0.1) — 2026-07-19  
**Tipo:** instrumento de observación / contraste de hipótesis — **no** demo de producto.  
**Pausa experimental OQ8:** intacta (D22). Este doc **no** abre H-ATT-d ni cand2.

Patrón de programa: [reference/OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md](reference/OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md).

---

## 1. Pregunta que responde la plataforma

> ¿Cómo observar de forma fiable (geo-referenciada, sincronizada en tiempo, multi-capa) fenómenos del EKF que **ya están caracterizados** en CSV/auditorías, de modo que se puedan contrastar hipótesis e instrumentos entre dominios?

No responde: “¿el filtro va bien?” ni “¿qué umbral usar?”.

---

## 2. Separación de niveles (obligatoria)

```
Hipótesis / OQ          (H-ATT-d, OQ8, …)     ← fuera de alcance del MVP
        ↓
Propiedad / observable  (onset, NIS, NHC arm, …)
        ↓
Session pack v1         ← contrato de datos del Explorer
        ↓
EKF Explorer (Unity+Cesium)  ← instrumento
```

El Explorer **consume** session packs. No calcula el EKF. No redefine umbrales.

---

## 3. Capacidades PASS (instrumento) vs fuera de alcance

| ID | Capacidad | MVP v0.1 |
|----|-----------|----------|
| **V1** | Superponer ≥2 trayectorias geo (estimate / truth / brazo B) | **PASS** |
| **V2** | Scrubber temporal único + sync de eventos críticos | **PASS** |
| **V3** | Capas de eventos: GPS outage, GNSS reject, NHC/ZUPT applied, fire/latch si viene en pack | **PASS** |
| **V4** | Panel de series internas alineadas al scrubber (NIS, innov, drift, flags) | **PASS** |
| **V5** | Proveniencia: origen LLA, rutas CSV fuente, seed, brazo | **PASS** |
| **V6** | Live UDP `UnityTelemetryPacket` (0x4E55) | Stretch (post-MVP) |
| **V7** | Editar umbrales / re-correr EKF desde Unity | **Prohibido** |
| **V8** | “Modo presentación” sin series ni eventos | **Prohibido** en v0.1 |

**PASS plataforma v0.1:** V1∧V2∧V3∧V4∧V5 sobre al menos un pack de sim (slalom/túnel) y uno de real_run baseline.

---

## 4. Contrato de datos — Session pack v1

**Schema id:** `navicore.ekf_explorer.session/v1`  
**Exportador:** `tools/export_ekf_explorer_session.py`  
**Artefactos:** `docs/ekf_explorer/sessions/<name>/session.json` (+ opcional `session.czml`)

### Campos mínimos

| Campo | Rol |
|-------|-----|
| `origin` | `{lat_deg, lon_deg, alt_m}` — ancla NED↔LLA |
| `tracks[]` | id, role (`estimate`\|`truth`\|`compare`), samples `{t_s, lat_deg, lon_deg, alt_m, yaw_rad?}` |
| `events[]` | `{t_s, type, label, track_id?}` |
| `series{}` | nombre → `[{t_s, v}]` (NIS, drift_m, nhc_applied, …) |
| `provenance` | paths, scenario, seed, notes |

Conversión NED→LLA: misma convención que `geodesy.hpp` / origen de escenario (sim: Barcelona 41.3874, 2.1686, 12; real_run: ref del replay).

Fuentes admitidas en MVP:

1. `docs/telemetria_navicore.csv` / `*_telemetry.csv` (NED + nis + drift)  
2. `replay_output.csv` + `constraint_pipeline_audit.csv` (+ opcional `gnss_nis_audit.csv`)

---

## 5. Arquitectura Unity (MVP)

```
ekf_explorer/
  README.md                 # install + open
  Sessions/                 # packs exportados (o symlink a docs/ekf_explorer/sessions)
  UnityProject/             # proyecto Unity (crear tras install Hub+Editor)
    Packages/manifest.json  # Cesium for Unity
    Assets/NaviCore/
      Scripts/
        SessionPack.cs
        SessionLoader.cs
        TrajectoryLayer.cs
        EventMarkerLayer.cs
        TimeScrubber.cs
        SeriesPanel.cs
        NedToLla.cs
      Scenarios/
        EkfExplorerBootstrap.cs
```

**Cesium:** tileset mundo + entidad por track; eventos = billboards/pines en t_s; scrubber mueve el reloj de sesión (no el time-of-day del globo salvo opción).

**Token:** Cesium ion (cuenta gratuita) — variable de entorno / ScriptableObject local **no commitear secretos**.

---

## 6. Principios (instrumento, no demo)

1. Toda vista es **falsable**: si falta serie/evento, el panel lo dice (no inventar).  
2. Dos tracks = contraste de hipótesis / brazos — no “bonito”.  
3. Tiempo es la dimensión primaria; el mapa es proyección.  
4. Sin modo que oculte NIS/eventos en v0.1.  
5. No mezclar con retune de cand1/T₂ desde la UI.  
6. **Stage II — Interactive EKF Laboratory:** produce≠interpreta. Contratos: RunPackage → Snapshot → Lab Session → Workspace (antes de Diff/plugins). Prueba de fuego: inspección sin conocer el EKF. [`LABORATORY.md`](../ekf_explorer/LABORATORY.md).

---

## 7. Install checklist (humano)

1. Unity Hub (`winget install Unity.UnityHub` o [unity.com/download](https://unity.com/download)).  
2. Editor **2022.3 LTS** (o el marcado compatible en README Cesium for Unity).  
3. Módulos: Windows Build Support (IL2CPP opcional).  
4. Abrir/`crear` proyecto en `ekf_explorer/UnityProject`.  
5. Package Manager → add **Cesium for Unity** (registry Cesium / OpenUPM según doc vigente).  
6. Cesium ion token (cuenta gratuita) → `CesiumIonToken` local.  
7. `python tools/export_ekf_explorer_session.py --preset slalom` y cargar `session.json`.

---

## 8. Fuera de alcance v0.1

OQ8 experimental; H-ATT-d re-run; cand2; edición de P/Q/R; multiplayer; mobile; photorealistic mesh obligatorio (ion default terrain/imagery basta).

---

## 9. Criterio de “plataforma útil”

Si en <30 s se puede: cargar un pack, scrub hasta un GPS outage, ver NHC applied + NIS, y comparar dos tracks — el instrumento cumple V1–V5. Si solo se ve un coche en un mapa bonito — **FAIL** de diseño.
