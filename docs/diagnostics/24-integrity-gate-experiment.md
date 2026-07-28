# Integrity gate experiment — qué es y para qué sirve

**Política:** solo inyección por software (teleport / mentira de velocidad).  
**No** spoof RF (ilegal en ES/EU sin autorización CNMC).

**Artefactos:** [`docs/benchmarks/integrity_gate_experiment/`](../benchmarks/integrity_gate_experiment/)  
**Código:** `tests/unit/test_integrity_gate_sweep.cpp` · campaña `tools/campaigns/run_integrity_gate_experiment.py`  
**Contexto GAP-7 (multipath real en REF):** [`22-gap7-ref-vertical-divergence.md`](22-gap7-ref-vertical-divergence.md)

---

## En una frase

El filtro **no solo usa el GPS**: comprueba si el fix es **físicamente compatible** con lo que dice la IMU / el estado INS.  
Si el GPS “está presente” pero es absurdo (salto de cientos de metros en 0,2 s mientras el coche sigue recto), **se rechaza** (`reject_reason=3`, INCONSISTENT) y **no se deja arrastrar** la posición.

Eso es **integridad**, no anti-jam militar ni CRPA.

---

## Ejemplos que se entienden sin ser ingeniero

### 1) App de flota / entrega (teléfono en el coche)

**Situación:** vas por una avenida. El GPS, por multipath (edificios de cristal), de repente te dibuja **en la calle paralela**, 150–300 m al lado. La app sin integridad cree que has “teletransportado” y puede:

- marcar una parada falsa,
- calcular un desvío absurdo,
- o disparar una alerta de “el conductor se ha salido de ruta”.

**Con el gate:** la IMU dice “seguimos a ~40 km/h hacia delante”; un salto lateral de 200 m en menos de 2 s es imposible → **fix rechazado**. La app sigue con la costa inercial hasta que el GPS vuelva a ser coherente.

**Potencial de uso:** menos falsas alarmas y kilometraje más creíble en cañón urbano (el mismo régimen que vimos en REF / GAP-7).

### 2) Seguro UBI / “pago según cómo conduces”

**Situación:** sales de un túnel o de un parking. El primer fix GPS a veces “salta” o inventa un frenazo aparente. El scoring del seguro interpreta un hard-brake o un accidente near-miss.

**Con el gate:**

- salto de posición absurdo en track continuo → rechazado como inconsistente;
- tras un **hueco largo** (túnel de verdad, gap &gt; 2 s) el mismo salto **no** se etiqueta como spoof: se trata como reaquisição (NIS), que es lo correcto.

**Potencial de uso:** integridad como señal de confianza para el score (“este fix es sospechoso”) sin hardware OBD.

### 3) Ataque spoof suave (inyección SW; en campo sería RF ilegal sin permiso)

**Situación:** alguien intenta “mover” tu posición 500 m al norte manteniendo `fix_valid=true` (como si el receptor siguiera “contento”).

**Experimento:** teleport SW de 200–800 m con gap corto → **100% rechazado**, estado y covarianza no se derrumban.

**Potencial de uso:** producto civil de *“IMU-consistent integrity”* — no es anti-spoof militar, pero sí un detector de “el GPS miente respecto al movimiento”.

### 4) Mentira de velocidad (GPS dice 216 km/h, IMU ~36 km/h)

**Situación:** el receptor reporta velocidad imposible frente al INS (Δv &gt; 35 m/s en track continuo).

**Experimento:** GPS a 60 m/s con INS a 10 m/s, gap 0,2 s → **INCONSISTENT**, estado retenido.

**Potencial de uso:** trailers / trackers que despiertan y reciben un fix “loco”; no actualizar la verdad del activo hasta coherencia.

### 5) Lo que **no** intenta ser

| No es | Por qué |
|-------|---------|
| Navegar 30 min sin ningún GPS con IMU de móvil | El MEMS deriva; el producto es **detectar mentiras**, no magia inercial eterna |
| Sustituir LiDAR en robots/drones | Ahí el stack es visión + mapas |
| Anti-jam RF / CRPA | Fuera de alcance y de licencia |

---

## Qué midió este experimento (barrido)

| Caso | Esperado | Resultado (claim) |
|------|----------|-------------------|
| Salto &gt; 120 m, gap ≤ 2 s | Rechazo INCONSISTENT + estado fijo | **OK** — 8/8 trials |
| Salto ≤ 2 m, gap corto | **No** clasificar como spoof | **OK** |
| Salto 500 m, gap 3 s (salida de túnel) | **No** reason=3 (reaquisición / NIS) | **OK** |
| Velocidad GPS 60 m/s vs INS 10 | INCONSISTENT | **OK** |
| RapidCheck teleport 200–800 m | INCONSISTENT | **OK** |
| `--safety-inject` spoof +500 m | INCONSISTENT | **OK** |

Corrida bancada: `docs/benchmarks/integrity_gate_experiment/SUMMARY.json` (`claims_ok: true`, UTC 2026-07-28).

Nota de frontera (gap 0,2–1 s): saltos ~20–40 m caen en **NIS** (no spoof); desde ~80 m dispara **INCONSISTENT**. Eso es coherente con `gate = min(120, max(plausible, 40))`.

Regenerar:

```powershell
cmake --build build --target navicore_unit_tests
python tools/campaigns/run_integrity_gate_experiment.py
```

---

## Relación con el orden de mercado

Este experimento es el **paso #1** acordado (integrity first):

1. Demostrar “detectamos GPS incompatible” (este doc).  
2. Aplicarlo a continuidad phone-drive / telemática.  
3. Portar a assets sin alimentación (duty-cycle).

El valor comercial no es “somos el mejor INS del mundo”, sino: **cuando el GPS miente o está sucio, no te creemos a ciegas**.
