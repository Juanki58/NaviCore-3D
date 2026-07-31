# Campaña adversaria DUT — desde 2026-08-03

**Espíritu:** pruebas lo más duras / “disparatadas” posibles **sin gastar dinero inútil** y **sin claims ilegales** (no spoof RF, no interferir a terceros).

**Objetivo de producto:** algo **robusto, serio, ágil y barato** — cada fallo debe producir: log + hipótesis + cambio mínimo + re-test.

**Regla de oro:** primero el checklist básico ([FIELD_CAMPAIGN_CHECKLIST.md](FIELD_CAMPAIGN_CHECKLIST.md)). Si el smoke falla, **no** entréis en el circo adversario.

**Carpeta de logs:** `data/campaign_YYYYMMDD/adversarial/`  
Cada run: `T##_nombre_intentoN.csv` + `notes.md` (una frase: qué hiciste, qué esperabas, qué pasó).

---

## Principios (para no volvernos locos)

1. **Una variable por run** cuando podáis (si movéis dos cosas, no sabéis qué rompió).
2. **Repetir ≥3 veces** lo que quiera entrar en README Evidence.
3. **Barato:** papel de aluminio, microondas apagado como jaula, parking, mochila, nevera, túnel peatonal — no laboratorio de 50 k€.
4. **Legal/ético:** denegar *vuestra* antena; no jammear espectro; no claims anti-spoof RF.
5. **Árbitro / multi-técnica:** solo *logging* de scores si queréis; no retocar priors hasta tener ≥1 semana de coast medido.

---

## Semana 0 (día 1–2): cimiento — aburrido y obligatorio

| ID | Prueba | Disparate level | Qué mide | Fail interesante |
|----|--------|-----------------|----------|------------------|
| S0 | Smoke HYBRID cielo abierto | 0 | Arranque | No hay producto |
| S1 | Coast 30/60/120 s (antena en caja Faraday casera) | 1 | Drift vs tiempo | Drift explosivo → IMU/cal |
| S2 | Reacquire tras coast | 1 | Reenganche | No vuelve a HYBRID |
| S3 | Estático 20–60 min (Allan / bias) | 0 | Estabilidad IMU | Random walk raro |

**Salida mínima:** tabla `t_coast_s → residual_m` (media ± spread) + SHA firmware.

---

## Semana 1: tortura GNSS (el cielo miente / desaparece)

| ID | Prueba “disparatada” | Cómo (barato) | Qué debe aguantar el sistema |
|----|----------------------|---------------|------------------------------|
| G1 | **Jaula de hojalata** | Antena en caja metálica / varias capas de foil | DR limpio; sin NaN; quality↓ monótona |
| G2 | **Mochila tumba** | DUT+antena bajo ropa/mochila llena andando | Coast intermitente; no oscilar modo 10 Hz |
| G3 | **Parking subterráneo** | Bajada de rampa hasta perder fix | Transición HYBRID→DR sin glitch de posición |
| G4 | **Bosque / dosel** (campo) | Bajo árboles densos 5–15 min | Calidad degradada; no diverger “en silencio” |
| G5 | **Cañón urbano** | Calle estrecha entre edificios altos | Multipath: integrity debe **rechazar** basura, no tragársela |
| G6 | **Parpadeo de antena** | Desconectar/reconectar SMA cada 2–5 s × 2 min | Histéresis de modo; sin reset del filtro si política lo evita |
| G7 | **GPS zombie** | Fix válido pero **congelar** posición/vel en inyección host/SIL si tenéis hook; en campo: antena casi tapada que da 3Dfix podrido | `reject_reason=3` / outlier → DR; **no** seguir el zombie |
| G8 | **Saltos teleport** | Si hay replay: saltar LLA 50–200 m en un sample | Gate tumba el update |

**Éxito de semana 1:** “cuando el cielo falla o miente en software, no nos creemos al mentiroso”.

---

## Semana 2: tortura IMU / mecánica (el cuerpo miente)

| ID | Prueba | Cómo | Qué buscamos |
|----|--------|------|--------------|
| I1 | **Mesa vibratoria pobre** | Altavoz fuerte / taladro lejos del snubber / coche al ralentí | ¿IMU silence / cross-check / quality×0.5? |
| I2 | **Giro loco** | Girar DUT en yaw 360°/s a mano (cuidado cables) | Heading wrap; sin explosión de covarianza |
| I3 | **Caída amortiguada** | Dejar caer sobre cojín 20–40 cm | Spike accel; ¿recovery o divergencia? |
| I4 | **Hot / cold casero** | Sol directo 20 min vs nevera 10 min (DUT off o on según rating) | Bias shift; re-HYBRID tras thermal |
| I5 | **Orientación imposible** | Montar IMU rotada 90° a propósito (un run) | Detectar mount error vs “navegar mal con cara seria” |
| I6 | **NHC traidor** | Empujar lateral en vehículo/carrito con NHC on vs off | Confirmar política: always-on puede empeorar (ya lab) |

**Éxito de semana 2:** el filtro **falla ruidoso y recuperable**, no silencioso y elegante.

---

## Semana 3: tortura sistema / barato-ágil (producto serio)

| ID | Prueba | Cómo | Qué buscamos |
|----|--------|------|--------------|
| P1 | **USB yank** | Desenchufar CDC a mitad de coast | Reconexión; estado coherente o safe reboot |
| P2 | **Brownout pobre** | Alimentar con powerbank casi muerto / cable malo | Health CRITICAL; no corrupción de estado |
| P3 | **CPU starve** | Log ultra-verbose + Wi-Fi on si existe | WCET / health; modo degradado |
| P4 | **Log flood** | 30 min continuo caminando ciudad | ¿llena disco? ¿timestamps monótonos? |
| P5 | **Double boot** | Reset cada 30 s × 20 | Seed GNSS; no brick |
| P6 | **Mentira de dominio** | Marcar dominio AIR en carrito de supermercado (absurdo) | API no debe “inventar” física; solo priors |

**Éxito de semana 3:** equipo de campo usable por un integrador impaciente.

---

## Semana 4 (opcional): circo controlado — solo si 1–3 están verdes

Estas son las más “disparatadas”; **una al día máximo**:

| ID | Circo | Nota |
|----|-------|------|
| X1 | Coast en **bicicleta** bajo puente + subida | Dinámica real low-cost |
| X2 | **Carrito de compra** en centro comercial (GPS horrible) | Nicho AMR indoor-adjacent |
| X3 | Antena en **bolsa de Faraday** + sacudir como coctelera | Worst-case IMU+outage |
| X4 | Dos referencias: DUT vs **móvil** en misma mochila | Residual grosero barato |
| X5 | Inmersión **parcial** (caja estanca en cubo, antena GNSS fuera del agua o apagada) | Demo marítima *sin* fingir DVL |

---

## Motor de ensayo-error (obligatorio tras cada fail)

Plantilla en `notes.md` del run:

```text
FAIL_ID: G5-run2
Symptom: ...
Hypothesis: ...
Change (mínimo): code | cal | mount | procedure | none
Retest: PASS/FAIL
Keep in Evidence?: yes/no (solo yes si N≥3 y método fijo)
```

**Prohibido:** cambiar 5 knobs del ESKF a la vez “porque salió feo un log”.

**Permitido:** un PR pequeño por hipótesis (gate, health, mount matrix, docs).

---

## Qué construimos encima (robustez seria)

Al cerrar 2–4 semanas:

1. **Evidence pack** README: coast, integrity, thermal, vibration — MEASURED.
2. **Matriz de fallos conocidos** (este doc, sección viva abajo).
3. **Sólo entonces** retocar priors del `technique_arbiter` con datos, no a ojo.
4. **SKU piloto:** “GNSS resilience core + campaign script” — barato de enseñar.

### Matriz viva de fallos (rellenar)

| Fecha | ID | Síntoma | Root cause | Fix | Estado |
|-------|----|---------|------------|-----|--------|
| | | | | | |

---

## Lo que NO pediré (aunque sea “disparatado”)

- Jamming / spoofing RF a terceros  
- Claims de certificación (ASIL, DO-178) tras dos semanas de parking  
- Reescribir el ESKF por un solo puente feo  
- Convertir NaviCore en visión+LiDAR+sonar antes de coast estable  

---

## Pactó de trabajo (tú + agente)

- **Tú (campo):** ejecutáis IDs, subís logs/notes.  
- **Agente:** analiza fallos, propone el **cambio mínimo**, tests de regresión, actualiza Evidence con honestidad.  
- Ritmo: **un bloque S/G/I/P por sesión**; circo X solo con cimiento verde.

Cuando llegue la pieza el 3 de agosto: empezar por **S0→S3**, luego **G1–G3** el mismo día si hay luz. El resto es combustible para volvernos robustos de verdad.
