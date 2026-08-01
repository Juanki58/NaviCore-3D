# Guía de montaje y validación — kit Adafruit (mesa)

**DUT activo:** Feather RP2040 Adalogger + Ultimate GPS FeatherWing + BNO055 + antena + bat + PPK2  
**microSD:** llega mañana — sin ella puedes encender y ver serie USB; **no** logs largos en tarjeta.  
**Firmware NaviCore Adalogger:** aún **no** existe en repo (`rp2040_adalogger/` pendiente). Día 1–2 = humo con sketches Adafruit; luego port.

Relacionado: [KIT_BOM…](KIT_BOM_ADALOGGER_BNO055_PA1010D.md) · [PORT](TARGET_RP2040_ADALOGGER_PORT.md) · [FIELD_CAMPAIGN_CHECKLIST.md](FIELD_CAMPAIGN_CHECKLIST.md)

---

## 1. Qué SÍ puedes hacer con lo que tienes

| Objetivo | Cómo | ¿Cuenta como Evidence NaviCore? |
|----------|------|----------------------------------|
| Encender stack, ver LED / USB | USB-C (ver §3) | Prep, no Evidence |
| Fix GNSS + NMEA en PC | GPS Wing + antena + Serial | Prep / antena OK |
| Outage casero (foil, parking) | Tapas antena / bajar a garaje; mirar fix age | **Parcial** — curva útil si logueas tiempo vs “fix sí/no” |
| Datos IMU (accel/gyro) | BNO055 en modo **AMG** (no NDOF como verdad) | Prep para ESKF |
| Consumo mA | **PPK2** en perfiles idle / GPS+IMU | **Sí** — tabla Power (DUT = Adalogger) |
| Log en microSD | Cuando llegue la tarjeta + sketch logger | Prep / campaña |
| Coast ESKF NaviCore en MCU | Hace falta port `rp2040_adalogger` | **Aún no** |

## 2. Qué NO puedes hacer (aún)

| No | Por qué |
|----|---------|
| Flashear `NaviCore3D_Pico2` | Archivado; otro hardware |
| Affirmar “NaviCore validado en campo” el día del unboxing | Falta FW + campaña medida |
| Usar fusión NDOF del BNO055 como navegación del producto | Doble filtro vs ESKF |
| Jamming / spoof RF | Ilegal |
| Confiar en el cable **USB-C → Micro USB** para el Adalogger | El Feather es **USB-C**; ese cable no habla con el PC en el puerto del Feather |

## 3. Qué falta para las pruebas de campo “de verdad”

| Falta | Para qué |
|-------|----------|
| **microSD** (mañana) | Logs largos en Adalogger |
| **Cable USB-C datos** (C→A o C→C) | Flash + CDC al PC |
| **Software** `src/targets/rp2040_adalogger/` | ESKF + NavState en el DUT |
| (Opcional) 2.ª STEMMA / breadboard | Solo si montáis GPS por I2C aparte (no hace falta: el Wing va apilado) |
| Protocolo + carpeta `data/campaign_YYYYMMDD/` | Evidence reproducible |

Con humo Adafruit + PPK2 + outages logueados ya sacáis valor. La Evidence “coast ESKF” espera el port.

---

## 4. Montaje mecánico / eléctrico (orden seguro)

### Piezas en mesa

1. Feather RP2040 **Adalogger** (5980)  
2. **Ultimate GPS FeatherWing**  
3. **BNO055** (ADA2472)  
4. Cable **STEMMA QT** 100 mm  
5. Pigtail **u.FL → SMA**  
6. Antena magnética SMA 3 m  
7. Batería 3,7 V JST-PH  
8. Interruptor SPST  
9. PPK2 (medidas después del humo)  
10. microSD — **mañana**  
11. Cable **USB-C datos** (comprar/usar si no lo tienes)

### Paso A — Solo USB (sin batería)

1. **No** conectes la batería todavía.  
2. Apila el **GPS FeatherWing** encima del Adalogger (todos los pines macho/hembra alineados; no forzar).  
3. En el GPS Wing, localiza el conector **u.FL** (pequeño, metal).  
4. Enchufa el pigtail **u.FL → SMA** (recto, sin girar a lo bruto).  
5. Enrosca la **antena SMA**; pon la base magnética en sitio con cielo (ventana / coche / terraza).  
6. STEMMA QT: del conector STEMMA del **Adalogger** → STEMMA del **BNO055** (da igual el sentido del cable).  
7. Conecta **USB-C** Adalogger → PC.  
8. El PC debería ver un disco o COM (según bootloader/CircuitPython/Arduino). Si no ves nada: cable datos, otro puerto, botón BOOT/RESET según doc Adafruit.

### Paso B — Batería + interruptor (después del humo USB)

1. **Polaridad JST-PH Adafruit:** suele ser cable rojo = BAT+, negro = GND. **Confirma** en la batería y en la serigrafía del Feather antes de enchufar. Polaridad invertida puede matar la placa.  
2. Corta el **positivo (+)** de la batería con el interruptor SPST en serie (un cable BAT+ → switch → BAT+ del JST hacia la placa; GND directo).  
3. Switch **OFF** → enchufa JST en el Feather → switch **ON**.  
4. Si también tienes USB, la placa puede cargar; no dejes el pack a cargo sin vigilancia la primera vez.

### Paso C — microSD (mañana)

1. Formato **FAT32**, preferible ≤32 GB para máxima compatibilidad.  
2. Insertar con la placa **apagada** (switch OFF / USB desconectado).  
3. Encender y comprobar que el sketch/logger escribe un fichero de prueba.

### Paso D — PPK2 (cuando el humo GPS+IMU funcione)

1. Lee el manual Nordic: modo source vs amperímetro.  
2. Mide la alimentación del DUT (BAT o USB según el perfil).  
3. Perfiles: idle MCU · GPS+IMU activos · +SD logging.  
4. Anota mA medios/pico → tabla Power del README (DUT = Adalogger).

---

## 5. Cómo validar las pruebas de campo (escalera)

### Nivel 0 — Humo (día llegada, sin NaviCore FW)

- [ ] PC ve el Adalogger por USB-C  
- [ ] NMEA / fix GPS con antena al cielo (sketch Adafruit Ultimate GPS)  
- [ ] BNO055 responde I2C; forzar/leer **AMG** (accel+gyro raw)  
- [ ] (Mañana) Escribe 1 archivo en microSD  

**Pass:** sensores vivos. **Fail:** no pases a outage largo.

### Nivel 1 — Campo “antena / denegación” (aún sin ESKF NaviCore)

- [ ] 5 min cielo abierto, log NMEA o CSV tiempo + lat/lon + fix  
- [ ] 30–120 s antena en caja foil / parking → fix perdido  
- [ ] Reacquire al salir  
- [ ] Tabla simple: `t_s, fix_ok, sats`  

**Pass:** sabéis denegar/reacquire GNSS en hardware real. Útil aunque el ESKF aún no corra en MCU.

### Nivel 2 — PPK2

- [ ] Tabla mA por perfil en README  

### Nivel 3 — Evidence NaviCore (cuando exista el port)

- [ ] Flash `rp2040_adalogger`  
- [ ] Modos HYBRID → DR en outage  
- [ ] Residual vs tiempo (checklist campo + campaña adversaria)  
- [ ] README Evidence con DUT = **Adalogger + BNO055 + GPS Wing**  

---

## 6. Software día 1 (mientras no hay port)

Opciones prácticas (elige una):

1. **Arduino + Adafruit boards** — ejemplos *Ultimate GPS* + *BNO055*  
2. **CircuitPython** en Adalogger — mismos sensores, más fácil log SD  

Objetivo del día: **NMEA en serie** + **IMU raw** + (mañana) **SD**.  
No hace falta Unity.

Cuando el port NaviCore esté listo, sustituís el sketch por el FW del repo.

---

## 7. Checklist anti-desastre

- [ ] Antena conectada **antes** de alimentar GPS con externa activa (buena práctica)  
- [ ] Polaridad batería comprobada  
- [ ] BNO055: no dejar NDOF como “verdad” de navegación  
- [ ] No mezclar pines Comarruga / Pico archivado  
- [ ] Etiquetar logs: `campaign_YYYYMMDD/` + nota de DUT  

---

*Si algo no encaja al abrir la caja (p. ej. el GPS Wing sin u.FL), para y foto: adaptamos el cableado sin improvisar.*
