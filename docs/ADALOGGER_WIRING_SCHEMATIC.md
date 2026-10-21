# Esquema de montaje — Adalogger DUT (kit real)

**Stack activo (1×):** Adalogger RP2040 + Ultimate GPS FeatherWing + BNO055 STEMMA + antena + microSD + bat + USB-C→A  
**Apartado:** 2.º Adalogger RP2040 = **SPARE** (no apilar)

```
                    [ Antena GPS SMA + base ]
                              |
                         cable SMA
                              |
                    [ pigtail u.FL → SMA ]
                              |
                         conector u.FL
                              |
              ┌───────────────────────────────┐
              │   Ultimate GPS FeatherWing    │  ← apilado encima
              │   (UART del Feather)          │
              └───────────────┬───────────────┘
                              │ pines header (stack)
              ┌───────────────┴───────────────┐
              │  Feather RP2040 Adalogger     │
              │  STEMMA QT · microSD · USB-C  │
              │  JST-PH batería               │
              └───────┬───────────┬───────────┘
                      │           │
             STEMMA QT│           │ USB-C ──► USB-A ──► PC (ranura azul)
                      │           │
              ┌───────┴───────┐   │
              │   BNO055      │   │
              │   STEMMA QT   │   │
              └───────────────┘   │
                                  │
              [ Batería 3.7 V JST ]──(opcional switch en +)──► JST Adalogger
                                  │
              [ 2º Adalogger ]  SPARE — sin conexiones
```

## Orden (números = secuencia)

| # | Acción |
|---|--------|
| 1 | Todo desconectado (sin USB, sin bat) |
| 2 | microSD → ranura Adalogger |
| 3 | GPS FeatherWing **apilado** sobre Adalogger |
| 4 | u.FL Wing → pigtail → antena SMA (cielo) |
| 5 | STEMMA Adalogger ↔ BNO055 |
| 6 | USB-C Adalogger → USB-A PC |
| 7 | (Después de humo USB) batería JST + switch en + |
| 8 | PPK2 más tarde (medida de consumo) |

## Señales (qué bus usa cada cosa)

| Pieza | Bus / conector |
|-------|----------------|
| GPS FeatherWing | UART del Feather (por el stack de pines) |
| BNO055 | I2C STEMMA QT |
| Antena | RF u.FL → SMA |
| PC | USB-C datos |
| Batería | JST-PH 3,7 V |

## No conectar

- Pico 2 W / WT61C / NEO-M9N (archivado)  
- Cable USB-C → Micro USB al PC  
- 2.º Adalogger en el mismo sandwich  
- Batería antes de probar USB  

Guía larga: [`ADALOGGER_MOUNT_AND_FIELD_GUIDE.md`](ADALOGGER_MOUNT_AND_FIELD_GUIDE.md)
