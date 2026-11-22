# Nivel 1 — GPS deny/reacquire log (CircuitPython)

**DUT:** Feather RP2040 Adalogger + Ultimate GPS FeatherWing + antena + microSD FAT32 (`NAVSD`).

## Qué hace

`code.py` escribe `/sd/gps_deny_log.csv`:

```text
t_s,fix_ok,sats,lat_deg,lon_deg,qual
```

Una fila por segundo a partir de `$GNGGA` / `$GPGGA`.

## Cómo correr

1. MicroSD en el Adalogger (FAT32).
2. Copia `code.py` a `CIRCUITPY/` (o abre Mu y guarda como `code.py`).
3. Antena al cielo / ventana. USB al PC. RESET.
4. Mu → Seriado: debe salir `logging to /sd/gps_deny_log.csv` y filas CSV.
5. **~60 s** cielo abierto → **tapa la antena** 30–120 s (foil / mano / caja) → destapa.
6. Para el log: en Mu Seriado Ctrl+C, o RESET; luego `storage.remount("/sd", readonly=True)` si hace falta ver `G:` en el PC, o lee el CSV desde la unidad SD.

## Pass

Tabla corta en notas / README Evidence desk:

| tramo | fix_ok típico | sats |
|-------|---------------|------|
| cielo | 1 | >0 |
| tapado | 0 | bajo / 0 |
| reopen | 1 | recupera |

## Repo

Fuente: `tools/circuitpython_adalogger/gps_deny_log/code.py`  
Checklist: `docs/ADALOGGER_MOUNT_AND_FIELD_GUIDE.md` § Nivel 1.
