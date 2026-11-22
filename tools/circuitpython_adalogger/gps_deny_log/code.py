# NaviCore desk — Nivel 1 GPS log → microSD (CircuitPython on Feather RP2040 Adalogger)
# CSV: t_s, fix_ok, sats, lat_deg, lon_deg, qual
# Denial test: after ~60 s open sky, cover antenna 30–120 s, then uncover.
import board
import busio
import storage
import time

LOG_PATH = "/sd/gps_deny_log.csv"
SAMPLE_S = 1.0
UART_BAUD = 9600


def remount_sd_for_cp():
    try:
        storage.remount("/sd", readonly=False)
        return True
    except Exception as e:
        print("remount /sd:", e)
        return False


def nmea_checksum_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, csum = line[1:].partition("*")
    if len(csum) < 2:
        return False
    x = 0
    for ch in body:
        x ^= ord(ch)
    try:
        return x == int(csum[:2], 16)
    except ValueError:
        return False


def parse_latlon(raw, hemi):
    if not raw or not hemi:
        return None
    try:
        val = float(raw)
    except ValueError:
        return None
    deg = int(val // 100)
    minutes = val - deg * 100
    out = deg + minutes / 60.0
    if hemi in ("S", "W"):
        out = -out
    return out


def parse_gga(line):
    """Return (fix_ok, sats, lat, lon, qual) or None."""
    if not (line.startswith("$GNGGA") or line.startswith("$GPGGA")):
        return None
    if not nmea_checksum_ok(line):
        return None
    p = line.split(",")
    if len(p) < 8:
        return None
    try:
        qual = int(p[6]) if p[6] else 0
        sats = int(p[7]) if p[7] else 0
    except ValueError:
        return None
    lat = parse_latlon(p[2], p[3])
    lon = parse_latlon(p[4], p[5])
    fix_ok = 1 if qual > 0 and lat is not None and lon is not None else 0
    return fix_ok, sats, lat, lon, qual


def main():
    print("Nivel 1 GPS→SD logger")
    if not remount_sd_for_cp():
        print("FAIL: cannot remount /sd — is NAVSD inserted?")
        while True:
            time.sleep(1)

    uart = busio.UART(board.TX, board.RX, baudrate=UART_BAUD, timeout=0.2)
    buf = b""
    last = None
    t0 = time.monotonic()
    rows = 0

    with open(LOG_PATH, "w") as f:
        f.write("t_s,fix_ok,sats,lat_deg,lon_deg,qual\n")
        f.flush()

    print("logging to", LOG_PATH)
    print("open sky ~60s, then cover antenna 30-120s, then uncover")

    while True:
        chunk = uart.read(128)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line_b, buf = buf.split(b"\n", 1)
                try:
                    line = line_b.decode("ascii", "ignore").strip()
                except Exception:
                    continue
                parsed = parse_gga(line)
                if parsed:
                    last = parsed

        now = time.monotonic()
        if last is not None and (now - t0) >= SAMPLE_S * (rows + 1):
            fix_ok, sats, lat, lon, qual = last
            t_s = now - t0
            lat_s = "" if lat is None else "{:.7f}".format(lat)
            lon_s = "" if lon is None else "{:.7f}".format(lon)
            row = "{:.1f},{},{},{},{},{}\n".format(t_s, fix_ok, sats, lat_s, lon_s, qual)
            try:
                with open(LOG_PATH, "a") as f:
                    f.write(row)
                    f.flush()
                rows += 1
                print(row.strip())
            except Exception as e:
                print("write fail:", e)
                remount_sd_for_cp()

        time.sleep(0.05)


main()
