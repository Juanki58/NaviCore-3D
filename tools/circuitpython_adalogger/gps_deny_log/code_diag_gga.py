# 90s GGA diagnostic → /sd/gga_diag.txt + /sd/gps_deny_log.csv then remount host-RW
import board
import busio
import storage
import time

DURATION_S = 90


def remount(rw_for_cp):
    try:
        storage.remount("/sd", readonly=not rw_for_cp)
        return True
    except Exception as e:
        print("remount:", e)
        return False


def main():
    print("GGA diag 90s")
    if not remount(True):
        while True:
            time.sleep(1)

    uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=0.2)
    buf = b""
    t0 = time.monotonic()
    n_lines = 0
    n_gga = 0
    n_fix = 0
    last_gga = ""

    with open("/sd/gga_diag.txt", "w") as diag, open("/sd/gps_deny_log.csv", "w") as csv:
        csv.write("t_s,fix_ok,sats,lat_deg,lon_deg,qual\n")
        next_row = t0 + 1.0
        latest = None
        latest_t = 0.0

        while time.monotonic() - t0 < DURATION_S:
            now = time.monotonic()
            chunk = uart.read(256)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    try:
                        line = raw.decode("ascii", "ignore").strip()
                    except Exception:
                        continue
                    if not line:
                        continue
                    n_lines += 1
                    if "GGA" in line:
                        n_gga += 1
                        last_gga = line[:120]
                        diag.write(line + "\n")
                        # crude fix: quality field
                        p = line.split(",")
                        try:
                            qual = int(p[6]) if len(p) > 6 and p[6] else 0
                        except ValueError:
                            qual = 0
                        if qual > 0:
                            n_fix += 1
                        latest = line
                        latest_t = now

            if now >= next_row:
                t_s = now - t0
                fix_ok = 0
                sats = 0
                lat = ""
                lon = ""
                qual = 0
                if latest and (now - latest_t) <= 3.0:
                    p = latest.split(",")
                    try:
                        qual = int(p[6]) if len(p) > 6 and p[6] else 0
                        sats = int(p[7]) if len(p) > 7 and p[7] else 0
                    except ValueError:
                        pass
                    if qual > 0 and len(p) > 5 and p[2] and p[4]:
                        fix_ok = 1
                        lat = p[2] + p[3]
                        lon = p[4] + p[5]
                csv.write("{:.1f},{},{},{},{},{}\n".format(t_s, fix_ok, sats, lat, lon, qual))
                csv.flush()
                diag.flush()
                print("t={:.0f} gga={} fix_lines={} last={}".format(t_s, n_gga, n_fix, last_gga[:60]))
                next_row += 1.0
            time.sleep(0.05)

        diag.write("\nSUMMARY lines={} gga={} fix_gga={}\n".format(n_lines, n_gga, n_fix))
        diag.write("last=" + last_gga + "\n")

    remount(False)
    print("DONE — read G:/gga_diag.txt")
    while True:
        time.sleep(1)


main()
