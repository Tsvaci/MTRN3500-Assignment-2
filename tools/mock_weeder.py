"""
Stand-in for the Weeder's three services (NOT the simulator), for testing the full build.

  LiDAR   23000  auth "<7 digits>\\n" -> "OK\\n", then sRN LMDscandata -> 361-point DIST1 telegram.
                 The scene is read from a JSON file on every scan, so a test can move obstacles
                 while the exe runs:  {"obstacles": [{"x": 0.8, "y": 0.0, "r": 0.1}, ...]}
                 x forward, y left, metres, circles. Anything not hit returns 8 m.
  GNSS    24000  replays GPSData.dat (BESTUTM records, 112 bytes) at 10 records/s. Each record
                 is sent in two pieces to test reassembly, and every 7th record has one byte
                 corrupted to test the CRC check.
  VC      25000  auth like the LiDAR, then logs every "# steer speed wdog #" command as a JSON
                 line to the VC log file.

Usage: python mock_weeder.py <scene.json> <vc_log.jsonl> <GPSData.dat> [gnss_drop_after]
gnss_drop_after: close each GNSS connection after this many records (tests the GNSS restart).
"""
import json, math, socket, sys, threading, time

SCENE_FILE, VC_LOG, GPS_FILE = sys.argv[1], sys.argv[2], sys.argv[3]
GNSS_DROP_AFTER = int(sys.argv[4]) if len(sys.argv) > 4 else 0
N = 361
FAR_MM = 8000


def listen(port, handler):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(4)
    print(f"listening on {port}", flush=True)

    def loop():
        while True:
            c, _ = srv.accept()
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=handler, args=(c,), daemon=True).start()
    threading.Thread(target=loop, daemon=True).start()


def authenticate(conn, name):
    auth = b""
    while not auth.endswith(b"\n") and len(auth) < 16:
        c = conn.recv(16)
        if not c:
            return False
        auth += c
    ok = len(auth.strip()) == 7 and auth.strip().isdigit()
    conn.sendall(b"OK\n" if ok else b"NO\n")
    print(f"{name} auth {auth.strip()} -> {'OK' if ok else 'NO'}", flush=True)
    return ok


# ---- LiDAR
def load_scene():
    try:
        with open(SCENE_FILE) as f:
            return json.load(f).get("obstacles", [])
    except Exception:
        return []


def beam_range_mm(angle_deg, obstacles):
    # Beam direction in the vehicle frame: x = sin(theta), y = -cos(theta)
    a = math.radians(angle_deg)
    dx, dy = math.sin(a), -math.cos(a)
    best = FAR_MM / 1000.0
    for o in obstacles:
        cx, cy, r = o["x"], o["y"], o["r"]
        t = cx * dx + cy * dy                  # closest approach along the beam
        d2 = cx * cx + cy * cy - t * t
        if t > 0 and d2 <= r * r:
            hit = t - math.sqrt(r * r - d2)
            if 0 < hit < best:
                best = hit
    return int(round(best * 1000))


def telegram(k, obstacles):
    hdr = ["sRN", "LMDscandata", "1", "1", "ABCD1234", "0", "0", f"{k:X}", f"{k:X}",
           "3A9C", "3A9C", "0", "0", "0", "1388", "168", "0", "1", "DIST1",
           "3F800000", "00000000", "0", "1388", f"{N:X}"]
    data = [f"{beam_range_mm(i * 0.5, obstacles):X}" for i in range(N)]
    return b"\x02" + " ".join(hdr + data + ["0", "0", "0", "0"]).encode() + b"\x03"


def lidar(conn):
    try:
        if not authenticate(conn, "LiDAR"):
            return
        buf, n = b"", 0
        while True:
            c = conn.recv(256)
            if not c:
                break
            buf += c
            while b"\x03" in buf:
                req, _, buf = buf.partition(b"\x03")
                if req.lstrip(b"\x02") == b"sRN LMDscandata":
                    n += 1
                    conn.sendall(telegram(n, load_scene()))
    except Exception as e:
        print("LiDAR:", e, flush=True)
    finally:
        conn.close()


# ---- GNSS
def gnss(conn):
    data = open(GPS_FILE, "rb").read()
    records = [data[i:i + 112] for i in range(0, len(data) - 111, 112)]
    try:
        k = 0
        while not (GNSS_DROP_AFTER and k >= GNSS_DROP_AFTER):
            rec = bytearray(records[k % len(records)])
            k += 1
            if k % 7 == 0:
                rec[60] ^= 0xFF                 # corrupt one body byte -> CRC must fail
            conn.sendall(bytes(rec[:50]))
            time.sleep(0.01)
            conn.sendall(bytes(rec[50:]))
            time.sleep(0.09)
    except Exception as e:
        print("GNSS:", e, flush=True)
    finally:
        conn.close()


# ---- Vehicle control
def vc(conn):
    try:
        if not authenticate(conn, "VC"):
            return
        buf = b""
        with open(VC_LOG, "a") as log:
            while True:
                c = conn.recv(256)
                if not c:
                    break
                buf += c
                while buf.count(b"#") >= 2:
                    start = buf.index(b"#")
                    end = buf.index(b"#", start + 1)
                    fields = buf[start + 1:end].split()
                    buf = buf[end + 1:]
                    try:
                        steer, speed, wdog = float(fields[0]), float(fields[1]), int(fields[2])
                        log.write(json.dumps({"t": time.time(), "steer": steer, "speed": speed, "wdog": wdog}) + "\n")
                        log.flush()
                    except (IndexError, ValueError):
                        print("VC: bad command", fields, flush=True)
    except Exception as e:
        print("VC:", e, flush=True)
    finally:
        conn.close()


listen(23000, lidar)
listen(24000, gnss)
listen(25000, vc)
while True:
    time.sleep(1)
