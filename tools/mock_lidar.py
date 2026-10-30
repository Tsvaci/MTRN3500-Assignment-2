"""
Stand-in for the LiDAR service only (NOT the simulator). Speaks the SICK LMS1xx
sRA LMDscandata telegram format as configured in the service details:
0 to 180 degrees, 0.5 degree step, 361 DIST1 points, scale factor 1, distances in mm.

Protocol: client sends "<7 digits>\n" -> server replies "OK\n"; client sends
0x02 "sRN LMDscandata" 0x03 -> server replies 0x02 "sRA LMDscandata ..." 0x03.

The scene: a flat wall 2.0 m straight ahead (angle 90 deg) that spans the whole view,
plus a post 0.8 m away at 45 deg. Zero return for the first three beams, to exercise
the no-echo case.
"""
import socket, math, sys, threading, time

PORT = 23000
N = 361

def scene_mm(i):
    ang = math.radians(i * 0.5)             # 0..180
    if i < 3:
        return 0                             # no echo
    # post at 45 deg, 0.8 m, about 6 deg wide
    if abs(i * 0.5 - 45.0) <= 3.0:
        return 800
    # wall straight ahead at x = 2.0 m (x = r*sin(theta))
    s = math.sin(ang)
    if s < 0.05:
        return 8000                           # far away to the sides
    return int(min(2000.0 / s, 8000))

def telegram():
    hdr = ["sRA", "LMDscandata", "1", "1", "ABCD1234", "0", "0", "1C", "1C",
           "3A9C", "3A9C", "0", "0", "0", "1388", "168", "0", "1", "DIST1",
           "3F800000", "00000000", "0", "1388", f"{N:X}"]
    data = [f"{scene_mm(i):X}" for i in range(N)]
    tail = ["0", "0", "0", "0"]   # 8-bit channels, position, name, comment
    return b"\x02" + " ".join(hdr + data + tail).encode("ascii") + b"\x03"

def handle(conn, addr):
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(10)
        auth = b""
        while not auth.endswith(b"\n") and len(auth) < 16:
            c = conn.recv(16)
            if not c:
                return
            auth += c
        print("auth:", auth.strip(), flush=True)
        if len(auth.strip()) == 7 and auth.strip().isdigit():
            conn.sendall(b"OK\n")
        else:
            conn.sendall(b"NO\n")
            return
        buf = b""
        n = 0
        while True:
            c = conn.recv(256)
            if not c:
                break
            buf += c
            while b"\x03" in buf:
                req, _, buf = buf.partition(b"\x03")
                req = req.lstrip(b"\x02")
                if req == b"sRN LMDscandata":
                    conn.sendall(telegram())
                    n += 1
                else:
                    conn.sendall(b"\x02sFA 1\x03")
        print(f"client gone after {n} scans", flush=True)
    except Exception as e:
        print("handler:", e, flush=True)
    finally:
        conn.close()

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", PORT))
srv.listen(2)
print("mock LiDAR listening on", PORT, flush=True)
while True:
    c, a = srv.accept()
    threading.Thread(target=handle, args=(c, a), daemon=True).start()
