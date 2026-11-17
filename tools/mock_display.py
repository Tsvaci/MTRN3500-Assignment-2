"""
Stand-in for the MATLAB display engine on 127.0.0.1:28000 (for testing only).
Receives X then Y as 361 little-endian doubles each (2888 bytes per array) and writes the
latest pair to a JSON file so a test can check the values. The provided engine plots X across
the screen (-8000..8000, + right) and Y ahead (0..8000), in millimetres.

Usage: python mock_display.py <out.json>
"""
import json, socket, struct, sys

OUT = sys.argv[1]
ARRAY_BYTES = 361 * 8

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", 28000))
srv.listen(1)
print("display listening on 28000", flush=True)
while True:
    conn, _ = srv.accept()
    buf, frames = b"", 0
    try:
        while True:
            c = conn.recv(8192)
            if not c:
                break
            buf += c
            while len(buf) >= 2 * ARRAY_BYTES:
                x = struct.unpack("<361d", buf[:ARRAY_BYTES])
                y = struct.unpack("<361d", buf[ARRAY_BYTES:2 * ARRAY_BYTES])
                buf = buf[2 * ARRAY_BYTES:]
                frames += 1
                with open(OUT, "w") as f:
                    json.dump({"frames": frames, "x": x, "y": y}, f)
    except Exception as e:
        print("display:", e, flush=True)
    finally:
        conn.close()
